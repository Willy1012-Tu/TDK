"""HTTP and USB bridge checks with a fake serial device; no real ports opened."""
import http.client
import json
import queue
import sys
import tempfile
import threading
import unittest
from pathlib import Path
from unittest.mock import patch

sys.path.insert(0,str(Path(__file__).resolve().parents[1]/"tools"))
from web_gui import Application,Bridge,GatewayError,document_hash,make_server,wire_document,PROFILES,ConfigError


class FakePort:
    def __init__(self,*args,**kwargs):
        self.queue=queue.Queue(); self.closed=False; self.commands=[]; self.reject=""; self.commit_delay=0; self.commit_fault=False; self.entry_remaining=0; self.entry_fault=False; self.entry_delay=2; self.entry_starts=0; self.phase_history=[]
        self.status={"protocol":1,"firmware_id":"Stage4_Mechanism_Test","firmware_revision":"stage2-id67-home-v1","startup_return_zero":True,"mode":"normal","phase":"ready","completed":0,"step":0,"branch":0,
            "steps":6,"uploading":False,"runtime_plan":False,"plan_version":0,"pulse2":1150,"pulse3":0,"pulse4":2000,"pulse6":1400,"pulse9":1800,"pulse5":2100,
            "servo_min_us":500,"servo_max_us":2500,"pin2":2,"pin5":5,"motor_id":6,
            "motor_ids":[6,7],"servo_pins":[2,5,4,6,9],"stage3_enabled":False,"position_known_mask":12,"stage_ids":[2,3,4],"max_runtime_actions":2048,"previous_available":False,
            "stage":4,"pending_stage":0,"position4":0,"position5":0,"position6":0,"position7":0,
            "chassis":{"state":"locked","healthy":False,"emergency":False,"reason":"waiting","wheels":[{"node":10,"bus":1,"heartbeat_age_ms":None,"error":0,"axis_state":0,"velocity":0}]},
            "rc":{"connected":False,"age_ms":999,"stage":4,"channels":[992]*16}}
    def write(self,packet):
        request,command=packet.decode().strip().split(" ",1); self.commands.append(command)
        error=command.startswith(self.reject) if self.reject else False
        if not error:
            parts=command.split()
            if parts[0]=="mode": self.status["mode"]=parts[1]
            if parts[0]=="stage":
                old=self.status["stage"]; self.status["stage"]=int(parts[1])
                if self.status["stage"]==2 and old!=2: self.begin_entry()
            if parts[0]=="plan.begin": self.status.update(uploading=True); self.step_count=int(parts[1]); self.plan_stage=int(parts[3]) if len(parts)>3 else 4
            if parts[0]=="plan.commit":
                if self.commit_delay: self.status.update(phase="busy",pending_stage=self.plan_stage)
                else: self.commit()
            if parts[0]=="status" and self.status["pending_stage"]:
                if self.commit_fault: self.status.update(phase="fault",pending_stage=0,uploading=False)
                else:
                    self.commit_delay-=1
                    if not self.commit_delay: self.commit()
            if parts[0]=="status" and self.entry_remaining and self.status["phase"]!="fault":
                if self.entry_fault: self.status.update(phase="fault",uploading=False)
                else:
                    self.entry_remaining-=1
                    if not self.entry_remaining: self.status.update(phase="ready",completed=1,step=0,position7=-8000,pulse9=1800,pulse4=1300,position_known_mask=self.status["position_known_mask"]|8)
            if parts[0]=="plan.abort": self.status["uploading"]=False
            if parts[0]=="stop": self.status.update(phase="fault",uploading=False)
        self.phase_history.append(self.status["phase"])
        result="rejected" if error else json.dumps(self.status) if command=="status" else "accepted"
        response=f"{request} {'ERR' if error else 'OK'} {result}\n".encode()
        # Deliver split lines to exercise reassembly in the reader.
        for i in range(0,len(response),23): self.queue.put(response[i:i+23])
        return len(packet)
    def commit(self):
        self.status.update(phase="ready",pending_stage=0,stage=self.plan_stage,uploading=False,runtime_plan=True,steps=self.step_count,plan_version=self.status["plan_version"]+1)
        if self.plan_stage==2: self.begin_entry()
    def begin_entry(self):
        self.entry_remaining=self.entry_delay; self.entry_starts+=1
        self.status.update(phase="busy",step=1,completed=0)
    def read(self,n):
        try:return self.queue.get(timeout=0.02)
        except queue.Empty:return b""
    @property
    def in_waiting(self):return self.queue.qsize()*23
    def close(self):self.closed=True


class BridgeTests(unittest.TestCase):
    def setUp(self):
        self.port=FakePort(); self.bridge=Bridge(port_factory=lambda *args,**kwargs:self.port)
        self.bridge.connect("COM99")
    def tearDown(self): self.bridge.disconnect()
    def test_connect_queries_only_and_reassembles(self):
        self.assertEqual(self.port.commands,["status"])
        self.assertEqual(self.bridge.snapshot()["firmware"]["phase"],"ready")
    def test_modes_and_debug_numbers(self):
        with self.assertRaises(GatewayError):self.bridge.command({"command":"debug.move","position":0,"rpm":320,"acc":5})
        self.bridge.command({"command":"mode","mode":"debug"})
        self.bridge.command({"command":"debug.servo","pin":5,"angle":90.1,"duration_ms":1000})
        self.assertIn("debug.servo 5 901 1000",self.port.commands)
        with self.assertRaises(GatewayError):self.bridge.command({"command":"debug.move","position":True,"rpm":320,"acc":5})
        with self.assertRaises(GatewayError):self.bridge.command({"command":"debug.servo","pin":5,"angle":float("nan"),"duration_ms":1000})
        self.bridge.command({"command":"debug.move","node":6,"position":-4200,"rpm":320,"acc":2})
        self.bridge.command({"command":"debug.gohome","node":6})
        self.bridge.command({"command":"debug.servo","pin":4,"angle":90,"duration_ms":0})
        self.assertIn("debug.move 6 -4200 320 2",self.port.commands)
        self.assertIn("debug.gohome 6",self.port.commands)
        self.bridge.command({"command":"debug.move","node":7,"position":-3000,"rpm":320,"acc":2})
        self.bridge.command({"command":"debug.gohome","node":7})
        self.assertIn("debug.move 7 -3000 320 2",self.port.commands)
        self.assertIn("debug.gohome 7",self.port.commands)
        self.assertIn("debug.servo 4 900 0",self.port.commands)
        with self.assertRaises(GatewayError):self.bridge.command({"command":"debug.move","node":8,"position":0,"rpm":320,"acc":0})
    def test_apply_transaction_and_reset_detection(self):
        doc={"steps":[{"actions":[{"parallel":[{"servo":{"pin":5,"pulse_us":1500}},{"stepper":{"position":100,"rpm":320}}]},{"delay":{"ms":500}}]}]}
        self.bridge.apply(doc)
        commands=self.port.commands
        self.assertIn("plan.begin 1 4 4 1",commands)
        self.assertLess(commands.index("plan.action 0 0 100 320 0 0 0 0 0 1 6"),commands.index("plan.action 1 1 0 0 0 5 1500 0 0 1 0"))
        self.assertNotIn("run",commands); self.assertEqual(self.bridge.applied_hash,document_hash(doc))
        self.port.status.update(runtime_plan=False,plan_version=0)
        self.bridge.refresh(); self.assertIsNone(self.bridge.applied_hash)
    def test_failed_transfer_aborts(self):
        self.port.reject="plan.action"
        with self.assertRaises(GatewayError):self.bridge.apply({"steps":[{"actions":[{"delay":{"ms":100}}]}]})
        self.assertIn("plan.abort",self.port.commands); self.assertFalse(self.port.status["runtime_plan"])
    def test_stop_bypasses_operation_lock(self):
        with self.bridge.operation_lock:
            self.bridge.command({"command":"stop"})
        self.assertEqual(self.port.status["phase"],"fault")
    def test_bad_pin_cannot_apply(self):
        self.port.status["servo_pins"].remove(9) # Older firmware must reject a new pin before upload.
        with self.assertRaises(GatewayError):self.bridge.apply({"steps":[{"actions":[{"servo":{"pin":9,"pulse_us":1500}}]}]})
        self.assertNotIn("plan.begin",[s.split()[0] for s in self.port.commands])
    def test_stage3_standby_and_stage2_complete_ram(self):
        for stage in (2,3):
            source=Path(__file__).resolve().parents[1]/f"stage{stage}_steps.yaml"
            from generate_steps import StrictLoader,yaml
            document=yaml.load(source.read_text(encoding="utf-8"),Loader=StrictLoader)
            result=self.bridge.apply(document)
            self.assertEqual(result["stage"],stage)
        self.assertIn("plan.begin 4 15 2 2",self.port.commands)
        self.assertIn("plan.action 0 0 -8000 320 2 0 0 0 0 1 7",self.port.commands)
        self.assertIn("plan.action 1 1 0 0 0 9 1800 0 0 1 0",self.port.commands)
        self.assertIn("plan.action 8 1 0 0 0 6 2200 0 0 1 0",self.port.commands)
        self.assertIn("plan.begin 1 1 3 1",self.port.commands)
        self.assertFalse(any(command.startswith("debug.") or command in ("next","run","home") for command in self.port.commands))

    def test_stage2_commit_waits_for_prepare_and_idempotent_selection(self):
        from generate_steps import StrictLoader,yaml
        document=yaml.load((Path(__file__).resolve().parents[1]/"stage2_steps.yaml").read_text(encoding="utf-8"),Loader=StrictLoader)
        self.port.entry_delay=4
        with patch("web_gui.time.sleep"):
            result=self.bridge.apply(document)
        self.assertEqual(result["phase"],"ready")
        self.assertEqual(result["completed"],1)
        self.assertEqual(result["position7"],-8000)
        self.assertEqual(result["position_known_mask"]&8,8)
        self.assertIn("busy",self.port.phase_history)
        self.assertTrue(any("STEP1 預備已完成" in log["text"] for log in self.bridge.logs))
        starts=self.port.entry_starts
        self.bridge.command({"command":"stage","stage":2})
        self.assertEqual(self.port.entry_starts,starts)
        self.bridge.command({"command":"stage","stage":4})
        self.bridge.command({"command":"stage","stage":2})
        self.assertEqual(self.port.entry_starts,starts+1)
    def test_stop_interrupts_stage2_prepare_without_success(self):
        from generate_steps import StrictLoader,yaml
        document=yaml.load((Path(__file__).resolve().parents[1]/"stage2_steps.yaml").read_text(encoding="utf-8"),Loader=StrictLoader)
        self.port.entry_delay=6
        with patch("web_gui.time.sleep",side_effect=lambda _:self.bridge.command({"command":"stop"})),self.assertRaises(GatewayError):
            self.bridge.apply(document)
        self.assertIn("stop",self.port.commands)
        self.assertEqual(self.port.status["phase"],"fault")
        self.assertIsNone(self.bridge.applied_hash)
    def test_stage2_prepare_wait_has_bounded_timeout(self):
        from generate_steps import StrictLoader,yaml
        document=yaml.load((Path(__file__).resolve().parents[1]/"stage2_steps.yaml").read_text(encoding="utf-8"),Loader=StrictLoader)
        self.port.entry_delay=6
        with patch("web_gui.time.monotonic",side_effect=[0,0,1,40]),patch("web_gui.time.sleep"),self.assertRaises(GatewayError):
            self.bridge.apply(document)
        self.assertIsNone(self.bridge.applied_hash)
        self.assertIn("plan.abort",self.port.commands)
    def test_stage2_prepare_fault_does_not_mark_apply_success(self):
        from generate_steps import StrictLoader,yaml
        document=yaml.load((Path(__file__).resolve().parents[1]/"stage2_steps.yaml").read_text(encoding="utf-8"),Loader=StrictLoader)
        self.port.entry_fault=True
        with self.assertRaises(GatewayError): self.bridge.apply(document)
        self.assertIsNone(self.bridge.applied_hash)
        self.assertIn("plan.abort",self.port.commands)
    def test_five_debug_pins_inactive_hardware_and_standby(self):
        with self.assertRaises(GatewayError): self.bridge.command({"command":True})
        self.bridge.command({"command":"mode","mode":"debug"})
        for pin in (2,5,4,6,9):
            self.bridge.command({"command":"debug.servo","pin":pin,"angle":90,"duration_ms":1000})
            self.assertIn(f"debug.servo {pin} 900 1000",self.port.commands)
        for pin in (3,7):
            with self.assertRaises(GatewayError): self.bridge.command({"command":"debug.servo","pin":pin,"angle":90,"duration_ms":0})
        for node in (4,5):
            with self.assertRaises(GatewayError): self.bridge.command({"command":"debug.gohome","node":node})
        self.port.status["stage"]=3
        before=len(self.port.commands)
        with self.assertRaisesRegex(GatewayError,"待機"): self.bridge.command({"command":"debug.servo","pin":4,"angle":90,"duration_ms":0})
        self.assertEqual(self.port.commands[before:],["status"])
    def test_deferred_commit_waits_before_success(self):
        self.port.status["stage"]=3; self.port.commit_delay=3
        doc={"stage":4,"steps":[{"actions":[{"delay":{"ms":1}}]}]}
        with patch("web_gui.time.sleep"):
            result=self.bridge.apply(doc)
        self.assertEqual(result["stage"],4); self.assertEqual(result["phase"],"ready")
        self.assertEqual(self.port.commit_delay,0); self.assertEqual(self.bridge.applied_version,1)
    def test_deferred_commit_fault_does_not_mark_applied(self):
        self.port.status["stage"]=3; self.port.commit_delay=3; self.port.commit_fault=True
        with patch("web_gui.time.sleep"),self.assertRaises(GatewayError):
            self.bridge.apply({"stage":4,"steps":[{"actions":[{"delay":{"ms":1}}]}]})
        self.assertIsNone(self.bridge.applied_hash)
        self.assertIn("plan.abort",self.port.commands)
    def test_deferred_commit_timeout_is_bounded(self):
        self.port.status["stage"]=3; self.port.commit_delay=5
        with patch("web_gui.time.monotonic",side_effect=[0,0,1,40]),patch("web_gui.time.sleep"),self.assertRaises(GatewayError):
            self.bridge.apply({"stage":4,"steps":[{"actions":[{"delay":{"ms":1}}]}]})
        self.assertIsNone(self.bridge.applied_hash)
        self.assertEqual(self.port.status["plan_version"],0)
        self.assertIn("plan.abort",self.port.commands)
    def test_stop_interrupts_deferred_commit_wait(self):
        self.port.status["stage"]=3; self.port.commit_delay=5
        with patch("web_gui.time.sleep",side_effect=lambda _:self.bridge.command({"command":"stop"})),self.assertRaises(GatewayError):
            self.bridge.apply({"stage":4,"steps":[{"actions":[{"delay":{"ms":1}}]}]})
        self.assertIn("stop",self.port.commands)
        self.assertIsNone(self.bridge.applied_hash)
    def test_unsupported_motor_rejected_before_upload(self):
        self.port.status["motor_ids"]=[6]
        with self.assertRaises((GatewayError,ConfigError)):
            self.bridge.apply({"stage":3,"steps":[{"actions":[{"stepper":{"node":4,"position":0,"rpm":320}}]}]})
        self.assertNotIn("plan.begin",[s.split()[0] for s in self.port.commands])
    def test_old_firmware_rejects_id7_before_transfer(self):
        self.port.status["motor_ids"]=[6]
        with self.assertRaises(GatewayError): self.bridge.apply({"stage":2,"steps":[{"actions":[{"stepper":{"node":7,"position":-8000,"rpm":320,"acc":2}}]}]})
        self.assertNotIn("plan.begin",[command.split()[0] for command in self.port.commands])
    def test_failed_handshake_closes_port(self):
        self.bridge.disconnect(); self.port=FakePort(); self.port.reject="status"
        self.bridge=Bridge(port_factory=lambda *args,**kwargs:self.port)
        with self.assertRaises(GatewayError):self.bridge.connect("COM99")
        self.assertFalse(self.bridge.snapshot()["connected"]); self.assertTrue(self.port.closed)

    def test_three_stages_and_unsupported_previous_command(self):
        for stage in PROFILES:
            self.bridge.command({"command":"stage","stage":stage})
            self.assertEqual(self.bridge.last_status["stage"],stage)
        self.assertEqual(self.bridge.last_status["rc"]["channels"],[992]*16)
        self.assertEqual(self.bridge.last_status["chassis"]["wheels"][0]["node"],10)
        with self.assertRaises(GatewayError): self.bridge.command({"command":"prev"})
        self.assertNotIn("prev",self.port.commands)

    def test_vase_and_placeholder_apply_keep_stage_metadata(self):
        for stage in (stage for stage in (1,2) if stage in PROFILES):
            document={"stage":stage,"repeat_from":1,"steps":[{"actions":[{"delay":{"ms":0}}]}]}
            self.bridge.apply(document)
            self.assertIn(f"plan.begin 1 1 {stage} 1",self.port.commands)
            self.assertEqual(self.bridge.last_status["stage"],stage)

    def test_legacy_stage_and_runtime_capacity_reject_before_transfer(self):
        del self.port.status["stage_ids"]
        with self.assertRaises(GatewayError):self.bridge.apply({"stage":2,"steps":[{"actions":[{"delay":{"ms":0}}]}]})
        self.port.status["max_runtime_actions"]=1
        with self.assertRaises(GatewayError):self.bridge.apply({"stage":4,"steps":[{"actions":[{"delay":{"ms":0}},{"delay":{"ms":1}}]}]})
        self.assertNotIn("plan.begin",[s.split()[0] for s in self.port.commands])


class HttpTests(unittest.TestCase):
    def setUp(self):
        self.temp=tempfile.TemporaryDirectory(); root=Path(self.temp.name)
        (root/"stage4_steps.yaml").write_text("steps: [{actions: [{delay: {ms: 100}}]}]",encoding="utf-8")
        (root/"stage3_steps.yaml").write_text("stage: 3\nrepeat_from: 1\nsteps: [{actions: [{delay: {ms: 0}}]}]",encoding="utf-8")
        for stage in (stage for stage in (1,2) if stage in PROFILES):
            (root/f"stage{stage}_steps.yaml").write_text(f"stage: {stage}\nsteps: [{{actions: [{{delay: {{ms: 0}}}}]}}]",encoding="utf-8")
        (root/"web").mkdir(); (root/"web"/"index.html").write_text("<html>Stage4</html>",encoding="utf-8")
        self.port=FakePort(); self.app=Application(root,Bridge(lambda *args,**kwargs:self.port))
        self.server=make_server("127.0.0.1",0,self.app)
        self.thread=threading.Thread(target=self.server.serve_forever,daemon=True); self.thread.start()
    def tearDown(self):
        self.app.bridge.disconnect(); self.server.shutdown(); self.server.server_close(); self.thread.join(); self.temp.cleanup()
    def request(self,path,payload=None,token=True,headers=None):
        connection=http.client.HTTPConnection("127.0.0.1",self.server.server_port,timeout=3)
        h={"Content-Type":"application/json"}
        if token:h["X-Stage4-Token"]=self.app.csrf
        if headers:h.update(headers)
        connection.request("GET" if payload is None else "POST",path,body=None if payload is None else json.dumps(payload),headers=h)
        response=connection.getresponse(); data=response.read(); status=response.status; connection.close()
        return status,json.loads(data) if path.startswith("/api/") else data.decode()
    def test_static_and_tokens(self):
        self.assertEqual(self.request("/")[0],200)
        self.assertEqual(self.request("/api/init")[1]["token"],self.app.csrf)
        self.assertEqual(self.request("/api/connect",{"port":"COM99"},token=False)[0],403)
        self.assertEqual(self.request("/api/connect",{"port":"COM99"},headers={"Origin":"http://evil.invalid"})[0],403)
        self.assertEqual(self.request("/api/init",headers={"Host":"evil.invalid"})[0],403)
        self.assertEqual(self.request("/../stage4_steps.yaml")[0],404)
    def test_save_conflict_and_apply_without_run(self):
        config=self.request("/api/init")[1]["config"]
        doc={"steps":[{"actions":[{"servo":{"pin":2,"pulse_us":1500}}]}]}
        saved=self.request("/api/config",{"document":doc,"expected_hash":config["hash"]})
        self.assertEqual(saved[0],200); self.assertIn("pulse_us: 1500",saved[1]["result"]["yaml"])
        self.assertEqual(self.request("/api/config",{"document":doc,"expected_hash":config["hash"]})[0],409)
        self.assertEqual(self.request("/api/connect",{"port":"COM99"})[0],200)
        self.assertEqual(self.request("/api/apply",{"document":doc})[0],200)
        self.assertNotIn("run",self.port.commands)
        self.assertEqual(self.request("/api/command",{"command":"run"})[0],200)
    def test_invalid_apply_leaves_device_untouched(self):
        self.request("/api/connect",{"port":"COM99"})
        self.assertEqual(self.request("/api/apply",{"document":{"steps":[]}})[0],400)
        self.assertEqual(self.port.commands,["status"])
    def test_profiles_load_and_save_independently_without_motion(self):
        third=self.request("/api/init?profile=3")[1]["config"]
        fourth=self.request("/api/init?profile=4")[1]["config"]
        self.assertEqual(third["document"]["stage"],3)
        self.assertNotEqual(third["hash"],fourth["hash"])
        doc={"stage":3,"steps":[{"actions":[{"delay":{"ms":0}}]}]}
        self.assertEqual(self.request("/api/config",{"document":doc,"expected_hash":third["hash"],"profile":3})[0],200)
        self.assertEqual(self.request("/api/init?profile=4")[1]["config"]["hash"],fourth["hash"])
        self.assertEqual(self.port.commands,[])
        self.assertEqual(self.request("/api/config",{"document":doc,"expected_hash":fourth["hash"],"profile":4})[0],409)
        self.assertEqual(self.request("/api/init?profile=../../stage3_steps.yaml")[0],400)
        self.assertEqual(self.request("/api/init?profile=5")[0],400)

    def test_stage2_http_roundtrip_ram_and_new_debug_pins(self):
        from generate_steps import StrictLoader,yaml
        source=Path(__file__).resolve().parents[1]/"stage2_steps.yaml"
        document=yaml.load(source.read_text(encoding="utf-8"),Loader=StrictLoader)
        config=self.request("/api/init?profile=2")[1]["config"]
        self.assertEqual(self.request("/api/config",{"document":document,"expected_hash":config["hash"],"profile":2})[0],200)
        self.assertEqual(self.request("/api/init?profile=2")[1]["config"]["document"],document)
        self.assertEqual(self.request("/api/connect",{"port":"COM99"})[0],200)
        self.assertEqual(self.request("/api/apply",{"document":document})[0],200)
        self.assertIn("plan.begin 4 15 2 2",self.port.commands)
        self.assertNotIn("run",self.port.commands)
        self.assertEqual(self.port.status["completed"],1)
        self.assertEqual(self.port.status["position7"],-8000)
        self.assertEqual(self.port.status["phase"],"ready")
        self.assertEqual(self.request("/api/command",{"command":"mode","mode":"debug"})[0],200)
        for pin in (4,6,9):
            self.assertEqual(self.request("/api/command",{"command":"debug.servo","pin":pin,"angle":90,"duration_ms":0})[0],200)
        self.assertEqual(self.request("/api/command",{"command":"debug.servo","pin":3,"angle":90,"duration_ms":0})[0],409)
        self.assertEqual(self.request("/api/command",{"command":"debug.gohome","node":4})[0],409)
        bad={"stage":3,"steps":[{"actions":[{"servo":{"pin":4,"pulse_us":1500}}]}]}
        before=len(self.port.commands)
        self.assertEqual(self.request("/api/apply",{"document":bad})[0],400)
        self.assertEqual(len(self.port.commands),before)

    def test_four_profile_metadata_and_placeholder_roundtrip(self):
        profiles=self.request("/api/init")[1]["profiles"]
        self.assertEqual([p["stage"] for p in profiles],list(PROFILES))
        for stage in (stage for stage in (1,2) if stage in PROFILES):
            result=self.request(f"/api/init?profile={stage}")
            self.assertEqual(result[0],200)
            config=result[1]["config"]
            self.assertEqual(config["document"]["stage"],stage)
            self.assertEqual(self.request("/api/config",{"document":config["document"],"expected_hash":config["hash"],"profile":stage})[0],200)
        self.assertEqual(self.port.commands,[])


if __name__=="__main__": unittest.main()
