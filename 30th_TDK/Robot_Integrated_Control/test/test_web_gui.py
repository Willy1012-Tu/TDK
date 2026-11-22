"""HTTP and USB bridge checks with a fake serial device; no real ports opened."""
import time
import copy
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
from web_gui import Application,Bridge,GatewayError,document_hash,make_server,wire_document,PROFILES,ConfigError,profile_identity,yaml,StrictLoader


class FakePort:
    def __init__(self,*args,**kwargs):
        self.plan_actions={}; self.plan_steps={}; self.profile_plans={}; self.motion_profile_before_write=None; self.servo_durations={}
        self.queue=queue.Queue(); self.closed=False; self.commands=[]; self.reject=""
        self.commit_delay=0; self.commit_fault=False; self.commit_pending=False
        self.return_remaining=0; self.return_fault=False; self.return_delay=2; self.return_starts=0
        self.return_completed=[]; self.return_snapshots=[]; self.phase_history=[]
        self.home_remaining=0; self.home_fault=False; self.home_delay=2; self.home_starts=0
        self.home_completed=[]; self.busy_remaining=0
        self.entry_home_remaining=0; self.entry_home_delay=2; self.entry_home_starts=0; self.entry_home_snapshots=[]
        self.return_cylinder_started=False; self.entry_home_entered=threading.Event()
        self.status={"protocol":1,"firmware_id":"Robot_Integrated_Control","firmware_revision":"bench-test-v7","bench_test_mode":True,"motor_online_mask":7,"motor_offline_mask":0,"startup_return_zero":True,"mode":"normal","phase":"ready","completed":0,"step":0,"branch":0,
            "steps":6,"uploading":False,"runtime_plan":False,"plan_version":0,"pulse2":1150,"pulse3":2250,"pulse4":2000,"pulse6":1400,"pulse9":1100,"pulse5":2100,"pulse10":1500,"pulse11":1500,"pulse12":1500,
            "servo_min_us":500,"servo_max_us":2500,"pin2":2,"pin5":5,"pin10":10,"pin11":11,"pin12":12,"motor_id":6,
            "current4_ma":3000,"current5_ma":3000,"current6_ma":5200,"current_configured_mask":7,
            "startup_current_settings":[{"node":4,"milliamps":3000},{"node":5,"milliamps":3000},{"node":6,"milliamps":5200}],
            "motor_ids":[4,5,6],"servo_pins":[2,3,4,5,6,9,10,11,12],"stage3_enabled":True,"position_known_mask":7,"stage_ids":[1,2,3,4,5],"stage1_prop_ids":[1,2,3,4,5],"prop":0,"pending_prop":0,"prop_complete":False,"max_runtime_actions":512,"previous_available":False,
            "stage":4,"pending_stage":0,"position4":0,"position5":0,"position6":0,
            "chassis":{"direction_stage":4,"state":"locked","healthy":False,"emergency":False,"reason":"waiting","wheels":[{"node":10,"bus":1,"heartbeat_age_ms":None,"error":0,"axis_state":0,"velocity":0}]},
            "pwm":{"available":True,"pin":14,"value":0,"max_value":55,"step_interval_ms":5,
                   "default_max_value":55,"default_step_interval_ms":5,"min_step_interval_ms":1,"max_step_interval_ms":1000},
            "cylinder":{"available":True,"action":0,"active":False,"duration_ms":0,"remaining_ms":0,"in1_pin":17,"in2_pin":16,"in1_high":False,"in2_high":False,"reference_action":2,"reference_known":True,"fully_retracted":True},
            "rc":{"connected":False,"age_ms":999,"stage":4,"channels":[992]*16}}
    def write(self,packet):
        request,command=packet.decode().strip().split(" ",1); self.commands.append(command)
        error=command.startswith(self.reject) if self.reject else False
        if not error:
            parts=command.split()
            if parts[0]=="chassis.motion":
                keys=("ramp_enabled","accel","decel","wheelbase_mm","track_mm","icr_x_mm","icr_left_y_mm","icr_right_y_mm")
                motion=self.status["chassis"]["motion"]
                if self.motion_profile_before_write is not None:
                    motion["profile_id"]=self.motion_profile_before_write
                    self.motion_profile_before_write=None
                error=len(parts)!=10 or int(parts[-1])!=motion["profile_id"]
                if not error:
                    settings=dict(zip(keys,map(int,parts[1:9])))
                    settings["ramp_enabled"]=bool(settings["ramp_enabled"])
                    motion.update(settings)
            if parts[0]=="pwm.set":
                maximum,interval=int(parts[1]),int(parts[2])
                self.status["pwm"].update(max_value=maximum,step_interval_ms=interval,
                                          value=min(self.status["pwm"]["value"],maximum))
            if parts[0]=="debug.servo":
                pin,angle10,duration=map(int,parts[1:])
                self.status[f"pulse{pin}"]=500+(angle10*2000+900)//1800
                self.servo_durations[pin]=duration
            if parts[0]=="mode": self.status["mode"]=parts[1]
            if parts[0]=="stage":
                self.stop_cylinder()
                target=int(parts[1]); prop=(int(parts[2]) if len(parts)>2 else 2) if target==1 else 0
                if self.status["phase"] in ("return_wait","home_wait"):
                    self.status.update(pending_stage=target,pending_prop=prop)
                elif self.status["phase"]=="busy":
                    self.status.update(pending_stage=target,pending_prop=prop)
                    if not self.busy_remaining: self.busy_remaining=2
                elif (target,prop)!=(self.status["stage"],self.status["prop"]): self.begin_return(target,prop)
            if parts[0]=="stage1.exit":
                self.status.update(pending_stage=int(parts[1]) if len(parts)>1 else 4,pending_prop=0); self.begin_home()
            if parts[0]=="prop":
                prop=int(parts[1]); count=self.profile_plans.get((1,prop),{}).get("steps",[2,3,2,4,4][prop-1])
                self.status.update(stage=1,prop=prop,phase="busy",step=1,completed=0,steps=count,prop_complete=False,previous_available=False)
                self.busy_remaining=2
            if parts[0]=="debug.cylinder":
                act,duration=int(parts[1]),int(parts[2])
                self.set_cylinder(act,duration)
                if act==0 and duration==0:
                    self.status.update(phase="ready"); self.busy_remaining=0
                else:
                    self.status.update(phase="busy"); self.busy_remaining=2
            if parts[0]=="home": self.begin_home()
            if parts[0]=="plan.begin": self.plan_actions={}; self.plan_steps={}; self.status.update(uploading=True); self.step_count=int(parts[1]); self.plan_stage=int(parts[3]) if len(parts)>3 else 4; self.plan_prop=(int(parts[5]) if len(parts)>5 else 2) if self.plan_stage==1 else 0
            if parts[0]=="plan.action": self.plan_actions[int(parts[1])]=list(map(int,parts[2:]))
            if parts[0]=="plan.step": self.plan_steps[int(parts[1])]=(int(parts[2]),int(parts[3]))
            if parts[0]=="plan.commit": self.commit_pending=True; self.begin_return(self.plan_stage,self.plan_prop)
            if parts[0]=="status":
                if self.status["phase"]=="busy" and self.busy_remaining:
                    self.busy_remaining-=1
                    if not self.busy_remaining:
                        if self.status["pending_stage"]: self.begin_return(self.status["pending_stage"],self.status["pending_prop"])
                        else:
                            if self.status["cylinder"]["active"]: self.complete_cylinder()
                            self.status.update(phase="ready",completed=self.status["step"],previous_available=True,prop_complete=self.status["stage"]==1 and self.status["step"]>=self.status["steps"])
                elif self.status["phase"]=="home_wait" and self.entry_home_remaining:
                    self.entry_home_snapshots.append(dict(self.status))
                    self.entry_home_remaining-=1
                    if not self.entry_home_remaining: self.finish_entry_home()
                    else: self.status["cylinder"]["remaining_ms"]=2500
                elif self.status["phase"]=="home_wait" and self.home_remaining:
                    if self.home_fault: self.status.update(phase="fault",pending_stage=0); self.home_remaining=0; self.stop_cylinder()
                    else:
                        self.home_remaining-=1
                        if not self.home_remaining: self.finish_home()
                elif self.status["phase"]=="return_wait" and self.return_remaining:
                    self.return_snapshots.append(dict(self.status))
                    if self.status["stage"]==2 and not self.return_cylinder_started and not self.status["cylinder"]["fully_retracted"]:
                        self.set_cylinder(2,5000); self.return_cylinder_started=True
                    if self.return_fault or (self.commit_pending and self.commit_fault):
                        self.status.update(phase="fault",pending_stage=0,uploading=False); self.return_remaining=0; self.stop_cylinder()
                    elif self.commit_delay: self.commit_delay-=1
                    else:
                        self.return_remaining-=1
                        if not self.return_remaining: self.finish_return()
            if parts[0] in ("next","run"):
                step=self.status["completed"]+1 if parts[0]=="next" and self.status["stage"]==1 else 1
                self.status.update(phase="busy",step=step)
                if self.status["stage"]==1: self.busy_remaining=2
            if parts[0]=="plan.abort": self.commit_pending=False; self.status.update(uploading=False,pending_stage=0)
            if parts[0]=="stop":
                self.status.update(phase="fault",pending_stage=0,uploading=False)
                self.home_remaining=0; self.entry_home_remaining=0; self.return_remaining=0; self.busy_remaining=0; self.commit_pending=False; self.stop_cylinder()
        self.status["chassis"]["direction_stage"]=self.status["pending_stage"] or self.status["stage"]
        self.phase_history.append(self.status["phase"])
        result="rejected" if error else json.dumps(self.status) if command=="status" else "accepted"
        response=f"{request} {'ERR' if error else 'OK'} {result}\n".encode()
        # Deliver split lines to exercise reassembly in the reader.
        for i in range(0,len(response),23): self.queue.put(response[i:i+23])
        return len(packet)
    def begin_return(self,target,prop=0):
        self.stop_cylinder(); self.return_cylinder_started=False
        self.return_remaining=self.return_delay; self.return_starts+=1
        self.status.update(phase="return_wait",step=0,branch=0,pending_stage=target,pending_prop=prop,previous_available=False)
    def reset_current_stage(self):
        stage=self.status["stage"]
        if stage==3: self.status.update(position4=0,position5=0,pulse3=2250)
        elif stage==2:
            self.complete_cylinder(reference=2)
            self.status.update(position4=0,position5=0,pulse3=2250,pulse4=2000,pulse6=1400,pulse9=1100)
        elif stage in (1,4): self.status.update(position6=0,pulse2=1150,pulse5=2100)
    def finish_return(self):
        self.return_completed.append(self.status["stage"]); self.reset_current_stage()
        target=self.status["pending_stage"] or self.status["stage"]; prop=self.status["pending_prop"] if target==1 else 0
        if self.commit_pending:
            target=self.plan_stage; prop=self.plan_prop
            self.profile_plans[(target,prop)]={"steps":self.step_count,"actions":dict(self.plan_actions)}
            self.status.update(runtime_plan=True,steps=self.step_count,plan_version=self.status["plan_version"]+1)
        self.commit_pending=False
        self.status.update(stage=target,prop=prop,pending_stage=0,pending_prop=0,prop_complete=False,phase="ready",uploading=False,step=0,branch=0,completed=0,previous_available=False)
        if target==2: self.begin_entry_home()
    def begin_home(self):
        if self.status["stage"]==2: self.set_cylinder(2,5000)
        self.home_remaining=self.home_delay; self.home_starts+=1
        self.status.update(phase="home_wait",step=0,branch=0,completed=0,previous_available=False)
    def finish_home(self):
        self.home_completed.append(self.status["stage"]); self.reset_current_stage()
        target=self.status["pending_stage"]; prop=self.status["pending_prop"]
        self.status.update(phase="ready",step=0,branch=0,completed=0,previous_available=False,pending_stage=0)
        if target and (target,prop)!=(self.status["stage"],self.status["prop"]): self.begin_return(target,prop)
    def set_cylinder(self,action,duration):
        c=self.status["cylinder"]
        if c["active"]: c["reference_known"]=False
        if action in (1,2) and duration>0: c["fully_retracted"]=False
        c.update(action=action,active=duration>0,duration_ms=duration,remaining_ms=duration,
                 in1_high=action==1 and duration>0,in2_high=action==2 and duration>0)
    def stop_cylinder(self):
        c=self.status["cylinder"]
        if c["active"]: c.update(reference_known=False,fully_retracted=False)
        c.update(action=0,active=False,duration_ms=0,remaining_ms=0,in1_high=False,in2_high=False)
    def complete_cylinder(self,reference=None):
        c=self.status["cylinder"]; reference=c["action"] if reference is None else reference
        full=c["fully_retracted"] or (reference==2 and c["duration_ms"]>=5000)
        self.stop_cylinder()
        if reference in (1,2): c.update(reference_action=reference,reference_known=True,fully_retracted=full)
    def begin_entry_home(self):
        self.status.update(pulse3=1580,pulse4=1300,pulse6=1400,pulse9=1800)
        if self.status["cylinder"]["fully_retracted"]:
            return
        self.entry_home_starts+=1; self.entry_home_remaining=self.entry_home_delay
        self.status.update(phase="home_wait",step=0,completed=0,previous_available=False)
        self.set_cylinder(2,5000); self.entry_home_entered.set()
        if not self.entry_home_remaining: self.finish_entry_home()
    def finish_entry_home(self):
        self.complete_cylinder(reference=2)
        self.status.update(pulse3=1580,pulse4=1300,pulse6=1400,pulse9=1800,
                           phase="ready",step=0,completed=0,previous_available=False)
        target=self.status["pending_stage"]; prop=self.status["pending_prop"]
        if target and (target,prop)!=(self.status["stage"],self.status["prop"]): self.begin_return(target,prop)
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
    def complete_motion(self):
        for _ in range(100):
            status=self.bridge.refresh()
            if status["phase"]=="ready": return status
            if status["phase"]=="fault": self.fail("fake motion fault")
        self.fail("fake return/HOME did not complete")
    def motion_setup(self):
        self.port.status["chassis"]["motion"]={"available":True,"editable":True,"profiles":True,"profile_id":0,"ramp_enabled":True,
            "accel":90,"decel":180,"wheelbase_mm":260,"track_mm":320,"icr_mode":"directional","icr_x_mm":0,"icr_left_y_mm":-500,"icr_right_y_mm":500,"target":[0]*4}
        self.bridge.refresh()
        return {"command":"chassis.motion","profile_id":0,"ramp_enabled":1,"accel":60,"decel":120,
                "wheelbase_mm":260,"track_mm":320,"icr_x_mm":130,"icr_left_y_mm":-350,"icr_right_y_mm":700}

    def test_motion_ram_command_roundtrip(self):
        payload=self.motion_setup()
        result=self.bridge.command(payload)
        self.assertEqual(result["chassis"]["motion"]["icr_x_mm"],130)
        self.assertEqual(result["chassis"]["motion"]["icr_left_y_mm"],-350)
        self.assertEqual(result["chassis"]["motion"]["icr_right_y_mm"],700)
        self.assertTrue(result["chassis"]["motion"]["ramp_enabled"])
        self.assertIn("chassis.motion 1 60 120 260 320 130 -350 700 0",self.port.commands)

    def test_motion_old_firmware_and_missing_second_y_rejected(self):
        payload=self.motion_setup()
        del payload["icr_right_y_mm"]
        with self.assertRaises(GatewayError): self.bridge.command(payload)
        payload=self.motion_setup()
        del self.port.status["chassis"]["motion"]["icr_mode"]
        self.bridge.refresh()
        with self.assertRaises(GatewayError): self.bridge.command(payload)
        self.assertFalse(any(c.startswith("chassis.motion") for c in self.port.commands))

    def test_motion_invalid_payload_is_nonmutating(self):
        valid=self.motion_setup()
        for key,bad in (("accel",0),("decel",10001),("wheelbase_mm",0),("track_mm",-1),
                        ("icr_x_mm",5001),("icr_left_y_mm",-5001),("icr_right_y_mm",5001),("accel",True),("ramp_enabled",2),
                        ("icr_x_mm",1.5),("accel","90"),("decel",float("inf")),("profile_id",True),("profile_id",2),("profile_id","0")):
            with self.subTest(key=key,bad=bad):
                with self.assertRaises(GatewayError): self.bridge.command({**valid,key:bad})
        self.assertFalse(any(c.startswith("chassis.motion") for c in self.port.commands))

    def test_motion_requires_stationary_fresh_supported_state(self):
        payload=self.motion_setup()
        self.port.status["chassis"]["motion"]["editable"]=False;self.bridge.refresh()
        with self.assertRaises(GatewayError):self.bridge.command(payload)
        self.port.status["chassis"]["motion"]["editable"]=True;self.bridge.refresh()
        self.bridge.status_time=time.monotonic()-5
        with self.assertRaises(GatewayError):self.bridge.command(payload)
        del self.port.status["chassis"]["motion"];self.bridge.refresh()
        with self.assertRaises(GatewayError):self.bridge.command(payload)
        self.assertFalse(any(c.startswith("chassis.motion") for c in self.port.commands))

    def test_motion_rechecks_after_initial_capability(self):
        payload=self.motion_setup()
        self.port.status["chassis"]["motion"]["editable"]=False
        with self.assertRaises(GatewayError):self.bridge.command(payload)
        self.assertFalse(any(c.startswith("chassis.motion") for c in self.port.commands))

    def test_motion_profiles_capability_and_profile_required(self):
        payload=self.motion_setup()
        for key in ("profiles","profile_id"):
            self.motion_setup()
            del self.port.status["chassis"]["motion"][key]
            self.bridge.refresh()
            with self.subTest(key=key),self.assertRaises(GatewayError): self.bridge.command(payload)
        self.motion_setup(); del payload["profile_id"]
        with self.assertRaises(GatewayError): self.bridge.command(payload)
        self.assertFalse(any(c.startswith("chassis.motion") for c in self.port.commands))

    def test_motion_cached_profile_mismatch_rejected(self):
        payload=self.motion_setup()
        self.port.status["chassis"]["motion"]["profile_id"]=1;self.bridge.refresh()
        with self.assertRaisesRegex(GatewayError,"模式已切換"): self.bridge.command(payload)
        self.assertFalse(any(c.startswith("chassis.motion") for c in self.port.commands))

    def test_motion_fresh_profile_mismatch_rejected(self):
        payload=self.motion_setup()
        self.port.status["chassis"]["motion"]["profile_id"]=1
        with self.assertRaisesRegex(GatewayError,"模式已切換"): self.bridge.command(payload)
        self.assertFalse(any(c.startswith("chassis.motion") for c in self.port.commands))

    def test_motion_firmware_rejects_profile_race_at_write(self):
        payload=self.motion_setup();self.port.motion_profile_before_write=1
        before=copy.deepcopy(self.port.status["chassis"]["motion"])
        with self.assertRaises(GatewayError): self.bridge.command(payload)
        after=self.port.status["chassis"]["motion"]
        self.assertEqual(after,{**before,"profile_id":1})
        self.assertIn("chassis.motion 1 60 120 260 320 130 -350 700 0",self.port.commands)

    def test_motion_procession_bank_command_and_exact_signed_parameters(self):
        payload=self.motion_setup()
        self.port.status["chassis"]["motion"]["profile_id"]=1;self.bridge.refresh()
        result=self.bridge.command({**payload,"profile_id":1,"accel":90,"decel":180,
                                    "icr_x_mm":120,"icr_left_y_mm":180,"icr_right_y_mm":-180})
        self.assertIn("chassis.motion 1 90 180 260 320 120 180 -180 1",self.port.commands)
        self.assertEqual(result["chassis"]["motion"]["profile_id"],1)
        self.assertEqual((result["chassis"]["motion"]["icr_left_y_mm"],result["chassis"]["motion"]["icr_right_y_mm"]),(180,-180))

    def test_pwm_settings_acknowledge_and_refresh_ram_status(self):
        self.port.status["pwm"]["value"]=32
        result=self.bridge.command({"command":"pwm.set","max_value":20,"step_interval_ms":12})
        self.assertEqual(self.port.commands,["status","status","pwm.set 20 12","status"])
        self.assertEqual(result["pwm"]["max_value"],20)
        self.assertEqual(result["pwm"]["step_interval_ms"],12)
        self.assertEqual(result["pwm"]["value"],20)
        self.assertEqual((result["pwm"]["default_max_value"],result["pwm"]["default_step_interval_ms"]),(55,5))
        self.assertEqual(self.bridge.snapshot()["firmware"]["pwm"],result["pwm"])
        for maximum,interval in ((0,1),(255,1000)):
            result=self.bridge.command({"command":"pwm.set","max_value":maximum,"step_interval_ms":interval})
            self.assertEqual((result["pwm"]["max_value"],result["pwm"]["step_interval_ms"]),(maximum,interval))
            if not maximum: self.assertEqual(result["pwm"]["value"],0)

    def test_pwm_rejects_invalid_parameters_before_any_serial_write(self):
        valid={"command":"pwm.set","max_value":55,"step_interval_ms":5}
        for field,values in (("max_value",(-1,256,True,False,55.0,float("nan"),float("inf"),"55",None)),
                             ("step_interval_ms",(0,-1,1001,True,False,5.0,float("nan"),float("inf"),"5",None))):
            for value in values:
                payload={**valid,field:value}; before=list(self.port.commands)
                with self.subTest(field=field,value=value),self.assertRaises(GatewayError):
                    self.bridge.command(payload)
                self.assertEqual(self.port.commands,before)
            payload=dict(valid);del payload[field];before=list(self.port.commands)
            with self.subTest(missing=field),self.assertRaises(GatewayError): self.bridge.command(payload)
            self.assertEqual(self.port.commands,before)

    def test_pwm_requires_supported_pin_and_verified_protocol(self):
        initial=copy.deepcopy(self.bridge.last_status)
        unsupported=(None,{"available":False,"pin":14},{"available":1,"pin":14},
                     {"available":True,"pin":16},{"available":True,"pin":14.0},{"available":True})
        for capability in unsupported:
            self.bridge.last_status=copy.deepcopy(initial)
            if capability is None: del self.bridge.last_status["pwm"]
            else: self.bridge.last_status["pwm"]=capability
            before=list(self.port.commands)
            with self.subTest(capability=capability),self.assertRaisesRegex(GatewayError,"更新.*韌體"):
                self.bridge.command({"command":"pwm.set","max_value":55,"step_interval_ms":5})
            self.assertEqual(self.port.commands,before)
        for protocol in (None,True,2,"1"):
            self.bridge.last_status=copy.deepcopy(initial); self.bridge.last_status["protocol"]=protocol
            with self.subTest(protocol=protocol),self.assertRaises(GatewayError):
                self.bridge.command({"command":"pwm.set","max_value":55,"step_interval_ms":5})
        self.assertEqual(self.port.commands,["status"])

    def test_pwm_requires_fresh_connection_then_recovers_after_refresh(self):
        for age in (0,self.bridge.status_time-5):
            self.bridge.status_time=age;before=list(self.port.commands)
            with self.assertRaisesRegex(GatewayError,"最近 4 秒"):
                self.bridge.command({"command":"pwm.set","max_value":55,"step_interval_ms":5})
            self.assertEqual(self.port.commands,before)
        self.bridge.refresh()
        self.bridge.command({"command":"pwm.set","max_value":10,"step_interval_ms":20})
        self.bridge.disconnect();before=list(self.port.commands)
        with self.assertRaises(GatewayError): self.bridge.command({"command":"pwm.set","max_value":55,"step_interval_ms":5})
        self.assertEqual(self.port.commands,before)

    def test_pwm_works_in_normal_debug_startup_busy_home_return_and_global_stop(self):
        for mode in ("normal","debug"):
            for phase in ("initializing","ready","busy","home_wait","return_wait","fault"):
                self.port.status.update(mode=mode,phase=phase,stopped=phase=="fault")
                self.bridge.refresh()
                before=copy.deepcopy(self.port.status)
                result=self.bridge.command({"command":"pwm.set","max_value":40,"step_interval_ms":25})
                self.assertEqual(result["pwm"]["max_value"],40)
                for key in before:
                    if key!="pwm": self.assertEqual(self.port.status[key],before[key],(mode,phase,key))
        self.assertTrue(all(c=="status" or c.startswith("pwm.set ") for c in self.port.commands))

    def test_pwm_does_not_change_running_cylinder_motor_servo_or_rc_state(self):
        self.port.set_cylinder(1,6000)
        self.port.status.update(phase="busy",position4=1234,position5=-1234,position6=3000,
                                pulse2=1200,pulse3=1800,pulse4=1300,pulse5=1800,pulse6=600,pulse9=1800)
        self.bridge.refresh();before=copy.deepcopy(self.port.status)
        self.bridge.command({"command":"pwm.set","max_value":0,"step_interval_ms":1000})
        for key in before:
            if key!="pwm": self.assertEqual(self.port.status[key],before[key],key)
        self.assertEqual(self.port.status["pwm"]["value"],0)
        self.assertEqual(set(c.split()[0] for c in self.port.commands),{"status","pwm.set"})

    def test_pwm_rechecks_capability_and_does_not_send_to_older_firmware(self):
        del self.port.status["pwm"]
        with self.assertRaisesRegex(GatewayError,"更新.*韌體"):
            self.bridge.command({"command":"pwm.set","max_value":55,"step_interval_ms":5})
        self.assertEqual(self.port.commands,["status","status"])

    def test_pwm_firmware_rejection_keeps_previous_values(self):
        before=copy.deepcopy(self.bridge.last_status["pwm"])
        self.port.reject="pwm.set"
        with self.assertRaises(GatewayError): self.bridge.command({"command":"pwm.set","max_value":100,"step_interval_ms":10})
        self.assertEqual(self.port.status["pwm"],before)
        self.assertEqual(self.bridge.last_status["pwm"],before)
        self.assertEqual(self.port.commands,["status","status","pwm.set 100 10"])

    def test_global_stop_keeps_priority_during_pwm_configuration(self):
        entered=threading.Event();release=threading.Event();errors=[];original=self.bridge.request
        def paused_request(command,*args,**kwargs):
            if command.startswith("pwm.set "):
                entered.set();release.wait(2)
            return original(command,*args,**kwargs)
        self.bridge.request=paused_request
        def configure():
            try:self.bridge.command({"command":"pwm.set","max_value":25,"step_interval_ms":10})
            except GatewayError as error:errors.append(str(error))
        worker=threading.Thread(target=configure,daemon=True);worker.start()
        try:
            self.assertTrue(entered.wait(2))
            self.assertFalse(self.bridge.operation_lock.acquire(blocking=False))
            self.bridge.command({"command":"stop"})
            self.assertIn("stop",self.port.commands)
            self.assertNotIn("pwm.set 25 10",self.port.commands)
        finally:release.set();worker.join(2)
        self.assertFalse(worker.is_alive());self.assertFalse(errors)
        self.assertEqual(self.port.status["phase"],"fault")
        self.assertEqual(self.port.status["pwm"]["max_value"],25)
        self.assertLess(self.port.commands.index("stop"),self.port.commands.index("pwm.set 25 10"))

    def test_connect_queries_only_and_reassembles(self):
        self.assertEqual(self.port.commands,["status"])
        self.assertEqual(self.bridge.snapshot()["firmware"]["phase"],"ready")
    def test_bench_status_offline_axes_and_locked_chassis_are_preserved(self):
        self.port.status.update(motor_online_mask=0,motor_offline_mask=7,position_known_mask=0)
        status=self.bridge.refresh()
        self.assertTrue(status["bench_test_mode"])
        self.assertEqual(status["firmware_revision"],"bench-test-v7")
        self.assertEqual(status["motor_ids"],[4,5,6])
        self.assertEqual(status["motor_online_mask"],0)
        self.assertEqual(status["motor_offline_mask"],7)
        self.assertEqual(status["chassis"]["state"],"locked")
        self.bridge.command({"command":"stage","stage":2})
        self.complete_motion()
        self.bridge.command({"command":"next"})
        self.assertIn("next",self.port.commands)
        self.assertEqual(self.port.status["step"],1)
        self.assertEqual(self.port.status["cylinder"]["reference_action"],2)
        self.assertEqual(self.port.status["phase"],"busy")

    def test_bench_offline_debug_motor_rejected_before_serial_write(self):
        self.port.status.update(motor_online_mask=0,motor_offline_mask=7,position_known_mask=0)
        self.bridge.command({"command":"mode","mode":"debug"})
        for node in (4,5,6):
            with self.subTest(node=node),self.assertRaisesRegex(GatewayError,"開機離線"):
                self.bridge.command({"command":"debug.move","node":node,"position":0,"rpm":320,"acc":0})
        with self.assertRaisesRegex(GatewayError,"開機離線"):
            self.bridge.command({"command":"debug.gohome","node":6})
        self.assertFalse(any(c.startswith(("debug.move","debug.gohome")) for c in self.port.commands))
        self.bridge.command({"command":"debug.servo","pin":4,"angle":90,"duration_ms":0})
        self.bridge.command({"command":"debug.cylinder","action":"extend","duration_ms":1000})
        self.assertIn("debug.cylinder 1 1000",self.port.commands)

    def test_bench_online_unreferenced_axis_allows_explicit_home_only(self):
        self.port.status.update(motor_online_mask=4,motor_offline_mask=3,position_known_mask=0)
        self.bridge.command({"command":"mode","mode":"debug"})
        with self.assertRaisesRegex(GatewayError,"尚未建立零點"):
            self.bridge.command({"command":"debug.move","node":6,"position":0,"rpm":320,"acc":0})
        self.bridge.command({"command":"debug.gohome","node":6})
        self.assertIn("debug.gohome 6",self.port.commands)
        self.assertFalse(any(c.startswith("debug.move") for c in self.port.commands))

    def test_bench_debug_masks_optional_and_false_mode_preserves_compatibility(self):
        self.bridge.command({"command":"mode","mode":"debug"})
        self.port.status.update(motor_online_mask=0,motor_offline_mask=7,position_known_mask=0,bench_test_mode=False)
        self.bridge.command({"command":"debug.move","node":6,"position":0,"rpm":320,"acc":0})
        self.port.status["bench_test_mode"]=True
        del self.port.status["motor_online_mask"]
        self.bridge.command({"command":"debug.move","node":6,"position":123,"rpm":320,"acc":0})
        self.assertIn("debug.move 6 0 320 0",self.port.commands)
        self.assertIn("debug.move 6 123 320 0",self.port.commands)

    def test_bench_offline_ram_axes_are_kept_for_firmware_skip(self):
        self.port.status.update(motor_online_mask=0,motor_offline_mask=7,position_known_mask=0)
        document={"stage":3,"repeat_from":1,"steps":[{"actions":[{"parallel":[
            {"stepper":{"node":4,"position":100,"rpm":320}},
            {"stepper":{"node":5,"position":-100,"rpm":320}},
            {"servo":{"pin":3,"pulse_us":1680}}]}]}]}
        result=self.bridge.apply(document)
        self.assertEqual(result["phase"],"ready")
        self.assertEqual(result["stage"],3)
        self.assertTrue(any(c.startswith("plan.action 0 0 100 320") for c in self.port.commands))
        self.assertTrue(any(c.startswith("plan.action 1 0 -100 320") for c in self.port.commands))

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
        self.bridge.command({"command":"debug.move","node":4,"position":-3000,"rpm":320,"acc":2})
        with self.assertRaises(GatewayError): self.bridge.command({"command":"debug.gohome","node":4})
        self.assertIn("debug.move 4 -3000 320 2",self.port.commands)
        self.assertNotIn("debug.gohome 4",self.port.commands)
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
    def test_stage3_paired_and_stage2_complete_ram(self):
        for stage in (2,3):
            source=Path(__file__).resolve().parents[1]/f"stage{stage}_steps.yaml"
            from generate_steps import StrictLoader,yaml,wire_document
            document=yaml.load(source.read_text(encoding="utf-8-sig"),Loader=StrictLoader)
            actions,steps=wire_document(document)
            result=self.bridge.apply(document)
            self.assertEqual(result["stage"],stage)
            self.assertEqual(result["completed"],0)
            self.assertIn(f"plan.begin {len(steps)} {len(actions)} {stage} {document.get('repeat_from',1)}",self.port.commands)
            self.complete_motion()
        first=document["steps"][0]["actions"][0]["parallel"]
        motor4=next(a["stepper"] for a in first if a.get("stepper",{}).get("node")==4)
        motor5=next(a["stepper"] for a in first if a.get("stepper",{}).get("node")==5)
        servo=next(a["servo"] for a in first if a.get("servo",{}).get("pin")==3)
        self.assertIn(f"plan.action 0 0 {motor4['position']} {motor4['rpm']} {motor4['acc']} 0 0 0 0 1 4",self.port.commands)
        self.assertIn(f"plan.action 1 0 {motor5['position']} {motor5['rpm']} {motor5['acc']} 0 0 0 0 1 5",self.port.commands)
        self.assertIn(f"plan.action 2 1 0 0 0 3 {servo['pulse_us']} {servo.get('duration_ms',0)} 0 1 0",self.port.commands)
        self.assertFalse(any(command.startswith("debug.") or command in ("next","run","home") for command in self.port.commands))
    def test_stage2_commit_resets_outgoing_stage_before_ready_without_auto_step(self):
        from generate_steps import StrictLoader,yaml
        document=yaml.load((Path(__file__).resolve().parents[1]/"stage2_steps.yaml").read_text(encoding="utf-8"),Loader=StrictLoader)
        self.port.return_delay=4; self.port.status.update(stage=1,position6=9000,pulse2=1800,pulse5=1500,pulse4=1300)
        with patch("web_gui.time.sleep"): result=self.bridge.apply(document)
        self.assertEqual((result["stage"],result["phase"],result["step"],result["completed"]),(2,"ready",0,0))
        self.assertEqual((result["position6"],result["pulse2"],result["pulse5"]),(0,1150,2100))
        self.assertEqual((result["pulse3"],result["pulse4"],result["pulse9"],result["cylinder"]["reference_action"]),
                         (1580,1300,1800,2)) # Incoming Stage2 open entry pose, not the manual HOME standby.
        self.assertEqual(self.port.return_completed,[1]); self.assertEqual(self.port.home_starts,0)
        for snapshot in self.port.return_snapshots:
            self.assertEqual((snapshot["stage"],snapshot["pending_stage"],snapshot["plan_version"]),(1,2,0))
        self.assertTrue(any("RAM交換及目標待機已確認" in log["text"] for log in self.bridge.logs))
        self.assertFalse(any(command in ("next","run","home") for command in self.port.commands))
        starts=self.port.return_starts
        self.bridge.command({"command":"stage","stage":2}); self.assertEqual(self.port.return_starts,starts)
        self.bridge.command({"command":"next"}); self.assertEqual(self.port.status["step"],1)
    def test_stop_interrupts_return_before_exchange_and_retains_old_table(self):
        self.port.status.update(stage=3,runtime_plan=True,plan_version=7)
        self.port.return_delay=100
        with patch("web_gui.time.sleep",side_effect=lambda _:self.bridge.command({"command":"stop"})),self.assertRaises(GatewayError):
            self.bridge.apply({"stage":2,"steps":[{"actions":[{"delay":{"ms":0}}]}]})
        self.assertEqual((self.port.status["stage"],self.port.status["phase"],self.port.status["plan_version"]),(3,"fault",7))
        self.assertTrue(self.port.status["runtime_plan"]); self.assertEqual(self.port.return_remaining,0)
        self.assertIsNone(self.bridge.applied_hash); self.assertIn("plan.abort",self.port.commands)
        self.assertFalse(any(command in ("next","run","home") for command in self.port.commands))
    def test_return_wait_blocks_actions_and_apply_but_allows_stage_and_stop(self):
        self.port.return_delay=100
        self.bridge.command({"command":"stage","stage":2})
        for payload in ({"command":"next"},{"command":"prev"},{"command":"run"},{"command":"home"},
                        {"command":"mode","mode":"debug"},{"command":"debug.servo","pin":5,"angle":90,"duration_ms":0}):
            with self.assertRaisesRegex(GatewayError,"復位"): self.bridge.command(payload)
        with self.assertRaises(GatewayError): self.bridge.apply({"steps":[{"actions":[{"delay":{"ms":0}}]}]})
        self.assertFalse(any(command.startswith("plan.begin") for command in self.port.commands))
        self.assertFalse(any(command in ("next","prev","run","home","mode debug") or command.startswith("debug.") for command in self.port.commands))
        self.bridge.command({"command":"stage","stage":3})
        self.assertEqual((self.port.status["stage"],self.port.status["pending_stage"]),(4,3))
    def test_commit_waits_for_return_and_does_not_mark_old_version_as_applied(self):
        self.port.status["cylinder"]["fully_retracted"]=False
        self.port.return_delay=4; self.port.status.update(stage=1,plan_version=9,runtime_plan=True)
        seen=[]
        def during_wait(_):
            seen.append((self.port.status["stage"],self.port.status["phase"],self.port.status["plan_version"],self.bridge.applied_hash))
        with patch("web_gui.time.sleep",side_effect=during_wait):
            result=self.bridge.apply({"stage":2,"steps":[{"actions":[{"delay":{"ms":0}}]}]})
        self.assertTrue(seen)
        self.assertTrue(any(row==(1,"return_wait",9,None) for row in seen))
        self.assertTrue(any(row==(2,"home_wait",10,None) for row in seen))
        self.assertTrue(all(row[3] is None for row in seen))
        self.assertEqual(self.port.entry_home_starts,1)
        self.assertEqual((result["stage"],result["phase"],result["plan_version"]),(2,"ready",10))
        self.assertEqual(self.port.home_starts,0)
    def test_commit_wrong_step_zero_snapshot_times_out_without_apply_success(self):
        self.port.status["cylinder"]["fully_retracted"]=False
        original=self.port.finish_entry_home
        def wrong_step():
            original(); self.port.status["step"]=1
        self.port.return_delay=1; self.port.entry_home_delay=1
        with patch.object(self.port,"finish_entry_home",side_effect=wrong_step),patch("web_gui.time.monotonic",side_effect=[0,0,1,2,3,70]),patch("web_gui.time.sleep"),self.assertRaises(GatewayError):
            self.bridge.apply({"stage":2,"steps":[{"actions":[{"delay":{"ms":0}}]}]})
        self.assertEqual((self.port.status["phase"],self.port.status["step"]),("ready",1))
        self.assertIsNone(self.bridge.applied_hash); self.assertIn("plan.abort",self.port.commands)

    def test_return_fault_does_not_exchange_or_mark_apply_success(self):
        self.port.return_fault=True
        with self.assertRaises(GatewayError): self.bridge.apply({"stage":2,"steps":[{"actions":[{"delay":{"ms":0}}]}]})
        self.assertEqual((self.port.status["stage"],self.port.status["plan_version"]),(4,0))
        self.assertFalse(self.port.status["runtime_plan"])
        self.assertIsNone(self.bridge.applied_hash); self.assertIn("plan.abort",self.port.commands)
    def test_return_keeps_latest_stage_request_without_homing_incoming(self):
        self.port.return_delay=100; self.port.status.update(stage=2,pulse9=1100,position6=888)
        self.bridge.command({"command":"stage","stage":1})
        self.bridge.command({"command":"stage","stage":3})
        self.bridge.command({"command":"stage","stage":4})
        self.assertEqual((self.port.status["stage"],self.port.status["pending_stage"],self.port.status["phase"]),(2,4,"return_wait"))
        self.port.return_remaining=1; status=self.bridge.refresh()
        self.assertEqual((status["stage"],status["phase"],status["pending_stage"],status["step"]),(4,"ready",0,0))
        self.assertEqual((status["cylinder"]["reference_action"],status["pulse9"]),(2,1100))
        self.assertEqual(status["position6"],888); self.assertEqual(self.port.home_starts,0)
        self.assertEqual(self.port.return_completed,[2])
    def test_every_selected_stage_returns_old_stage_before_activation(self):
        for old,target in [(4,1),(1,2),(2,3),(3,4)]:
            self.port.return_delay=4
            self.bridge.command({"command":"stage","stage":target})
            self.assertEqual((self.bridge.last_status["stage"],self.bridge.last_status["phase"],self.bridge.last_status["pending_stage"]),(old,"return_wait",target))
            self.complete_motion(); self.assertEqual(self.port.status["stage"],target)
            if old==3: self.assertEqual((self.port.status["position4"],self.port.status["position5"],self.port.status["pulse3"]),(0,0,2250))
        self.assertEqual(self.port.return_completed,[4,1,2,3]); self.assertEqual(self.port.home_starts,0)
        self.assertFalse(any(command in ("next","run","home") for command in self.port.commands))
    def test_busy_all_stages_accept_latest_request_then_return_after_step(self):
        for stage in (1,2,3,4):
            self.port.status.update(stage=stage,phase="busy",step=2,pending_stage=0)
            self.port.busy_remaining=100; self.port.return_delay=100
            first=stage%4+1; latest=(stage+1)%4+1
            self.bridge.command({"command":"stage","stage":first})
            self.bridge.command({"command":"stage","stage":latest})
            self.assertEqual((self.port.status["stage"],self.port.status["pending_stage"],self.port.status["phase"]),(stage,latest,"busy"))
            self.port.busy_remaining=1; status=self.bridge.refresh()
            self.assertEqual((status["stage"],status["phase"]),(stage,"return_wait"))
            self.port.return_remaining=1; status=self.bridge.refresh()
            if latest==2: status=self.complete_motion()
            self.assertEqual((status["stage"],status["phase"],status["step"]),(latest,"ready",0))
    def test_manual_home_latest_request_returns_old_stage_and_no_new_home(self):
        self.port.status.update(stage=2,position6=500)
        self.port.home_delay=100; self.port.return_delay=100
        self.bridge.command({"command":"home"})
        with self.assertRaises(GatewayError): self.bridge.command({"command":"next"})
        self.bridge.command({"command":"stage","stage":1}); self.bridge.command({"command":"stage","stage":4})
        self.assertEqual((self.port.status["stage"],self.port.status["pending_stage"],self.port.status["phase"]),(2,4,"home_wait"))
        self.port.home_remaining=1; status=self.bridge.refresh()
        self.assertEqual((status["stage"],status["phase"]),(2,"return_wait"))
        self.port.return_remaining=1; status=self.bridge.refresh()
        self.assertEqual((status["stage"],status["phase"],status["position6"]),(4,"ready",500))
        self.assertEqual(self.port.home_completed,[2]); self.assertEqual(self.port.home_starts,1)
    def test_manual_home_and_stage3_switching_both_return_to_zero(self):
        self.port.status.update(stage=3,position4=5700,position5=-5668,pulse3=1640)
        self.bridge.command({"command":"home"}); status=self.complete_motion()
        self.assertEqual((status["position4"],status["position5"],status["pulse3"]),(0,0,2250))
        self.bridge.command({"command":"stage","stage":4}); status=self.complete_motion()
        self.assertEqual((status["stage"],status["phase"],status["position4"],status["position5"],status["pulse3"]),(4,"ready",0,0,2250))
        self.assertEqual(self.port.home_starts,1); self.assertEqual(self.port.return_completed,[3])
    def test_stage3_park_zero_reentry_then_next_stage1_and_stage2(self):
        for target in (1,2):
            self.port.status.update(stage=3,phase="ready",position4=7100,position5=-7100,pulse3=1630,pending_stage=0,step=0,completed=0)
            self.port.return_delay=4
            self.bridge.command({"command":"stage","stage":target})
            self.assertEqual((self.port.status["stage"],self.port.status["pending_stage"],self.port.status["phase"]),(3,target,"return_wait"))
            with self.assertRaises(GatewayError): self.bridge.command({"command":"next"})
            status=self.complete_motion()
            self.assertEqual((status["stage"],status["step"],status["completed"],status["position4"],status["position5"],status["pulse3"]),(target,0,0,0,0,1580 if target==2 else 2250))
            before=self.port.commands.count("next")
            self.bridge.command({"command":"next"}); self.assertEqual(self.port.commands.count("next"),before+1)
            self.port.status.update(phase="ready",completed=1)
            self.bridge.command({"command":"stage","stage":3}); status=self.complete_motion()
            self.assertEqual((status["stage"],status["step"],status["completed"],status["position4"],status["position5"]),(3,0,0,0,0))
            self.bridge.command({"command":"next"}); self.assertEqual(self.port.commands.count("next"),before+2)
        self.assertEqual(self.port.home_starts,0)
        self.assertFalse(any(command=="home" for command in self.port.commands))
    def test_same_stage_ram_commit_still_resets_before_install(self):
        self.port.status.update(stage=3,position4=5700,position5=-5668,pulse3=1640)
        result=self.bridge.apply({"stage":3,"steps":[{"actions":[{"delay":{"ms":0}}]}]})
        self.assertEqual((result["stage"],result["phase"],result["position4"],result["position5"],result["pulse3"]),(3,"ready",0,0,2250))
        self.assertEqual(self.port.return_completed,[3]); self.assertEqual(self.port.home_starts,0)
    def test_nine_servos_three_fe_motors_and_selective_gohome(self):
        with self.assertRaises(GatewayError): self.bridge.command({"command":True})
        self.bridge.command({"command":"mode","mode":"debug"})
        for pin in (2,3,4,5,6,9,10,11,12):
            self.bridge.command({"command":"debug.servo","pin":pin,"angle":90,"duration_ms":1000})
            self.assertIn(f"debug.servo {pin} 900 1000",self.port.commands)
        for node in (4,5,6):
            self.bridge.command({"command":"debug.move","node":node,"position":-2000,"rpm":320,"acc":2})
            self.assertIn(f"debug.move {node} -2000 320 2",self.port.commands)
        for pin in (7,8):
            with self.assertRaises(GatewayError): self.bridge.command({"command":"debug.servo","pin":pin,"angle":90,"duration_ms":0})
        for node in (4,5):
            with self.assertRaisesRegex(GatewayError,"原位零點"): self.bridge.command({"command":"debug.gohome","node":node})
            self.assertNotIn(f"debug.gohome {node}",self.port.commands)
        self.port.status["stage"]=3
        self.bridge.command({"command":"debug.servo","pin":3,"angle":90,"duration_ms":0})
        self.assertIn("debug.servo 3 900 0",self.port.commands)
    def test_debug_new_servos_independent_values_duration_and_boundaries(self):
        self.bridge.command({"command":"mode","mode":"debug"})
        for pin,angle,duration,pulse in ((10,0,0,500),(11,90,1750,1500),(12,180,60000,2500)):
            self.bridge.command({"command":"debug.servo","pin":pin,"angle":angle,"duration_ms":duration})
            self.assertIn(f"debug.servo {pin} {angle*10} {duration}",self.port.commands)
            self.assertEqual(self.port.status[f"pulse{pin}"],pulse)
            self.assertEqual(self.port.servo_durations[pin],duration)
        self.assertEqual(tuple(self.port.status[f"pulse{pin}"] for pin in (10,11,12)),(500,1500,2500))
        self.assertEqual(tuple(self.port.status[f"pulse{pin}"] for pin in (2,3,4,5,6,9)),(1150,2250,2000,2100,1400,1100))
        before=len([c for c in self.port.commands if c.startswith("debug.servo")])
        for pin in (10,11,12):
            for key,bad in (("angle",-0.1),("angle",180.1),("duration_ms",-1),("duration_ms",60001)):
                with self.subTest(pin=pin,key=key,bad=bad),self.assertRaises(GatewayError):
                    self.bridge.command({"command":"debug.servo","pin":pin,"angle":90,"duration_ms":1000,key:bad})
        self.assertEqual(len([c for c in self.port.commands if c.startswith("debug.servo")]),before)

    def test_new_servo_capability_and_legacy_explicit_pin_fallback(self):
        self.bridge.command({"command":"mode","mode":"debug"})
        self.port.status["servo_pins"]=[2,3,4,5,6,9]
        for pin in (10,11,12):
            with self.assertRaisesRegex(GatewayError,f"Pin {pin}"):
                self.bridge.command({"command":"debug.servo","pin":pin,"angle":90,"duration_ms":0})
        self.assertFalse(any(c.startswith(("debug.servo 10 ","debug.servo 11 ","debug.servo 12 ")) for c in self.port.commands))
        del self.port.status["servo_pins"]
        for pin in (10,11,12):
            self.bridge.command({"command":"debug.servo","pin":pin,"angle":90,"duration_ms":1000})
            self.assertIn(f"debug.servo {pin} 900 1000",self.port.commands)
        del self.port.status["pin12"]
        with self.assertRaises(GatewayError):
            self.bridge.command({"command":"debug.servo","pin":12,"angle":0,"duration_ms":0})

    def test_new_servo_ram_actions_and_unsupported_upload_are_nonmutating(self):
        document={"stage":4,"steps":[{"actions":[{"parallel":[
            {"servo":{"pin":10,"pulse_us":500,"duration_ms":0}},
            {"servo":{"pin":11,"pulse_us":1500,"duration_ms":1750}},
            {"servo":{"pin":12,"pulse_us":2500,"duration_ms":60000}}]}]}]}
        result=self.bridge.apply(document)
        self.assertEqual(result["phase"],"ready")
        from generate_steps import wire_document
        actions,_=wire_document(document)
        for index,action in enumerate(actions):
            self.assertIn("plan.action "+" ".join(map(str,[index]+action)),self.port.commands)
        for pin in (10,11,12):
            self.port.status["servo_pins"]=[p for p in (2,3,4,5,6,9,10,11,12) if p!=pin]
            before=len([c for c in self.port.commands if c.startswith("plan.")])
            with self.assertRaises(GatewayError): self.bridge.apply(document)
            self.assertEqual(len([c for c in self.port.commands if c.startswith("plan.")]),before)

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
        with patch("web_gui.time.monotonic",side_effect=[0,0,1,70]),patch("web_gui.time.sleep"),self.assertRaises(GatewayError):
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
        with self.assertRaises(GatewayError):
            self.bridge.apply({"stage":3,"steps":[{"actions":[{"stepper":{"node":4,"position":0,"rpm":320}}]}]})
        self.assertNotIn("plan.begin",[s.split()[0] for s in self.port.commands])
    def test_old_firmware_rejects_cylinder_before_transfer(self):
        self.port.status["cylinder"]["available"]=False
        with self.assertRaises(GatewayError): self.bridge.apply({"stage":2,"steps":[{"actions":[{"cylinder":{"action":"extend","duration_ms":6000}}]}]})
        self.assertNotIn("plan.begin",[command.split()[0] for command in self.port.commands])
    def test_retired_id7_rejects_debug_and_yaml_before_transfer(self):
        self.bridge.command({"command":"mode","mode":"debug"})
        with self.assertRaises(GatewayError): self.bridge.command({"command":"debug.move","node":7,"position":0,"rpm":320,"acc":0})
        with self.assertRaises(ConfigError): self.bridge.apply({"stage":2,"steps":[{"actions":[{"stepper":{"node":7,"position":0,"rpm":320,"acc":0}}]}]})
        self.assertFalse(any(command.startswith("plan.") or command.startswith("debug.move 7") for command in self.port.commands))

    def test_failed_handshake_closes_port(self):
        self.bridge.disconnect(); self.port=FakePort(); self.port.reject="status"
        self.bridge=Bridge(port_factory=lambda *args,**kwargs:self.port)
        with self.assertRaises(GatewayError):self.bridge.connect("COM99")
        self.assertFalse(self.bridge.snapshot()["connected"]); self.assertTrue(self.port.closed)

    def test_pending_chassis_direction_diagnostic_preserves_mechanical_return(self):
        self.port.status.update(stage=2,pending_stage=0,phase="ready")
        self.port.return_delay=100
        self.bridge.command({"command":"stage","stage":1})
        wire=self.bridge.refresh()
        self.assertEqual((wire["stage"],wire["pending_stage"],wire["phase"]),(2,1,"return_wait"))
        self.assertEqual(wire["chassis"]["direction_stage"],1)
        self.bridge.command({"command":"stage","stage":4})
        wire=self.bridge.refresh()
        self.assertEqual((wire["stage"],wire["pending_stage"]),(2,4))
        self.assertEqual(wire["chassis"]["direction_stage"],4)
        self.assertEqual(self.port.return_starts,1)

    def test_stage3_disabled_rejects_selection_and_ram_before_motion(self):
        self.port.status.update(stage3_enabled=False,stage_ids=[1,2,3,4,5,6])
        self.bridge.refresh()
        for payload in ({"command":"stage","stage":3},):
            with self.assertRaises(GatewayError): self.bridge.command(payload)
        document={"stage":3,"repeat_from":1,"steps":[{"actions":[{"delay":{"ms":0}}]}]}
        with self.assertRaises(GatewayError): self.bridge.apply(document)
        self.assertFalse(any(command.startswith(("stage ","plan.begin")) for command in self.port.commands))
        self.bridge.command({"command":"stage","stage":2})
        self.complete_motion()
        self.assertEqual(self.bridge.last_status["stage"],2)

    def test_stage3_disabled_preserves_shared_debug_hardware(self):
        self.port.status.update(stage3_enabled=False,stage_ids=[1,2,4,5,6],stage=2,mode="debug")
        self.bridge.refresh()
        for node,position in ((4,500),(5,-500)):
            self.bridge.command({"command":"debug.move","node":node,"position":position,"rpm":100,"acc":5})
            self.assertIn(f"debug.move {node} {position} 100 5",self.port.commands)
        self.bridge.command({"command":"debug.servo","pin":3,"angle":97.2,"duration_ms":0})
        self.assertIn("debug.servo 3 972 0",self.port.commands)
        # Stale stage metadata must not disable all hardware Debug controls either.
        self.port.status["stage"]=3; self.bridge.refresh()
        self.bridge.command({"command":"debug.move","node":4,"position":0,"rpm":100,"acc":5})
        self.assertIn("debug.move 4 0 100 5",self.port.commands)

    def test_stage3_disabled_keeps_stage1_vase_and_other_stages(self):
        self.port.status.update(stage3_enabled=False,stage_ids=[1,2,4,5,6])
        self.bridge.refresh()
        self.bridge.command({"command":"stage","stage":1,"prop":2});self.complete_motion()
        self.bridge.command({"command":"prop","prop":2});self.complete_motion()
        self.assertIn("prop 2",self.port.commands)
        for stage in (2,4,5,6):
            self.bridge.command({"command":"stage","stage":stage});self.complete_motion()
            self.assertEqual(self.bridge.last_status["stage"],stage)

    def test_four_stages_and_previous_command(self):
        for key in PROFILES:
            stage,prop=profile_identity(key)
            self.bridge.command({"command":"stage","stage":stage,"prop":prop})
            self.complete_motion()
            self.assertEqual((self.bridge.last_status["stage"],self.bridge.last_status["prop"]),(stage,prop))
        self.bridge.command({"command":"prev"})
        self.assertIn("prev",self.port.commands)
        self.assertEqual(self.bridge.last_status["rc"]["channels"],[992]*16)
        self.assertEqual(self.bridge.last_status["chassis"]["wheels"][0]["node"],10)

    def test_vase_and_placeholder_apply_keep_stage_metadata(self):
        for stage in (1,2):
            document={"stage":stage,"repeat_from":1,"steps":[{"actions":[{"delay":{"ms":0}}]}]}
            self.bridge.apply(document)
            self.assertIn(f"plan.begin 1 1 {stage} 1"+(" 2" if stage==1 else ""),self.port.commands)
            self.complete_motion()
            self.assertEqual(self.bridge.last_status["stage"],stage)

    def test_legacy_stage_and_runtime_capacity_reject_before_transfer(self):
        del self.port.status["stage_ids"]
        with self.assertRaises(GatewayError):self.bridge.apply({"stage":2,"steps":[{"actions":[{"delay":{"ms":0}}]}]})
        self.port.status["max_runtime_actions"]=1
        with self.assertRaises(GatewayError):self.bridge.apply({"stage":4,"steps":[{"actions":[{"delay":{"ms":0}},{"delay":{"ms":1}}]}]})
        self.assertNotIn("plan.begin",[s.split()[0] for s in self.port.commands])

    def test_five_prop_uploads_keep_distinct_identity_and_ready_step0(self):
        for prop,filename in enumerate(("basket","vase","tube","chicken","bird"),1):
            document=yaml.load((Path(__file__).resolve().parents[1]/f"stage1_{filename}.yaml").read_text(encoding="utf-8-sig"),Loader=StrictLoader)
            actions,steps=wire_document(document)
            result=self.bridge.apply(document)
            self.assertEqual((result["stage"],result["prop"],result["phase"],result["step"],result["completed"]),(1,prop,"ready",0,0))
            self.assertIn(f"plan.begin {len(steps)} {len(actions)} 1 1 {prop}",self.port.commands)
        self.assertEqual(set(self.port.profile_plans),{(1,n) for n in range(1,6)})
        self.assertFalse(any(c in ("next","run","home") or c.startswith("prop ") for c in self.port.commands))
    def test_prop_lift_and_release_upload_as_separate_steps(self):
        # Protocol fixtures remain independent of the operator's tuned YAML defaults.
        # The preceding profile roundtrip test loads all current files from disk.
        for prop,position,rpm,acc in ((1,6000,500,150),(4,4000,1500,255),(5,4000,1500,255)):
            with self.subTest(prop=prop):
                document={"stage":1,"prop":prop,"repeat_from":1,"steps":[{"actions":[{"parallel":[
                    {"stepper":{"node":6,"position":8000 if prop==1 else 7900,"rpm":1000,"acc":230}},
                    {"servo":{"pin":5,"pulse_us":{1:2056,4:1833,5:1944}[prop]}},
                    {"servo":{"pin":2,"pulse_us":1611}}]}]}]}
                if prop!=1:
                    document["steps"].append({"actions":[{"servo":{"pin":2,"pulse_us":1111 if prop==4 else 1056}}]})
                document["steps"].append({"actions":[{"stepper":{"node":6,"position":position,"rpm":rpm,"acc":acc}}]})
                if prop!=1:
                    document["steps"].append({"actions":[{"servo":{"pin":2,"pulse_us":1667}}]})
                actions,steps=wire_document(document)
                lift_index=1 if prop==1 else 2
                self.assertEqual(document["steps"][lift_index]["actions"],[{"stepper":{"node":6,"position":position,"rpm":rpm,"acc":acc}}])
                lift=[0,position,rpm,acc,0,0,0,1,1,6]
                self.assertEqual(steps[lift_index][1],1)
                self.assertEqual(actions[steps[lift_index][0]],lift)
                if prop!=1:
                    self.assertEqual(document["steps"][3]["actions"],[{"servo":{"pin":2,"pulse_us":1667}}])
                    self.assertEqual(steps[3][1],1)
                    self.assertEqual(actions[steps[3][0]],[1,0,0,0,2,1667,0,1,1,0])
                before=len(self.port.commands)
                result=self.bridge.apply(document)
                sent=self.port.commands[before:]
                self.assertEqual((result["stage"],result["prop"],result["steps"],result["phase"],result["step"]),(1,prop,2 if prop==1 else 4,"ready",0))
                for index,action in enumerate(actions):
                    self.assertIn("plan.action "+" ".join(map(str,[index]+action)),sent)
                for index,(start,count) in enumerate(steps):
                    self.assertIn(f"plan.step {index} {start} {count}",sent)
                self.assertFalse(any(c in ("next","run","home") or c.startswith("prop ") for c in sent))

    def test_fruit_profile_load_and_ram(self):
        from pathlib import Path
        document=yaml.load((Path(__file__).resolve().parents[1]/"stage6_steps.yaml").read_text(encoding="utf-8"),Loader=StrictLoader)
        self.port.status["stage_ids"]=[1,2,3,4,5,6]
        self.bridge.refresh()
        actions,steps=wire_document(document)
        self.assertEqual(len(steps),5)
        self.assertEqual([(a[1],a[2],a[3]) for a in actions if a[0]==0],[(9400,320,200),(5500,320,200),(-1000,320,200)])
        result=self.bridge.apply(document)
        self.assertEqual(result["stage"],6)
        self.assertTrue(any(c.startswith("plan.begin 5 ") and c.endswith(" 6 1") for c in self.port.commands))
        self.assertFalse(any(c in ("next","run") for c in self.port.commands))
    def test_prop_trigger_next_terminal_and_retrigger(self):
        for prop,count in enumerate((2,3,2,4,4),1):
            self.bridge.command({"command":"stage","stage":1,"prop":prop}); self.complete_motion()
            self.assertEqual(self.port.status["step"],0)
            self.bridge.command({"command":"prop","prop":prop}); self.complete_motion()
            self.assertIn(f"prop {prop}",self.port.commands)
            self.assertEqual(self.port.status["steps"],count)
            self.assertFalse(self.port.status["prop_complete"])
            for step in range(2,count+1):
                self.bridge.command({"command":"next"}); self.complete_motion()
                self.assertEqual(self.port.status["completed"],step)
                self.assertEqual(self.port.status["prop_complete"],step==count)
            self.assertTrue(self.port.status["prop_complete"])
            self.bridge.command({"command":"next"})
            self.assertEqual([c for c in self.port.commands if c != "status"][-1],"next")
            self.complete_motion()
            self.bridge.command({"command":"prop","prop":prop}); self.complete_motion()
            self.assertEqual(self.port.status["completed"],1)
    def test_prop_rejects_other_stage_busy_debug_invalid_and_old_capability(self):
        with self.assertRaises(GatewayError):self.bridge.command({"command":"prop","prop":1})
        self.port.status.update(stage=1,prop=2)
        for field,value in (("phase","busy"),("phase","return_wait"),("mode","debug")):
            self.port.status.update(phase="ready",mode="normal"); self.port.status[field]=value
            with self.assertRaises(GatewayError):self.bridge.command({"command":"prop","prop":1})
        self.port.status.update(phase="ready",mode="normal")
        for prop in (0,6,True,"1"):
            with self.assertRaises(GatewayError):self.bridge.command({"command":"prop","prop":prop})
        self.port.status["stage1_prop_ids"]=[2]
        with self.assertRaises(GatewayError):self.bridge.apply({"stage":1,"prop":1,"steps":[{"actions":[{"delay":{"ms":0}}]}]})
    def test_wrong_prop_ready_snapshot_does_not_confirm_commit(self):
        original=self.port.finish_return
        def wrong_prop():
            original();self.port.status["prop"]=2
        self.port.return_delay=1
        with patch.object(self.port,"finish_return",side_effect=wrong_prop),patch("web_gui.time.monotonic",side_effect=[0,0,1,70]),patch("web_gui.time.sleep"),self.assertRaises(GatewayError):
            self.bridge.apply({"stage":1,"prop":1,"steps":[{"actions":[{"delay":{"ms":0}}]}]})
        self.assertIsNone(self.bridge.applied_hash);self.assertIn("plan.abort",self.port.commands)
    def test_stage5_placeholder_home_does_not_move_other_stage_axes(self):
        self.port.status.update(stage=5,prop=0,position6=1234,pulse2=1600)
        self.bridge.command({"command":"home"});self.complete_motion()
        self.assertEqual((self.port.status["position6"],self.port.status["pulse2"]),(1234,1600))
    def test_stage1_exit_homes_current_before_fixed_stage4(self):
        self.port.status.update(stage=1,prop=2,position6=8000)
        self.bridge.command({"command":"stage1.exit"});self.complete_motion()
        self.assertEqual((self.port.status["stage"],self.port.status["prop"],self.port.status["position6"]),(4,0,0))
        self.assertEqual(self.port.home_starts,1);self.assertIn("stage1.exit 4",self.port.commands)

    def test_boot_full_retract_reference_skips_stage2_entry_wait(self):
        self.assertTrue(self.bridge.refresh()["cylinder"]["fully_retracted"])
        self.bridge.command({"command":"stage","stage":2}); status=self.complete_motion()
        self.assertEqual((status["stage"],status["phase"],status["step"]),(2,"ready",0))
        self.assertEqual(self.port.entry_home_starts,0)
        self.assertEqual((status["pulse3"],status["pulse4"],status["pulse6"],status["pulse9"]),(1580,1300,1400,1800))
        # Completed boot retraction is reused on later entries without another timed retract.
        with patch.object(self.port,"set_cylinder",wraps=self.port.set_cylinder) as motion:
            self.bridge.command({"command":"stage","stage":4}); self.complete_motion()
            self.bridge.command({"command":"stage","stage":2}); status=self.complete_motion()
            motion.assert_not_called()
        self.assertEqual(self.port.entry_home_starts,0)
        self.assertEqual((status["pulse3"],status["pulse4"],status["pulse9"]),(1580,1300,1800))
        self.bridge.command({"command":"next"}); self.assertIn("next",self.port.commands)
    def test_stage2_manual_home_still_runs_full_retract_after_boot(self):
        self.port.status.update(stage=2,phase="ready")
        self.assertTrue(self.port.status["cylinder"]["fully_retracted"])
        self.bridge.command({"command":"home"})
        cylinder=self.port.status["cylinder"]
        self.assertEqual((cylinder["action"],cylinder["duration_ms"],cylinder["active"]),(2,5000,True))
        self.assertFalse(cylinder["fully_retracted"])
        status=self.complete_motion()
        self.assertTrue(status["cylinder"]["fully_retracted"])
        self.assertEqual((status["position4"],status["position5"]),(0,0))
        self.assertEqual((status["pulse3"],status["pulse4"],status["pulse6"],status["pulse9"]),(2250,2000,1400,1100))
    def test_partial_cylinder_retract_does_not_skip_stage2_full_home(self):
        for duration in (600,1800,3000):
            with self.subTest(duration=duration):
                self.port.status.update(stage=4,phase="ready",pending_stage=0)
                self.port.set_cylinder(2,duration); self.port.complete_cylinder()
                self.assertTrue(self.port.status["cylinder"]["reference_known"])
                self.assertFalse(self.port.status["cylinder"]["fully_retracted"])
                starts=self.port.entry_home_starts
                self.bridge.command({"command":"stage","stage":2}); status=self.complete_motion()
                self.assertEqual(self.port.entry_home_starts,starts+1)
                self.assertTrue(status["cylinder"]["fully_retracted"])
                self.assertEqual((status["pulse3"],status["pulse4"],status["pulse9"]),(1580,1300,1800))
                self.assertTrue(self.port.entry_home_snapshots)
                for snapshot in self.port.entry_home_snapshots:
                    self.assertEqual((snapshot["pulse3"],snapshot["pulse4"],snapshot["pulse9"]),(1580,1300,1800))
    def test_boot_cylinder_wait_blocks_next_and_stop_cannot_restart_retract(self):
        self.port.status.update(phase="booting",stopped=False)
        self.port.set_cylinder(2,5000)
        self.bridge.refresh()
        for action in ("next","prev","run","home"):
            with self.subTest(action=action),self.assertRaises(GatewayError): self.bridge.command({"command":action})
        self.bridge.command({"command":"stop"})
        for _ in range(3): status=self.bridge.refresh()
        self.assertEqual((status["phase"],status["cylinder"]["active"]),("fault",False))
        self.assertFalse(status["cylinder"]["fully_retracted"])
        self.assertFalse(any(c.startswith("debug.cylinder") for c in self.port.commands))

    def test_cylinder_debug_numbers_and_stop_bypass_operation_lock(self):
        self.bridge.command({"command":"mode","mode":"debug"})
        self.bridge.command({"command":"debug.cylinder","action":"extend","duration_ms":6000})
        self.assertIn("debug.cylinder 1 6000",self.port.commands)
        with self.bridge.operation_lock:
            self.bridge.command({"command":"debug.cylinder","action":"stop","duration_ms":0})
        c=self.port.status["cylinder"]
        self.assertEqual((c["in1_high"],c["in2_high"],c["active"]),(False,False,False))
        for action,duration in (("bad",6000),("extend",0),("retract",-1),(True,1000),("stop",60001)):
            with self.assertRaises(GatewayError):self.bridge.command({"command":"debug.cylinder","action":action,"duration_ms":duration})
    def test_stage2_cylinder_wire_is_eleven_fields_legacy_actions_are_ten(self):
        self.port.status["cylinder"]["fully_retracted"]=False
        document=yaml.load((Path(__file__).resolve().parents[1]/"stage2_steps.yaml").read_text(encoding="utf-8-sig"),Loader=StrictLoader)
        actions,steps=wire_document(document)
        self.assertEqual([len(a) for a in actions if a[0]==4],[11,11])
        self.assertEqual([(a[6],a[10]) for a in actions if a[0]==4],[(3000,1),(3000,2)])
        self.assertTrue(all(len(a)==10 for a in actions if a[0]!=4))
        result=self.bridge.apply(document)
        for index,a in enumerate(actions):
            self.assertIn("plan.action "+" ".join(map(str,[index]+a)),self.port.commands)
        self.assertEqual((result["phase"],result["step"],result["cylinder"]["reference_action"]),("ready",0,2))
        self.assertTrue(self.port.entry_home_snapshots)
        self.assertTrue(all(s["stage"]==2 and s["plan_version"]==1 and s["phase"]=="home_wait" for s in self.port.entry_home_snapshots))
    def test_global_stop_concurrent_with_ram_entry_home_wait(self):
        self.port.status["cylinder"]["fully_retracted"]=False
        entered=threading.Event(); release=threading.Event(); stop_written=threading.Event(); errors=[]
        self.port.entry_home_delay=100
        original=self.port.write
        def observed_write(packet):
            if packet.decode().strip().endswith(" stop"): stop_written.set()
            return original(packet)
        self.port.write=observed_write
        def paused_sleep(_):
            if self.port.status["stage"]==2 and self.port.status["phase"]=="home_wait":
                entered.set();release.wait(2)
        def apply():
            try:self.bridge.apply({"stage":2,"steps":[{"actions":[{"cylinder":{"action":"extend","duration_ms":6000}}]}]})
            except GatewayError as error:errors.append(str(error))
        worker=threading.Thread(target=apply,daemon=True); stopper=None
        with patch("web_gui.time.sleep",side_effect=paused_sleep):
            worker.start()
            try:
                self.assertTrue(entered.wait(2))
                self.assertFalse(self.bridge.operation_lock.acquire(blocking=False))
                stopper=threading.Thread(target=lambda:self.bridge.command({"command":"stop"}),daemon=True)
                stopper.start();self.assertTrue(stop_written.wait(1),"STOP blocked behind RAM wait")
            finally:release.set()
            worker.join(2)
            if stopper:stopper.join(2)
        self.assertFalse(worker.is_alive());self.assertTrue(errors)
        self.assertEqual(self.port.status["phase"],"fault")
        self.assertEqual((self.port.status["cylinder"]["in1_high"],self.port.status["cylinder"]["in2_high"]),(False,False))
        self.assertIsNone(self.bridge.applied_hash)


class HttpTests(unittest.TestCase):
    def setUp(self):
        self.temp=tempfile.TemporaryDirectory(); root=Path(self.temp.name)
        (root/"stage4_steps.yaml").write_text("steps: [{actions: [{delay: {ms: 100}}]}]",encoding="utf-8")
        (root/"stage3_steps.yaml").write_text("stage: 3\nrepeat_from: 1\nsteps: [{actions: [{delay: {ms: 0}}]}]",encoding="utf-8")
        for key,filename in PROFILES.items():
            stage,prop=profile_identity(key)
            doc={"stage":stage,"steps":[{"actions":[{"delay":{"ms":0}}]}]}
            if stage==1: doc["prop"]=prop
            (root/filename).write_text(yaml.safe_dump(doc),encoding="utf-8")
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
    def test_http_pwm_configuration_returns_live_status_without_actuator_commands(self):
        self.assertEqual(self.request("/api/connect",{"port":"COM99"})[0],200)
        self.port.status.update(phase="busy",position6=8000,pulse6=600)
        self.port.set_cylinder(1,6000)
        self.assertEqual(self.request("/api/refresh",{})[0],200)
        before=copy.deepcopy(self.port.status)
        code,body=self.request("/api/command",{"command":"pwm.set","max_value":120,"step_interval_ms":40})
        self.assertEqual(code,200)
        self.assertEqual((body["result"]["pwm"]["max_value"],body["result"]["pwm"]["step_interval_ms"]),(120,40))
        state=self.request("/api/state")[1]
        self.assertEqual(state["firmware"]["pwm"],body["result"]["pwm"])
        for key in before:
            if key!="pwm": self.assertEqual(self.port.status[key],before[key],key)
        self.assertTrue(all(c=="status" or c=="pwm.set 120 40" for c in self.port.commands))
        self.assertEqual(self.request("/api/command",{"command":"stop"})[0],200)
        self.assertEqual(self.request("/api/refresh",{})[0],200)
        code,body=self.request("/api/command",{"command":"pwm.set","max_value":0,"step_interval_ms":1})
        self.assertEqual(code,200);self.assertEqual(body["result"]["phase"],"fault")
        self.assertEqual(body["result"]["pwm"]["value"],0)
        self.assertFalse(body["result"]["cylinder"]["active"])

    def test_http_pwm_disconnected_stale_unsupported_and_invalid_requests_are_rejected(self):
        payload={"command":"pwm.set","max_value":55,"step_interval_ms":5}
        self.assertEqual(self.request("/api/command",payload)[0],409)
        self.assertEqual(self.port.commands,[])
        self.assertEqual(self.request("/api/connect",{"port":"COM99"})[0],200)
        for field,value in (("max_value",True),("max_value",256),("max_value",55.0),
                            ("step_interval_ms",0),("step_interval_ms",1001),("step_interval_ms","5")):
            before=list(self.port.commands)
            self.assertEqual(self.request("/api/command",{**payload,field:value})[0],409)
            self.assertEqual(self.port.commands,before)
        self.app.bridge.status_time-=5
        before=list(self.port.commands);self.assertEqual(self.request("/api/command",payload)[0],409)
        self.assertEqual(self.port.commands,before)
        del self.port.status["pwm"]
        self.assertEqual(self.request("/api/refresh",{})[0],200)
        before=list(self.port.commands);code,body=self.request("/api/command",payload)
        self.assertEqual(code,409);self.assertIn("更新",body["error"])
        self.assertEqual(self.port.commands,before)

    def test_http_bench_masks_and_offline_debug_guard(self):
        self.port.status.update(motor_online_mask=0,motor_offline_mask=7,position_known_mask=0)
        self.assertEqual(self.request("/api/connect",{"port":"COM99"})[0],200)
        state=self.request("/api/state")[1]
        self.assertTrue(state["firmware"]["bench_test_mode"])
        self.assertEqual(state["firmware"]["motor_online_mask"],0)
        self.assertEqual(state["firmware"]["motor_offline_mask"],7)
        self.assertEqual(state["firmware"]["chassis"]["state"],"locked")
        self.assertEqual(self.request("/api/command",{"command":"mode","mode":"debug"})[0],200)
        rejected=self.request("/api/command",{"command":"debug.move","node":6,"position":0,"rpm":320,"acc":0})
        self.assertEqual(rejected[0],409)
        self.assertIn("開機離線",rejected[1]["error"])
        self.assertFalse(any(c.startswith("debug.move") for c in self.port.commands))
        self.assertEqual(self.request("/api/command",{"command":"debug.cylinder","action":"extend","duration_ms":1500})[0],200)
        self.assertIn("debug.cylinder 1 1500",self.port.commands)
        self.assertEqual(self.request("/api/command",{"command":"stop"})[0],200)
        self.assertFalse(self.port.status["cylinder"]["in1_high"])
        self.assertFalse(self.port.status["cylinder"]["in2_high"])

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
        while self.port.status["phase"] in ("home_wait","return_wait"): self.request("/api/refresh",{})
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
        self.assertEqual(self.request("/api/init?profile=6")[0],400)

    def test_stage2_http_roundtrip_ram_and_new_debug_pins(self):
        from generate_steps import StrictLoader,yaml
        source=Path(__file__).resolve().parents[1]/"stage2_steps.yaml"
        document=yaml.load(source.read_text(encoding="utf-8"),Loader=StrictLoader)
        config=self.request("/api/init?profile=2")[1]["config"]
        self.assertEqual(self.request("/api/config",{"document":document,"expected_hash":config["hash"],"profile":2})[0],200)
        self.assertEqual(self.request("/api/init?profile=2")[1]["config"]["document"],document)
        self.assertEqual(self.request("/api/connect",{"port":"COM99"})[0],200)
        self.assertEqual(self.request("/api/apply",{"document":document})[0],200)
        from generate_steps import wire_document
        actions,steps=wire_document(document)
        self.assertIn(f"plan.begin {len(steps)} {len(actions)} 2 {document.get('repeat_from',1)}",self.port.commands)
        self.assertNotIn("run",self.port.commands)
        self.assertEqual(self.port.status["completed"],0)
        self.assertEqual(self.port.status["cylinder"]["reference_action"],2)
        self.assertEqual(self.port.status["phase"],"ready")
        while self.port.status["phase"] in ("home_wait","return_wait"): self.request("/api/refresh",{})
        self.assertEqual((self.port.status["phase"],self.port.status["step"],self.port.status["completed"]),("ready",0,0))
        self.assertEqual(self.request("/api/command",{"command":"mode","mode":"debug"})[0],200)
        for pin in (3,4,6,9,10,11,12):
            self.assertEqual(self.request("/api/command",{"command":"debug.servo","pin":pin,"angle":90,"duration_ms":0})[0],200)
        self.assertEqual(self.request("/api/command",{"command":"debug.servo","pin":7,"angle":90,"duration_ms":0})[0],409)
        self.assertEqual(self.request("/api/command",{"command":"debug.gohome","node":4})[0],409)
        bad={"stage":3,"steps":[{"actions":[{"servo":{"pin":8,"pulse_us":1500}}]}]}
        before=len(self.port.commands)
        self.assertEqual(self.request("/api/apply",{"document":bad})[0],400)
        self.assertEqual(len(self.port.commands),before)

    def test_stage3_disabled_http_profiles_and_ram_keep_shared_hardware(self):
        self.assertEqual(self.request("/api/connect",{"port":"COM99"})[0],200)
        self.port.status.update(stage3_enabled=False,stage_ids=[1,2,3,4,5,6])
        self.assertEqual(self.request("/api/refresh",{})[0],200)
        code,payload=self.request("/api/init")
        self.assertEqual(code,200)
        keys=[item["key"] for item in payload["profiles"]]
        self.assertNotIn("3",keys);self.assertIn("1_2",keys);self.assertIn("2",keys)
        document={"stage":3,"repeat_from":1,"steps":[{"actions":[{"delay":{"ms":0}}]}]}
        self.assertEqual(self.request("/api/apply",{"document":document})[0],409)
        self.assertFalse(any(command.startswith("plan.begin") for command in self.port.commands))
        self.assertEqual(self.request("/api/command",{"command":"mode","mode":"debug"})[0],200)
        self.assertEqual(self.request("/api/command",{"command":"debug.move","node":4,"position":500,"rpm":100,"acc":5})[0],200)
        self.assertEqual(self.request("/api/command",{"command":"debug.servo","pin":3,"angle":97.2,"duration_ms":0})[0],200)

    def test_stage3_http_complete_pair_ram_without_auto_step(self):
        from generate_steps import yaml,StrictLoader
        document=yaml.load((Path(__file__).resolve().parents[1]/"stage3_steps.yaml").read_text(encoding="utf-8-sig"),Loader=StrictLoader)
        config=self.request("/api/init?profile=3")[1]["config"]
        self.assertEqual(self.request("/api/config",{"document":document,"expected_hash":config["hash"],"profile":3})[0],200)
        self.assertEqual(self.request("/api/connect",{"port":"COM99"})[0],200)
        result=self.request("/api/apply",{"document":document})
        self.assertEqual(result[0],200)
        self.assertEqual(result[1]["result"]["completed"],0)
        self.assertEqual(self.port.home_starts,0)
        self.assertEqual(self.port.return_completed,[4])
        self.assertEqual(result[1]["result"]["phase"],"ready")
        self.assertIn("plan.begin 4 16 3 2",self.port.commands)
        motor5=next(a["stepper"] for a in document["steps"][0]["actions"][0]["parallel"] if a.get("stepper",{}).get("node")==5)
        self.assertIn(f"plan.action 1 0 {motor5['position']} {motor5['rpm']} {motor5['acc']} 0 0 0 0 1 5",self.port.commands)
        self.assertFalse(any(command in ("next","run") for command in self.port.commands))
        while self.port.status["phase"] in ("home_wait","return_wait"): self.request("/api/refresh",{})
        self.assertEqual(self.request("/api/command",{"command":"next"})[0],200)
        self.assertEqual(self.port.commands.count("next"),1)
    def test_http_return_wait_busy_guards_latest_selection_and_stop(self):
        self.port.return_delay=100
        self.assertEqual(self.request("/api/connect",{"port":"COM99"})[0],200)
        self.assertEqual(self.request("/api/command",{"command":"stage","stage":2})[0],200)
        for command in ("next","prev","run","home"):
            self.assertEqual(self.request("/api/command",{"command":command})[0],409)
        self.assertEqual(self.request("/api/apply",{"document":{"steps":[{"actions":[{"delay":{"ms":0}}]}]}})[0],409)
        self.assertEqual(self.request("/api/command",{"command":"stage","stage":3})[0],200)
        self.assertEqual(self.request("/api/command",{"command":"stage","stage":1})[0],200)
        f=self.request("/api/state")[1]["firmware"]
        self.assertEqual((f["stage"],f["pending_stage"],f["phase"]),(4,1,"return_wait"))
        self.assertEqual(self.request("/api/command",{"command":"stop"})[0],200)
        self.assertEqual(self.port.status["phase"],"fault")
        self.assertFalse(any(command in ("next","prev","run","home") for command in self.port.commands))
    def test_http_commit_returns_ready_only_after_outgoing_reset(self):
        self.port.status.update(stage=2,pulse9=1100,position6=321)
        self.assertEqual(self.request("/api/connect",{"port":"COM99"})[0],200)
        self.port.return_delay=4
        document={"stage":4,"steps":[{"actions":[{"delay":{"ms":0}}]}]}
        response=self.request("/api/apply",{"document":document})
        self.assertEqual(response[0],200); f=response[1]["result"]
        self.assertEqual((f["stage"],f["phase"],f["cylinder"]["reference_action"],f["pulse9"]),(4,"ready",2,1100))
        self.assertEqual(f["position6"],321)
        self.assertEqual(self.port.return_completed,[2]); self.assertEqual(self.port.home_starts,0)
        self.assertTrue(all(s["plan_version"]==0 and s["stage"]==2 for s in self.port.return_snapshots))
    def test_status_without_imu_and_no_motion(self):
        self.assertEqual(self.request("/api/connect",{"port":"COM99"})[0],200)
        self.assertEqual(self.request("/api/refresh",{})[0],200)
        f=self.request("/api/state")[1]["firmware"]
        self.assertFalse(any(key in f for key in ("imu","yaw","pitch","roll")))
        self.assertEqual(set(self.port.commands),{"status"})
    def test_four_profile_metadata_and_placeholder_roundtrip(self):
        profiles=self.request("/api/init")[1]["profiles"]
        self.assertEqual([p["key"] for p in profiles],list(PROFILES)); self.assertEqual(len(profiles),9)
        for stage in (1,2):
            result=self.request(f"/api/init?profile={stage}")
            self.assertEqual(result[0],200)
            config=result[1]["config"]
            self.assertEqual(config["document"]["stage"],stage)
            self.assertEqual(self.request("/api/config",{"document":config["document"],"expected_hash":config["hash"],"profile":stage})[0],200)
        self.assertEqual(self.port.commands,[])

    def test_nine_profile_files_alias_conflicts_and_prop_identity(self):
        configs={}
        for key in PROFILES:
            code,payload=self.request(f"/api/init?profile={key}")
            self.assertEqual(code,200); configs[key]=payload["config"]
            self.assertEqual((payload["config"]["document"]["stage"],payload["config"]["document"].get("prop",0)),profile_identity(key))
        self.assertEqual(self.request("/api/init?profile=1")[1]["config"],configs["1_2"])
        for key in ("1_0","1_6","1_1/../../x",True):
            self.assertEqual(self.request(f"/api/init?profile={key}")[0],400)
        for key in list(PROFILES)[:5]:
            document=configs[key]["document"]
            document["steps"][0]["actions"][0]["delay"]["ms"]=100+document["prop"]
            self.assertEqual(self.request("/api/config",{"document":document,"profile":key,"expected_hash":configs[key]["hash"]})[0],200)
        wrong=dict(configs["1_1"]["document"]);wrong["prop"]=2
        self.assertEqual(self.request("/api/config",{"document":wrong,"profile":"1_1","expected_hash":configs["1_1"]["hash"]})[0],409)
        self.assertEqual(self.port.commands,[])
    def test_legacy_stage1_save_writes_explicit_vase_prop(self):
        config=self.request("/api/init?profile=1")[1]["config"]
        doc={"stage":1,"steps":[{"actions":[{"delay":{"ms":100}}]}]}
        result=self.request("/api/config",{"document":doc,"profile":1,"expected_hash":config["hash"]})
        self.assertEqual(result[0],200)
        self.assertEqual(result[1]["result"]["document"]["prop"],2)
        self.assertIn("prop: 2",(self.app.project/"stage1_vase.yaml").read_text(encoding="utf-8"))
    def test_http_prop_ram_metadata_trigger_and_stage5_placeholder(self):
        self.assertEqual(self.request("/api/connect",{"port":"COM99"})[0],200)
        for prop in range(1,6):
            doc={"stage":1,"prop":prop,"steps":[{"actions":[{"servo":{"pin":2,"pulse_us":1600+prop}}]}]}
            result=self.request("/api/apply",{"document":doc})
            self.assertEqual(result[0],200);self.assertEqual((result[1]["result"]["stage"],result[1]["result"]["prop"]),(1,prop))
            self.assertEqual(self.request("/api/command",{"command":"prop","prop":prop})[0],200)
            for _ in range(4):self.request("/api/refresh",{})
        self.assertEqual(self.request("/api/command",{"command":"stage","stage":5})[0],200)
        for _ in range(4):self.request("/api/refresh",{})
        self.assertEqual((self.port.status["stage"],self.port.status["prop"]),(5,0))


if __name__=="__main__": unittest.main()
