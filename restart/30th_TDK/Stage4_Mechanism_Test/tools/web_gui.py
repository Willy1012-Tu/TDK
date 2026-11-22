"""LAN control gateway. No serial port is opened until the user clicks Connect."""
import argparse
import hashlib
import json
import re
import secrets
import socket
import sys
import threading
import time
from collections import deque
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path
from urllib.parse import urlsplit, parse_qs

PROJECT = Path(__file__).resolve().parents[1]
PROFILE_NAMES = {2: "第二關 · 文蛤分級", 3: "第三關 · 待機／未接線", 4: "第四關 · 北港迎媽祖／擲筊"}
PROFILES = {stage: f"stage{stage}_steps.yaml" for stage in PROFILE_NAMES}
sys.path.insert(0, str(PROJECT / ".pio" / "python-deps"))
import serial
from serial.tools import list_ports
from generate_steps import ConfigError, StrictLoader, compile_document, wire_document, yaml, MOTOR_IDS, SERVO_PINS, MAX_ACTIONS


class GatewayError(RuntimeError):
    pass


def bounded(value, low, high, name):
    if type(value) is not int or not low <= value <= high:
        raise GatewayError(f"{name} 必須為 {low}～{high} 的整數")
    return value


def profile_number(value=4):
    # File names and paths are never accepted from the browser.
    if isinstance(value, str) and value in ("2", "3", "4"):
        value = int(value)
    if type(value) is not int or value not in PROFILES:
        raise GatewayError("關卡只能選擇第 2、3、4 關")
    return value


def status_pins(status):
    declared = status.get("servo_pins")
    if declared is None:
        declared = [status[key] for key in ("pin2", "pin5", "pin4", "pin6", "pin9") if key in status]
    return tuple(pin for pin in declared if type(pin) is int and pin in SERVO_PINS)


def status_nodes(status):
    return tuple(node for node in status.get("motor_ids", [status.get("motor_id", 6)])
                 if type(node) is int and node in MOTOR_IDS)


def status_stages(status):
    return status.get("stage_ids", [3, 4])


class Bridge:
    def __init__(self, port_factory=serial.Serial):
        self.factory=port_factory
        self.port=None
        self.port_name=""
        self.lock=threading.RLock()
        self.write_lock=threading.Lock()
        self.operation_lock=threading.Lock()
        self.pending={}
        self.request_id=0
        self.generation=0
        self.logs=deque(maxlen=500)
        self.log_id=0
        self.last_status=None
        self.status_time=0
        self.last_error=""
        self.applied_hash=None
        self.applied_version=None

    def log(self, line, direction="rx"):
        with self.lock:
            self.log_id+=1
            self.logs.append({"id":self.log_id,"time":time.strftime("%H:%M:%S"),"direction":direction,"text":line})

    def connect(self, name):
        if not isinstance(name,str) or not re.fullmatch(r"COM[1-9][0-9]*",name,re.I):
            raise GatewayError("請選擇有效的 COM 串口")
        with self.operation_lock:
            self.disconnect()
            try:
                port=self.factory(name,115200,timeout=0.2,write_timeout=1)
            except serial.SerialException as exc:
                raise GatewayError("無法開啟串口；請先關閉 PlatformIO Monitor。"+str(exc)) from exc
            with self.lock:
                self.port=port; self.port_name=name; self.generation+=1
                generation=self.generation; self.last_error=""; self.last_status=None
                self.applied_hash=None; self.applied_version=None
            threading.Thread(target=self._reader,args=(port,generation),daemon=True).start()
            # Query only; connecting does not issue a movement command.
            try:
                return self.refresh()
            except GatewayError:
                self.disconnect()
                raise

    def disconnect(self):
        with self.write_lock, self.lock:
            port=self.port; self.port=None; self.port_name=""; self.generation+=1
            self.last_status=None; self.status_time=0; self.applied_hash=None; self.applied_version=None
            for entry in self.pending.values():
                entry["error"]="串口已中斷"; entry["event"].set()
            self.pending.clear()
            if port:
                port.close()

    def _reader(self, port, generation):
        buffer=b""
        try:
            while True:
                with self.lock:
                    if self.port is not port or generation!=self.generation:
                        return
                # Read available bytes, rather than waiting 200ms to fill 512
                # bytes for every small acknowledgement during a large upload.
                data=port.read(max(1,min(512,port.in_waiting)))
                if not data:
                    continue
                buffer+=data
                if len(buffer)>16384:
                    buffer=b""; self.log("接收內容過長，已清除", "gateway")
                while b"\n" in buffer:
                    raw,buffer=buffer.split(b"\n",1)
                    line=raw.decode("utf-8",errors="replace").strip()
                    if not line:
                        continue
                    match=re.fullmatch(r"@(\d+) (OK|ERR)(?: (.*))?",line)
                    if match:
                        with self.lock:
                            entry=self.pending.get(int(match[1]))
                            if entry:
                                entry["error"]=match[3] if match[2]=="ERR" else None
                                entry["result"]=match[3] or ""
                                entry["event"].set()
                        if match[2]=="ERR": self.log(line)
                    else:
                        self.log(line)
        except (serial.SerialException,OSError) as exc:
            with self.lock:
                if self.port is port:
                    self.last_error="USB 串口中斷，請重新連線。"+str(exc)
                    for entry in self.pending.values():
                        entry["error"]=self.last_error; entry["event"].set()
                    self.port=None; self.last_status=None; self.applied_hash=None
            try: port.close()
            except OSError: pass

    def request(self, command, timeout=2):
        with self.write_lock:
            with self.lock:
                if not self.port:
                    raise GatewayError("尚未連接 Teensy")
                self.request_id=self.request_id%1000000000+1
                request=self.request_id; port=self.port
                entry={"event":threading.Event(),"error":None,"result":""}
                self.pending[request]=entry
            packet=f"@{request} {command}\n".encode("ascii")
            if len(packet)>128:
                with self.lock: self.pending.pop(request,None)
                raise GatewayError("命令過長")
            try:
                written=port.write(packet)
                if written!=len(packet): raise GatewayError("串口命令未完整發送")
            except (serial.SerialException,OSError,GatewayError) as exc:
                with self.lock: self.pending.pop(request,None)
                raise GatewayError("串口發送失敗："+str(exc)) from exc
        try:
            if not entry["event"].wait(timeout):
                raise GatewayError("韌體未回覆；請確認已上傳 Web GUI 版韌體。命令結果未知，請查狀態再操作。")
            if entry["error"]:
                raise GatewayError("韌體拒絕命令（模式、忙碌、停止或數值不符）："+entry["error"])
            return entry["result"]
        finally:
            with self.lock: self.pending.pop(request,None)

    def refresh(self):
        result=self.request("status")
        try:
            status=json.loads(result)
            if status.get("protocol")!=1: raise ValueError("protocol")
        except (ValueError,AttributeError) as exc:
            raise GatewayError("韌體狀態格式不相容") from exc
        with self.lock:
            if self.last_status and (status["plan_version"]!=self.applied_version or not status["runtime_plan"]):
                self.applied_hash=None
            self.last_status=status; self.status_time=time.monotonic()
        return status

    def command(self, payload):
        kind=payload.get("command")
        if not isinstance(kind,str): raise GatewayError("命令格式錯誤")
        if kind=="stop":
            # Does not wait for an ongoing configuration transfer's operation lock.
            self.log("stop","tx"); return self.request("stop")
        with self.operation_lock:
            status=self.refresh()
            if kind and kind.startswith("debug.") and status.get("stage") == 3 and status.get("stage3_enabled") is False:
                raise GatewayError("第三關待機／未接線；請切換到第二或第四關調試")
            if kind=="mode":
                mode=payload.get("mode")
                if mode not in ("normal","debug"): raise GatewayError("模式錯誤")
                command="mode "+mode
            elif kind in ("next","run","home"):
                if status["mode"]!="normal": raise GatewayError("請先切換至 Normal 模式")
                command=kind
            elif kind=="stage":
                if status["mode"]!="normal": raise GatewayError("請先切換至 Normal 模式")
                stage=profile_number(payload.get("stage"))
                if stage not in status_stages(status): raise GatewayError("韌體不支援這個關卡；請更新整合韌體")
                command=f"stage {stage}"
            elif kind=="debug.gohome":
                node=bounded(payload.get("node",6),4,7,"CAN ID")
                if node not in status_nodes(status): raise GatewayError("韌體不支援這顆步進馬達")
                command=f"{kind} {node}"
            elif kind=="debug.move":
                position=bounded(payload.get("position"),-8388607,8388607,"位置")
                rpm=bounded(payload.get("rpm"),1,3000,"RPM")
                acc=bounded(payload.get("acc"),0,255,"ACC")
                node=bounded(payload.get("node",6),4,7,"CAN ID")
                if node not in status_nodes(status): raise GatewayError("韌體不支援這顆步進馬達")
                command=f"debug.move {node} {position} {rpm} {acc}"
            elif kind=="debug.servo":
                pin=bounded(payload.get("pin"),0,39,"pin")
                if pin not in status_pins(status): raise GatewayError("伺服腳位不符")
                angle=payload.get("angle")
                if type(angle) not in (int,float) or not 0<=angle<=180: raise GatewayError("角度須為 0～180 度")
                duration=bounded(payload.get("duration_ms"),0,60000,"duration_ms")
                command=f"debug.servo {pin} {int(angle*10+0.5)} {duration}"
            else:
                raise GatewayError("不支援的命令")
            if kind.startswith("debug.") and status["mode"]!="debug": raise GatewayError("請先切換至 Debug 模式")
            self.log(command,"tx"); result=self.request(command)
            self.refresh(); return result

    def apply(self, document):
        actions,steps=wire_document(document)
        stage=profile_number(document.get("stage",4))
        repeat_from=document.get("repeat_from",1)
        with self.operation_lock:
            status=self.refresh()
            if status["mode"]!="normal" or status["phase"]!="ready" or status["uploading"]:
                raise GatewayError("立即套用需要 Normal 模式且機構閒置")
            if stage not in status_stages(status): raise GatewayError("韌體不支援這個關卡；請更新整合韌體")
            if len(actions)>status.get("max_runtime_actions",MAX_ACTIONS): raise GatewayError("動作數超過韌體 RAM 容量")
            for a in actions:
                if a[0]==1 and a[4] not in status_pins(status): raise GatewayError("YAML 伺服腳位與韌體不符")
                if a[0]==0 and a[9] not in status_nodes(status): raise GatewayError("YAML 步進 ID 與韌體不符")
            old_version=status["plan_version"]
            began=False
            try:
                self.request(f"plan.begin {len(steps)} {len(actions)} {stage} {repeat_from}"); began=True
                for index,a in enumerate(actions):
                    self.request("plan.action "+" ".join(map(str,[index]+a)))
                for index,(start,count) in enumerate(steps):
                    self.request(f"plan.step {index} {start} {count}")
                self.request("plan.commit")
                # Wait for the validated RAM commit and active-stage snapshot.
                # The acknowledgement accepts that operation, not completion.
                deadline=time.monotonic()+35
                while True:
                    status=self.refresh()
                    if status["phase"]=="fault": raise GatewayError("套用期間機構已停止，未確認新設定生效")
                    if status["phase"]=="ready" and not status["uploading"] and status["runtime_plan"] and status["plan_version"]!=old_version and status.get("stage",4)==stage:
                        break
                    if time.monotonic()>=deadline: raise GatewayError("等待設定切換逾時，請檢查控制器狀態")
                    time.sleep(0.1)
            except GatewayError:
                if began:
                    try: self.request("plan.abort",timeout=0.5)
                    except GatewayError: pass
                self.applied_hash=None
                raise
            with self.lock:
                self.applied_hash=document_hash(document); self.applied_version=status["plan_version"]
            detail=f"第二關 STEP1 預備已完成；NEXT 從 STEP{2 if len(steps)>1 else repeat_from} 開始。" if stage==2 else "尚未執行 STEP。"
            self.log(f"已切換至第 {stage} 關：{len(steps)} 段 STEP、{len(actions)} 個內部動作；{detail}","gateway")
            return status

    def snapshot(self, after=0):
        with self.lock:
            return {"connected":self.port is not None,"port":self.port_name,"firmware":self.last_status,
                "status_age":time.monotonic()-self.status_time if self.status_time else None,
                "error":self.last_error,"logs":[item for item in self.logs if item["id"]>after],
                "applied_hash":self.applied_hash}


def document_hash(document):
    return hashlib.sha256(json.dumps(document,sort_keys=True,separators=(",",":"),ensure_ascii=False).encode()).hexdigest()


class Application:
    def __init__(self, project=PROJECT, bridge=None):
        self.project=Path(project); self.bridge=bridge or Bridge(); self.csrf=secrets.token_urlsafe(32)
        self.file_lock=threading.Lock()
        self.simulation=False
        self.ports=list_ports.comports

    def load_config(self, profile=4):
        profile=profile_number(profile)
        with self.file_lock:
            text=(self.project/PROFILES[profile]).read_text(encoding="utf-8-sig")
        document=yaml.load(text,Loader=StrictLoader); compile_document(document)
        if profile_number(document.get("stage",4))!=profile: raise GatewayError("YAML 的 stage 與所選關卡不符")
        return {"document":document,"yaml":text,"hash":document_hash(document),"profile":profile}

    def save_config(self, document, expected_hash, profile=4):
        profile=profile_number(profile)
        compile_document(document)
        if profile_number(document.get("stage",4))!=profile: raise GatewayError("YAML 的 stage 與所選關卡不符")
        target=self.project/PROFILES[profile]
        with self.file_lock:
            current=yaml.load(target.read_text(encoding="utf-8-sig"),Loader=StrictLoader)
            if expected_hash!=document_hash(current): raise GatewayError("YAML 已被其他視窗或 IDE 修改，請重新載入後編輯。")
            text="# Web GUI 編輯；立即套用至 RAM，或重新 Build / Upload 作為開機預設。\n"+yaml.safe_dump(document,allow_unicode=True,sort_keys=False)
            temporary=target.with_suffix(".yaml.tmp")
            temporary.write_text(text,encoding="utf-8"); temporary.replace(target)
        return {"document":document,"yaml":text,"hash":document_hash(document),"profile":profile}


class Handler(BaseHTTPRequestHandler):
    server_version="TDKRobotLAN/1"

    def valid_host(self):
        host=urlsplit("http://"+self.headers.get("Host","")).hostname
        if not host or host.lower() not in self.server.allowed_hosts:
            self.json_response({"error":"Host not allowed"},403); return False
        return True

    def json_response(self, payload, status=200):
        data=json.dumps(payload,ensure_ascii=False,allow_nan=False).encode()
        self.send_response(status); self.send_header("Content-Type","application/json; charset=utf-8")
        self.send_header("Content-Length",str(len(data))); self.send_header("Cache-Control","no-store")
        self.send_header("X-Content-Type-Options","nosniff"); self.end_headers(); self.wfile.write(data)

    def do_GET(self):
        if not self.valid_host(): return
        app=self.server.app; parsed=urlsplit(self.path)
        try:
            if parsed.path=="/api/init":
                profile=parse_qs(parsed.query).get("profile",["4"])[0]
                self.json_response({"token":app.csrf,"config":app.load_config(profile),"profiles":[{"stage":stage,"name":name} for stage,name in PROFILE_NAMES.items()],"simulation":app.simulation}); return
            if parsed.path=="/api/ports":
                self.json_response({"ports":[{"device":p.device,"description":p.description} for p in app.ports()]}); return
            if parsed.path=="/api/state":
                after=int(parse_qs(parsed.query).get("after",[0])[0])
                self.json_response(app.bridge.snapshot(after)); return
            files={"/":"index.html","/app.js":"app.js","/style.css":"style.css"}
            if parsed.path not in files: self.json_response({"error":"Not found"},404); return
            target=app.project/"web"/files[parsed.path]
            data=target.read_bytes(); content_type={".html":"text/html; charset=utf-8",".js":"text/javascript; charset=utf-8",".css":"text/css; charset=utf-8"}[target.suffix]
            self.send_response(200); self.send_header("Content-Type",content_type); self.send_header("Content-Length",str(len(data)))
            self.send_header("Cache-Control","no-store"); self.send_header("X-Content-Type-Options","nosniff")
            self.send_header("Content-Security-Policy","default-src 'self'; style-src 'self'; script-src 'self'; connect-src 'self'; frame-ancestors 'none'")
            self.end_headers(); self.wfile.write(data)
        except (OSError,ValueError,ConfigError,GatewayError,yaml.YAMLError) as exc:
            self.json_response({"error":str(exc)},400)

    def do_POST(self):
        if not self.valid_host(): return
        app=self.server.app
        # No CORS. A per-process token and same-origin check prevent drive-by commands.
        origin=self.headers.get("Origin")
        if origin and urlsplit(origin).netloc!=self.headers.get("Host"):
            self.json_response({"error":"Origin mismatch"},403); return
        if self.headers.get("X-Stage4-Token")!=app.csrf:
            self.json_response({"error":"Invalid session token"},403); return
        try:
            size=int(self.headers.get("Content-Length","0"))
            if not 0<size<=1024*1024: raise GatewayError("請求內容大小錯誤")
            payload=json.loads(self.rfile.read(size))
            if not isinstance(payload,dict): raise GatewayError("請求格式錯誤")
            path=urlsplit(self.path).path
            if path=="/api/connect": result=app.bridge.connect(payload.get("port"))
            elif path=="/api/disconnect": app.bridge.disconnect(); result={}
            elif path=="/api/command": result=app.bridge.command(payload)
            elif path=="/api/refresh": result=app.bridge.refresh()
            elif path=="/api/apply": result=app.bridge.apply(payload.get("document"))
            elif path=="/api/config": result=app.save_config(payload.get("document"),payload.get("expected_hash"),payload.get("profile",4))
            else: self.json_response({"error":"Not found"},404); return
            self.json_response({"result":result})
        except (GatewayError,ConfigError,yaml.YAMLError,ValueError,TypeError,OSError) as exc:
            self.json_response({"error":str(exc)},409 if isinstance(exc,GatewayError) else 400)

    def log_message(self, format, *args):
        if args and str(args[0]).startswith("GET /api/state"): return
        super().log_message(format,*args)


def make_server(host, port, app=None):
    server=ThreadingHTTPServer((host,port),Handler); server.app=app or Application(); server.daemon_threads=True
    server.allowed_hosts={"localhost","127.0.0.1",socket.gethostname().lower()}
    server.allowed_hosts.update(item[4][0] for item in socket.getaddrinfo(socket.gethostname(),None,socket.AF_INET))
    if host!="0.0.0.0": server.allowed_hosts.add(host.lower())
    return server


def main():
    parser=argparse.ArgumentParser()
    parser.add_argument("--host",default="0.0.0.0")
    parser.add_argument("--port",type=int,default=8765)
    args=parser.parse_args()
    app=Application(); server=make_server(args.host,args.port,app)
    local_address="127.0.0.1" if args.host=="0.0.0.0" else args.host
    print(f"TDK Robot Web GUI: http://{local_address}:{server.server_port}",flush=True)
    if args.host=="0.0.0.0":
        for address in sorted({item[4][0] for item in socket.getaddrinfo(socket.gethostname(),None,socket.AF_INET)}):
            if not address.startswith("127."): print(f"LAN: http://{address}:{server.server_port}",flush=True)
    print("No serial connection yet. Select a COM port in the browser. Ctrl+C exits.",flush=True)
    try: server.serve_forever()
    except KeyboardInterrupt: pass
    finally: app.bridge.disconnect(); server.server_close()


if __name__=="__main__": main()
