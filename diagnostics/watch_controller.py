"""Read-only XInput and NT4 observation; never publishes or commands the robot."""
import ctypes
import datetime
import json
import pathlib
import statistics
import struct
import threading
import time

import msgpack
import websocket

class Gamepad(ctypes.Structure):
    _fields_ = [("buttons", ctypes.c_ushort), ("lt", ctypes.c_ubyte), ("rt", ctypes.c_ubyte),
                ("lx", ctypes.c_short), ("ly", ctypes.c_short),
                ("rx", ctypes.c_short), ("ry", ctypes.c_short)]

class State(ctypes.Structure):
    _fields_ = [("packet", ctypes.c_uint), ("pad", Gamepad)]

start = time.monotonic()
duration = 180
raw = []
raw_events = []
finished = threading.Event()

def poll_xinput():
    api = ctypes.WinDLL("xinput1_4.dll")
    previous = None
    active = False
    while not finished.is_set():
        for port in range(4):
            state = State()
            if api.XInputGetState(port, ctypes.byref(state)) != 0:
                continue
            p = state.pad
            values = [p.lx/32768, -p.ly/32768, p.rx/32768]
            value = [port, state.packet, *values, p.buttons, p.lt, p.rt]
            if value != previous:
                now = time.monotonic() - start
                raw.append([now, value])
                moved = max(abs(v) for v in values) > .12
                if moved != active:
                    raw_events.append([now, moved])
                    print("RAW stick", "moved" if moved else "released", round(now, 3), flush=True)
                active = moved
                previous = value
            break
        finished.wait(.005)

thread = threading.Thread(target=poll_xinput, daemon=True)
thread.start()
prefixes = ["/AdvantageKit/DriverStation/", "/AdvantageKit/Drive/Module",
            "/AdvantageKit/Drive/Gyro/", "/AdvantageKit/RealOutputs/LoopTiming/",
            "/AdvantageKit/RealOutputs/Timing/", "/AdvantageKit/RealOutputs/Health/",
            "/AdvantageKit/RealOutputs/SwerveChassisSpeeds/", "/AdvantageKit/Timestamp"]
ws = websocket.create_connection("ws://10.18.99.2:5810/nt/controller-test-observer",
    subprotocols=["networktables.first.wpi.edu"], timeout=1, http_proxy_host=None)
ws.send(json.dumps([{"method": "subscribe", "params": {"subuid": 1,
    "topics": prefixes, "options": {"prefix": True, "periodic": .02, "all": True}}}]))
names, history = {}, {}
axis_active = False
robot_events = []
last_report = 0
utc = datetime.datetime.now(datetime.timezone.utc).isoformat()
print("MONITOR READY: 180 seconds of raw Windows input, robot axes, drive outputs and timing", flush=True)
try:
    while time.monotonic() - start < duration:
        try:
            packet = ws.recv()
        except websocket.WebSocketTimeoutException:
            continue
        if isinstance(packet, str):
            for m in json.loads(packet):
                if m.get("method") == "announce":
                    p = m["params"]
                    names[p["id"]] = p
            continue
        unpacker = msgpack.Unpacker(raw=False)
        unpacker.feed(packet)
        for tid, ts, dtype, value in unpacker:
            if tid not in names:
                continue
            name = names[tid]["name"]
            if isinstance(value, bytes):
                value = list(struct.unpack("<" + "d"*(len(value)//8), value)) if len(value)%8 == 0 else {"hex": value.hex()}
            now = time.monotonic() - start
            history.setdefault(name, []).append([now, ts, value])
            if name.endswith("/Joystick0/AxisValues") and len(value) >= 5:
                moved = max(abs(value[i]) for i in [0,1,4]) > .12
                if moved != axis_active:
                    robot_events.append([now, ts, moved])
                    print("ROBOT stick", "moved" if moved else "released", round(now, 3), flush=True)
                axis_active = moved
            if name.endswith("/DriverStation/Enabled"):
                print("ROBOT enabled:", value, flush=True)
        if time.monotonic()-start-last_report >= 20:
            last_report = time.monotonic()-start
            periods = [r[2] for r in history.get("/AdvantageKit/RealOutputs/LoopTiming/PeriodMS", [])[-1000:]]
            enabled = history.get("/AdvantageKit/DriverStation/Enabled", [[0,0,None]])[-1][2]
            print("STATUS", json.dumps({"elapsed": round(last_report), "enabled": enabled,
                "axis_transitions": len(robot_events), "loop_median_ms": statistics.median(periods) if periods else None,
                "loop_p95_ms": sorted(periods)[int(.95*(len(periods)-1))] if periods else None}), flush=True)
finally:
    finished.set()
    thread.join(1)
    ws.close()
    output = pathlib.Path(__file__).parent / "controller_teleop_test.json"
    output.write_text(json.dumps({"capture_utc": utc, "duration_sec": time.monotonic()-start,
        "history": history, "raw_xinput": raw, "raw_events": raw_events,
        "robot_events": robot_events, "topics": list(names.values())}, indent=2), encoding="utf-8")
    print("SAVED", output, flush=True)
