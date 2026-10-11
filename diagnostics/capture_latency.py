"""Subscribe to robot telemetry only; never publish or command hardware."""
import datetime
import json
import pathlib
import statistics
import time

import msgpack
import websocket

output = pathlib.Path(__file__).parent
prefixes = ["/AdvantageKit/RealOutputs/Timing/", "/AdvantageKit/RealOutputs/LoopTiming/",
            "/AdvantageKit/RealOutputs/AngularSubsystems/", "/AdvantageKit/DriverStation/",
            "/AdvantageKit/SystemStats/", "/AdvantageKit/RealMetadata/",
            "/AdvantageKit/Hood/", "/AdvantageKit/Timestamp", "/AdvantageKit/Drive/Module"]
ws = websocket.create_connection("ws://10.18.99.2:5810/nt/controller-latency-readonly",
    subprotocols=["networktables.first.wpi.edu"], timeout=1, http_proxy_host=None)
ws.send(json.dumps([{"method": "subscribe", "params": {"subuid": 1,
    "topics": prefixes, "options": {"prefix": True, "periodic": .02, "all": True}}}]))
names, history = {}, {}
start = time.monotonic()
utc = datetime.datetime.now(datetime.timezone.utc).isoformat()
try:
    while time.monotonic() - start < 20:
        try:
            packet = ws.recv()
        except websocket.WebSocketTimeoutException:
            continue
        if isinstance(packet, str):
            for message in json.loads(packet):
                if message.get("method") == "announce":
                    p = message["params"]
                    names[p["id"]] = p
            continue
        unpacker = msgpack.Unpacker(raw=False)
        unpacker.feed(packet)
        for topic_id, timestamp, dtype, value in unpacker:
            if topic_id not in names:
                continue
            if isinstance(value, bytes):
                value = {"hex": value.hex()}
            history.setdefault(names[topic_id]["name"], []).append(
                [time.monotonic() - start, timestamp, value])
finally:
    ws.close()
result = {"capture_utc": utc, "duration_sec": time.monotonic() - start,
          "topics": list(names.values()), "history": history}
(output / "live_latency.json").write_text(json.dumps(result, indent=2), encoding="utf-8")
summary = {}
for name, rows in sorted(history.items()):
    values = [r[2] for r in rows]
    s = {"samples": len(rows), "last": values[-1], "span_seconds": rows[-1][0]-rows[0][0]}
    if all(isinstance(v, (int, float)) and not isinstance(v, bool) for v in values):
        ordered = sorted(values)
        s.update(min=min(values), median=statistics.median(values), max=max(values),
                 mean=statistics.mean(values), p95=ordered[int(.95*(len(values)-1))])
    summary[name] = s
    if any(word in name for word in ["Timing/", "LoopTiming/", "Enabled", "GitSHA", "BuildDate", "CANBus", "ControllerConnected", "CPUTemp", "CPUUsage"]):
        print(name, json.dumps(s))
(output / "live_latency_summary.json").write_text(json.dumps(summary, indent=2), encoding="utf-8")
print("Saved", output / "live_latency.json")
