"""Read-only NT4 capture. Does not publish commands, enable the robot, or move motors."""
import argparse
import datetime
import json
import pathlib
import struct
import time

import msgpack
import websocket


def capture(host, seconds, output):
    prefixes = ["/AdvantageKit/Drive/Module", "/AdvantageKit/RealOutputs/Drive/",
                "/AdvantageKit/RealOutputs/DriveDiagnostics/",
                "/AdvantageKit/RealOutputs/LoopTiming/", "/AdvantageKit/DriverStation/",
                "/AdvantageKit/RealMetadata/", "/SmartDashboard/DriveDiagnostics/",
                "/AdvantageKit/RealOutputs/Vision/", "/AdvantageKit/RealOutputs/AngularSubsystems/"]
    ws = websocket.create_connection(
        f"ws://{host}:5810/nt/bordie-drive-observer",
        subprotocols=["networktables.first.wpi.edu"], timeout=1, http_proxy_host=None)
    ws.send(json.dumps([{"method": "subscribe", "params": {
        "subuid": 1, "topics": prefixes, "options": {
            "prefix": True, "periodic": .02, "all": True}}}]))
    names, latest, history = {}, {}, {}
    started = time.monotonic()
    utc = datetime.datetime.now(datetime.timezone.utc).isoformat()
    try:
        while time.monotonic() - started < seconds:
            try:
                data = ws.recv()
            except websocket.WebSocketTimeoutException:
                continue
            if isinstance(data, str):
                for message in json.loads(data):
                    if message.get("method") == "announce":
                        params = message["params"]
                        names[params["id"]] = params
                continue
            unpacker = msgpack.Unpacker(raw=False)
            unpacker.feed(data)
            for topic_id, server_us, data_type, value in unpacker:
                topic = names.get(topic_id)
                if not topic:
                    continue
                if isinstance(value, bytes):
                    if topic["type"] == "struct:Rotation2d" and len(value) == 8:
                        value = struct.unpack("<d", value)[0]
                    else:
                        value = {"hex": value.hex()}
                name = topic["name"]
                latest[name] = value
                history.setdefault(name, []).append([
                    time.monotonic() - started, server_us, value])
    finally:
        ws.close()
    result = {"capture_utc": utc, "duration_sec": time.monotonic() - started,
              "topics": list(names.values()), "latest": latest, "history": history}
    output.parent.mkdir(parents=True, exist_ok=True)
    output.write_text(json.dumps(result, indent=2), encoding="utf-8")
    for name, value in sorted(latest.items()):
        if any(key in name for key in ["/Diagnostics/", "DriveDiagnostics/",
                                      "ConfigurationHealthy", "AppliedDriveK", "AppliedTurnK",
                                      "/DriverStation/Enabled", "PeriodMS"]):
            if not name.endswith("/Trace"):
                print(name, value)
    print("Saved", output.resolve())


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("output", type=pathlib.Path)
    parser.add_argument("--host", default="10.18.99.2")
    parser.add_argument("--seconds", type=float, default=15)
    args = parser.parse_args()
    capture(args.host, args.seconds, args.output)
