"""Read-only intake NT4 recording. Never publishes controls or enables Driver Station."""

import argparse
import datetime
import json
import math
import pathlib
import struct
import time

import msgpack
import websocket


PREFIXES = [
    "/AdvantageKit/RealOutputs/AngularControllers/",
    "/AdvantageKit/RealOutputs/Shooter/",
    "/AdvantageKit/RealOutputs/Intake/",
    "/AdvantageKit/AngularSubsystems/",
    "/AdvantageKit/RealOutputs/AngularSubsystems/",
    "/AdvantageKit/RealOutputs/IntakeDiagnostics/",
    "/AdvantageKit/RealOutputs/DriveDiagnostics/",
    "/AdvantageKit/RealOutputs/LoopTiming/",
    "/AdvantageKit/Drive/Module",
    "/AdvantageKit/DriverStation/",
    "/AdvantageKit/RealMetadata/",
    "/Tuning/AngularSubsystems/",
    "/SmartDashboard/IntakeDiagnostics/",
    "/SmartDashboard/DriveDiagnostics/",
]


def capture(host, seconds, output):
    if not math.isfinite(seconds) or not 0 < seconds <= 300:
        raise ValueError("Capture duration must be between zero and 300 seconds")
    ws = websocket.create_connection(
        f"ws://{host}:5810/nt/bordie-intake-observer",
        subprotocols=["networktables.first.wpi.edu"],
        timeout=2,
        http_proxy_host=None,
    )
    ws.settimeout(0.25)
    ws.send(json.dumps([{"method": "subscribe", "params": {
        "subuid": 1, "topics": PREFIXES,
        "options": {"prefix": True, "periodic": 0.02, "all": True},
    }}]))
    names, latest, history = {}, {}, {}
    started = time.monotonic()
    utc = datetime.datetime.now(datetime.timezone.utc).isoformat()
    output.parent.mkdir(parents=True, exist_ok=True)
    last_saved = started
    interrupted = False

    def save_recording():
        result = {"capture_utc": utc, "duration_sec": time.monotonic() - started,
                  "topics": list(names.values()), "latest": latest, "history": history,
                  "interrupted": interrupted}
        output.write_text(json.dumps(result, indent=2), encoding="utf-8")
        return result

    try:
        while time.monotonic() - started < seconds:
            try:
                data = ws.recv()
            except websocket.WebSocketTimeoutException:
                continue
            if data in ("", b""):
                raise RuntimeError("NT4 connection closed during capture")
            if isinstance(data, str):
                for message in json.loads(data):
                    params = message.get("params", {})
                    if message.get("method") == "announce":
                        names[params["id"]] = params
                    elif message.get("method") == "unannounce":
                        names.pop(params["id"], None)
                continue
            unpacker = msgpack.Unpacker(raw=False)
            unpacker.feed(data)
            for topic_id, server_us, _, value in unpacker:
                topic = names.get(topic_id)
                if topic is None:
                    continue
                if isinstance(value, bytes):
                    if topic["type"] == "struct:Rotation2d" and len(value) == 8:
                        value = struct.unpack("<d", value)[0]
                    else:
                        value = {"hex": value.hex()}
                name = topic["name"]
                latest[name] = value
                history.setdefault(name, []).append([
                    time.monotonic() - started, server_us, value,
                ])
            if time.monotonic() - last_saved >= 5.0:
                save_recording()
                last_saved = time.monotonic()
    except BaseException:
        interrupted = True
        raise
    finally:
        ws.close()
        result = save_recording()
    for name, value in sorted(latest.items()):
        if any(part in name for part in (
            "IntakePivot", "IntakeRollers", "/Feeder/", "RealMetadata/",
            "/DriverStation/Enabled", "/DriverStation/Test",
            "/DriverStation/DSAttached", "/IntakeDiagnostics/",
        )):
            samples = history[name]
            print(json.dumps({"topic": name, "value": value, "samples": len(samples),
                              "first_server_us": samples[0][1],
                              "last_server_us": samples[-1][1]}))
    print("Saved", output.resolve())
    return result


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("output", type=pathlib.Path)
    parser.add_argument("--host", default="10.18.99.2")
    parser.add_argument("--seconds", type=float, default=10)
    args = parser.parse_args()
    capture(args.host, args.seconds, args.output)
