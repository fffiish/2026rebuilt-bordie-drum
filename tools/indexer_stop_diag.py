"""Why does the indexer (CAN 36 + follower 22) run briefly and then stop?

Captures the indexer's own telemetry alongside battery voltage and brownout state over
NetworkTables 4, then works out which of these explains each stop:

  * secondary current cutoff - current climbs to the secondary limit, output drops,
    battery stays healthy. The indexer's secondary limit (60 A) sits below its smart
    limit (80 A), so it can trip before the smart limit ever regulates.
  * brownout / voltage sag  - battery voltage collapses or the RIO reports brownout.
  * command ended           - the goal velocity itself went to zero (trigger released,
                              or code stopped it on purpose).
  * device dropout          - a controller stopped reporting as connected.
  * thermal                 - motor temperature high at the moment of the stop.

NT4 connection code follows the team's tools/capture_intake_diagnostics.py.

Usage:
  python indexer_stop_diag.py capture out.json --seconds 30   # then reproduce the stop
  python indexer_stop_diag.py analyze out.json
"""

import argparse
import datetime
import json
import math
import pathlib
import time

import msgpack
import websocket

PREFIXES = [
    "/AdvantageKit/AngularSubsystems/Indexer/",
    "/AdvantageKit/RealOutputs/AngularSubsystems/Indexer/",
    "/AdvantageKit/SystemStats/",
    "/AdvantageKit/PowerDistribution/",
    "/AdvantageKit/DriverStation/",
]

IDX = "/AdvantageKit/AngularSubsystems/Indexer/"
SECONDARY_LIMIT_A = 60.0


def capture(host, seconds, output):
    ws = websocket.create_connection(
        f"ws://{host}:5810/nt/indexer-stop-diag",
        subprotocols=["networktables.first.wpi.edu"],
        timeout=3,
        http_proxy_host=None,
    )
    ws.settimeout(0.25)
    ws.send(json.dumps([{"method": "subscribe", "params": {
        "subuid": 1, "topics": PREFIXES,
        "options": {"prefix": True, "periodic": 0.02, "all": True},
    }}]))
    names, history = {}, {}
    started = time.monotonic()
    print(f"Connected to {host}. Capturing {seconds:.0f} s - reproduce the stop now.")
    try:
        while time.monotonic() - started < seconds:
            try:
                data = ws.recv()
            except websocket.WebSocketTimeoutException:
                continue
            if isinstance(data, str):
                for message in json.loads(data):
                    params = message.get("params", {})
                    if message.get("method") == "announce":
                        names[params["id"]] = params
                continue
            unpacker = msgpack.Unpacker(raw=False)
            unpacker.feed(data)
            for topic_id, server_us, _, value in unpacker:
                topic = names.get(topic_id)
                if topic is None:
                    continue
                if isinstance(value, bytes):
                    value = {"hex": value.hex()}
                history.setdefault(topic["name"], []).append([server_us / 1e6, value])
    finally:
        ws.close()
    output.write_text(json.dumps({
        "capture_utc": datetime.datetime.now(datetime.timezone.utc).isoformat(),
        "history": history,
    }), encoding="utf-8")
    print(f"Saved {output} ({sum(len(v) for v in history.values())} samples, "
          f"{len(history)} topics)")


def _series(history, name):
    return [(t, v) for t, v in history.get(name, []) if isinstance(v, (int, float))]


def _scalar(v):
    if isinstance(v, list):
        nums = [x for x in v if isinstance(x, (int, float))]
        return max(nums) if nums else None
    return v if isinstance(v, (int, float)) else None


def _at(series, t):
    """Most recent value at or before time t."""
    best = None
    for ts, v in series:
        if ts <= t:
            best = v
        else:
            break
    return best


def _window(series, t0, t1):
    return [v for ts, v in series if t0 <= ts <= t1]


def analyze(path):
    h = json.loads(pathlib.Path(path).read_text())["history"]
    goal = _series(h, IDX + "GoalVel")
    vel = _series(h, IDX + "Velocity")
    volts = _series(h, IDX + "AppliedVolts")
    stator = _series(h, IDX + "StatorCurrent")
    bus = _series(h, IDX + "BusVolts")
    temps = [(t, _scalar(v)) for t, v in h.get(IDX + "MotorTemperatures", [])]
    battery = _series(h, "/AdvantageKit/SystemStats/BatteryVoltage")
    brownout = [(t, v) for t, v in h.get("/AdvantageKit/SystemStats/BrownedOut", [])]

    if not goal and not volts:
        print("No indexer telemetry captured. Check the robot code is running and the")
        print("indexer's logKey is still 'Indexer'.")
        return

    print("=== Overall ===")
    for label, s, unit in (("Stator current", stator, "A"), ("Indexer bus volts", bus, "V"),
                           ("Battery voltage", battery, "V")):
        vals = [v for _, v in s]
        if vals:
            print(f"  {label:18} min {min(vals):6.2f} {unit}   max {max(vals):6.2f} {unit}")
    if brownout:
        print(f"  Brownout ever true: {any(v is True for _, v in brownout)}")
    t_vals = [v for _, v in temps if v is not None]
    if t_vals:
        print(f"  Motor temperature  max {max(t_vals):.0f} C")

    # A stop = commanded to spin, but output collapses to ~0.
    print("\n=== Stops (commanded to run, output dropped) ===")
    running, stops = False, []
    for t, v in volts:
        g = _at(goal, t) or 0.0
        if abs(v) > 1.0:
            running = True
        elif running and abs(v) < 0.3:
            running = False
            stops.append((t, g))
    if not stops:
        print("  None detected in this capture.")
    for t, g in stops:
        peak = max((abs(x) for x in _window(stator, t - 0.5, t)), default=float("nan"))
        bmin = min(_window(battery, t - 0.5, t + 0.2), default=float("nan"))
        busmin = min(_window(bus, t - 0.5, t + 0.2), default=float("nan"))
        bo = any(v is True for ts, v in brownout if t - 0.5 <= ts <= t + 0.2)
        temp = _at(temps, t)
        if abs(g) < 1e-3:
            verdict = "COMMAND ENDED - goal velocity is zero, the code stopped it"
        elif bo or (not math.isnan(bmin) and bmin < 7.0):
            verdict = "BROWNOUT / VOLTAGE SAG"
        elif not math.isnan(peak) and peak >= SECONDARY_LIMIT_A * 0.9:
            verdict = (f"SECONDARY CURRENT CUTOFF - current hit {peak:.0f} A, near the "
                       f"{SECONDARY_LIMIT_A:.0f} A secondary limit, with voltage healthy")
        elif temp is not None and temp > 80:
            verdict = "THERMAL - motor hot"
        else:
            verdict = "UNCLEAR - output dropped with goal non-zero, no current or voltage cause"
        print(f"  t={t:8.2f}s  goal {g:7.1f}  peak current {peak:5.1f} A  "
              f"battery min {bmin:5.2f} V  bus min {busmin:5.2f} V  brownout {bo}")
        print(f"             -> {verdict}")


if __name__ == "__main__":
    p = argparse.ArgumentParser()
    sub = p.add_subparsers(dest="cmd", required=True)
    c = sub.add_parser("capture")
    c.add_argument("output", type=pathlib.Path)
    c.add_argument("--host", default="10.18.99.2")
    c.add_argument("--seconds", type=float, default=30)
    a = sub.add_parser("analyze")
    a.add_argument("input", type=pathlib.Path)
    args = p.parse_args()
    if args.cmd == "capture":
        capture(args.host, args.seconds, args.output)
        analyze(args.output)
    else:
        analyze(args.input)
