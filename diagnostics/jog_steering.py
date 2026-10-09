"""Operator-authorized visible steering jog using existing bounded robot pulses.

Never enables Driver Station, changes gains, or commands a drive motor. Each pulse
retains the robot's 0.25 V / 150 ms / 2 degree guards and independent watchdog.
Four pulses at most; stop requesting pulses once cumulative travel reaches 10 degrees.
"""
import argparse
import datetime
import json
import math
from pathlib import Path
import struct
import sys
import threading
import time

sys.path.insert(0, r"C:\Users\otgra\2026rebuilt-bordie-drum\tools")
from control_drive_diagnostics import NT4, OUT, DASH, MODULE, ControlError
from capture_drive_diagnostics import capture

parser = argparse.ArgumentParser()
parser.add_argument("module", type=int, choices=range(4))
args = parser.parse_args()
writer = NT4("10.18.99.2")
observer = NT4("10.18.99.2")
recording = None
capture_errors = []
capture_path = Path(__file__).parent / ("pid-visible-" + datetime.datetime.now().strftime("%Y%m%d-%H%M%S")
                                       + "-m" + str(args.module) + ".json")

def pump():
    writer.pump()
    observer.pump()

def guard():
    writer.require_test()
    observer.require_test()
    if observer.get(OUT + "Ready") is not True:
        raise ControlError("Diagnostic readiness lost: " + str(observer.get(OUT + "Status")))

def angle(field):
    value = observer.get(MODULE + str(args.module) + "/" + field)
    if isinstance(value, bytes) and len(value) == 8:
        value = struct.unpack("<d", value)[0]
    if not isinstance(value, (float, int)) or not math.isfinite(value):
        raise ControlError("Invalid steering angle")
    return value

def stopped():
    return observer.get(OUT + "Active") is False and all(
        abs(observer.get(MODULE + str(i) + "/" + field, math.inf)) < 0.01
        for i in range(4) for field in ("DriveAppliedVolts", "TurnAppliedVolts"))

def await_condition(predicate, seconds=2):
    deadline = time.monotonic() + seconds
    while time.monotonic() < deadline:
        pump()
        guard()
        if predicate():
            return
    raise ControlError("Timed out waiting for acknowledgement")

try:
    # Drain the writer's subscription backlog after opening the observer. No
    # activation is published until both clients have fresh enabled Test state.
    deadline = time.monotonic() + 0.5
    while time.monotonic() < deadline:
        pump()
    deadline = time.monotonic() + 45
    while time.monotonic() < deadline:
        pump()
        ds = observer.fresh_robot()
        writer_ds = writer.fresh_robot()
        if (ds["enabled"] and ds["test"] and ds["attached"] and not ds["estop"]
                and writer_ds["enabled"] and writer_ds["test"]):
            break
    else:
        raise ControlError("Waiting for enabled Test; no movement requested")
    guard()
    if observer.get(DASH + "Prepare") is not True or observer.get(OUT + "RemoteAllowedLatched") is not True:
        raise ControlError("Disabled-only diagnostic preparation is required")
    if not stopped():
        raise ControlError("All eight motor outputs must be zero before the jog")
    writer.publish(DASH + "RemoteActivation", False)
    writer.publish(DASH + "Stage", "FEEDBACK")
    writer.publish(DASH + "Module", float(args.module))
    await_condition(lambda: observer.get(DASH + "Stage") == "FEEDBACK"
                    and observer.get(DASH + "Module") == args.module
                    and observer.get(DASH + "RemoteActivation") is False)
    def observe():
        try:
            capture("10.18.99.2", 12, capture_path)
        except Exception as exc:
            capture_errors.append(str(exc))
    recording = threading.Thread(target=observe)
    recording.start()
    deadline = time.monotonic() + 0.4
    while time.monotonic() < deadline:
        pump()
        guard()
    if capture_errors:
        raise ControlError("Recording failed")
    starting = angle("TurnAbsolutePosition")
    total_started = time.monotonic()
    for pulse in range(4):
        travel = math.degrees(math.remainder(angle("TurnAbsolutePosition") - starting, 2 * math.pi))
        if abs(travel) >= 10 or time.monotonic() - total_started > 7:
            break
        await_condition(lambda: stopped() and all(
            abs(observer.get(MODULE + str(i) + "/TurnVelocityRadPerSec", math.inf)) < 0.1
            for i in range(4)))
        guard()
        nonce = max(int(time.time_ns() // 1_000_000),
                    int(observer.get(OUT + "LastConsumedNonce", 0)) + 1,
                    int(observer.get(DASH + "StartNonce", 0)) + 1)
        heartbeat = max(nonce, int(observer.get(DASH + "RemoteHeartbeat", 0))) + 1
        writer.publish(DASH + "RemoteHeartbeat", heartbeat)
        writer.publish(DASH + "RemoteActivation", True)
        writer.publish(DASH + "StartNonce", nonce)
        deadline = time.monotonic() + 2
        last_heartbeat = time.monotonic()
        matched_since = None
        while time.monotonic() < deadline:
            pump()
            guard()
            now = time.monotonic()
            if now - last_heartbeat >= 0.04:
                heartbeat += 1
                writer.publish(DASH + "RemoteHeartbeat", heartbeat)
                last_heartbeat = now
            if observer.get(OUT + "LastRunNonce") == nonce and observer.get(OUT + "Active") is False:
                matched_since = now if matched_since is None else matched_since
                if now - matched_since >= 0.1:
                    if observer.get(OUT + "LastRunPassed") is not True:
                        raise ControlError("Pulse stopped: " + str(observer.get(OUT + "LastRunReason")))
                    break
        else:
            raise ControlError("Pulse result unavailable; do not retry movement")
        writer.publish(DASH + "RemoteActivation", False)
        await_condition(lambda: stopped() and observer.get(DASH + "RemoteActivation") is False)
        travel = math.degrees(math.remainder(angle("TurnAbsolutePosition") - starting, 2 * math.pi))
        print(json.dumps({"module": args.module, "pulse": pulse + 1, "totalSteeringDegrees": travel,
                          "driveOutputsZero": True, "steeringOutputsStopped": True}), flush=True)
    await_condition(stopped)
finally:
    try:
        writer.publish(DASH + "RemoteActivation", False)
    finally:
        writer.close()
        observer.close()
        if recording:
            recording.join()
            print("Evidence:", capture_path)
            if capture_errors:
                raise ControlError("Capture failed: " + str(capture_errors))
