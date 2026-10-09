"""Run only an authorized, robot-bounded steering stage with read-only capture."""
import argparse
import datetime
import json
from pathlib import Path
import sys
import threading
import time
from types import SimpleNamespace

sys.path.insert(0, r"C:\Users\otgra\2026rebuilt-bordie-drum\tools")
from control_drive_diagnostics import NT4, OUT, MODULE, ControlError, run
from capture_drive_diagnostics import capture

parser = argparse.ArgumentParser()
parser.add_argument("stage", choices=["FEEDBACK", "STEER_POSITIVE", "STEER_NEGATIVE"])
parser.add_argument("module", type=int, choices=range(4))
args = parser.parse_args()
client = NT4("10.18.99.2")
observer_thread = None
errors = []
run_error = None
capture_path = Path(__file__).parent / (
    "pid-" + datetime.datetime.now().strftime("%Y%m%d-%H%M%S")
    + "-m" + str(args.module) + "-" + args.stage.lower() + ".json")
try:
    deadline = time.monotonic() + 45
    while time.monotonic() < deadline:
        client.pump()
        ds = client.fresh_robot()
        if ds["enabled"] and ds["test"] and ds["attached"] and not ds["estop"]:
            break
    else:
        raise ControlError("Waiting for enabled Test; no movement requested")
    client.require_test()
    if client.get(OUT + "Ready") is not True:
        raise ControlError("Diagnostic not ready: " + str(client.get(OUT + "Status")))
    baseline_nonce = client.get(OUT + "LastRunNonce", 0)
    def observe():
        try:
            capture("10.18.99.2", 6 if args.stage == "FEEDBACK" else 17, capture_path)
        except Exception as exc:
            errors.append(str(exc))
    observer_thread = threading.Thread(target=observe)
    observer_thread.start()
    # Establish the independent recording before publishing an activation nonce.
    wait_until = time.monotonic() + 0.6
    while time.monotonic() < wait_until:
        client.pump()
    if errors:
        raise ControlError("Recording failed: " + errors[0])
    try:
        run(client, SimpleNamespace(host="10.18.99.2", stage=args.stage,
                                   module=args.module, magnitude=0.5))
    except ControlError as exc:
        # NT topics arrive separately. The existing client can see the new result
        # nonce before its pass flag. Do not repeat movement; verify the recording.
        run_error = exc
finally:
    client.close()
    if observer_thread:
        observer_thread.join()
        if errors:
            print("CAPTURE ERROR:", errors, file=sys.stderr)
        print("Evidence:", capture_path)
if capture_path.exists():
    latest = json.loads(capture_path.read_text())["latest"]
    result = {"stage": args.stage, "module": args.module,
              "nonce": latest.get(OUT + "LastRunNonce"),
              "passed": latest.get(OUT + "LastRunPassed"),
              "reason": latest.get(OUT + "LastRunReason"),
              "active": latest.get(OUT + "Active"),
              "driveVolts": [latest.get(MODULE + str(i) + "/DriveAppliedVolts") for i in range(4)],
              "turnVolts": [latest.get(MODULE + str(i) + "/TurnAppliedVolts") for i in range(4)]}
    print("Recorded final result:", json.dumps(result, indent=2))
    stopped = (result["active"] is False
               and all(isinstance(v, (int, float)) and abs(v) < 0.01
                       for v in result["driveVolts"] + result["turnVolts"]))
    if not stopped or errors:
        raise ControlError("Stopped outputs or capture could not be verified")
    current_run = (isinstance(result["nonce"], (int, float)) and result["nonce"] > baseline_nonce
                   and latest.get(OUT + "StageActive") == args.stage
                   and latest.get(OUT + "SelectedModule") == args.module)
    if not current_run or result["passed"] is not True:
        raise run_error or ControlError("Recorded run did not pass: " + str(result["reason"]))
