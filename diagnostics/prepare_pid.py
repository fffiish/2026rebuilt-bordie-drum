"""Prepare existing bounded steering diagnostics while disabled; never enables DS."""
import argparse
import json
import math
import sys
import time

sys.path.insert(0, r"C:\Users\otgra\2026rebuilt-bordie-drum\tools")
from control_drive_diagnostics import NT4, OUT, DASH, MODULE, TUNING, ControlError

parser = argparse.ArgumentParser()
parser.add_argument("--turn-kp", type=float)
args = parser.parse_args()
if args.turn_kp is not None and not (math.isfinite(args.turn_kp) and 0 <= args.turn_kp <= 1):
    raise ControlError("P must be finite and between zero and one")

writer = NT4("10.18.99.2")
observer = NT4("10.18.99.2")
try:
    deadline = time.monotonic() + 50
    while time.monotonic() < deadline:
        writer.pump()
        observer.pump()
        state = observer.fresh_robot()
        writer_state = writer.fresh_robot()
        if (not state["enabled"] and state["test"] and state["attached"] and not state["estop"]
                and not writer_state["enabled"] and writer_state["attached"]):
            break
    else:
        raise ControlError("Still waiting for disabled Test mode; nothing configured")
    values = {DASH + "RemoteActivation": False, DASH + "Stage": "NONE",
              DASH + "Prepare": True, DASH + "RemoteAllowed": True,
              DASH + "Module": 0.0}
    if args.turn_kp is not None:
        values[TUNING + "TurnKp"] = args.turn_kp
        values[TUNING + "TurnKd"] = 0.0
    for name, value in values.items():
        writer.require_disabled()
        writer.publish(name, value)
    def applied():
        return (all(observer.get(k) == v for k, v in values.items())
                and observer.get(OUT + "RemoteAllowedLatched") is True
                and all(observer.get(MODULE + str(i) + "/DiagnosticLimitsApplied") is True
                        and observer.get(MODULE + str(i) + "/ConfigurationHealthy") is True
                        and (args.turn_kp is None or
                             abs(observer.get(MODULE + str(i) + "/AppliedTurnKp", math.inf)
                                 - args.turn_kp) < 1e-5)
                        for i in range(4)))
    observer.wait(applied, seconds=12, guard=observer.require_disabled)
    result = {"prepared": True, "ds": observer.fresh_robot(),
              "ready": observer.get(OUT + "Ready"),
              "healthFault": observer.get(OUT + "HealthFault"),
              "loopMedianMs": observer.get(OUT + "LoopMedianMs"),
              "loopP95Ms": observer.get(OUT + "LoopP95Ms"),
              "gains": [{"module": i,
                         "P": observer.get(MODULE + str(i) + "/AppliedTurnKp"),
                         "D": observer.get(MODULE + str(i) + "/AppliedTurnKd"),
                         "limitsApplied": observer.get(MODULE + str(i) + "/DiagnosticLimitsApplied")}
                        for i in range(4)]}
    print(json.dumps(result, indent=2))
finally:
    observer.close()
    writer.close()
