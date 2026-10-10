"""Explicit bounded pivot diagnostics. Never enables Driver Station; no movement by default.

Configuration/reference changes require fresh disabled telemetry. A run requires
already-enabled Test, robot-side readiness and disabled-latched remote permission.
Run failures never trigger an automatic retry. Uses websocket-client and msgpack.
"""

import argparse
import json
import math
from pathlib import Path
import threading
import time

from control_drive_diagnostics import ControlError, MAX_AGE, NT4
from capture_intake_diagnostics import PREFIXES, capture


DASH = "/SmartDashboard/IntakeDiagnostics/"
OUT = "/AdvantageKit/RealOutputs/IntakeDiagnostics/"
TUNING = "/Tuning/AngularSubsystems/IntakePivot/"


class TelemetryPending(ControlError):
    """A fresh coherent DS/sensor pair has not arrived yet; no command is authorized."""


class IntakeNT4(NT4):
    def __init__(self, host):
        super().__init__(host)
        self.send_json("subscribe", {"subuid": 2, "topics": PREFIXES,
                                    "options": {"prefix": True, "periodic": .02, "all": True}})

    def fresh_robot(self):
        ds = self.values.get(OUT + "DSState")
        trace = self.values.get(OUT + "Trace")
        if ds is None or trace is None:
            raise TelemetryPending("Deploy intake diagnostic code before issuing commands")
        if time.monotonic() - ds[2] > MAX_AGE or time.monotonic() - trace[2] > MAX_AGE:
            raise TelemetryPending("Intake/Driver Station telemetry is stale")
        state, snapshot = ds[1], trace[1]
        if not isinstance(state, (list, tuple)) or len(state) != 5:
            raise ControlError("Invalid intake DSState")
        if not isinstance(snapshot, (list, tuple)) or len(snapshot) < 10:
            raise ControlError("Invalid intake snapshot")
        if not all(isinstance(v, (int, float)) and math.isfinite(v) for v in state):
            raise ControlError("Nonfinite DSState")
        if not isinstance(snapshot[9], (int, float)) or not math.isfinite(snapshot[9]):
            raise ControlError("Invalid sample timestamp")
        age = state[0] - snapshot[9]
        if not -.05 <= age <= MAX_AGE:
            raise TelemetryPending("Intake snapshot and DSState do not agree")
        return {"enabled": state[1] == 1, "test": state[2] == 1,
                "attached": state[3] == 1, "estop": state[4] == 1}

    def wait(self, predicate, seconds=5, guard=None):
        deadline = time.monotonic() + seconds
        while time.monotonic() < deadline:
            self.pump()
            if OUT + "DSState" not in self.values or OUT + "Trace" not in self.values:
                continue
            if guard:
                try:
                    guard()
                except ControlError:
                    # NT topics arrive independently. During a disabled write,
                    # wait for a coherent sensor/DS pair; never issue a command
                    # from an incoherent pair or ignore a fresh enabled/lost-DS state.
                    entry = self.values.get(OUT + "DSState")
                    if entry is not None and time.monotonic() - entry[2] <= MAX_AGE:
                        state = entry[1]
                        if isinstance(state, (list, tuple)) and len(state) == 5:
                            if state[1] == 1 or state[3] != 1:
                                raise
                    continue
            if predicate():
                return
        raise ControlError("Timed out waiting for intake acknowledgement")


def finite_number(value):
    value = float(value)
    if not math.isfinite(value):
        raise argparse.ArgumentTypeError("Value must be finite")
    return value


def boolean_value(value):
    if value not in ("true", "false"):
        raise argparse.ArgumentTypeError("Use true or false")
    return value == "true"


def next_nonce(*values):
    valid = [v for v in values if isinstance(v, (int, float)) and math.isfinite(v)]
    result = max([0] + valid) + 1
    if result != math.floor(result) or result > 9_007_199_254_740_991:
        raise ControlError("Cannot allocate a safe increasing request nonce")
    return result


def post_run_stopped(observer, nonce):
    observer.fresh_robot()
    trace = observer.get(OUT + "Trace")
    finished_at = observer.get(OUT + "LastRunTimestampSec")
    return (observer.get(OUT + "LastRunNonce") == nonce
            and observer.get(OUT + "Active") is False
            and isinstance(finished_at, (int, float)) and math.isfinite(finished_at)
            and isinstance(trace, (list, tuple)) and len(trace) >= 12
            and math.isfinite(trace[9]) and trace[9] >= finished_at and trace[11] == nonce
            and math.isfinite(trace[6]) and abs(trace[6]) < .01)


def wait_for_post_run_stopped(observer, nonce, seconds=3):
    """Drain queued telemetry after recorder.join without issuing or retrying a motor request."""
    deadline = time.monotonic() + seconds
    while time.monotonic() < deadline:
        # Transport errors must propagate; only an incomplete telemetry pair is transient.
        observer.pump()
        try:
            if post_run_stopped(observer, nonce):
                return
        except TelemetryPending:
            # NT topics arrive independently, and this connection was not pumped while the
            # separate recorder finished. Require a new coherent pair before accepting zero.
            continue
    raise ControlError("Timed out waiting for fresh post-run stop evidence; no retry will be issued")


def recorded_run_stopped(recorded, nonce):
    latest = recorded["latest"]
    trace = latest.get(OUT + "Trace", [])
    finished_at = latest.get(OUT + "LastRunTimestampSec")
    return (latest.get(OUT + "LastRunNonce") == nonce
            and latest.get(OUT + "Active") is False and len(trace) >= 12
            and trace[11] == nonce and math.isfinite(trace[6]) and abs(trace[6]) < .01
            and isinstance(finished_at, (int, float)) and math.isfinite(finished_at)
            and math.isfinite(trace[9]) and trace[9] >= finished_at)


def configure(writer, observer, args):
    writer.wait(lambda: True, guard=writer.require_disabled)
    observer.wait(lambda: True, guard=observer.require_disabled)
    if args.stop_requested is False:
        # Establish the explicit disabled true->false edge even if a dashboard
        # tried to clear the abort while enabled (that edit is intentionally ignored).
        writer.publish(DASH + "StopRequested", True)
        observer.wait(lambda: observer.get(OUT + "StopRequestedLatched") is True,
                      seconds=3, guard=observer.require_disabled)
    values = {DASH + "RemoteActivation": False}
    names = {
        "prepare": "Prepare", "allow_remote": "AllowRemote", "mode": "Mode",
        "stop_requested": "StopRequested",
        "reference_degrees": "ReferenceDegrees", "minimum_degrees": "MinimumDegrees",
        "maximum_degrees": "MaximumDegrees", "reference_confirmed": "ReferenceConfirmed",
        "scaling_and_travel_confirmed": "ScalingAndTravelConfirmed",
        "direction_confirmed": "DirectionConfirmed", "voltage": "Voltage",
        "step_degrees": "StepDegrees", "duration_seconds": "DurationSeconds",
        "output_duty": "OutputDuty", "current_limit_amps": "CurrentLimitAmps",
        "temperature_limit_celsius": "TemperatureLimitCelsius",
    }
    for arg, key in names.items():
        value = getattr(args, arg, None)
        if value is not None:
            values[DASH + key] = value
    gain_names = {"kp": "KP", "ki": "KI", "kd": "KD", "ks": "KS", "kv": "KV", "kg": "KG",
                  "cruise_rad_per_sec": "CruiseVelocityRadiansPerSecond",
                  "accel_rad_per_sec2": "AccelerationRadiansPerSecondPerSecond"}
    for arg, key in gain_names.items():
        value = getattr(args, arg, None)
        if value is not None:
            if value < 0:
                raise ControlError("Negative gains/profile constraints are not accepted")
            values[TUNING + key] = value
    for name, value in values.items():
        writer.wait(lambda: True, seconds=3, guard=writer.require_disabled)
        writer.publish(name, value)
        writer.pump()
        observer.pump()
    # Publications are acknowledged on another connection, never by a local echo.
    deadline = time.monotonic() + 8
    while time.monotonic() < deadline:
        writer.pump(); observer.pump()
        writer.wait(lambda: True, seconds=3, guard=writer.require_disabled)
        observer.wait(lambda: True, seconds=3, guard=observer.require_disabled)
        if all(observer.get(name) == value for name, value in values.items()):
            break
    else:
        raise ControlError("Settings were not acknowledged; no activation requested")
    expected = [(getattr(args, k, None), index) for index, k in enumerate(
        ("kp", "ki", "kd", "ks", "kv", "kg", "cruise_rad_per_sec", "accel_rad_per_sec2", "output_duty"))]
    def settings_applied(include_output):
        actual = observer.get(OUT + "ConfigReadback")
        if observer.get(OUT + "ConfigurationReady") is not True:
            return False
        if not isinstance(actual, (list, tuple)) or len(actual) != 9:
            return False
        return all(value is None or index == 8 and not include_output
                   or math.isfinite(actual[index]) and abs(actual[index] - value) <= 1e-5
                   for value, index in expected)
    # Reference requests cannot be consumed while the controller is still
    # applying gains/profiles. Verify those first; the cap follows calibration.
    observer.wait(lambda: settings_applied(False), seconds=10, guard=observer.require_disabled)
    if args.calibrate:
        nonce = next_nonce(observer.get(OUT + "LastCalibrationNonce"),
                           observer.get(DASH + "CalibrationNonce"))
        writer.require_disabled()
        writer.publish(DASH + "CalibrationNonce", nonce)
        observer.wait(lambda: observer.get(OUT + "LastCalibrationNonce") == nonce
                      and observer.get(OUT + "Calibrated") is True,
                      seconds=8, guard=observer.require_disabled)
    if args.confirm_direction:
        nonce = next_nonce(observer.get(OUT + "LastDirectionConfirmationNonce"),
                           observer.get(DASH + "DirectionConfirmationNonce"))
        writer.require_disabled()
        writer.publish(DASH + "DirectionConfirmed", True)
        writer.publish(DASH + "DirectionConfirmationNonce", nonce)
        observer.wait(lambda: observer.get(OUT + "DirectionConfirmedLatched") is True
                      and observer.get(OUT + "LastDirectionConfirmationNonce") == nonce,
                      seconds=5, guard=observer.require_disabled)
    observer.wait(lambda: settings_applied(True),
                  seconds=10, guard=observer.require_disabled)
    actual = observer.get(OUT + "ConfigReadback")
    if not isinstance(actual, (list, tuple)) or len(actual) != 9:
        raise ControlError("Controller configuration readback unavailable")
    for value, index in expected:
        if value is not None and (not math.isfinite(actual[index]) or abs(actual[index] - value) > 1e-5):
            raise ControlError("Controller readback differs from requested value")
    print(json.dumps({"configured": values, "calibrated": observer.get(OUT + "Calibrated"),
                      "readback": actual, "driverStationDisabled": True}, indent=2))


def run(writer, observer, args):
    writer.wait(lambda: True, guard=writer.require_test)
    observer.wait(lambda: observer.get(OUT + "Ready") is True,
                  seconds=5, guard=observer.require_test)
    if observer.get(OUT + "Active") is not False or observer.get(OUT + "RemoteAllowedLatched") is not True:
        raise ControlError("Requires inactive diagnostics and remote permission prepared while disabled")
    if observer.get(DASH + "Mode") not in ("FEEDBACK", "POSITION"):
        raise ControlError("Choose a bounded mode while disabled first")
    nonce = next_nonce(observer.get(OUT + "LastConsumedNonce"), observer.get(DASH + "StartNonce"))
    heartbeat = next_nonce(observer.get(DASH + "RemoteHeartbeat"))
    duration = observer.get(DASH + "DurationSeconds")
    if not isinstance(duration, (int, float)) or not math.isfinite(duration) or not 0 < duration <= 1:
        raise ControlError("Unbounded/invalid duration")
    errors = []
    def record():
        try:
            capture(args.host, duration + 3, args.output)
        except Exception as exc:
            errors.append(str(exc))
    recorder = threading.Thread(target=record)
    recorder.start()
    try:
        # Establish recording and both independent observers before any request.
        until = time.monotonic() + .6
        while time.monotonic() < until:
            writer.pump(); observer.pump(); writer.require_test()
        if errors:
            raise ControlError("Recording failed: " + errors[0])
        writer.publish(DASH + "RemoteHeartbeat", heartbeat)
        writer.publish(DASH + "RemoteActivation", True)
        until = time.monotonic() + .08
        while time.monotonic() < until:
            writer.pump(); observer.pump(); writer.require_test()
        writer.publish(DASH + "StartNonce", nonce)
        deadline = time.monotonic() + duration + 1
        last_beat = 0
        while time.monotonic() < deadline:
            writer.pump(); observer.pump(); writer.require_test()
            now = time.monotonic()
            if now - last_beat >= .04:
                heartbeat += 1
                writer.publish(DASH + "RemoteHeartbeat", heartbeat)
                last_beat = now
            if observer.get(OUT + "LastRunNonce") == nonce and observer.get(OUT + "Active") is False:
                break
        else:
            raise ControlError("Run result unavailable; no retry will be issued")
    finally:
        writer.publish(DASH + "RemoteActivation", False)
        recorder.join()
    # A prior zero or inactive flag is insufficient: require a fresh post-result
    # sensor sample and compare the matching nonce in the independent recording.
    wait_for_post_run_stopped(observer, nonce, seconds=3)
    if not args.output.exists():
        raise ControlError("Independent recording was not saved")
    recorded = json.loads(args.output.read_text(encoding="utf-8"))
    if not recorded_run_stopped(recorded, nonce):
        raise ControlError("Recording does not verify this run stopped; do not repeat movement")
    passed = observer.get(OUT + "LastRunPassed") is True
    print(json.dumps({"nonce": nonce, "passed": passed,
                      "reason": observer.get(OUT + "LastRunReason"),
                      "recording": str(args.output.resolve()), "captureErrors": errors}, indent=2))
    if errors or not passed:
        raise ControlError("Run failed or recording incomplete; inspect evidence before another run")


def stop(writer, observer):
    # Abort either physical or remote activation while retaining diagnostic
    # isolation through a disable/enable cycle. De-preparing is a separate action.
    baseline = observer.get(OUT + "DSState", [float("nan")])[0]
    writer.publish(DASH + "RemoteActivation", False)
    writer.publish(DASH + "StopRequested", True)
    if not isinstance(baseline, (int, float)) or not math.isfinite(baseline):
        observer.wait(lambda: True, seconds=3)
        baseline = observer.fresh_robot()
        baseline = observer.get(OUT + "DSState")[0]
    def stopped():
        observer.fresh_robot()
        trace = observer.get(OUT + "Trace")
        return (observer.get(OUT + "Active") is False
                and observer.get(OUT + "StopRequestedLatched") is True
                and observer.get(DASH + "RemoteActivation") is False
                and isinstance(trace, (list, tuple)) and len(trace) >= 10
                and trace[9] > baseline and abs(trace[6]) < .01)
    observer.wait(stopped, seconds=5)
    print(json.dumps({"stopped": True, "freshSampleVerified": True,
                      "isolationLatched": observer.get(OUT + "Selected")}, indent=2))


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--host", default="10.18.99.2")
    sub = parser.add_subparsers(dest="command", required=True)
    cfg = sub.add_parser("configure")
    for flag in ("prepare", "allow-remote", "reference-confirmed", "scaling-and-travel-confirmed",
                 "direction-confirmed", "stop-requested"):
        cfg.add_argument("--" + flag, type=boolean_value)
    cfg.add_argument("--mode", choices=["OFF", "FEEDBACK", "POSITION"])
    for flag in ("reference-degrees", "minimum-degrees", "maximum-degrees", "voltage", "step-degrees",
                 "duration-seconds", "output-duty", "current-limit-amps", "temperature-limit-celsius",
                 "kp", "ki", "kd", "ks", "kv", "kg", "cruise-rad-per-sec", "accel-rad-per-sec2"):
        cfg.add_argument("--" + flag, type=finite_number)
    cfg.add_argument("--calibrate", action="store_true")
    cfg.add_argument("--confirm-direction", action="store_true")
    execute = sub.add_parser("run")
    execute.add_argument("--output", type=Path, required=True)
    sub.add_parser("stop")
    args = parser.parse_args()
    writer = IntakeNT4(args.host)
    observer = None
    try:
        observer = IntakeNT4(args.host)
        if args.command == "stop":
            stop(writer, observer)
        elif args.command == "configure":
            configure(writer, observer, args)
        else:
            run(writer, observer, args)
    finally:
        if observer is not None:
            observer.close()
        writer.close()


if __name__ == "__main__":
    try:
        main()
    except (ControlError, OSError) as exc:
        raise SystemExit(str(exc))
