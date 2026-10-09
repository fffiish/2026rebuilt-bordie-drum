"""Explicit NT4 drivetrain diagnostic controls. Never enables Driver Station.

Requires websocket-client and msgpack. Configuration is allowed only with fresh
disabled robot telemetry; movement requires already-enabled Test mode, verified
readiness, RemoteAllowed, a new nonce, and continuing heartbeats. Robot-side
deadlines/watchdogs remain responsible for stopping if this client disappears.
"""

import argparse
import json
import math
import sys
import time

import msgpack
import websocket


DASH = "/SmartDashboard/DriveDiagnostics/"
OUT = "/AdvantageKit/RealOutputs/DriveDiagnostics/"
MODULE = "/AdvantageKit/Drive/Module"
TUNING = "/Tuning/Drive/Module/"
STAGES = ("NONE", "FEEDBACK", "STEER_POSITIVE", "STEER_NEGATIVE", "DRIVE_OPEN", "DRIVE_CLOSED")
MAX_AGE = 0.25


class ControlError(RuntimeError):
    pass


class NT4:
    def __init__(self, host):
        self.ws = websocket.create_connection(
            f"ws://{host}:5810/nt/bordie-diagnostic-control",
            subprotocols=["networktables.first.wpi.edu"], timeout=2,
            http_proxy_host=None)
        self.ws.settimeout(0.01)
        self.topics = {}
        self.values = {}
        self.publishers = {}
        self.offset_us = None
        self.best_rtt_us = float("inf")
        self.last_sync = 0.0
        self.send_json("subscribe", {
            "subuid": 1,
            "topics": [DASH, OUT, MODULE, TUNING, "/AdvantageKit/DriverStation/"],
            "options": {"prefix": True, "periodic": 0.02, "all": True}})
        # NT4 timestamp 0 means weak setDefault, not a normal command. Synchronize
        # before publishing so existing robot/dashboard values can be overridden.
        for _ in range(3):
            self.sync_request()
            deadline = time.monotonic() + 0.25
            while time.monotonic() < deadline:
                self.pump()
        if self.offset_us is None or self.best_rtt_us > 100_000:
            self.close()
            raise ControlError("NT4 timestamp synchronization unavailable or RTT exceeds 100 ms")

    def send_json(self, method, params):
        self.ws.send(json.dumps([{"method": method, "params": params}]))

    def sync_request(self):
        client_us = time.monotonic_ns() // 1000
        self.ws.send_binary(msgpack.packb([-1, 0, 2, client_us]))
        self.last_sync = time.monotonic()

    def server_us(self):
        return int(time.monotonic_ns() // 1000 + self.offset_us)

    def pump(self):
        if time.monotonic() - self.last_sync > 1:
            self.sync_request()
        try:
            data = self.ws.recv()
        except websocket.WebSocketTimeoutException:
            return
        if data == "" or data == b"":
            raise ControlError("NT4 connection closed")
        if isinstance(data, str):
            for message in json.loads(data):
                params = message.get("params", {})
                if message.get("method") == "announce":
                    self.topics[params["id"]] = params
                elif message.get("method") == "unannounce":
                    old = self.topics.pop(params["id"], None)
                    if old:
                        self.values.pop(old["name"], None)
            return
        unpacker = msgpack.Unpacker(raw=False)
        unpacker.feed(data)
        for topic_id, server_us, _, value in unpacker:
            if topic_id == -1:
                now_us = time.monotonic_ns() // 1000
                rtt_us = now_us - value
                if 0 <= rtt_us < self.best_rtt_us:
                    self.best_rtt_us = rtt_us
                    self.offset_us = server_us + rtt_us / 2 - now_us
            elif topic_id in self.topics:
                name = self.topics[topic_id]["name"]
                old = self.values.get(name)
                if old is None or server_us >= old[0]:
                    self.values[name] = (server_us, value, time.monotonic())

    def get(self, name, default=None):
        entry = self.values.get(name)
        return default if entry is None else entry[1]

    def publish(self, name, value):
        if type(value) is bool:
            kind, code = "boolean", 0
        elif isinstance(value, (int, float)):
            if not math.isfinite(value):
                raise ControlError("Refusing nonfinite published value")
            kind, code, value = "double", 1, float(value)
        elif isinstance(value, str):
            kind, code = "string", 4
        else:
            raise ControlError("Unsupported command type")
        if name not in self.publishers:
            uid = len(self.publishers) + 100
            self.publishers[name] = uid
            self.send_json("publish", {"name": name, "pubuid": uid,
                                       "type": kind, "properties": {}})
        self.ws.send_binary(msgpack.packb([
            self.publishers[name], self.server_us(), code, value]))

    def fresh_robot(self):
        entry = self.values.get(OUT + "DSState")
        if entry is None or not isinstance(entry[1], (list, tuple)) or len(entry[1]) != 5:
            raise ControlError("Fresh timestamped DSState is unavailable; deploy diagnostic code first")
        if time.monotonic() - entry[2] > MAX_AGE:
            raise ControlError("Robot/Driver Station telemetry is stale")
        state = entry[1]
        if not all(isinstance(x, (int, float)) and math.isfinite(x) for x in state):
            raise ControlError("Invalid DSState values")
        # NT4 timestamps may drift from the host monotonic clock across a roboRIO
        # power cycle. Compare the robot FPGA timestamps directly, then use local
        # receipt age to prove this pair is still arriving over Ethernet.
        module_snapshot = self.values.get(MODULE + "0/SnapshotTimestampSec")
        if module_snapshot is None or time.monotonic() - module_snapshot[2] > MAX_AGE:
            raise ControlError("Fresh module snapshot is unavailable")
        snapshot_age = state[0] - module_snapshot[1]
        if snapshot_age < -0.05 or snapshot_age > MAX_AGE:
            raise ControlError("Robot DS and module snapshots do not agree")
        return {"enabled": state[1] == 1, "test": state[2] == 1,
                "attached": state[3] == 1, "estop": state[4] == 1}

    def require_disabled(self):
        ds = self.fresh_robot()
        if ds["enabled"] or not ds["attached"]:
            raise ControlError("Configuration requires attached Driver Station with robot disabled")

    def require_test(self):
        ds = self.fresh_robot()
        if not ds["enabled"] or not ds["test"] or not ds["attached"] or ds["estop"]:
            raise ControlError("Movement requires attached Driver Station already enabled in Test mode")

    def wait(self, predicate, seconds=5, guard=None):
        deadline = time.monotonic() + seconds
        while time.monotonic() < deadline:
            self.pump()
            if OUT + "DSState" not in self.values:
                continue
            if guard:
                guard()
            if predicate():
                return
        raise ControlError("Timed out waiting for robot acknowledgement")

    def close(self):
        self.ws.close()


def selection_values(args):
    values = {}
    for arg, name in (("stage", "Stage"), ("module", "Module"), ("magnitude", "Magnitude")):
        value = getattr(args, arg, None)
        if value is not None:
            values[DASH + name] = value
    return values


def acknowledge(client, values):
    return all(client.get(name) == value for name, value in values.items())


def configure(client, args):
    client.wait(lambda: True, guard=client.require_disabled)
    values = selection_values(args)
    for arg, topic in (("prepare", "Prepare"), ("remote_allowed", "RemoteAllowed")):
        value = getattr(args, arg)
        if value is not None:
            values[DASH + topic] = value
    gain_fields = (("drive_kp", "DriveKp", "AppliedDriveKp"),
                   ("drive_kv", "DriveKv", "AppliedDriveKv"),
                   ("turn_kp", "TurnKp", "AppliedTurnKp"),
                   ("turn_kd", "TurnKd", "AppliedTurnKd"))
    for arg, key, _ in gain_fields:
        value = getattr(args, arg)
        if value is not None:
            if not math.isfinite(value) or value < 0 or value > (2 if arg == "drive_kv" else 1):
                raise ControlError(f"Invalid {arg} gain")
            values[TUNING + key] = value
    client.publish(DASH + "RemoteActivation", False)
    for name, value in values.items():
        client.require_disabled()
        client.publish(name, value)
    client.wait(lambda: acknowledge(client, values), guard=client.require_disabled)

    def gains_applied():
        for index in range(4):
            prefix = MODULE + str(index) + "/"
            if client.get(prefix + "ConfigurationHealthy") is not True:
                return False
            snapshot_topic = client.values.get(prefix + "SnapshotTimestampSec")
            if snapshot_topic is None or time.monotonic() - snapshot_topic[2] > MAX_AGE:
                return False
            for arg, _, field in gain_fields:
                expected = getattr(args, arg)
                if expected is not None:
                    actual = client.get(prefix + field)
                    if not isinstance(actual, (int, float)) or not math.isfinite(actual) or abs(actual - expected) > 1e-5:
                        return False
        return True

    client.wait(gains_applied, seconds=10, guard=client.require_disabled)
    print(json.dumps({"configured": values, "configurationHealthy": True,
                      "driverStationDisabled": True}, indent=2))


def run(client, args):
    if args.stage == "NONE":
        raise ControlError("Choose a movement stage")
    if args.stage.startswith("STEER") or args.stage == "FEEDBACK":
        if args.module == 4:
            raise ControlError("Steering requires one module 0..3")
    if args.stage == "DRIVE_OPEN" and args.magnitude not in (0.5, 1, 1.5):
        raise ControlError("Open-loop voltage must be 0.5, 1.0, or 1.5")
    if args.stage == "DRIVE_CLOSED" and (args.module != 4 or args.magnitude not in (0.25, 0.5)):
        raise ControlError("Closed-loop comparison requires module4 and speed0.25 or0.5")
    client.wait(lambda: True, guard=client.require_test)
    if client.get(DASH + "RemoteAllowed") is not True or client.get(DASH + "Prepare") is not True:
        raise ControlError("Prepare and RemoteAllowed must first be configured while disabled")
    if client.get(OUT + "Active") is not False or client.get(OUT + "Ready") is not True:
        raise ControlError("Diagnostic must be inactive and ready before starting")
    values = selection_values(args)
    # NT4 does not echo a client's own publications to that same connection.
    # Read selection acknowledgements through an independent subscriber before
    # issuing a movement nonce, while continuing to refresh the writer's DS guard.
    selection_observer = NT4(args.host)
    try:
        client.publish(DASH + "RemoteActivation", False)
        for name, value in values.items():
            client.publish(name, value)

        def writer_guard():
            client.pump()
            client.require_test()

        selection_observer.wait(lambda: acknowledge(selection_observer, values),
                                guard=writer_guard)
    finally:
        selection_observer.close()
    baseline = client.get(OUT + "TimestampSec", -math.inf)
    client.wait(lambda: client.get(OUT + "TimestampSec", -math.inf) > baseline,
                guard=client.require_test)
    client.wait(lambda: client.get(DASH + "StartNonce", 0)
                <= client.get(OUT + "LastConsumedNonce", 0), guard=client.require_test)
    if client.get(OUT + "Active") is not False or client.get(OUT + "Ready") is not True:
        raise ControlError("Readiness lost before activation")
    nonce = max(int(time.time_ns() // 1_000_000),
                int(client.get(DASH + "StartNonce", 0)) + 1,
                int(client.get(OUT + "LastConsumedNonce", 0)) + 1)
    heartbeat = max(nonce, int(client.get(DASH + "RemoteHeartbeat", 0))) + 1
    started = time.monotonic()
    last_heartbeat = -math.inf
    seen_active = False
    try:
        client.publish(DASH + "RemoteHeartbeat", heartbeat)
        client.publish(DASH + "RemoteActivation", True)
        # The robot consumes new nonces even when ineligible. Set activation and
        # heartbeat first; only the final new nonce can request movement.
        client.publish(DASH + "StartNonce", nonce)
        while time.monotonic() - started < 14:
            client.pump()
            client.require_test()
            now = time.monotonic()
            if now - last_heartbeat >= 0.05:
                heartbeat += 1
                client.publish(DASH + "RemoteHeartbeat", heartbeat)
                last_heartbeat = now
            active = client.get(OUT + "Active") is True
            seen_active |= active
            result_nonce = client.get(OUT + "LastRunNonce")
            if result_nonce == nonce:
                result = {"nonce": nonce, "stage": args.stage, "module": args.module,
                          "passed": client.get(OUT + "LastRunPassed"),
                          "reason": client.get(OUT + "LastRunReason"),
                          "status": client.get(OUT + "Status"),
                          "watchdog": client.get(OUT + "WatchdogReason"),
                          "durationSec": now - started}
                print(json.dumps(result, indent=2))
                if result["passed"] is not True:
                    raise ControlError("Robot stopped or rejected the diagnostic: " + str(result["reason"]))
                return
            if not seen_active and now - started > 2:
                raise ControlError("Robot did not acknowledge/start diagnostic: " + str(client.get(OUT + "Status")))
        raise ControlError("Session exceeded14-second hard timeout")
    finally:
        # Best effort on network failure; independent robot watchdog must stop it.
        try:
            client.publish(DASH + "RemoteActivation", False)
        except Exception:
            pass


def stop(client, _):
    baseline = client.get(OUT + "TimestampSec", -math.inf)
    client.publish(DASH + "RemoteActivation", False)
    client.publish(DASH + "Stage", "NONE")
    client.wait(lambda: client.get(OUT + "Active") is False
                and client.get(DASH + "RemoteActivation") is False
                and client.get(OUT + "TimestampSec", -math.inf) > baseline, seconds=2)
    print("Remote activation cleared; robot reports diagnostic inactive.")


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--host", default="10.18.99.2")
    commands = parser.add_subparsers(dest="command", required=True)
    config = commands.add_parser("configure", help="Write selections/gains only while disabled")
    movement = commands.add_parser("run", help="Run one bounded stage in already-enabled Test mode")
    commands.add_parser("stop", help="Clear activation and cancel stage in any Driver Station mode")
    for sub in (config, movement):
        sub.add_argument("--stage", choices=STAGES, required=sub is movement)
        sub.add_argument("--module", type=int, choices=range(5), default=0 if sub is movement else None)
        sub.add_argument("--magnitude", type=float, default=0.5 if sub is movement else None)
    config.add_argument("--prepare", action=argparse.BooleanOptionalAction, default=None)
    config.add_argument("--remote-allowed", action=argparse.BooleanOptionalAction, default=None)
    for gain in ("drive-kp", "drive-kv", "turn-kp", "turn-kd"):
        config.add_argument("--" + gain, type=float)
    args = parser.parse_args()
    client = None
    try:
        client = NT4(args.host)
        {"configure": configure, "run": run, "stop": stop}[args.command](client, args)
    except (ControlError, OSError, websocket.WebSocketException, ValueError) as exc:
        print("Stopped:", exc, file=sys.stderr)
        return 1
    finally:
        if client:
            client.close()
    return 0


if __name__ == "__main__":
    sys.exit(main())
