"""No-network checks for stale/result attribution guards in the pivot control client."""
import copy
import unittest

from control_intake_diagnostics import OUT, IntakeNT4, post_run_stopped, recorded_run_stopped
from control_drive_diagnostics import ControlError


class Observer:
    def __init__(self, stale=False):
        self.stale = stale
        trace = [0.0] * 12
        trace[9], trace[11] = 11.0, 7
        self.data = {OUT + "Trace": trace, OUT + "LastRunTimestampSec": 10.0,
                     OUT + "LastRunNonce": 7, OUT + "Active": False}

    def get(self, key, default=None):
        return self.data.get(key, default)

    def fresh_robot(self):
        if self.stale:
            raise ControlError("stale")
        return {"enabled": True, "test": True, "attached": True, "estop": False}


class IntakeControlGuardTest(unittest.TestCase):
    def test_stale_zero_sample_cannot_report_stopped(self):
        with self.assertRaises(ControlError):
            post_run_stopped(Observer(stale=True), 7)

    def test_sample_before_finish_cannot_report_stopped(self):
        observer = Observer()
        observer.data[OUT + "Trace"][9] = 9.0
        self.assertFalse(post_run_stopped(observer, 7))

    def test_previous_nonce_cannot_report_new_run_stopped(self):
        self.assertFalse(post_run_stopped(Observer(), 8))

    def test_trace_nonce_voltage_and_timestamp_must_match(self):
        observer = Observer()
        self.assertTrue(post_run_stopped(observer, 7))
        for index, value in ((11, 6), (6, .02), (6, float("nan")), (9, float("nan"))):
            changed = copy.deepcopy(observer)
            changed.data[OUT + "Trace"][index] = value
            self.assertFalse(post_run_stopped(changed, 7))

    def test_recording_requires_post_finish_trace_and_same_run(self):
        data = {"latest": Observer().data}
        self.assertTrue(recorded_run_stopped(data, 7))
        self.assertFalse(recorded_run_stopped(data, 8))
        data["latest"][OUT + "Trace"][9] = 9.0
        self.assertFalse(recorded_run_stopped(data, 7))

    def test_fresh_ds_cannot_bless_an_old_sensor_snapshot(self):
        client = IntakeNT4.__new__(IntakeNT4)
        import time
        received = time.monotonic()
        client.values = {OUT + "DSState": (0, [100, 1, 1, 1, 0], received),
                         OUT + "Trace": (0, [0] * 12, received)}
        with self.assertRaises(ControlError):
            client.fresh_robot()


if __name__ == "__main__":
    unittest.main()
