"""No-network regressions for post-run observer catch-up after the recorder joins."""
import unittest
from unittest.mock import patch

from control_drive_diagnostics import ControlError
from control_intake_diagnostics import OUT, IntakeNT4, wait_for_post_run_stopped


class PostRunCatchUpTest(unittest.TestCase):
    def setUp(self):
        self.clock = 100.0
        self.client = IntakeNT4.__new__(IntakeNT4)  # No socket, subscription or command publisher.
        self.client.values = {}
        self.pumps = 0
        self.trace = [0.0] * 12
        self.trace[9], self.trace[11] = 100.0, 7
        self.client.values.update({
            OUT + "LastRunNonce": (0, 7, 98.0),
            OUT + "Active": (0, False, 98.0),
            OUT + "LastRunTimestampSec": (0, 99.0, 98.0),
        })
        self.set_topic("DSState", [100.0, 1, 1, 1, 0], received=98.0)
        self.set_topic("Trace", self.trace, received=98.0)

    def set_topic(self, key, value, received=None):
        self.client.values[OUT + key] = (0, value, self.clock if received is None else received)

    def tick(self):
        self.clock += .05
        self.pumps += 1

    def wait(self, seconds=.5):
        with patch("control_intake_diagnostics.time.monotonic", side_effect=lambda: self.clock):
            wait_for_post_run_stopped(self.client, 7, seconds=seconds)

    def test_stale_and_independently_arriving_topics_are_drained_before_confirmation(self):
        def pump():
            self.tick()
            if self.pumps == 2:
                self.set_topic("DSState", [100.0, 1, 1, 1, 0])
            elif self.pumps == 3:
                old_sensor = list(self.trace)
                old_sensor[9] = 98.0  # Fresh receipt still contains an incoherent sensor timestamp.
                self.set_topic("Trace", old_sensor)
            elif self.pumps == 4:
                self.set_topic("Trace", self.trace)
        self.client.pump = pump
        self.wait()
        self.assertEqual(4, self.pumps)

    def test_permanently_stale_data_times_out_without_retrying_motion(self):
        self.client.pump = self.tick
        with self.assertRaisesRegex(ControlError, "fresh post-run stop evidence; no retry"):
            self.wait(seconds=.2)
        self.assertGreaterEqual(self.pumps, 4)
        self.assertNotIn("publishers", self.client.__dict__)

    def test_disconnected_transport_is_not_treated_as_pending_telemetry(self):
        def pump():
            self.tick()
            raise ControlError("NT4 connection closed")
        self.client.pump = pump
        with self.assertRaisesRegex(ControlError, "NT4 connection closed"):
            self.wait()
        self.assertEqual(1, self.pumps)

    def test_malformed_fresh_state_is_not_retried(self):
        def pump():
            self.tick()
            self.set_topic("DSState", [100.0, 1, 1, float("nan"), 0])
            self.set_topic("Trace", self.trace)
        self.client.pump = pump
        with self.assertRaisesRegex(ControlError, "Nonfinite DSState"):
            self.wait()
        self.assertEqual(1, self.pumps)

    def test_fresh_zero_from_another_nonce_cannot_confirm_this_run(self):
        def pump():
            self.tick()
            wrong_run = list(self.trace)
            wrong_run[11] = 6
            self.set_topic("DSState", [100.0, 1, 1, 1, 0])
            self.set_topic("Trace", wrong_run)
        self.client.pump = pump
        with self.assertRaisesRegex(ControlError, "fresh post-run stop evidence"):
            self.wait(seconds=.2)

    def test_matching_nonce_with_nonzero_voltage_cannot_report_stopped(self):
        def pump():
            self.tick()
            energized = list(self.trace)
            energized[6] = .02
            self.set_topic("DSState", [100.0, 1, 1, 1, 0])
            self.set_topic("Trace", energized)
        self.client.pump = pump
        with self.assertRaisesRegex(ControlError, "fresh post-run stop evidence"):
            self.wait(seconds=.2)


if __name__ == "__main__":
    unittest.main()
