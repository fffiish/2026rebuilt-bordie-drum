import json
from pathlib import Path
import subprocess
import sys
import tempfile
import unittest


class IntakeSummaryTest(unittest.TestCase):
    script = Path(__file__).resolve().parents[1] / "diagnostics" / "summarize_intake_test.py"
    prefix = "/AdvantageKit/"

    def summarize(self, states):
        history = dict(states)
        for suffix, value in {
            "AppliedVolts": 4.0,
            "StatorCurrent": 12.0,
            "BusVolts": 12.4,
            "Angle": 0.0,
        }.items():
            history[self.prefix + "AngularSubsystems/IntakePivot/" + suffix] = [
                [0, 0, value],
                [0, 1500000, value],
            ]
        capture = {"capture_utc": "2026-10-09T00:00:00Z", "duration_sec": 3, "history": history}
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "capture.json"
            path.write_text(json.dumps(capture), encoding="utf-8")
            result = subprocess.run(
                [sys.executable, "-B", str(self.script), str(path)],
                check=True,
                capture_output=True,
                text=True,
            )
        return json.loads(result.stdout)

    def assert_single_hold(self, result):
        self.assertEqual(len(result["holds"]), 1)
        self.assertEqual(result["holds"][0]["duration_sec"], 1.0)
        self.assertEqual(result["holds"][0]["AppliedVolts"]["median"], 4.0)

    def test_legacy_target_state_captures_still_summarize(self):
        result = self.summarize({
            self.prefix + "RealOutputs/Intake/TargetState": [
                [0, 1000000, "kIntaking"], [0, 2000000, "kStowed"]
            ]
        })
        self.assert_single_hold(result)

    def test_split_roller_state_captures_summarize(self):
        result = self.summarize({
            self.prefix + "RealOutputs/Intake/TargetState": [
                [0, 1000000, "kDeployed"], [0, 2000000, "kRaised"]
            ],
            self.prefix + "RealOutputs/Intake/TargetRollerState": [
                [0, 1000000, "kIntaking"], [0, 2000000, "kOff"]
            ],
        })
        self.assert_single_hold(result)

    def test_new_roller_state_takes_precedence_over_legacy_state(self):
        result = self.summarize({
            self.prefix + "RealOutputs/Intake/TargetState": [
                [0, 1000000, "kIntaking"], [0, 2000000, "kStowed"]
            ],
            self.prefix + "RealOutputs/Intake/TargetRollerState": [
                [0, 1000000, "kOff"], [0, 2000000, "kOff"]
            ],
        })
        self.assertEqual(result["holds"], [])


if __name__ == "__main__":
    unittest.main()
