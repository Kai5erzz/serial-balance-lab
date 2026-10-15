import unittest
from tools.probes.jump_flight_trace import analyze_flight, analyze_jump_yaw


def complete_flight():
    rows = []
    for tick in range(0, 1001, 10):
        t = tick/1000
        phase = 0 if t < .05 or t >= .95 else 1 if t < .15 else 2 if t < .25 else 3 if t < .70 else 4
        length = .14 if .35 <= t <= .45 else .28
        rows.append({"t": t, "state": [phase, int(.25 <= t < .70), 0, 256 if .46 <= t < .70 else 0],
                     "length": [length, length], "ref": [.13, .13] if .25 <= t < .46 else [.27, .27],
                     "contact": [0., 0., 0.] if .25 <= t < .70 else [65., 65., 0.], "roll": 1.0})
    return rows


class JumpFlightTraceTests(unittest.TestCase):
    def test_physical_yaw_wrap_and_old_drift_are_distinguished(self):
        rows = complete_flight()
        for row in rows:
            row["actual_yaw"] = 179.0 if row["t"] < .25 else -178.0
        report = analyze_jump_yaw(rows)
        self.assertTrue(report["passed"], report)
        self.assertAlmostEqual(report["max_air_yaw_deg"], 3.0)
        for row in rows:
            if row["t"] >= .25: row["actual_yaw"] = -169.0
        report = analyze_jump_yaw(rows)
        self.assertFalse(report["passed"])
        self.assertAlmostEqual(report["max_air_yaw_deg"], 12.0)

    def test_missing_or_nonfinite_physical_yaw_cannot_pass(self):
        rows = complete_flight()
        self.assertFalse(analyze_jump_yaw(rows)["passed"])
        for row in rows: row["actual_yaw"] = 0.0
        rows[45]["actual_yaw"] = float("nan")
        self.assertFalse(analyze_jump_yaw(rows)["passed"])

    def test_real_compact_hold_then_deploy_then_contact(self):
        report = analyze_flight(complete_flight())
        self.assertTrue(report["passed"], report)
        self.assertGreaterEqual(report["longest_actual_compact_hold_s"], .080)

    def test_phase_labels_do_not_hide_missing_tuck(self):
        rows = complete_flight()
        for r in rows: r["length"] = [.31, .32]
        report = analyze_flight(rows)
        self.assertFalse(report["passed"])
        self.assertIn("both legs did not complete the measured airborne compact dwell", report["errors"])

    def test_one_leg_or_interrupted_compact_hold_is_insufficient(self):
        for index in (None, 39):
            rows = complete_flight()
            if index is None:
                for r in rows: r["length"][1] = .16
            else: rows[index]["length"][1] = .16
            self.assertIsNone(analyze_flight(rows)["compact_hold_completed_s"])

    def test_false_landing_and_sideways_takeoff_are_detected(self):
        rows = complete_flight()
        for r in rows:
            if .5 <= r["t"] < .7:
                r["state"][0] = 4
                r["roll"] = 8.
        report = analyze_flight(rows)
        self.assertFalse(report["passed"])
        self.assertIn("controller entered LANDING before physical two-wheel contact", report["errors"])
        self.assertEqual(report["max_air_roll_deg"], 8.)

    def test_deploy_bit_without_physical_extension_is_detected(self):
        rows = complete_flight()
        for r in rows:
            if .35 <= r["t"] < .70: r["length"] = [.14, .14]
        report = analyze_flight(rows)
        self.assertIn("both actual legs did not extend 20 mm before touchdown", report["errors"])

    def test_timer_deployment_without_dwell_is_detected(self):
        rows = complete_flight()
        for r in rows:
            if .40 <= r["t"] < .46: r["state"][3] = 256
        report = analyze_flight(rows)
        self.assertIn("deployment started before the measured compact dwell", report["errors"])


if __name__ == "__main__": unittest.main(verbosity=2)
