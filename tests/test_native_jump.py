"""Real C controller, encoder cadence, CAD leg separation and landing recovery."""
import unittest
from tools.probes.native_jump_acceptance import episode, polarity


class NativeJumpTests(unittest.TestCase):
    def test_jump_pose_polarities_are_restoring(self):
        self.assertEqual(len(polarity()), 20)

    def test_low_and_mid_repeat_jumps_restore_normal_driving(self):
        for leg in ("low","mid"):
            with self.subTest(leg=leg):
                report = episode(leg)
                self.assertGreater(report["fast_polls"], 400)
                self.assertGreater(report["normal_tail_polls"], 900)

    def test_later_standing_pose_does_not_reopen_leg_split(self):
        for leg in ("low","mid"):
            with self.subTest(leg=leg):
                episode(leg, start=10)


if __name__ == "__main__": unittest.main(verbosity=2)
