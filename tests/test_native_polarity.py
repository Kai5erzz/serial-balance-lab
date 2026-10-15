"""Physical direction and real C wire-path regressions for the vehicle adapter."""
from pathlib import Path
import sys
import tempfile
import unittest

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
from tools.probes.native_polarity_audit import (
    VmcDiagnostic, actuator_pulse, geometry_and_vmc, gimbal_closed_loop,
    hip_wire_channel, imu_rotation, live_hip_callbacks, longitudinal_acceleration,
    vmc_pulse, wheel_wire_channel,
)


class NativePolarityTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.directory = tempfile.TemporaryDirectory(prefix="native-polarity-test-")
        cls.diag = VmcDiagnostic(cls.directory.name)

    @classmethod
    def tearDownClass(cls):
        cls.directory.cleanup()

    def test_cad_axes_and_vmc_virtual_work_signs(self):
        for variant in ("fast", "full"):
            for pose in ("stand", "encoder"):
                with self.subTest(variant=variant, pose=pose):
                    result = geometry_and_vmc(self.diag, variant, pose)
                    self.assertAlmostEqual(result["sensor_frame_determinant"], 1)
                    self.assertTrue(all(w["matches"] for w in result["wheels"]), result)
                    self.assertTrue(all(l["jacobian_signs_match"] for l in result["legs"]), result)

    def test_dm_and_dji_feedback_keep_signed_channels(self):
        for index in range(4):
            for sign in (-1, 1):
                result = hip_wire_channel(index, sign)
                self.assertTrue(result["passed"], result)
        for index in range(2):
            for sign in (-1, 1):
                result = wheel_wire_channel(self.diag, index, sign)
                self.assertTrue(result["passed"], result)

    def test_original_callbacks_keep_vmc_front_back_and_sign(self):
        result = live_hip_callbacks()
        self.assertTrue(result["passed"], result)

    def test_original_ins_rotation_and_longitudinal_acceleration(self):
        for axis in range(3):
            for sign in (-1, 1):
                result = imu_rotation(axis, sign)
                self.assertTrue(result["passed"], result)
        for sign in (-1, 1):
            result = longitudinal_acceleration(sign)
            self.assertTrue(result["passed"], result)

    def test_physical_leg_extension_and_swing_match_c_vmc(self):
        for side in range(2):
            for kind in ("extension", "swing"):
                for sign in (-1, 1):
                    result = vmc_pulse(self.diag, side, kind, sign)
                    self.assertTrue(result["passed"], result)

    def test_wheel_yaw_motor_pulses_and_original_yaw_loop(self):
        for name in ("l_wheel", "r_wheel", "gimbal_yaw"):
            for sign in (-1, 1):
                result = actuator_pulse(name, sign)
                self.assertTrue(result["passed"], result)
        for sign in (-1, 1):
            result = gimbal_closed_loop(sign)
            self.assertTrue(result["passed"], result)


if __name__ == "__main__":
    unittest.main(verbosity=2)
