"""Shared firmware ABI, timing, real driver, sensor and replay regressions."""
from pathlib import Path
import json
import sys
import tempfile
import unittest
import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
from balance_lab.firmware import VehicleRuntime, Feedback, Command, ChassisMode, GimbalMode
from balance_lab.firmware.calibration import load_calibration
from balance_lab.firmware.simulation import VehicleSimulation
from balance_lab.firmware.recording import Recorder, replay, fields
from tools.sync_firmware import sync, DEFAULT_SOURCE


def stationary(time_us=1000, *, raw=False):
    f = Feedback(time_us=time_us, feedback_mask=511, raw_imu=int(raw))
    f.accel[:] = (0,0,9.81)
    return f


class NativeFirmwareTests(unittest.TestCase):
    @unittest.skipUnless(DEFAULT_SOURCE.is_dir(), "vehicle checkout is optional in a standalone clone")
    def test_snapshot_matches_vehicle(self):
        result = sync(DEFAULT_SOURCE, check=True)
        self.assertIn("src/task/gimbal/gimbal_task.c", result["files"])
        self.assertIn("Core/Src/tim.c", result["files"])

    def test_zero_record_matches_closed_pose(self):
        zeros, identity = load_calibration()
        pose = json.loads((ROOT/"firmware/calibration/evidence/encoder_zero_candidate_pose.json").read_text())
        self.assertTrue(identity["sha256"])
        for name, value in zeros.items():
            self.assertEqual(value, pose[name])

    def test_real_vmc_zero_geometry(self):
        with VehicleRuntime() as v:
            out = v.step(stationary(raw=True), Command())
            np.testing.assert_allclose(out.length, [.332557082, .332557082], atol=1e-5)
            np.testing.assert_allclose(np.degrees(out.phi0), [18.495,18.495], atol=.03)
            self.assertEqual(v.parameters["wheel_current_per_nm"], 2220)
            self.assertAlmostEqual(v.parameters["controller_support_mass_kg"], 12.801, places=5)

    def test_original_ins_integrates_raw_gyro(self):
        with VehicleRuntime() as v:
            f = stationary(raw=True); f.gyro[2]=.5
            for i in range(200):
                f.time_us=(i+1)*1000; out=v.step(f, Command())
            self.assertAlmostEqual(out.imu[5], -.5, places=6)
            self.assertAlmostEqual(out.imu[2], -np.degrees(.5*.2), delta=.04)
            self.assertTrue(np.isfinite(out.imu).all())

    def test_dm_round_robin_and_holds(self):
        with VehicleRuntime() as v:
            self.assertEqual(v.timing, {"control_tick_us":1000, "dm_poll_us":2000, "cpu_hz":480000000})
            expected=[0]*5
            for tick in range(1,21):
                if tick%2==0:
                    expected[(0,3,1,2,4)[(tick//2-1)%5]] += 1
                out=v.step(stationary(tick*1000),Command())
                self.assertEqual(list(out.motor_tx_count),expected)

    def test_runtime_instances_are_isolated(self):
        with VehicleRuntime() as a, VehicleRuntime() as b, VehicleRuntime() as c:
            for tick in range(1,101):
                a.step(stationary(tick*1000),Command(chassis_mode=2,gimbal_mode=2,gimbal_yaw_raw=100))
            self.assertEqual(fields(b.step(stationary(),Command())), fields(c.step(stationary(),Command())))

    def test_timestamps_cannot_skip_control_ticks(self):
        with VehicleRuntime() as v:
            with self.assertRaises(ValueError): v.step(stationary(0),Command())
            with self.assertRaises(ValueError): v.step(stationary(2000),Command())
            v.step(stationary(),Command())
            with self.assertRaises(ValueError): v.step(stationary(3000),Command())
            v.step(stationary(2000),Command())

    def test_command_dispatch_preserves_vehicle_pipeline(self):
        with VehicleRuntime() as v:
            cmd=Command(chassis_mode=ChassisMode.INIT)
            first=v.step(stationary(),cmd)
            self.assertEqual(first.chassis_mode,ChassisMode.RELAX)
            second=v.step(stationary(2000),cmd)
            self.assertEqual(second.chassis_mode,ChassisMode.INIT)

    def test_yaw_manual_cascade_and_packet_limits(self):
        with VehicleRuntime() as v:
            f=stationary()
            for tick in range(1,101):
                f.time_us=tick*1000
                out=v.step(f,Command(chassis_mode=2,gimbal_mode=2,gimbal_yaw_raw=100))
            self.assertLess(out.yaw_target,-.05)
            self.assertGreater(out.yaw_command[4],0)
            self.assertLessEqual(abs(out.yaw_command[4]),3.52)
            for packet in list(out.hip_command)+[out.yaw_command]:
                wire=(packet[4]+35)/70*4095
                self.assertAlmostEqual(wire,round(wire),delta=.002)

    def test_yaw_homing_uses_original_readiness(self):
        with VehicleRuntime() as v:
            for tick in range(1,401):
                out=v.step(stationary(tick*1000,raw=True),Command(chassis_mode=2,gimbal_mode=1))
            self.assertEqual(out.gimbal_ready,1)
            self.assertAlmostEqual(out.yaw_relative,0,delta=.001)

    def test_rc_loss_disables_held_hip_and_wheel_commands(self):
        with VehicleRuntime() as v:
            for tick in range(1,121):
                out=v.step(stationary(tick*1000),Command.remote(right_switch=3,lost=tick>100))
            self.assertEqual(out.chassis_mode,0)
            np.testing.assert_array_equal(out.hip_command,np.zeros((4,5)))
            np.testing.assert_array_equal(out.wheel_current,[0,0])
            np.testing.assert_array_equal(out.yaw_command,np.zeros(5))

    def test_invalid_rc_frame_uses_vehicle_fail_closed_path(self):
        with VehicleRuntime() as v:
            out=v.step(stationary(),Command.remote(channels=(0,701,0,0,0),right_switch=3))
            self.assertEqual(out.chassis_mode,0)

    def test_sensor_freshness_can_be_withheld(self):
        with VehicleRuntime() as v:
            f=stationary();cmd=Command(chassis_mode=2,gimbal_mode=2)
            v.step(f,cmd)
            f.feedback_mask=0
            for tick in range(2,102):
                f.time_us=tick*1000;out=v.step(f,cmd)
            # Page64 is the actual source's sensor validity diagnostic.
            self.assertTrue(np.isfinite(v.debug_page(64)).all())
            self.assertEqual(out.imu[1],0)
            self.assertTrue(np.isfinite(out.vmc).all())

    def test_physics_adapter_only_actuates_and_replay_matches(self):
        sim=VehicleSimulation(pose="stand",plant="nominal")
        try:
            before=sim.data.qpos.copy()
            f=sim.read_feedback()
            out=sim.runtime.step(f,Command())
            sim.apply_outputs(out,f)
            np.testing.assert_array_equal(sim.data.qpos,before)
            self.assertGreater(np.linalg.norm(sim.data.qfrc_applied),0)
            # Restart the C instance before recording a complete episode.
            sim.ticks=1;sim.reset("stand")
            with tempfile.TemporaryDirectory() as directory:
                path=Path(directory)/"episode.jsonl"
                recorder=Recorder(path,sim.metadata(),pages=(47,54,55,57,62,64))
                masks=[]
                for _ in range(100):
                    cmd=Command.remote(right_switch=3)
                    sim.step(cmd);recorder.tick(sim,cmd);masks.append(sim.feedback.feedback_mask)
                recorder.close()
                self.assertGreater(len(set(masks)),2)
                result=replay(path)
                self.assertEqual(result["ticks"],100)
                self.assertTrue(result["matched"])
        finally:
            sim.close()


if __name__ == "__main__":
    unittest.main(verbosity=2)
