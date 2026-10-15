"""Input edge/focus semantics and conjugate hardware mapping regressions."""
from pathlib import Path
import sys
import unittest
import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from balance_lab.firmware.operator_console import OperatorInput, firmware_input_contract, telemetry
from balance_lab.firmware.simulation import VehicleSimulation, WHEEL_NAMES
from balance_lab.firmware.runtime import Output


class OperatorInputTests(unittest.TestCase):
    def test_contract_comes_from_current_vehicle_snapshot(self):
        contract = firmware_input_contract()
        self.assertEqual(contract["drive_limits_mps"], [2.30, 1.85, 1.20])
        self.assertEqual(contract["rc_max"], 660)
        self.assertEqual(contract["gimbal_rate_rad_s"], 5.3)

    def test_modifiers_and_opposing_keys_never_exceed_rc_envelope(self):
        state = OperatorInput(paused=False)
        for keys, expected in (({"w"}, 363), ({"w","shift"},660), ({"s","control"},-132),
                               ({"w","s","shift"},0)):
            state.keys = keys
            self.assertEqual(state.command(0).rc_channels[1],expected)
        state.mouse_dx=10000
        state.begin_frame(.04)
        self.assertEqual(state.command(0).rc_channels[2],660)
        state.keys=set()
        state.begin_frame(.04)
        self.assertEqual(state.command(.04).rc_channels[2],0)

    def test_mouse_drag_is_quarter_sensitivity_and_frame_rate_independent(self):
        # The same 100 pixels over 0.2 s must request the same total angle at
        # either supported UI rate, without retaining movement next frame.
        for fps in (20,25,30):
            state=OperatorInput(paused=False)
            angle=0.
            for _ in range(round(.2*fps)):
                state.mouse_dx=100/(.2*fps)
                state.begin_frame(1/fps)
                angle += state.yaw_raw/state.rc_max*state.gimbal_rate_rad_s/fps
            self.assertAlmostEqual(angle,.075,delta=.002)
            self.assertAlmostEqual(state.mouse_sensitivity_rad_px,.003/4)
            state.begin_frame(1/fps)
            self.assertEqual(state.yaw_raw,0)

    def test_rm_actions_use_real_remote_switch_and_wheel_semantics(self):
        state=OperatorInput(paused=False)
        state.press("e",0)
        self.assertEqual(state.command(0).rc_switches[0],2)
        state.press("q",0)
        self.assertEqual(state.command(0).rc_switches[0],3)
        for key, switch in (("z",2),("x",3),("c",1)):
            state.press(key,0)
            self.assertEqual(state.command(0).rc_switches[1],switch)
        state.press("space",1)
        self.assertEqual(state.command(1.01).rc_channels[4],660)
        self.assertEqual(state.command(1.06).rc_channels[4],0)
        state.press("space",1.07)  # Held-key repeat is not a second edge.
        self.assertEqual(state.command(1.07).rc_channels[4],0)

    def test_pause_autorepeat_and_focus_release_do_not_latch_motion(self):
        state=OperatorInput(paused=False,keys={"w"},mouse_dx=100,yaw_raw=250,jump_until=1)
        state.press("p",0)
        self.assertTrue(state.paused)
        state.press("p",0.1)
        self.assertTrue(state.paused)
        self.assertEqual(state.command(.1).rc_switches[0],1)
        state.keys.discard("p")
        state.press("p",.2)
        self.assertFalse(state.paused)
        state.release()
        cmd=state.command(.2)
        self.assertEqual(list(cmd.rc_channels),[0,0,0,0,0])
        state.paused=True
        state.press("space",.2)
        state.paused=False
        self.assertEqual(state.command(.21).rc_channels[4],0)

    def test_reset_drops_pending_actions_and_requests_encoder_spawn(self):
        state=OperatorInput(paused=False,keys={"w"},spin=True,leg_switch=1,jump_until=10)
        self.assertEqual(state.press("r",2),"reset")
        self.assertTrue(state.paused)
        self.assertFalse(state.spin)
        self.assertEqual(state.leg_switch,2)
        self.assertLess(state.jump_until,0)

    def test_map_overview_does_not_change_driving_or_gimbal_request(self):
        state = OperatorInput(paused=False, keys={"w"}, yaw_raw=123)
        before = state.command(1)
        state.press("m", 1)
        self.assertEqual(state.camera, "overview")
        after = state.command(1)
        self.assertEqual(list(after.rc_channels), list(before.rc_channels))
        state.keys.discard("m")
        state.press("m", 1.1)
        self.assertEqual(state.camera, "third")

    def test_help_and_fullscreen_preserve_live_commands_and_ignore_repeat(self):
        state = OperatorInput(paused=False, keys={"w"}, yaw_raw=123)
        before = state.command(1)
        self.assertTrue(state.show_help)
        state.press("h", 1)
        self.assertFalse(state.show_help)
        state.press("h", 1.1)
        self.assertFalse(state.show_help)
        self.assertEqual(state.press("f11", 1.2), "fullscreen")
        self.assertTrue(state.fullscreen)
        self.assertIsNone(state.press("f11", 1.3))
        after = state.command(1.4)
        self.assertEqual(list(after.rc_channels), list(before.rc_channels))
        self.assertEqual(list(after.rc_switches), list(before.rc_switches))
        state.keys.discard("h")
        state.press("h", 1.5)
        self.assertTrue(state.show_help)


class CandidateMappingTests(unittest.TestCase):
    controller_profile = None

    def test_operator_forward_reverse_and_live_c_velocity_limits(self):
        velocities=[]
        for key in ("w","s"):
            sim=VehicleSimulation(interface_mapping="coherent_candidate", yaw_damping_nms_rad=0,
                                  controller_profile=self.controller_profile)
            try:
                state=OperatorInput(paused=False)
                samples=[]
                for tick in range(6000):
                    if tick==5000:
                        start=sim.data.xpos[sim.base].copy()
                        # The CAD barrel runs along yaw-head local +X.
                        muzzle=sim.data.xmat[sim.gimbal].reshape(3,3)[:,0].copy()
                    state.keys={key,"control"} if tick>=5000 else set()
                    state.begin_frame(sim.tick_s)
                    sim.step(state.command(tick*sim.tick_s))
                    if tick>=5800:
                        samples.append(float(np.dot(sim._velocity(sim.base)[3:],muzzle)))
                self.assertEqual(sim.output.stand_state,2)
                page=sim.runtime.debug_page(39)
                self.assertAlmostEqual(page[4],2.30,places=5)
                self.assertAlmostEqual(page[3],.46 if key=="w" else -.46,places=5)
                velocities.append(float(np.mean(samples)))
                displacement=float(np.dot(sim.data.xpos[sim.base]-start,muzzle))
                direction=1 if key=="w" else -1
                self.assertGreater(direction*displacement,.04,"movement must agree with the CAD gun muzzle")
                self.assertGreater(direction*telemetry(sim)["speed_mps"],.05)
                # MID is a normal leg selection. The real HIGH switch also
                # requests BUMP, whose command owner explicitly forces LOW.
                # Check that behavior instead of inventing a high-leg bypass.
                for leg_key,limit in (("x",1.85),("c",2.30)):
                    state.press(leg_key,6)
                    state.keys={key,"shift"}
                    for _ in range(30):
                        state.begin_frame(sim.tick_s)
                        sim.step(state.command(sim.ticks*sim.tick_s))
                    page=sim.runtime.debug_page(39)
                    self.assertAlmostEqual(page[4],limit,places=5)
                    self.assertLessEqual(abs(page[3]),limit+1e-5)
                self.assertNotEqual(sim.output.bump_state,0)
            finally:
                sim.close()
        self.assertGreater(velocities[0],.05,"W must physically move toward the gun muzzle")
        self.assertLess(velocities[1],-.05,"S must physically move away from the gun muzzle")

    def test_spin_holds_world_heading_through_multiple_chassis_rotations(self):
        sim=VehicleSimulation(interface_mapping="coherent_candidate", yaw_damping_nms_rad=0,
                              controller_profile=self.controller_profile)
        try:
            state=OperatorInput(paused=False)
            gun_angles=[]
            chassis_angles=[]
            targets=[]
            torques=[]
            for tick in range(10000):
                state.spin=tick>=5000
                sim.step(state.command(tick*sim.tick_s))
                if tick>=4999:
                    gun=sim.data.xmat[sim.gimbal].reshape(3,3)[:,0]
                    chassis=sim.data.xmat[sim.base].reshape(3,3)@sim.sensor_body[:,1]
                    gun_angles.append(np.arctan2(gun[1],gun[0]))
                    chassis_angles.append(np.arctan2(chassis[1],chassis[0]))
                    targets.append(sim.runtime.debug_page(49)[18])
                    torques.append(sim.output.yaw_command[4])
            gun=np.unwrap(gun_angles)
            chassis=np.unwrap(chassis_angles)
            self.assertEqual(sim.output.chassis_mode,6)
            self.assertEqual(sim.output.stand_state,2)
            self.assertGreater(abs(chassis[-1]-chassis[0]),7*2*np.pi)
            self.assertLess(np.max(abs(gun-gun[0])),np.radians(4))
            self.assertLess(abs(gun[-1]-gun[0]),np.radians(2))
            self.assertLess(max(targets)-min(targets),1e-6,"world yaw target must stay fixed")
            self.assertLess(max(abs(np.array(torques))),2.4,"hold must not rely on saturated torque")
            self.assertIn("uncalibrated",sim.metadata()["yaw_damping_status"])
        finally:
            sim.close()

    def test_original_default_and_candidate_position_velocity_torque_power(self):
        sim=VehicleSimulation(plant="nominal", controller_profile=self.controller_profile)
        try:
            self.assertEqual(sim.interface_mapping,"original")
            self.assertAlmostEqual(sim.model.dof_damping[sim.dadr["gimbal_yaw"]],.35)
            for name in WHEEL_NAMES+("gimbal_yaw",):
                sim.data.qvel[sim.dadr[name]]=.4
                sim.data.actuator_force[sim.actuators[name]]=1.2
            original=sim.read_feedback()
            sim.interface_mapping="coherent_candidate"
            candidate=sim.read_feedback()
            for field in ("wheel_angle","wheel_speed","wheel_torque"):
                np.testing.assert_allclose(list(getattr(candidate,field)),-np.array(list(getattr(original,field))))
            for field in ("yaw_motor_angle","yaw_motor_speed","yaw_motor_torque","gimbal_yaw_deg","gimbal_yaw_rate"):
                self.assertAlmostEqual(getattr(candidate,field),-getattr(original,field))
            self.assertAlmostEqual(candidate.yaw_motor_speed*candidate.yaw_motor_torque,
                                   original.yaw_motor_speed*original.yaw_motor_torque)
            out=Output()
            out.wheel_current[:]=(2220,-2220)
            out.yaw_command[4]=2
            sim.apply_outputs(out,candidate)
            self.assertEqual(sim.data.ctrl[sim.actuators["l_wheel"]],-1)
            self.assertEqual(sim.data.ctrl[sim.actuators["r_wheel"]],1)
            self.assertEqual(sim.data.ctrl[sim.actuators["gimbal_yaw"]],-2)
            self.assertIn("candidate",sim.metadata()["interface_mapping_status"])
        finally:
            sim.close()


class SimulationProfileMappingTests(CandidateMappingTests):
    controller_profile = "jump46"


if __name__=="__main__":
    unittest.main(verbosity=2)
