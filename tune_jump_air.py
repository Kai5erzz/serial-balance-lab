"""Batch-search physically actuated jump air-attitude allocations."""

from __future__ import annotations

from dataclasses import dataclass
import os
from pathlib import Path

import mujoco
import numpy as np

from traditional_lqr import (
    CALIBRATED_STAND_POSE_PATH,
    TraditionalLQR,
    WHEEL_RADIUS_M,
    apply_saved_pose,
)


MODEL = Path(os.environ.get(
    "MUJOCO_MODEL",
    Path(__file__).resolve().parent / "model" / "whole_body_lqr_fast.xml",
))
SIM_DURATION = float(os.environ.get("TUNE_JUMP_DURATION", "4.0"))


@dataclass(frozen=True)
class Config:
    name: str
    hip_kp: float
    hip_kd: float
    theta_tuck: float
    theta_deploy: float
    wheel_kp: float
    wheel_kd: float
    wheel_sign: float
    wheel_limit: float = 3.69
    thrust_kp: float = 8.0
    thrust_kd: float = 20.0
    thrust_limit: float = 40.0
    thrust_rate: float = 1.48
    thrust_rate_max: float = 1.48
    thrust_rate_start_l0: float = 0.16
    thrust_rate_full_l0: float = 0.26
    thrust_pitch_slew: float = 100.0
    thrust_theta_kd: float = 0.0
    thrust_theta_kp: float = 0.0
    thrust_theta_limit: float = 22.0
    theta_kp: float = 80.0
    theta_kd: float = 10.0
    theta_limit: float = 40.0
    common_limit: float = 40.0
    hip_limit: float = 40.0
    tuck_length: float = 0.14
    tuck_time: float = 0.11
    tuck_hold: float = 0.12
    tuck_rate: float = 3.50
    landing_length: float = 0.27
    deploy_rate: float = 1.20
    deploy_kp: float = 1000.0
    deploy_kd: float = 26.0
    retract_limit: float = 200.0
    retract_peak: float = 260.0
    tuck_pitch_bias_deg: float = 2.0
    deploy_pitch_bias_deg: float = 3.0
    pitch_reference_limit_deg: float = 15.0
    wheel_theta_ff: float = 0.50
    wheel_taper_delay: float = 0.0
    wheel_taper_time: float = 0.15
    wheel_speed_kd: float = 0.0
    wheel_speed_damping_start: float = 0.0
    common_filter_tau: float = 0.0
    hip_motor_filter_tau: float = 0.0
    rate_filter_tau: float = 0.0
    landing_wheel_kp: float = 10.0
    landing_wheel_kd: float = 1.0
    landing_wheel_limit: float = 3.69
    landing_capture_slew: float = 1100.0
    landing_slew: float = 1000.0
    landing_strong_hold: float = 0.0
    landing_position_ratio: float = 3.0


CONFIGS = [
    Config("baseline", 195, 13, 2.0, 5.0, 20, 2, 1),
    Config(
        "wheel_soft", 195, 13, 2.0, 5.0, 6, .6, 1,
        wheel_limit=1.2,
    ),
    Config(
        "decoupled_0", 0, 0, .7, 1.0, 8, .8, 1,
        wheel_limit=1.2, theta_kp=25, theta_kd=2,
        theta_limit=18, common_limit=18, hip_limit=24,
        tuck_pitch_bias_deg=0, deploy_pitch_bias_deg=0,
        wheel_theta_ff=.10,
    ),
    Config(
        "decoupled_20", 20, 2, .7, 1.0, 8, .8, 1,
        wheel_limit=1.2, theta_kp=25, theta_kd=2,
        theta_limit=18, common_limit=18, hip_limit=24,
        tuck_pitch_bias_deg=0, deploy_pitch_bias_deg=0,
        wheel_theta_ff=.10,
    ),
    Config(
        "decoupled_40", 40, 3, .7, 1.0, 8, .8, 1,
        wheel_limit=1.2, theta_kp=25, theta_kd=2,
        theta_limit=18, common_limit=18, hip_limit=24,
        tuck_pitch_bias_deg=0, deploy_pitch_bias_deg=0,
        wheel_theta_ff=.10,
    ),
    Config(
        "damped_40", 40, 5, .5, .8, 6, 1.0, 1,
        wheel_limit=1.0, theta_kp=22, theta_kd=3,
        theta_limit=16, common_limit=16, hip_limit=22,
        tuck_pitch_bias_deg=0, deploy_pitch_bias_deg=0,
        wheel_theta_ff=.05,
    ),
    Config(
        "damped_25", 25, 4, .5, .8, 6, 1.0, 1,
        wheel_limit=1.0, theta_kp=22, theta_kd=3,
        theta_limit=16, common_limit=16, hip_limit=22,
        tuck_pitch_bias_deg=0, deploy_pitch_bias_deg=0,
        wheel_theta_ff=.05,
    ),
    Config(
        "smooth_len_w20d3", 80, 6, 1.0, 2.0, 20, 3, 1,
        wheel_limit=3.0, theta_kp=35, theta_kd=4,
        theta_limit=22, common_limit=24, hip_limit=28,
        tuck_length=.16, tuck_rate=2.0, deploy_kp=700, deploy_kd=32,
        retract_limit=140, retract_peak=180,
        tuck_pitch_bias_deg=0, deploy_pitch_bias_deg=0,
        wheel_theta_ff=.15,
    ),
    Config(
        "smooth_len_w15d3", 80, 6, 1.0, 2.0, 15, 3, 1,
        wheel_limit=3.0, theta_kp=35, theta_kd=4,
        theta_limit=22, common_limit=24, hip_limit=28,
        tuck_length=.16, tuck_rate=2.0, deploy_kp=700, deploy_kd=32,
        retract_limit=140, retract_peak=180,
        tuck_pitch_bias_deg=0, deploy_pitch_bias_deg=0,
        wheel_theta_ff=.15,
    ),
    Config(
        "smooth_len_w20d5", 80, 6, 1.0, 2.0, 20, 5, 1,
        wheel_limit=3.0, theta_kp=35, theta_kd=4,
        theta_limit=22, common_limit=24, hip_limit=28,
        tuck_length=.16, tuck_rate=2.0, deploy_kp=700, deploy_kd=32,
        retract_limit=140, retract_peak=180,
        tuck_pitch_bias_deg=0, deploy_pitch_bias_deg=0,
        wheel_theta_ff=.15,
    ),
    Config(
        "smooth_len_hip40", 40, 5, .8, 1.5, 20, 4, 1,
        wheel_limit=3.0, theta_kp=30, theta_kd=4,
        theta_limit=20, common_limit=20, hip_limit=26,
        tuck_length=.16, tuck_rate=2.0, deploy_kp=700, deploy_kd=32,
        retract_limit=140, retract_peak=180,
        tuck_pitch_bias_deg=0, deploy_pitch_bias_deg=0,
        wheel_theta_ff=.10,
    ),
    *[
        Config(
            f"launch_rate_{rate:.1f}", 195, 13, 2.0, 5.0, 20, 2, 1,
            thrust_rate=rate, thrust_rate_max=rate,
            tuck_pitch_bias_deg=0, deploy_pitch_bias_deg=0,
        )
        for rate in (0.0, 0.3, 0.6, 0.9)
    ],
    *[
        Config(
            f"launch_bias_rate_{rate:.1f}", 195, 13, 2.0, 5.0, 20, 2, 1,
            thrust_rate=rate, thrust_rate_max=rate,
        )
        for rate in (1.0, 1.15, 1.3, 1.4)
    ],
    Config(
        "landing_off", 195, 13, 2.0, 5.0, 20, 2, 1,
        landing_wheel_kp=0, landing_wheel_kd=0,
        landing_wheel_limit=0,
    ),
    Config(
        "landing_gentle", 195, 13, 2.0, 5.0, 20, 2, 1,
        landing_wheel_kp=4, landing_wheel_kd=.5,
        landing_wheel_limit=1.2,
        landing_capture_slew=80, landing_slew=80,
    ),
    Config(
        "landing_mid", 195, 13, 2.0, 5.0, 20, 2, 1,
        landing_wheel_kp=6, landing_wheel_kd=.7,
        landing_wheel_limit=1.5,
        landing_capture_slew=100, landing_slew=100,
    ),
    Config(
        "landing_moderate", 195, 13, 2.0, 5.0, 20, 2, 1,
        landing_wheel_kp=8, landing_wheel_kd=1.0,
        landing_wheel_limit=2.0,
        landing_capture_slew=140, landing_slew=120,
    ),
    Config(
        "landing_reverse_full", 195, 13, 2.0, 5.0, 20, 2, 1,
        landing_wheel_kp=-10, landing_wheel_kd=-1,
    ),
    Config(
        "landing_reverse_mid", 195, 13, 2.0, 5.0, 20, 2, 1,
        landing_wheel_kp=-6, landing_wheel_kd=-.7,
        landing_wheel_limit=1.5,
        landing_capture_slew=100, landing_slew=100,
    ),
    Config(
        "landing_reverse_soft", 195, 13, 2.0, 5.0, 20, 2, 1,
        landing_wheel_kp=-4, landing_wheel_kd=-.5,
        landing_wheel_limit=1.2,
        landing_capture_slew=80, landing_slew=80,
    ),
    *[
        Config(
            f"landing_stage_{hold:.2f}", 195, 13, 2.0, 5.0, 20, 2, 1,
            landing_wheel_kp=-4, landing_wheel_kd=-.5,
            landing_wheel_limit=1.2,
            landing_capture_slew=1100, landing_slew=160,
            landing_strong_hold=hold,
        )
        for hold in (.04, .08, .12, .16)
    ],
    *[
        Config(
            f"air_deploy_bias_{bias:.0f}",
            195, 13, 2.0, 5.0, 20, 2, 1,
            deploy_pitch_bias_deg=bias,
        )
        for bias in (4, 6, 8, 10, 12, 15)
    ],
    *[
        Config(
            f"air_taper_d{delay:.2f}_t{duration:.2f}",
            195, 13, 2.0, 5.0, 20, 2, 1,
            wheel_taper_delay=delay,
            wheel_taper_time=duration,
        )
        for delay, duration in (
            (.03, .15), (.06, .15), (.09, .15),
            (.03, .20), (.06, .20), (.09, .20),
        )
    ],
    *[
        Config(
            f"air_alloc_h{hip_kp:.0f}_d{hip_kd:.0f}_w{wheel_kd:.0f}",
            hip_kp, hip_kd, .8, 1.5, 20, wheel_kd, 1,
            theta_kp=30, theta_kd=4,
            theta_limit=22, common_limit=24, hip_limit=30,
            wheel_theta_ff=.10,
            tuck_pitch_bias_deg=0, deploy_pitch_bias_deg=0,
        )
        for hip_kp, hip_kd, wheel_kd in (
            (0, 0, 2), (30, 3, 2), (60, 5, 2),
            (90, 7, 2), (60, 5, 4), (90, 7, 4),
        )
    ],
    *[
        Config(
            f"air_speed_kd_{speed_kd:.2f}",
            90, 7, .8, 1.5, 20, 4, 1,
            theta_kp=30, theta_kd=4,
            theta_limit=22, common_limit=24, hip_limit=30,
            wheel_theta_ff=.10, wheel_speed_kd=speed_kd,
            tuck_pitch_bias_deg=0, deploy_pitch_bias_deg=0,
        )
        for speed_kd in (.03, .06, .10, .16, .24)
    ],
    *[
        Config(
            f"air_common_lpf_{tau:.4f}",
            195, 13, 2.0, 5.0, 20, 2, 1,
            common_filter_tau=tau,
        )
        for tau in (.0025, .005, .010, .015, .020)
    ],
    *[
        Config(
            f"air_motor_lpf_{tau:.4f}",
            195, 13, 2.0, 5.0, 20, 2, 1,
            hip_motor_filter_tau=tau,
        )
        for tau in (.0005, .0010, .0015, .0025, .004, .006, .010)
    ],
    *[
        Config(
            f"air_rate_lpf_{tau:.3f}",
            195, 13, 2.0, 5.0, 20, 2, 1,
            rate_filter_tau=tau,
        )
        for tau in (.003, .005, .008, .012, .020, .030)
    ],
    *[
        Config(
            f"air_wheel_only_k{kp:.0f}_d{kd:.0f}_td{delay:.2f}",
            0, 0, .8, 1.5, kp, kd, 1,
            theta_kp=30, theta_kd=4,
            theta_limit=22, common_limit=24, hip_limit=30,
            wheel_theta_ff=.10,
            wheel_taper_delay=delay, wheel_taper_time=.20,
            tuck_pitch_bias_deg=0, deploy_pitch_bias_deg=0,
        )
        for kp, kd, delay in (
            (20, 2, .03), (20, 2, .06), (20, 2, .10),
            (25, 3, .06), (25, 3, .10),
            (30, 4, .06), (30, 4, .10),
        )
    ],
    *[
        Config(
            f"air_wheel_refine_k{kp:.0f}_d{kd:.0f}_b{bias:.0f}",
            0, 0, .8, 1.5, kp, kd, 1,
            theta_kp=30, theta_kd=4,
            theta_limit=22, common_limit=24, hip_limit=30,
            wheel_theta_ff=.10,
            wheel_taper_delay=.10, wheel_taper_time=.20,
            tuck_length=.135,
            tuck_pitch_bias_deg=bias, deploy_pitch_bias_deg=bias,
        )
        for kp, kd, bias in (
            (35, 5, 0), (40, 6, 0), (50, 8, 0),
            (30, 4, 2), (35, 5, 2), (40, 6, 2),
            (35, 5, 4), (40, 6, 4),
        )
    ],
    *[
        Config(
            f"air_wheel_tuck_{tuck_bias:.0f}",
            0, 0, .8, 1.5, 35, 5, 1,
            theta_kp=30, theta_kd=4,
            theta_limit=22, common_limit=24, hip_limit=30,
            wheel_theta_ff=.10,
            wheel_taper_delay=.10, wheel_taper_time=.20,
            tuck_length=.135,
            tuck_pitch_bias_deg=tuck_bias, deploy_pitch_bias_deg=4,
        )
        for tuck_bias in (5, 6, 8, 10, 12)
    ],
    *[
        Config(
            f"air_wheel_launch_{rate:.2f}",
            0, 0, .8, 1.5, 35, 5, 1,
            thrust_rate=rate, thrust_rate_max=rate,
            theta_kp=30, theta_kd=4,
            theta_limit=22, common_limit=24, hip_limit=30,
            wheel_theta_ff=.10,
            wheel_taper_delay=.10, wheel_taper_time=.20,
            tuck_length=.135,
            tuck_pitch_bias_deg=12, deploy_pitch_bias_deg=4,
        )
        for rate in (.9, 1.05, 1.2, 1.3, 1.4, 1.48)
    ],
    *[
        Config(
            f"air_wheel_brake_kd{speed_kd:.2f}_s{start:.2f}",
            0, 0, .8, 1.5, 35, 5, 1,
            theta_kp=30, theta_kd=4,
            theta_limit=22, common_limit=24, hip_limit=30,
            wheel_theta_ff=.10,
            wheel_speed_kd=speed_kd,
            wheel_speed_damping_start=start,
            wheel_taper_delay=.10, wheel_taper_time=.20,
            tuck_length=.135,
            tuck_pitch_bias_deg=12, deploy_pitch_bias_deg=4,
        )
        for speed_kd, start in (
            (.03, .28), (.05, .28), (.08, .28),
            (.03, .32), (.05, .32), (.08, .32), (.12, .32),
            (.20, .32), (.30, .32), (.50, .32),
        )
    ],
    *[
        Config(
            f"combo_landing_{name}",
            0, 0, .8, 1.5, 35, 5, 1,
            theta_kp=30, theta_kd=4,
            theta_limit=22, common_limit=24, hip_limit=30,
            wheel_theta_ff=.10,
            wheel_taper_delay=.10, wheel_taper_time=.20,
            tuck_length=.135,
            tuck_pitch_bias_deg=12, deploy_pitch_bias_deg=4,
            landing_wheel_kp=kp, landing_wheel_kd=kd,
            landing_wheel_limit=limit,
            landing_capture_slew=1100, landing_slew=slew,
        )
        for name, kp, kd, limit, slew in (
            ("off", 0, 0, 0, 160),
            ("rev2", -2, -.25, .8, 120),
            ("rev4", -4, -.5, 1.2, 160),
            ("pos2", 2, .25, .8, 120),
            ("pos4", 4, .5, 1.2, 160),
        )
    ],
    *[
        Config(
            f"combo_length_k{length_kp:.0f}_d{length_kd:.0f}_l{tuck_length:.3f}",
            0, 0, .8, 1.5, 35, 5, 1,
            theta_kp=30, theta_kd=4,
            theta_limit=22, common_limit=24, hip_limit=30,
            wheel_theta_ff=.10,
            wheel_taper_delay=.10, wheel_taper_time=.20,
            tuck_length=tuck_length,
            deploy_kp=length_kp, deploy_kd=length_kd,
            retract_limit=160, retract_peak=200,
            tuck_pitch_bias_deg=12, deploy_pitch_bias_deg=4,
        )
        for length_kp, length_kd, tuck_length in (
            (600, 30, .120), (600, 35, .120),
            (700, 30, .120), (700, 35, .120),
            (700, 30, .125), (800, 35, .125),
            (800, 40, .130),
        )
    ],
    *[
        Config(
            f"combo_position_ratio_{ratio:.0f}",
            0, 0, .8, 1.5, 35, 5, 1,
            theta_kp=30, theta_kd=4,
            theta_limit=22, common_limit=24, hip_limit=30,
            wheel_theta_ff=.10,
            wheel_taper_delay=.10, wheel_taper_time=.20,
            tuck_length=.135,
            tuck_pitch_bias_deg=12, deploy_pitch_bias_deg=4,
            landing_position_ratio=ratio,
        )
        for ratio in (4, 5, 6, 7, 8, 10)
    ],
    *[
        Config(
            f"combo_force_f{force:.0f}_l{tuck_length:.3f}",
            0, 0, .8, 1.5, 35, 5, 1,
            theta_kp=30, theta_kd=4,
            theta_limit=22, common_limit=24, hip_limit=30,
            wheel_theta_ff=.10,
            wheel_taper_delay=.10, wheel_taper_time=.20,
            tuck_length=tuck_length,
            deploy_kp=1000, deploy_kd=26,
            retract_limit=force, retract_peak=force + 30,
            tuck_pitch_bias_deg=12, deploy_pitch_bias_deg=4,
        )
        for force, tuck_length in (
            (140, .115), (160, .115), (180, .115),
            (140, .125), (160, .125), (180, .125),
        )
    ],
]


def simulate(model: mujoco.MjModel, config: Config) -> dict[str, float | str | bool]:
    data = mujoco.MjData(model)
    controller = TraditionalLQR(model)
    apply_saved_pose(model, data, CALIBRATED_STAND_POSE_PATH)
    controller.reset(data, calibrate_frame=True)
    controller.enable_calibrated_lqr(data)
    controller.jump_air_pitch_kp = config.hip_kp
    controller.jump_air_pitch_kd = config.hip_kd
    controller.jump_air_pitch_limit = config.hip_limit
    controller.jump_air_tuck_pitch_bias = np.radians(
        config.tuck_pitch_bias_deg
    )
    controller.jump_air_deploy_pitch_bias = np.radians(
        config.deploy_pitch_bias_deg
    )
    controller.jump_air_pitch_reference_limit = np.radians(
        config.pitch_reference_limit_deg
    )
    controller.jump_air_theta_weight_tuck = config.theta_tuck
    controller.jump_air_theta_weight_deploy = config.theta_deploy
    controller.jump_air_wheel_pitch_kp = config.wheel_kp
    controller.jump_air_wheel_pitch_kd = config.wheel_kd
    controller.jump_air_wheel_sign = config.wheel_sign
    controller.jump_air_wheel_theta_ff = config.wheel_theta_ff
    controller.jump_air_wheel_torque_limit = config.wheel_limit
    controller.jump_air_wheel_taper_delay = config.wheel_taper_delay
    controller.jump_air_wheel_taper_time = config.wheel_taper_time
    controller.jump_air_wheel_speed_kd = config.wheel_speed_kd
    controller.jump_air_wheel_speed_damping_start = (
        config.wheel_speed_damping_start
    )
    controller.jump_air_common_tp_filter_tau = config.common_filter_tau
    controller.jump_hip_motor_filter_tau = config.hip_motor_filter_tau
    controller.jump_air_rate_filter_tau = config.rate_filter_tau
    controller.jump_landing_wheel_pitch_kp = config.landing_wheel_kp
    controller.jump_landing_wheel_pitch_kd = config.landing_wheel_kd
    controller.jump_landing_wheel_torque_limit = config.landing_wheel_limit
    controller.jump_landing_capture_wheel_slew_rate = (
        config.landing_capture_slew
    )
    controller.jump_landing_wheel_slew_rate = config.landing_slew
    controller.jump_landing_strong_hold_time = config.landing_strong_hold
    controller.jump_landing_position_x_ratio = config.landing_position_ratio
    controller.jump_thrust_pitch_kp = config.thrust_kp
    controller.jump_thrust_pitch_kd = config.thrust_kd
    controller.jump_thrust_pitch_limit = config.thrust_limit
    controller.jump_thrust_pitch_rate_target = config.thrust_rate
    controller.jump_thrust_pitch_rate_max = config.thrust_rate_max
    controller.jump_thrust_pitch_rate_start_l0 = config.thrust_rate_start_l0
    controller.jump_thrust_pitch_rate_full_l0 = config.thrust_rate_full_l0
    controller.jump_thrust_pitch_tp_slew_rate = config.thrust_pitch_slew
    controller.jump_thrust_theta_kd = config.thrust_theta_kd
    controller.jump_thrust_theta_kp = config.thrust_theta_kp
    controller.jump_thrust_theta_limit = config.thrust_theta_limit
    controller.jump_air_theta_kp = config.theta_kp
    controller.jump_air_theta_kd = config.theta_kd
    controller.jump_air_theta_limit = config.theta_limit
    controller.jump_air_common_tp_limit = config.common_limit
    controller.jump_air_tuck_length = config.tuck_length
    controller.jump_air_tuck_time = config.tuck_time
    controller.jump_air_tuck_hold_time = config.tuck_hold
    controller.jump_air_tuck_ref_rate = config.tuck_rate
    controller.jump_air_landing_length = config.landing_length
    controller.jump_air_deploy_ref_rate = config.deploy_rate
    controller.jump_air_deploy_kp = config.deploy_kp
    controller.jump_air_deploy_kd = config.deploy_kd
    controller.jump_air_retract_force_limit = config.retract_limit
    controller.jump_air_retract_force_peak = config.retract_peak
    controller.jump_abort_pitch_limit = np.pi
    controller.jump_abort_roll_limit = np.pi

    settle_end = data.time + 3.0
    while data.time < settle_end:
        controller.step(data, model.opt.timestep)
        mujoco.mj_step(model, data)
    accepted, reason = controller.begin_jump(data)
    if not accepted:
        return {"name": config.name, "error": reason}

    start_z = float(data.xpos[controller.base, 2])
    apex_z = start_z
    apex_pitch = 0.0
    apex_wheel_clearance = 0.0
    apex_l0 = 0.0
    apex_theta = 0.0
    max_air_pitch = 0.0
    min_air_pitch = np.inf
    max_signed_air_pitch = -np.inf
    first_contact_pitch = np.nan
    first_contact_theta = np.nan
    first_contact_wheel_rate = np.nan
    first_contact_forward = np.nan
    first_contact_time = np.nan
    first_ground = "none"
    first_ground_time = np.nan
    any_other_after_takeoff = False
    other_ground_labels: set[str] = set()
    takeoff_pitch_rate = np.nan
    max_motor = 0.0
    peak_motor_time = 0.0
    max_hip_step = 0.0
    max_wheel_step = 0.0
    previous_hip = {side: np.zeros(2) for side in ("l", "r")}
    previous_air_wheel = 0.0
    start_position = data.xpos[controller.base].copy()
    tail_pitch: list[float] = []
    tail_roll: list[float] = []
    tail_z: list[float] = []
    tail_both_contact: list[bool] = []
    tail_other_contact: list[bool] = []
    ground_geom = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_GEOM, "ground")
    wheel_bodies = {ids.wheel_body for ids in controller.sides.values()}

    def other_ground_contact() -> bool:
        for index in range(data.ncon):
            contact = data.contact[index]
            if contact.geom1 == ground_geom:
                other_geom = int(contact.geom2)
            elif contact.geom2 == ground_geom:
                other_geom = int(contact.geom1)
            else:
                continue
            if int(model.geom_bodyid[other_geom]) not in wheel_bodies:
                return True
        return False
    was_airborne = False
    end_time = data.time + SIM_DURATION
    while data.time < end_time:
        controller.step(data, model.opt.timestep)
        mujoco.mj_step(model, data)
        z = float(data.xpos[controller.base, 2])
        pitch = float(np.degrees(controller.body_pitch(data)))
        roll = float(np.degrees(controller.body_roll(data)))
        if z > apex_z:
            apex_z = z
            apex_pitch = pitch
            apex_wheel_clearance = min(
                float(data.xpos[ids.wheel_body, 2]) - WHEEL_RADIUS_M
                for ids in controller.sides.values()
            )
            apex_l0 = 0.5 * sum(
                controller.vmc_kinematics(data, side).l0
                for side in ("l", "r")
            )
            pitch_rad = controller.body_pitch(data)
            apex_theta = float(np.degrees(0.5 * sum(
                np.pi / 2.0
                - controller.vmc_kinematics(data, side).phi0
                + pitch_rad
                for side in ("l", "r")
            )))
        if controller.jump_off_ground:
            if not was_airborne:
                takeoff_pitch_rate = float(np.degrees(
                    -controller.last_state["l"][5]
                ))
            was_airborne = True
            max_air_pitch = max(max_air_pitch, abs(pitch))
            min_air_pitch = min(min_air_pitch, pitch)
            max_signed_air_pitch = max(max_signed_air_pitch, pitch)
        if was_airborne:
            ground_labels: list[str] = []
            for contact_index in range(data.ncon):
                contact = data.contact[contact_index]
                if ground_geom not in (contact.geom1, contact.geom2):
                    continue
                other_geom = (
                    contact.geom2
                    if contact.geom1 == ground_geom
                    else contact.geom1
                )
                other_body = int(model.geom_bodyid[other_geom])
                if other_body in wheel_bodies:
                    ground_labels.append("wheel")
                else:
                    any_other_after_takeoff = True
                    label = mujoco.mj_id2name(
                        model, mujoco.mjtObj.mjOBJ_BODY, other_body
                    ) or f"body#{other_body}"
                    ground_labels.append(label)
                    other_ground_labels.add(label)
            if ground_labels and np.isnan(first_ground_time):
                first_ground_time = float(data.time)
                first_ground = ",".join(sorted(set(ground_labels)))
        if (
            was_airborne
            and np.isnan(first_contact_time)
            and any(controller.jump_raw_wheel_contact.values())
        ):
            first_contact_time = float(data.time)
            first_contact_pitch = pitch
            pitch_rad = controller.body_pitch(data)
            first_contact_theta = float(np.degrees(0.5 * sum(
                np.pi / 2.0
                - controller.vmc_kinematics(data, side).phi0
                + pitch_rad
                for side in ("l", "r")
            )))
            first_contact_wheel_rate = 0.5 * sum(
                controller.wheel_sign[side]
                * float(data.qvel[model.jnt_dofadr[controller.sides[side].wheel_joint]])
                for side in ("l", "r")
            )
            first_contact_forward = -float(np.dot(
                data.xpos[controller.base] - start_position,
                controller.forward_world,
            ))
        motor = max(
            abs(float(value))
            for values in controller.last_motor_torque.values()
            for value in values
        )
        max_motor = max(max_motor, motor)
        peak_motor_time += model.opt.timestep * int(motor > 20.0)
        if controller.jump_off_ground:
            max_hip_step = max(
                max_hip_step,
                *(float(np.max(np.abs(
                    controller.last_motor_torque[side] - previous_hip[side]
                ))) for side in ("l", "r")),
            )
            max_wheel_step = max(
                max_wheel_step,
                abs(controller.jump_air_wheel_torque - previous_air_wheel),
            )
        previous_hip = {
            side: controller.last_motor_torque[side].copy()
            for side in ("l", "r")
        }
        previous_air_wheel = controller.jump_air_wheel_torque
        if data.time >= end_time - 0.5:
            tail_pitch.append(pitch)
            tail_roll.append(roll)
            tail_z.append(z)
            tail_both_contact.append(all(controller._wheel_contacts(data).values()))
            tail_other_contact.append(other_ground_contact())

    return {
        "name": config.name,
        "apex_dz": apex_z - start_z,
        "apex_pitch": apex_pitch,
        "apex_wheel": apex_wheel_clearance,
        "apex_l0": apex_l0,
        "apex_theta": apex_theta,
        "max_air_pitch": max_air_pitch,
        "min_air_pitch": min_air_pitch,
        "max_signed_air_pitch": max_signed_air_pitch,
        "contact_t": first_contact_time,
        "takeoff_rate": takeoff_pitch_rate,
        "contact_pitch": first_contact_pitch,
        "contact_theta": first_contact_theta,
        "contact_wheel_rate": first_contact_wheel_rate,
        "contact_forward": first_contact_forward,
        "first_ground": first_ground,
        "first_ground_t": first_ground_time,
        "other_after": any_other_after_takeoff,
        "other_labels": ",".join(sorted(other_ground_labels)) or "none",
        "tail_pitch": float(np.mean(tail_pitch)),
        "tail_roll": float(np.mean(tail_roll)),
        "tail_z_span": float(np.ptp(tail_z)),
        "tail_contact": float(np.mean(tail_both_contact)),
        "tail_other": float(np.mean(tail_other_contact)),
        "off_end": controller.jump_off_ground,
        "fault": controller.jump_fault,
        "max_motor": max_motor,
        "peak_time": peak_motor_time,
        "max_hip_step": max_hip_step,
        "max_wheel_step": max_wheel_step,
        "forward_delta": -float(np.dot(
            data.xpos[controller.base] - start_position,
            controller.forward_world,
        )),
    }


def main() -> None:
    model = mujoco.MjModel.from_xml_path(str(MODEL))
    name_filter = os.environ.get("TUNE_JUMP_FILTER", "")
    for config in CONFIGS:
        if name_filter and name_filter not in config.name:
            continue
        result = simulate(model, config)
        print(
            "{name:27s} apex dz={apex_dz:.3f} wheel={apex_wheel:.3f} "
            "P={apex_pitch:+5.1f}[{min_air_pitch:+4.0f},"
            "{max_signed_air_pitch:+4.0f}] "
            "L/th={apex_l0:.3f}/{apex_theta:+4.0f} "
            "takeoff_dP={takeoff_rate:+5.1f} "
            "first={first_ground}@{first_ground_t:.3f} "
            "other={other_after}:{other_labels} "
            "contact P/theta={contact_pitch:+5.1f}/{contact_theta:+5.1f} "
            "wr={contact_wheel_rate:+6.1f}rad/s "
            "xcontact={contact_forward:+.3f}m "
            "tail P/R={tail_pitch:+5.1f}/{tail_roll:+4.1f} "
            "zspan={tail_z_span:.3f} wheels={tail_contact:.2f} other={tail_other:.2f} "
            "off={off_end} fault={fault} motor={max_motor:.1f}@{peak_time:.2f}s "
            " dHip/dTw={max_hip_step:.1f}/{max_wheel_step:.2f} "
            "dx={forward_delta:+.3f}m".format(
                **result
            )
        )


if __name__ == "__main__":
    main()
