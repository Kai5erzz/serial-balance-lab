

import sys
from pathlib import Path

REPO_ROOT = Path(__file__).resolve().parents[2]
if str(REPO_ROOT) not in sys.path:
    sys.path.insert(0, str(REPO_ROOT))

from __future__ import annotations

import sys

import mujoco
import numpy as np

from traditional_lqr import (
    CALIBRATED_STAND_POSE_PATH,
    TraditionalLQR,
    apply_saved_pose,
)


leg_length = float(sys.argv[1])
drive_speed = float(sys.argv[2]) if len(sys.argv) > 2 else 0.15
model = mujoco.MjModel.from_xml_path(
    "model/whole_body_lqr_fast_platform.xml"
)
data = mujoco.MjData(model)
controller = TraditionalLQR(model)
apply_saved_pose(model, data, CALIBRATED_STAND_POSE_PATH)
data.qpos[2] += 0.6
mujoco.mj_forward(model, data)
controller.reset(data, calibrate_frame=True)
controller.enable_calibrated_lqr(data)
controller.set_leg_length(leg_length)

# Match the regression test: let the closed-loop stand settle before the
# forward command is applied, otherwise the first contact event is a startup
# transient rather than a representative ledge crossing.
for _ in range(int(np.ceil(2.0 / model.opt.timestep))):
    controller.step(data, model.opt.timestep)
    mujoco.mj_step(model, data)
controller.set_drive_command(drive_speed, 0.0)

last_print = -1.0
next_print = 7.0
previous_state = None
while data.time < 17.0:
    controller.step(data, model.opt.timestep)
    mujoco.mj_step(model, data)
    if data.time < 7.0:
        continue
    contacts = set()
    normals = []
    for index in range(data.ncon):
        contact = data.contact[index]
        names = [
            mujoco.mj_id2name(
                model, mujoco.mjtObj.mjOBJ_GEOM, int(geom_id)
            )
            for geom_id in (contact.geom1, contact.geom2)
        ]
        if (
            any(name in {"fast_collision_l_wheel", "fast_collision_r_wheel"}
                for name in names)
            and any(name and name.startswith("platform") for name in names)
        ):
            contacts.update(name for name in names if name and name.startswith("platform"))
            normals.append(tuple(np.round(
                np.asarray(contact.frame).reshape(3, 3)[0], 2
            )))
    state = (
        controller.descent_guard_active,
        controller.off_ground,
        controller.gbuf_phase,
    )
    # Print densely around the first geometric edge and then at 100 Hz.  The
    # important distinction is between a real vertical tread support and a
    # riser scrape, so include both contact masks and the commanded channels.
    if state != previous_state or data.time + 1e-9 >= next_print:
        kin = {
            side: controller.vmc_kinematics(data, side)
            for side in ("l", "r")
        }
        print(
            f"t={data.time:6.3f} z={data.xpos[controller.base, 2]:.3f} "
            f"x={data.xpos[controller.base, 0]:+.3f} "
            f"vz={data.qvel[2]:+.2f} "
            f"travel={np.dot(data.xpos[controller.base] - controller.base_position0, controller.forward_world):+.2f} "
            f"v={-controller.chassis_vx:+.2f} "
            f"p={np.degrees(controller.body_pitch(data)):+.1f} "
            f"r={np.degrees(controller.body_roll(data)):+.1f} "
            f"l0={kin['l'].l0:.3f}/{kin['r'].l0:.3f} "
            f"fn={controller.offground_fn_filtered['l']:.0f}/"
            f"{controller.offground_fn_filtered['r']:.0f} "
            f"sup={controller.last_support['l']:.0f}/"
            f"{controller.last_support['r']:.0f} "
            f"out={controller.last_output['l'][0]:+.2f}/"
            f"{controller.last_output['r'][0]:+.2f} "
            f"tp={controller.last_output['l'][1]:+.2f}/"
            f"{controller.last_output['r'][1]:+.2f} "
            f"airF={controller.last_support['l']:+.0f}/"
            f"{controller.last_support['r']:+.0f} "
            f"edge={controller._ground_edge_distance_ahead(data, controller.forward_world)} "
            f"raw={controller._wheel_contacts(data)} supc={controller._wheel_support_contacts(data)} "
            f"state={int(controller.descent_guard_active)}/"
            f"{int(controller.off_ground)}/{controller.gbuf_phase} "
            f"etr={int(controller.descent_edge_transition_active)} "
            f"airref={controller.landing_air_deploy_ref['l']:.3f}/"
            f"{controller.landing_air_deploy_ref['r']:.3f} "
            f"contact={sorted(contacts)} n={normals[:3]}"
        )
        previous_state = state
        last_print = data.time
        next_print = data.time + (0.01 if data.time < 14.5 else 0.10)
