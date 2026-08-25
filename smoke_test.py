"""Headless finite-value check for the floating-base LQR experiment."""

from __future__ import annotations

from pathlib import Path
import sys

import mujoco
import numpy as np

from traditional_lqr import TraditionalLQR, apply_code_zero_pose


MODEL = Path(__file__).resolve().parent / "model" / "whole_body_lqr.xml"


def main() -> None:
    model = mujoco.MjModel.from_xml_path(str(MODEL))
    data = mujoco.MjData(model)
    mujoco.mj_resetData(model, data)
    apply_code_zero_pose(model, data)
    controller = TraditionalLQR(model)
    controller.reset(data)
    if "--lqr" in sys.argv:
        controller.enable_cad_zero_lqr(data)
    for _ in range(2_000):
        controller.step(data, model.opt.timestep)
        mujoco.mj_step(model, data)

    finite = bool(np.isfinite(data.qpos).all() and np.isfinite(data.qvel).all())
    base = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, "base_link")
    print(f"finite={finite}; t={data.time:.3f}; contacts={data.ncon}; lqr={controller.enabled}")
    print("base_position=", np.array2string(data.xpos[base], precision=4))
    print("left_T_Tp=", np.array2string(controller.last_output["l"], precision=4))
    print("right_T_Tp=", np.array2string(controller.last_output["r"], precision=4))
    print("support_N=", controller.last_support)
    if not finite:
        raise SystemExit("Non-finite simulation state")


if __name__ == "__main__":
    main()
