"""Choose the CAD-zero wheel polarity with symmetric pitch-pulse tests."""

from __future__ import annotations

import sys
from pathlib import Path

REPO_ROOT = Path(__file__).resolve().parents[2]
if str(REPO_ROOT) not in sys.path:
    sys.path.insert(0, str(REPO_ROOT))

from pathlib import Path

import mujoco
import numpy as np

from traditional_lqr import TraditionalLQR, apply_code_zero_pose


MODEL = REPO_ROOT / "model" / "whole_body_lqr.xml"
PULSE_START_S = 1.75
PULSE_DURATION_S = 0.10
PULSE_TORQUE_NM = 2.0
END_SECONDS = 8.0


def run_candidate(wheel_global_sign: float, pulse_torque: float) -> tuple[float, float, float, bool]:
    model = mujoco.MjModel.from_xml_path(str(MODEL))
    data = mujoco.MjData(model)
    apply_code_zero_pose(model, data)
    controller = TraditionalLQR(model)
    controller.reset(data)
    controller.cad_stand_wheel_sign = {"l": -wheel_global_sign, "r": wheel_global_sign}
    controller.enable_cad_zero_lqr(data)

    samples: list[tuple[float, float]] = []
    while data.time < END_SECONDS:
        controller.step(data, model.opt.timestep)
        if PULSE_START_S <= data.time < PULSE_START_S + PULSE_DURATION_S:
            _, _, lateral = controller._frame(data)
            mujoco.mj_applyFT(
                model, data, np.zeros(3), pulse_torque * lateral,
                data.xpos[controller.base], controller.base, data.qfrc_applied,
            )
        mujoco.mj_step(model, data)
        samples.append((data.time, float(np.degrees(controller.body_pitch(data)))))

    timeline = np.asarray(samples)
    pulse = timeline[(timeline[:, 0] >= PULSE_START_S) & (timeline[:, 0] <= PULSE_START_S + 1.0)]
    tail = timeline[-200:]
    peak = float(np.max(np.abs(pulse[:, 1])))
    final_mean = float(np.mean(tail[:, 1]))
    final_peak = float(np.max(np.abs(tail[:, 1])))
    finite = bool(np.isfinite(data.qpos).all() and np.isfinite(data.qvel).all())
    return peak, final_mean, final_peak, finite


def main() -> None:
    results = []
    for wheel_sign in (1.0, -1.0):
        for pulse_torque in (PULSE_TORQUE_NM, -PULSE_TORQUE_NM):
            peak, final_mean, final_peak, finite = run_candidate(wheel_sign, pulse_torque)
            score = peak + 3.0 * abs(final_mean) + final_peak
            results.append((score, wheel_sign, pulse_torque, peak, final_mean, final_peak, finite))

    results.sort(key=lambda row: row[0])
    for score, wheel_sign, pulse_torque, peak, final_mean, final_peak, finite in results:
        print(
            f"wheel_global={wheel_sign:+.0f} pulse={pulse_torque:+.1f} Nm "
            f"score={score:.3f} peak={peak:.3f} final_mean={final_mean:+.3f} "
            f"tail_peak={final_peak:.3f} finite={finite}"
        )


if __name__ == "__main__":
    main()
