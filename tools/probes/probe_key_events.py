"""Probe whether the passive viewer key_callback receives REPEAT/RELEASE.

Logs every callback invocation to probe_key_events.log with a monotonic
timestamp. Run it, focus the window, press/hold/release a key, then Ctrl+C
(or close the window); the log shows which GLFW actions reach Python.
"""

from __future__ import annotations

import sys
from pathlib import Path

REPO_ROOT = Path(__file__).resolve().parents[2]
if str(REPO_ROOT) not in sys.path:
    sys.path.insert(0, str(REPO_ROOT))

import time
from pathlib import Path

import mujoco
import mujoco.viewer
from mujoco import glfw

LOG = REPO_ROOT / "probe_key_events.log"
MODEL = REPO_ROOT / "model" / "whole_body_lqr_fast.xml"


def main() -> None:
    if LOG.exists():
        LOG.unlink()
    model = mujoco.MjModel.from_xml_path(str(MODEL))
    data = mujoco.MjData(model)
    start = time.perf_counter()

    def key_cb(key: int) -> None:
        with LOG.open("a", encoding="utf-8") as fh:
            fh.write(f"{time.perf_counter() - start:9.4f} key={key}\n")

    with mujoco.viewer.launch_passive(model, data, key_callback=key_cb) as viewer:
        print("window open; press/hold/release arrow keys, then close the window", flush=True)
        while viewer.is_running():
            mujoco.mj_step(model, data)
            viewer.sync()
            time.sleep(0.005)
    print("done")


if __name__ == "__main__":
    main()
