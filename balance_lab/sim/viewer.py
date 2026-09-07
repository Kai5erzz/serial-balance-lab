"""Entry point for the interactive MuJoCo viewer.

Run ``python run_lqr.py`` from the repository root.  Key 6 restarts the
session on the 600 mm platform terrain scene.
"""

from __future__ import annotations

import argparse
from ..paths import MODEL_FAST
from .session import run_session


def main() -> None:
    """Launch viewer sessions; key 6 restarts on the platform terrain."""
    parser = argparse.ArgumentParser(description="Interactive whole-body MuJoCo LQR viewer")
    parser.add_argument(
        "--plant",
        type=str,
        default="nominal",
        help="Plant profile to load: 'nominal' or 'identified_2026-09-07_v2' (or path to yaml)",
    )
    parser.add_argument(
        "--sysid",
        action="store_true",
        help="Shortcut to load the latest V2 identified profile (identified_2026-09-07_v2)",
    )
    args, _ = parser.parse_known_args()
    plant = "identified_2026-09-07_v2" if args.sysid else args.plant

    config: dict = {
        "model": str(MODEL_FAST),
        "z_offset": 0.0,
        "platform_spawn": False,
        "start_mode": "recovery",
        "plant": plant,
    }
    while True:
        restart = run_session(config)
        if restart is None:
            break
        config = restart


if __name__ == "__main__":
    main()
