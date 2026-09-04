"""Entry point for the interactive MuJoCo viewer.

Run ``python run_lqr.py`` from the repository root.  Key 6 restarts the
session on the 600 mm platform terrain scene.
"""

from __future__ import annotations

from ..paths import MODEL_FAST
from .session import run_session


def main() -> None:
    """Launch viewer sessions; key 6 restarts on the platform terrain."""
    config: dict = {
        "model": str(MODEL_FAST),
        "z_offset": 0.0,
        "platform_spawn": False,
        "start_mode": "recovery",
    }
    while True:
        restart = run_session(config)
        if restart is None:
            break
        config = restart


if __name__ == "__main__":
    main()
