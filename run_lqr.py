"""Interactive whole-body MuJoCo viewer (see balance_lab.sim).

Starts paused at the calibrated encoder-zero recovery pose; press Space to
run recovery and hand off to LQR.  The full key map is in the viewer overlay
and in README.md.
"""

from balance_lab.sim.viewer import main

if __name__ == "__main__":
    main()
