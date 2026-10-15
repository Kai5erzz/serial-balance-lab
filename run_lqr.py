"""MuJoCo trial ground using the shared vehicle C controller by default.

Use --headless for recorded trials or --controller legacy for the old viewer.
"""

from balance_lab.firmware.viewer import main

if __name__ == "__main__":
    main()
