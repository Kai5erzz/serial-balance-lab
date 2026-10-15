"""RM operator console. Automatically serializes GPU/physics work with heavy_run."""
import os
from pathlib import Path
import sys


def inherited_heavy_lock():
    """mujoco-gpu/heavy_run passes its held lock on descriptor 9."""
    try:
        inherited = Path(os.readlink("/proc/self/fd/9")).resolve()
    except OSError:
        return False
    directory = os.environ.get("SERIAL_BALANCE_HEAVY_LOCK_DIR")
    if directory:
        return inherited == (Path(directory)/"heavy.lock").resolve()
    return inherited.name == "heavy.lock" and "rl-serial-balance-jointpd" in inherited.parts


def main():
    guard = Path("/home/kaiser/robomaster/rl-serial-balance-jointpd/scripts/heavy_run.sh")
    gpu_launcher = Path("/home/kaiser/.local/bin/mujoco-gpu")
    if os.environ.get("RL_LAB_OPERATOR_HEAVY") != "1" and not inherited_heavy_lock() and not any(x in sys.argv for x in ("-h", "--help")):
        env = dict(os.environ, RL_LAB_OPERATOR_HEAVY="1", OPENBLAS_NUM_THREADS="1", OMP_NUM_THREADS="1")
        if gpu_launcher.is_file():
            os.execvpe(str(gpu_launcher), [str(gpu_launcher), sys.executable,
                         str(Path(__file__).resolve()), *sys.argv[1:]], env)
        env.update(MUJOCO_GL="egl", MUJOCO_EGL_DEVICE_ID="0")
        if guard.is_file():
            os.execvpe(str(guard), [str(guard), "4096", sys.executable,
                                   str(Path(__file__).resolve()), *sys.argv[1:]], env)
        portable_guard = Path(__file__).resolve().parent / "balance_lab/firmware/launch_guard.py"
        os.execvpe(sys.executable, [sys.executable, str(portable_guard), sys.executable,
                                  str(Path(__file__).resolve()), *sys.argv[1:]], env)
    os.environ.setdefault("MUJOCO_GL", "egl")
    from balance_lab.firmware.operator_console import main as console_main
    console_main()


if __name__ == "__main__":
    main()
