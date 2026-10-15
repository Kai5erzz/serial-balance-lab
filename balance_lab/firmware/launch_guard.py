"""Portable Linux operator lock and available-memory guard."""
from pathlib import Path
import fcntl
import os
import signal
import subprocess
import sys
import time

FLOOR_MIB = 2500


def available_mib():
    for line in Path("/proc/meminfo").read_text().splitlines():
        if line.startswith("MemAvailable:"):
            return int(line.split()[1]) // 1024
    raise RuntimeError("Linux MemAvailable is unavailable")


def main():
    command = sys.argv[1:]
    if not command:
        raise SystemExit("Expected an operator command")
    directory = Path(os.environ.get("XDG_CACHE_HOME", str(Path.home() / ".cache"))) / "rl-lab"
    directory.mkdir(parents=True, exist_ok=True)
    with (directory / "operator.lock").open("w") as lock:
        try:
            fcntl.flock(lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
        except BlockingIOError:
            raise SystemExit("Another rl-lab operator is running; close it before starting this one")
        if available_mib() < FLOOR_MIB:
            raise SystemExit(f"Operator requires at least {FLOOR_MIB} MiB available RAM")
        env = dict(os.environ, RL_LAB_OPERATOR_HEAVY="1", OPENBLAS_NUM_THREADS="1", OMP_NUM_THREADS="1")
        env.setdefault("MUJOCO_GL", "egl")
        child = subprocess.Popen(command, env=env, start_new_session=True, pass_fds=(lock.fileno(),))

        def stop(signum=signal.SIGTERM, _frame=None):
            if child.poll() is None:
                try:
                    os.killpg(child.pid, signum)
                except ProcessLookupError:
                    pass

        signal.signal(signal.SIGTERM, stop)
        signal.signal(signal.SIGINT, stop)
        try:
            while child.poll() is None:
                if available_mib() < FLOOR_MIB:
                    print(f"Available RAM below {FLOOR_MIB} MiB; stopping this operator", file=sys.stderr)
                    stop()
                    break
                time.sleep(.1)
            try:
                return child.wait(timeout=5)
            except subprocess.TimeoutExpired:
                stop(signal.SIGKILL)
                return child.wait()
        finally:
            if child.poll() is None:
                stop(signal.SIGKILL)
                child.wait()


if __name__ == "__main__":
    raise SystemExit(main())
