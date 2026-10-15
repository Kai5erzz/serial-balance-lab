"""Build original code_914 C sources against desktop hardware interfaces."""
from __future__ import annotations
import hashlib
import json
from pathlib import Path
import subprocess
import fcntl
from .schedule import contract

ROOT = Path(__file__).resolve().parents[2]
FIRMWARE = ROOT / "firmware"
VEHICLE = FIRMWARE / "vehicle"
HOST = FIRMWARE / "host"
SOURCES = [
    "src/task/chassis/chassis_task.c", "src/task/chassis/control/chassis_control.c",
    "src/task/chassis/recovery/chassis_recovery.c", "src/task/chassis/safety/chassis_safety.c",
    "src/task/chassis/debug/chassis_debug.c", "src/task/gimbal/gimbal_task.c",
    "src/task/cmd/dr16_control.c", "src/modules/leg_vmc/drv_leg.c",
    "src/modules/motor/DM_motor/DM_Motor.c", "src/modules/motor/DJI_motor/dji_motor.c",
    "src/modules/ipc/uMCN.c", "src/algorithm/PID/PID.c", "src/algorithm/kalman_filter/kalman_filter.c",
    "src/algorithm/Ramp/ramp.c", "src/algorithm/Filter/filter32.c", "src/algorithm/User_Lib/user_lib.c",
    "src/task/ins/ins_task.c", "src/algorithm/QuaternionEKF/QuaternionEKF.c", "src/modules/dm_imu/dm_imu.c",
]


def build(profile: str | None = None) -> Path:
    if profile is not None:
        from .profiles import build_profile
        return build_profile(build(), profile)
    from tools.sync_firmware import sync, inventory
    manifest = json.loads((FIRMWARE / "manifest.json").read_text())
    source = Path(manifest["source"])
    if source.is_dir():
        # The vehicle checkout is the sole controller source. Import its latest
        # worktree automatically; sync refuses to overwrite edited snapshots.
        if inventory(source) != manifest["files"]:
            manifest = sync(source)
    if inventory(VEHICLE) != manifest["files"]:
        raise RuntimeError("firmware snapshot differs from its manifest; preserve/export edits before synchronising")
    for name, expected in manifest["files"].items():
        path = VEHICLE / name
        if not path.is_file() or hashlib.sha256(path.read_bytes()).hexdigest() != expected:
            raise RuntimeError(f"firmware snapshot differs from manifest: {name}; synchronise before building")
    timing = contract(VEHICLE)
    files = sorted(p for base in (HOST, VEHICLE) for p in base.rglob("*") if p.is_file())
    fingerprint = hashlib.sha256()
    fingerprint.update(Path(__file__).read_bytes())
    fingerprint.update(json.dumps(timing, sort_keys=True).encode())
    for path in files:
        fingerprint.update(str(path.relative_to(FIRMWARE)).encode())
        fingerprint.update(path.read_bytes())
    key = fingerprint.hexdigest()
    target = ROOT / "out" / "firmware" / key / "vehicle.so"
    if target.exists():
        return target
    target.parent.mkdir(parents=True, exist_ok=True)
    # The hardware aggregate headers are substituted; all controller/driver
    # headers are the original ones. Single-precision arithmetic, no fast math/FMA.
    include = [HOST / "include", VEHICLE / "src", VEHICLE / "src/task/chassis"]
    include.extend(sorted({p.parent for p in VEHICLE.rglob("*.h")}
                          - {VEHICLE / "src/algorithm/arm_math"}))
    cmd = ["gcc", "-std=c11", "-shared", "-fPIC", "-O2", "-ffp-contract=off",
           "-DARM_MATH_MATRIX_CHECK", "-Werror=implicit-function-declaration", "-Wl,-z,defs"]
    cmd += [f"-DLAB_{name.upper()}={value}" for name, value in timing.items()]
    cmd += [f"-I{p}" for p in include]
    cmd += [str(VEHICLE / p) for p in SOURCES]
    cmd += [str(HOST / "bridge.c"), str(HOST / "memory.c")]
    cmd += [str(p) for p in sorted((HOST / "cmsis").glob("*.c"))]
    cmd += ["-Wl,--wrap=malloc", "-Wl,--wrap=free"]
    temporary = target.with_suffix(".building.so")
    cmd += ["-lm", "-o", str(temporary)]
    with (target.parent / "build.lock").open("w") as lock:
        fcntl.flock(lock, fcntl.LOCK_EX)
        if target.exists():
            return target
        result = subprocess.run(cmd, capture_output=True, text=True)
        (target.parent / "build.log").write_text(result.stdout + result.stderr)
        (target.parent / "build.json").write_text(json.dumps({"fingerprint": key, "command": cmd}, indent=2) + "\n")
        if result.returncode:
            temporary.unlink(missing_ok=True)
            raise RuntimeError(f"native firmware build failed; {target.parent / 'build.log'}\n{result.stderr[-8000:]}")
        temporary.replace(target)
    return target


if __name__ == "__main__":
    print(build())
