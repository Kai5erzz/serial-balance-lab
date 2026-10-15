"""Typed hardware boundary for the original vehicle C control tasks.

Each runtime loads its own shared object because the vehicle firmware owns
static state. No PID/gain/state-machine implementation is duplicated in Python.
"""
from __future__ import annotations

import ctypes as ct
from enum import IntEnum
import json
from pathlib import Path
import shutil
import tempfile
import _ctypes

from .build import build, FIRMWARE
from .schedule import contract

F2 = ct.c_float * 2
F3 = ct.c_float * 3
F4 = ct.c_float * 4
F5 = ct.c_float * 5


class ChassisMode(IntEnum):
    RELAX = 0
    INIT = 1
    OPEN_LOOP = 2
    RECOVERY = 3
    JUMP = 4
    FOLLOW_GIMBAL = 5
    SPIN = 6
    AUTO = 7


class LegLevel(IntEnum):
    LOW = 0
    MID = 1
    HIGH = 2


class GimbalMode(IntEnum):
    RELAX = 0
    INIT = 1
    GYRO = 2
    AUTO = 3


class Feedback(ct.Structure):
    _fields_ = [("time_us", ct.c_uint64), ("hip_angle", F4), ("hip_speed", F4), ("hip_torque", F4),
                ("wheel_angle", F2), ("wheel_speed", F2), ("wheel_torque", F2),
                ("gyro", F3), ("accel", F3), ("motion_accel", F3),
                ("roll_deg", ct.c_float), ("pitch_deg", ct.c_float), ("yaw_deg", ct.c_float),
                ("pitch_rate", ct.c_float), ("yaw_motor_angle", ct.c_float),
                ("yaw_motor_speed", ct.c_float), ("yaw_motor_torque", ct.c_float),
                ("gimbal_yaw_deg", ct.c_float), ("gimbal_yaw_rate", ct.c_float),
                ("feedback_mask", ct.c_uint32), ("raw_imu", ct.c_int32)]


class Command(ct.Structure):
    _fields_ = [("use_remote", ct.c_int32), ("chassis_mode", ct.c_int32),
                ("leg_level", ct.c_int32), ("gimbal_mode", ct.c_int32),
                ("vx_raw", ct.c_float), ("yaw_raw", ct.c_float),
                ("gimbal_yaw_raw", ct.c_float), ("gimbal_auto_deg", ct.c_float),
                ("jump_request", ct.c_int32), ("bump_request", ct.c_int32),
                ("rc_channels", ct.c_int32 * 5), ("rc_switches", ct.c_int32 * 2),
                ("rc_lost", ct.c_int32)]

    @classmethod
    def remote(cls, *, channels=(0, 0, 0, 0, 0), right_switch=1, left_switch=2, lost=False):
        """Actual DR16 mapping: right 1 relax / 3 follow / 2 spin; left 1 bump/high / 3 mid / 2 low."""
        c = cls(use_remote=1, rc_lost=int(lost))
        c.rc_channels[:] = channels
        c.rc_switches[:] = (right_switch, left_switch)
        return c


class Output(ct.Structure):
    _fields_ = [("hip_command", F5 * 4), ("wheel_current", F2), ("yaw_command", F5),
                ("length", F2), ("phi0", F2), ("theta", F2), ("support", F2),
                ("vmc", F4), ("lqr", F4), ("obs", ct.c_float * 12),
                ("yaw_relative", ct.c_float), ("yaw_target", ct.c_float),
                ("imu", ct.c_float * 9), ("motor_tx_count", ct.c_uint32 * 5)] + [
                    (name, ct.c_int32) for name in (
                        "chassis_mode", "stand_state", "jump_state", "jump_fault", "recovery_state",
                        "recovery_fault", "off_ground", "landing_phase", "gimbal_ready", "bump_state")]

    def as_dict(self) -> dict:
        def convert(x):
            return [convert(v) for v in x] if isinstance(x, ct.Array) else x
        return {name: convert(getattr(self, name)) for name, _ in self._fields_}


class VehicleRuntime:
    def __init__(self, *, profile=None):
        library = build(profile=profile) if profile is not None else build()
        self.profile_name = profile
        self.controller_profile = json.loads((library.parent / "build.json").read_text()).get("simulation_profile") if profile else None
        self.native_fingerprint = library.parent.name
        self.timing = contract(FIRMWARE / "vehicle")
        self.manifest = json.loads((FIRMWARE / "manifest.json").read_text())
        self._directory = tempfile.TemporaryDirectory(prefix="rl-lab-vehicle-")
        path = Path(self._directory.name) / "vehicle.so"
        shutil.copyfile(library, path)
        self.lib = ct.CDLL(str(path))
        self.lib.lab_close.argtypes = []
        self.lib.lab_close.restype = None
        self.lib.lab_abi_size.argtypes = [ct.c_uint32]
        self.lib.lab_abi_size.restype = ct.c_uint32
        for i, struct in enumerate((Feedback, Command, Output)):
            if self.lib.lab_abi_size(i) != ct.sizeof(struct):
                raise RuntimeError(f"ABI size mismatch: {struct.__name__}")
        self.lib.lab_init.argtypes = []
        self.lib.lab_init.restype = None
        self.lib.lab_step.argtypes = [ct.POINTER(Feedback), ct.POINTER(Command), ct.POINTER(Output)]
        self.lib.lab_step.restype = None
        self.lib.lab_debug.argtypes = [ct.c_uint32, ct.POINTER(ct.c_float)]
        self.lib.lab_debug.restype = None
        self.lib.lab_init()
        self.lib.lab_parameters.argtypes = [ct.POINTER(ct.c_float)]
        self.lib.lab_parameters.restype = None
        params = (ct.c_float * 12)()
        self.lib.lab_parameters(params)
        self.parameters = dict(zip(("wheel_radius_m", "wheel_current_per_nm", "wheel_torque_max_nm",
                                    "dm_position_min", "dm_position_max", "dm_velocity_min", "dm_velocity_max",
                                    "dm_torque_min", "dm_torque_max", "gas_force_left_n", "gas_force_right_n",
                                    "controller_support_mass_kg"), params))
        self._last_time = -1

    def step(self, feedback: Feedback, command: Command) -> Output:
        if self.lib is None:
            raise RuntimeError("firmware runtime has been closed")
        if feedback.time_us < self.timing["control_tick_us"] or feedback.time_us <= self._last_time:
            raise ValueError("firmware time must increase; construct a new runtime to reset")
        if self._last_time < 0 and feedback.time_us != self.timing["control_tick_us"]:
            raise ValueError("start at the first 1 ms tick; replay the complete startup history")
        if self._last_time >= 0 and feedback.time_us - self._last_time != self.timing["control_tick_us"]:
            raise ValueError("dispatch the 1 ms control tick explicitly; do not skip held-feedback ticks")
        out = Output()
        self.lib.lab_step(ct.byref(feedback), ct.byref(command), ct.byref(out))
        self._last_time = feedback.time_us
        return out

    def debug_page(self, page: int) -> list[float]:
        if self.lib is None:
            raise RuntimeError("firmware runtime has been closed")
        data = (ct.c_float * 64)()
        self.lib.lab_debug(page, data)
        return list(data)

    def close(self):
        if getattr(self, "lib", None) is not None:
            lib = self.lib
            self.lib = None
            lib.lab_close()
            _ctypes.dlclose(lib._handle)
            self._directory.cleanup()

    def __enter__(self):
        return self

    def __exit__(self, *exc):
        self.close()
