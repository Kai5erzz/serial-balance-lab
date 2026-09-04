"""Shared leg geometry types and small vector helpers."""

from __future__ import annotations

from dataclasses import dataclass

import numpy as np


def unit(vector: np.ndarray) -> np.ndarray:
    length = float(np.linalg.norm(vector))
    if length < 1e-9:
        raise ValueError("A required controller direction is degenerate.")
    return vector / length

@dataclass
class SideIds:
    j1: int
    j2: int
    wheel_body: int
    wheel_joint: int
    wheel_motor: int
    hip_motors: tuple[int, int]
    gas_thigh_site: int
    gas_leg_site: int

@dataclass
class VmcKinematics:
    """The same four-bar variables and VMC Jacobian used by drv_leg.c."""

    l0: float
    phi0: float
    phi1: float
    phi2: float
    phi3: float
    phi4: float
    jacobian: np.ndarray

@dataclass
class RecoverySwingAxis:
    actual: float
    start_actual: float
    last_raw: float
    ref: float
    target: float
    omega_actual: float = 0.0
    omega_ref: float = 0.0
    tp_cmd: float = 0.0

    @property
    def direction(self) -> float:
        return 1.0 if self.target >= self.start_actual else -1.0
