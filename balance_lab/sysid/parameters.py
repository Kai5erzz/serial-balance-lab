"""Identified-parameter file loader (schema v1).

The file format and the fingerprint function are kept compatible with
``serial_balance_isaac.parameters`` in the rl-serial-balance workspace, so
identified profiles produced there (``configs/model/identified_*.yaml``)
load here unchanged and keep the same fingerprint -- the fingerprint is the
cross-repository provenance key for "which measured parameter set am I
running on?".

A profile is a plain YAML mapping with these sections::

    schema_version: 1
    identity / geometry / rigid_bodies / actuators / gas_springs /
    contacts / sensors / latency / theta_compensation / fit

``geometry`` is calibration-review-only: a fit result may never change it.
"""

from __future__ import annotations

import hashlib
import json
import math
from dataclasses import dataclass
from pathlib import Path
from typing import Any

import yaml

MODEL_SCHEMA_VERSION = 1


def _required(section: str, data: dict[str, Any], key: str) -> Any:
    if key not in data:
        raise KeyError(f"model parameter section {section!r} is missing key {key!r}")
    return data[key]


def _as_number(section: str, key: str, value: Any) -> float:
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        raise ValueError(f"model parameter {section}.{key} must be a number, got {value!r}")
    number = float(value)
    if not math.isfinite(number):
        raise ValueError(f"model parameter {section}.{key} must be finite, got {value!r}")
    return number


@dataclass(frozen=True)
class ModelParameters:
    """Validated view over one identified-parameter YAML mapping."""

    data: dict[str, Any]

    @property
    def schema_version(self) -> int:
        return int(self.data["schema_version"])

    @property
    def fingerprint(self) -> str:
        """First 16 hex of sha256 over the canonical mapping; provenance key."""
        canonical = json.dumps(self.data, sort_keys=True, separators=(",", ":"))
        return hashlib.sha256(canonical.encode("utf-8")).hexdigest()[:16]

    @property
    def identity(self) -> dict[str, Any]:
        return dict(self.data.get("identity", {}))

    def section(self, name: str) -> dict[str, Any]:
        value = self.data.get(name)
        if not isinstance(value, dict):
            raise KeyError(f"missing model parameter section {name!r}")
        return value

    def number(self, section: str, key: str) -> float:
        """Required finite number; KeyError on missing key, ValueError on type."""
        return _as_number(section, key, _required(section, self.section(section), key))

    def numbers(self, section: str, key: str) -> list[float]:
        """Required sequence of finite numbers."""
        value = _required(section, self.section(section), key)
        if not isinstance(value, (list, tuple)):
            raise ValueError(
                f"model parameter {section}.{key} must be a sequence of numbers, got {value!r}"
            )
        return [_as_number(section, f"{key}[{index}]", item) for index, item in enumerate(value)]

    def validate(self) -> None:
        if self.schema_version != MODEL_SCHEMA_VERSION:
            raise ValueError(
                f"unsupported model schema {self.schema_version}; expected {MODEL_SCHEMA_VERSION}"
            )
        required = {
            "identity",
            "geometry",
            "rigid_bodies",
            "actuators",
            "gas_springs",
            "contacts",
            "sensors",
            "latency",
            "theta_compensation",
            "fit",
        }
        missing = required.difference(self.data)
        if missing:
            raise ValueError(f"missing model parameter sections: {sorted(missing)}")


def load_model_parameters(path: str | Path) -> ModelParameters:
    with Path(path).open("r", encoding="utf-8") as stream:
        raw = yaml.safe_load(stream)
    if not isinstance(raw, dict):
        raise ValueError("model parameter file must contain a mapping")
    result = ModelParameters(raw)
    result.validate()
    return result
