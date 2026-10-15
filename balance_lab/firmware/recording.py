"""Record/replay the hardware boundary, independent of MuJoCo."""
from __future__ import annotations
import ctypes as ct
import json
from pathlib import Path
from .runtime import Feedback, Command, Output, VehicleRuntime


def fields(struct):
    def value(x):
        return [value(v) for v in x] if isinstance(x, ct.Array) else x
    return {key: value(getattr(struct, key)) for key, _ in struct._fields_}


def restore(kind, values):
    result = kind()
    allowed = {key for key, _ in kind._fields_}
    unknown = values.keys() - allowed
    if unknown:
        raise ValueError(f"unknown {kind.__name__} fields: {sorted(unknown)}")
    for key, val in values.items():
        destination = getattr(result, key)
        if isinstance(destination, ct.Array):
            if len(val) != len(destination):
                raise ValueError(f"wrong array length for {key}")
            destination[:] = val
        else:
            setattr(result, key, val)
    return result


class Recorder:
    def __init__(self, path, metadata, pages=()):
        self.path = Path(path)
        self.path.parent.mkdir(parents=True, exist_ok=True)
        self.file = self.path.open("w")
        self.pages = tuple(pages)
        self.file.write(json.dumps({"metadata": metadata}, allow_nan=False) + "\n")

    def tick(self, simulation, command):
        row = {"feedback": fields(simulation.feedback), "command": fields(command),
               "output": fields(simulation.output)}
        if self.pages and simulation.ticks % 10 == 0:
            row["pages"] = {str(p): simulation.runtime.debug_page(p) for p in self.pages}
        self.file.write(json.dumps(row, allow_nan=False) + "\n")

    def close(self):
        self.file.close()


def replay(path, *, compare=True):
    ticks, mismatches = 0, []
    with Path(path).open() as file:
        first = json.loads(file.readline())
        profile = first.get("metadata", {}).get("controller_profile")
    with VehicleRuntime(profile=profile) as runtime, Path(path).open() as file:
        for line in file:
            row = json.loads(line)
            if "metadata" in row:
                expected = row["metadata"].get("firmware")
                if compare and expected and expected != runtime.manifest["fingerprint"]:
                    raise ValueError("recording firmware differs from the current snapshot")
                native = row["metadata"].get("native_build")
                if compare and native and native != runtime.native_fingerprint:
                    raise ValueError("recording desktop hardware/schedule build differs from the current runtime")
                continue
            output = runtime.step(restore(Feedback, row["feedback"]), restore(Command, row["command"]))
            ticks += 1
            if compare and "output" in row and fields(output) != row["output"]:
                if len(mismatches) < 10:
                    mismatches.append(int(row["feedback"]["time_us"]))
            if compare:
                for page, expected_values in row.get("pages", {}).items():
                    if runtime.debug_page(int(page)) != expected_values and len(mismatches) < 10:
                        mismatches.append({"time_us": row["feedback"]["time_us"], "page": page})
    if mismatches:
        raise AssertionError(f"firmware replay differs at {mismatches}")
    return {"ticks": ticks, "compared": compare, "matched": not mismatches}
