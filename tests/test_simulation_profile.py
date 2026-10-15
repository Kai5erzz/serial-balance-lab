"""Simulation profile isolation and record/replay identity, without vehicle edits."""
import json
from pathlib import Path
import tempfile
import unittest

from balance_lab.firmware.build import FIRMWARE, build
from balance_lab.firmware.runtime import Command, Feedback, VehicleRuntime
from balance_lab.firmware.recording import fields, replay
from tools.sync_firmware import inventory


class SimulationProfileTests(unittest.TestCase):
    def test_profile_isolated_from_vehicle_checkout_and_mirror(self):
        source = Path(json.loads((FIRMWARE / "manifest.json").read_text())["source"])
        before_source = inventory(source)
        before_mirror = inventory(FIRMWARE / "vehicle")
        with VehicleRuntime(profile="jump46") as runtime:
            self.assertEqual(runtime.parameters["dm_torque_max"], 40)
            self.assertEqual(runtime.parameters["dm_torque_min"], -40)
            self.assertEqual(runtime.controller_profile["hip_total_request_nm"], 160)
        self.assertEqual(inventory(source), before_source)
        self.assertEqual(inventory(FIRMWARE / "vehicle"), before_mirror)
        with VehicleRuntime() as original:
            self.assertIsNone(original.controller_profile)
            self.assertNotEqual(original.native_fingerprint, build(profile="jump46").parent.name)

    def test_recording_replays_its_profile_and_rejects_wrong_identity(self):
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "ticks.jsonl"
            with VehicleRuntime(profile="jump46") as runtime, path.open("w") as file:
                metadata = {"firmware": runtime.manifest["fingerprint"],
                            "native_build": runtime.native_fingerprint,
                            "controller_profile": runtime.profile_name}
                file.write(json.dumps({"metadata": metadata}) + "\n")
                for tick in range(1, 101):
                    feedback = Feedback(time_us=tick * 1000, feedback_mask=511, raw_imu=1)
                    feedback.accel[:] = (0, 0, 9.81)
                    command = Command()
                    output = runtime.step(feedback, command)
                    file.write(json.dumps({"feedback": fields(feedback), "command": fields(command),
                                           "output": fields(output)}) + "\n")
            self.assertEqual(replay(path), {"ticks": 100, "compared": True, "matched": True})
            lines = path.read_text().splitlines()
            header = json.loads(lines[0])
            header["metadata"]["controller_profile"] = None
            lines[0] = json.dumps(header)
            path.write_text("\n".join(lines) + "\n")
            with self.assertRaisesRegex(ValueError, "build differs"):
                replay(path)


if __name__ == "__main__":
    unittest.main()
