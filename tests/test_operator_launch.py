"""A fresh clone must not require this workstation's external launch scripts."""
import os
from pathlib import Path
import sys
import unittest
from unittest.mock import patch

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
import run_operator


class ExecRequested(Exception):
    pass


class OperatorLaunchTests(unittest.TestCase):
    def test_fresh_clone_uses_bundled_guard(self):
        with patch.dict(os.environ, {}, clear=True), \
             patch.object(sys, "argv", ["run_operator.py", "--variant", "rmuc2027"]), \
             patch.object(run_operator, "inherited_heavy_lock", return_value=False), \
             patch.object(Path, "is_file", return_value=False), \
             patch.object(os, "execvpe", side_effect=ExecRequested) as execute:
            with self.assertRaises(ExecRequested):
                run_operator.main()
        executable, command, env = execute.call_args.args
        self.assertEqual(executable, sys.executable)
        self.assertEqual(command[1], str(ROOT / "balance_lab/firmware/launch_guard.py"))
        self.assertEqual(command[-2:], ["--variant", "rmuc2027"])
        self.assertEqual(env["MUJOCO_GL"], "egl")
        self.assertEqual(env["RL_LAB_OPERATOR_HEAVY"], "1")

    def test_existing_workstation_launcher_is_preserved(self):
        with patch.dict(os.environ, {}, clear=True), \
             patch.object(sys, "argv", ["run_operator.py"]), \
             patch.object(run_operator, "inherited_heavy_lock", return_value=False), \
             patch.object(Path, "is_file", return_value=True), \
             patch.object(os, "execvpe", side_effect=ExecRequested) as execute:
            with self.assertRaises(ExecRequested):
                run_operator.main()
        self.assertEqual(execute.call_args.args[0], "/home/kaiser/.local/bin/mujoco-gpu")


if __name__ == "__main__":
    unittest.main(verbosity=2)
