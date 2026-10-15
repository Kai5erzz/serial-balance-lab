"""A display snapshot cannot race control stepping, pause or source reset."""
import time
import unittest

import numpy as np

from balance_lab.firmware.operator_console import OperatorInput
from balance_lab.firmware.operator_worker import OperatorWorker
from balance_lab.firmware.simulation import VehicleSimulation


class OperatorWorkerTests(unittest.TestCase):
    def test_physics_snapshot_pause_reset_and_shutdown(self):
        sim = VehicleSimulation(interface_mapping="coherent_candidate", controller_profile="jump46")
        state = OperatorInput()
        worker = OperatorWorker(sim, state)
        try:
            state.paused = False
            deadline = time.monotonic()+3
            while sim.ticks < 250 and worker.error is None and time.monotonic() < deadline:
                time.sleep(.005)
            self.assertIsNone(worker.error)
            self.assertGreaterEqual(sim.ticks, 250)
            state.paused = True
            with worker.lock:
                worker.snapshot()
                ticks = sim.ticks
                self.assertIsNot(worker.view.data, sim.data)
                np.testing.assert_array_equal(worker.view.data.qpos, sim.data.qpos)
                self.assertEqual(worker.view.ticks, ticks)
            time.sleep(.025)
            self.assertEqual(sim.ticks, ticks)
            with worker.lock:
                sim.reset("encoder")
                worker.snapshot()
                self.assertEqual(sim.ticks, 0)
                self.assertEqual(worker.view.ticks, 0)
            state.paused = False
            deadline = time.monotonic()+3
            while sim.ticks < 50 and worker.error is None and time.monotonic() < deadline:
                time.sleep(.005)
            self.assertIsNone(worker.error)
            self.assertGreaterEqual(sim.ticks, 50)
        finally:
            worker.close()
            sim.close()
        self.assertFalse(worker.thread.is_alive())
