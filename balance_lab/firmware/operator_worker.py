"""Real-time physical stepping independent of the operator's GPU/Tk frames."""
import copy
import threading
import time

import mujoco


class OperatorWorker:
    def __init__(self, sim, state, *, view=None):
        self.sim, self.state = sim, state
        self.lock = threading.Lock()
        self.stopping = threading.Event()
        self.snapshot_requested = threading.Event()
        self.error = None
        self.realtime = 1.0
        self.view = view if view is not None else copy.copy(sim)
        if view is None:
            self.view.data = mujoco.MjData(sim.model)
        self.snapshot()
        self.thread = threading.Thread(target=self.run, name="operator-physics", daemon=True)
        self.thread.start()

    def snapshot(self):
        """Caller holds lock once the worker is started; only the view is copied."""
        if self.view.model is self.sim.model:
            mujoco.mj_copyData(self.view.data, self.sim.model, self.sim.data)
        else:
            self.view.data.qpos[:] = self.sim.data.qpos
            self.view.data.qvel[:] = self.sim.data.qvel
            self.view.data.time = self.sim.data.time
        self.view.ticks = self.sim.ticks

    def run(self):
        deadline = measured_wall = time.monotonic()
        measured_tick = self.sim.ticks
        try:
            while not self.stopping.is_set():
                if self.state.paused:
                    deadline = measured_wall = time.monotonic()
                    measured_tick = self.sim.ticks
                    self.stopping.wait(.002)
                    continue
                now = time.monotonic()
                if now < deadline:
                    self.stopping.wait(min(.002, deadline-now))
                    continue
                # Small batches bound snapshot/reset latency. Every C and
                # MuJoCo tick still executes, in order, at the original 1 ms.
                count = min(5, int((now-deadline)/self.sim.tick_s)+1)
                with self.lock:
                    for _ in range(count):
                        if self.state.paused or self.stopping.is_set():
                            break
                        self.sim.step(self.state.command(self.sim.ticks*self.sim.tick_s))
                        deadline += self.sim.tick_s
                    if now-measured_wall >= .5:
                        self.realtime = ((self.sim.ticks-measured_tick)*self.sim.tick_s /
                                         (now-measured_wall))
                        measured_wall, measured_tick = now, self.sim.ticks
                # CPython locks are not fair. An overloaded physics loop can
                # immediately reacquire this lock for seconds while Tk waits.
                # Give a pending nonblocking UI snapshot a bounded handoff.
                if self.snapshot_requested.is_set():
                    self.stopping.wait(.003)
                # A desktop stall must not leave seconds of catch-up work.
                # Re-anchor wall time; no simulation/control ticks are skipped.
                if time.monotonic()-deadline > .1:
                    deadline = time.monotonic()
        except Exception as error:
            self.error = error
            self.state.paused = True
            self.stopping.set()

    def close(self):
        self.stopping.set()
        self.thread.join()
