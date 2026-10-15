"""Original-C jump acceptance and matched-pose polarity checks. Run via mujoco-gpu."""
from pathlib import Path
import argparse
import ctypes as ct
import json
import sys
import tempfile

import mujoco
import numpy as np

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT))
from balance_lab.firmware.operator_console import OperatorInput, OperatorRenderer, telemetry
from balance_lab.firmware.recording import Recorder, replay
from balance_lab.firmware.simulation import VehicleSimulation
from tools.probes.native_polarity_audit import VmcDiagnostic, cad_leg, suspended, pulse_steps
from tools.probes.native_friction_audit import contacts
from tools.probes.jump_flight_trace import analyze_flight, analyze_jump_yaw


def episode(leg, *, start=5.0, output=None):
    sim = VehicleSimulation(interface_mapping="coherent_candidate", yaw_damping_nms_rad=0)
    state = OperatorInput(paused=False, leg_switch=2 if leg == "low" else 3)
    fast = sim.runtime.lib.chassis_control_module_jump_fast_poll_active
    fast.argtypes, fast.restype = [], ct.c_uint8
    recorder = renderer = writer = None
    if output is not None:
        import imageio.v2 as imageio
        output.mkdir(parents=True, exist_ok=False)
        renderer = OperatorRenderer(sim)
        # The original 16-channel page39 leaves the wide scratch tail intact.
        # Read full page62 first so every recorded/replayed page has the same
        # deterministic tail even when the flight observer also reads page62.
        recorder = Recorder(output/"commands.jsonl", sim.metadata(), pages=(62,39,52,57,58,59))
        writer = imageio.get_writer(output/"jump.mp4", fps=20, codec="libx264",
            pixelformat="yuv420p", macro_block_size=2,
            ffmpeg_params=["-crf","22","-preset","fast","-threads","2","-movflags","+faststart"])
    transitions, rows, phases = [], [], set()
    max_split = max_cad_split = max_pitch = max_roll = max_hip_l1 = 0.0
    fast_polls = 0
    normal_tail_polls = 0
    previous = tuple(sim.output.motor_tx_count)
    last_status = None
    jumps = (round(start*1000), round((start+7)*1000))
    flight_rows = [[], []]
    try:
        for tick in range(round((start+15)*1000)):
            if tick in jumps:
                state.keys.discard("space")
                state.press("space", tick*sim.tick_s)
            command = state.command(tick*sim.tick_s)
            o = sim.step(command)
            if tick % 2:
                for index, jump_tick in enumerate(jumps):
                    if jump_tick-50 <= tick < jump_tick+4000:
                        p = sim.runtime.debug_page(62)
                        forces = contacts(sim)
                        frame = sim.data.xmat[sim.base].reshape(3,3)@sim.sensor_body
                        flight_rows[index].append({"t":float(sim.data.time),
                            "state":[o.jump_state, o.off_ground, o.landing_phase, int(p[5])&384],
                            "length":list(o.length),
                            "cad_length":[cad_leg(sim, side)["length_m"] for side in (0,1)],
                            "ref":p[16:18], "roll":o.imu[0],
                            "actual_roll":float(np.degrees(np.arcsin(np.clip(frame[2,0],-1,1)))),
                            "actual_yaw":float(np.degrees(np.arctan2(frame[1,1],frame[0,1]))),
                            "contact":[forces[key] for key in ("left_n","right_n","other_n")]})
            count = tuple(o.motor_tx_count)
            delta = np.array(count, dtype=np.int64)-previous
            previous = count
            if fast() and (tick+1) % 2 == 0:
                assert np.array_equal(delta, np.ones(5)), delta
                fast_polls += 1
            if tick >= jumps[-1]+5000 and (tick+1) % 2 == 0:
                assert not fast(), "jump polling must release after recovery"
                assert delta.sum() == 1, delta
                normal_tail_polls += 1
            if tick >= jumps[0]:
                split = abs(float(o.theta[0]-o.theta[1]))
                cad_split = abs(cad_leg(sim, 0)["phi0_rad"]-cad_leg(sim, 1)["phi0_rad"])
                max_split = max(max_split, split)
                max_cad_split = max(max_cad_split, cad_split)
                max_pitch = max(max_pitch, abs(o.imu[1]))
                max_roll = max(max_roll, abs(o.imu[0]))
                max_hip_l1 = max(max_hip_l1, sum(abs(p[4]) for p in o.hip_command))
                assert o.jump_fault == 0, (tick, o.jump_fault)
                assert o.chassis_mode != 3, "accepted jump must not enter recovery"
                phases.add(o.jump_state)
                status = (o.chassis_mode, o.jump_state, o.landing_phase)
                if status != last_status:
                    transitions.append({"time_s":(tick+1)*.001, "mode_jump_buffer":status})
                    last_status = status
            if tick % 1000 == 999:
                rows.append(telemetry(sim))
            if recorder is not None:
                recorder.tick(sim, command)
                if tick % 50 == 49:
                    frame = renderer.render(state, telemetry(sim))
                    writer.append_data(np.asarray(frame))
                    if tick in (5299, 5499, 6999):
                        frame.save(output/f"jump_{tick+1}.png")
        result = {"leg":leg, "jump_times_s":[t*.001 for t in jumps],
            "max_theta_split_deg":float(np.degrees(max_split)),
            "max_cad_split_deg":float(np.degrees(max_cad_split)),
            "max_pitch_deg":max_pitch, "max_roll_deg":max_roll,
            "max_hip_wire_l1_nm":max_hip_l1, "fast_polls":fast_polls,
            "normal_tail_polls":normal_tail_polls,
            "phases":sorted(phases), "transitions":transitions,
            "final":telemetry(sim), "trace_1hz":rows, "metadata":sim.metadata(),
            "flights":[analyze_flight(trace) for trace in flight_rows],
            "yaw":[analyze_jump_yaw(trace) for trace in flight_rows]}
        for flight in result["flights"]:
            assert flight["passed"], flight
        for yaw in result["yaw"]:
            assert yaw["passed"], yaw
        assert max_split < np.radians(10), result
        assert max_cad_split < np.radians(10), result
        assert phases == {0,1,2,3,4}, phases
        assert max_hip_l1 < 80.08, max_hip_l1  # 12-bit wire quantization
        # Original firmware preserves IS_JUMPOK (5) after successful landing.
        assert o.chassis_mode == 5 and o.stand_state == 5 and not o.off_ground
        assert o.landing_phase == 0 and o.jump_state == 0
        assert abs(o.imu[1]) < 2 and abs(o.imu[0]) < 2
        if recorder is not None:
            writer.close(); writer = None
            recorder.close()
            result["gpu"] = renderer.gpu
            result["replay"] = replay(output/"commands.jsonl")
            (output/"report.json").write_text(json.dumps(result, ensure_ascii=False, indent=2)+"\n")
        return result
    finally:
        if writer is not None: writer.close()
        if recorder is not None and not recorder.file.closed: recorder.close()
        if renderer is not None: renderer.close()
        sim.close()


def polarity():
    """Central +/- pulses at actual jump poses, measuring CAD geometry/gyro.

    The diagnostic disables resistance only in its own suspended clones. Its
    virtual forces come from the real C VMC; measured directions come from CAD.
    """
    result = []
    with tempfile.TemporaryDirectory() as directory:
        diag = VmcDiagnostic(directory)
        for stop in (5000,5240,5340,5650):
            sim = VehicleSimulation(interface_mapping="coherent_candidate", yaw_damping_nms_rad=0)
            state = OperatorInput(paused=False)
            try:
                for tick in range(stop):
                    if tick == 5000: state.press("space", 5)
                    sim.step(state.command(tick*.001))
                mode = sim.output.jump_state
                suspended(sim)
                sim.data.ctrl[:] = 0
                sim.data.qfrc_applied[:] = 0
                spec = mujoco.mjtState.mjSTATE_INTEGRATION
                saved = np.zeros(mujoco.mj_stateSize(sim.model, spec))
                mujoco.mj_getState(sim.model, sim.data, saved, spec)
                for channel in ("left_extension", "right_extension", "theta_sync", "wheel_pitch", "roll_force"):
                    measured = []
                    for sign in (-1,1):
                        mujoco.mj_setState(sim.model, sim.data, saved, spec)
                        mujoco.mj_forward(sim.model, sim.data)
                        for side, prefix in enumerate(("l","r")):
                            force = sign*10 if channel == ("left_extension","right_extension")[side] else 0
                            if channel == "roll_force": force = sign*10*(-1 if side == 0 else 1)
                            tp = sign*.4*(1 if side == 0 else -1) if channel == "theta_sync" else 0
                            c = diag.evaluate(sim, side, force=force, tp=tp)
                            for name, torque in zip((prefix+"_j2", prefix+"_j1"), c["torque_front_back_nm"]):
                                sim.data.ctrl[sim.actuators[name]] = (-1 if side == 0 else 1)*torque
                        if channel == "wheel_pitch":
                            sim.data.ctrl[sim.actuators["l_wheel"]] = sign*.1
                            sim.data.ctrl[sim.actuators["r_wheel"]] = -sign*.1
                        pulse_steps(sim, 20)
                        a, b = cad_leg(sim, 0), cad_leg(sim, 1)
                        frame = sim.data.xmat[sim.base].reshape(3,3)@sim.sensor_body
                        gyro = frame.T@sim._velocity(sim.base)[:3]
                        measured.append({"left_extension":a["length_m"], "right_extension":b["length_m"],
                            "theta_sync":b["phi0_rad"]-a["phi0_rad"],
                            "wheel_pitch":float(gyro[0]), "roll_force":float(-gyro[1])}[channel])
                    response = measured[1]-measured[0]
                    expected = 1 if "extension" in channel else -1
                    assert expected*response > 1e-6, (stop, channel, response)
                    result.append({"pose_time_s":stop*.001, "jump_phase":mode,
                        "channel":channel, "plus_minus_response":response, "restoring_direction":True})
            finally: sim.close()
    return result


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()
    args.output.mkdir(parents=True, exist_ok=False)
    report = {"polarity":polarity()}
    for leg in ("low","mid"):
        report[leg] = episode(leg, output=args.output/leg)
        print(json.dumps({"leg":leg, "split_deg":report[leg]["max_theta_split_deg"],
            "replay":report[leg]["replay"]}), flush=True)
    (args.output/"report.json").write_text(json.dumps(report, ensure_ascii=False, indent=2)+"\n")


if __name__ == "__main__": main()
