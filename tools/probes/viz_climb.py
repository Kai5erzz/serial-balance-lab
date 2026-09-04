

import sys
from pathlib import Path

REPO_ROOT = Path(__file__).resolve().parents[2]
if str(REPO_ROOT) not in sys.path:
    sys.path.insert(0, str(REPO_ROOT))

import mujoco, numpy as np, sys
from pathlib import Path
from PIL import Image, ImageEnhance
import traditional_lqr as TL

model = mujoco.MjModel.from_xml_path("model/whole_body_lqr_fast_platform.xml")
data = mujoco.MjData(model)
c = TL.TraditionalLQR(model)
TL.apply_saved_pose(model, data, Path(TL.CALIBRATED_STAND_POSE_PATH))
mujoco.mj_forward(model, data)
c.reset(data, calibrate_frame=True)
h, _ = c._heading(data)
data.qpos[0] += float(h[0]) * 2.6
data.qpos[1] += float(h[1]) * 2.6
mujoco.mj_forward(model, data)
c.enable_calibrated_lqr(data)
c.disable_descent_guard = True
c.disable_danger_check = True
end = data.time + 2.0
while data.time < end:
    c.step(data, model.opt.timestep); mujoco.mj_step(model, data)

renderer = mujoco.Renderer(model, height=360, width=480)
cam = mujoco.MjvCamera(); cam.type = mujoco.mjtCamera.mjCAMERA_FREE
cam.distance = 3.4; cam.elevation = -12.0; cam.azimuth = 100.0
frames = []
def snap():
    p = data.xpos[c.base]
    cam.lookat[:] = (float(p[0]), float(p[1]), float(p[2]) + 0.1)
    renderer.update_scene(data, camera=cam)
    frames.append(ImageEnhance.Brightness(
        Image.fromarray(renderer.render())).enhance(1.6))

next_t = data.time
def play(sec, drive=0.0, redrive=None):
    global next_t
    end = data.time + sec
    while data.time < end:
        c.step(data, model.opt.timestep); mujoco.mj_step(model, data)
        if redrive is not None and c.jump_state in ("preload", "thrust"):
            c.set_drive_command(redrive, 0.0)
        while data.time >= next_t:
            snap(); next_t += 0.04

play(1.5)
c.set_drive_command(0.30, 0.0)
while data.time < 12.0:
    c.step(data, model.opt.timestep); mujoco.mj_step(model, data)
    d = float(np.hypot(data.xpos[c.base][0], data.xpos[c.base][1]))
    if d <= 2.05:
        break
ok, reason = c.begin_jump(data)
print("jump:", ok, reason, f"vx={c.chassis_vx:+.2f}", flush=True)
play(1.2, redrive=0.30)
c.stop_drive()
play(2.0)
play(1.5, drive=0.18)
c.stop_drive()
play(2.0)
renderer.close()

lvl = {"l": 0.0, "r": 0.0}
for i in range(data.ncon):
    con = data.contact[i]
    names = [mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_GEOM, int(g)) for g in (con.geom1, con.geom2)]
    for g in (con.geom1, con.geom2):
        bid = int(model.geom_bodyid[int(g)])
        for side in ("l", "r"):
            if bid == c.sides[side].wheel_body:
                for nm in names:
                    if nm == "platform_step2": lvl[side] = max(lvl[side], .2)
                    elif nm == "platform_step1": lvl[side] = max(lvl[side], .4)
                    elif nm == "platform_top": lvl[side] = max(lvl[side], .6)
print(f"FINAL support={lvl} pitch={np.degrees(c.body_pitch(data)):+.2f} "
      f"z={data.xpos[c.base][2]:.3f}", flush=True)
Path("out").mkdir(exist_ok=True)
frames[0].save("out/climb_tier1.gif", save_all=True,
               append_images=frames[1:], duration=40, loop=0)
print("wrote out/climb_tier1.gif", len(frames), "frames", flush=True)
