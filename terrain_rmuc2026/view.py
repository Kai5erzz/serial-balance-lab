"""Standalone arena viewer; R resets simulation state and the overview camera."""
from pathlib import Path
import argparse,time,threading,resource
import mujoco
import mujoco.viewer
ROOT=Path(__file__).resolve().parent
p=argparse.ArgumentParser(description=__doc__)
p.add_argument('--scene',type=Path,default=ROOT/'pack/scene.xml')
a=p.parse_args()
resource.setrlimit(resource.RLIMIT_AS,(6*1024**3,)*2)
resource.setrlimit(resource.RLIMIT_CORE,(0,0))
model=mujoco.MjModel.from_xml_path(str(a.scene));data=mujoco.MjData(model)
reset=threading.Event()
def key(k):
 if k in (ord('R'),ord('r')):reset.set()
with mujoco.viewer.launch_passive(model,data,key_callback=key) as viewer:
 def reset_view():
  mujoco.mj_resetData(model,data);mujoco.mj_forward(model,data)
  viewer.cam.lookat[:]=(0,0,.2);viewer.cam.distance=35;viewer.cam.azimuth=135;viewer.cam.elevation=-42
  viewer.opt.geomgroup[3]=0
 reset_view()
 while viewer.is_running():
  started=time.monotonic()
  if reset.is_set():
   with viewer.lock():reset_view()
   reset.clear()
  mujoco.mj_step(model,data);viewer.sync()
  time.sleep(max(0,model.opt.timestep-(time.monotonic()-started)))
