"""Stream an EGL GPU orbit video and four views of the standalone arena."""
from pathlib import Path
import argparse
import hashlib
import json
import resource
import time

import imageio.v2 as imageio
import mujoco
import numpy as np
from OpenGL import GL
from PIL import Image, ImageDraw, ImageFont

ROOT = Path(__file__).resolve().parent


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument("--pack", type=Path, default=ROOT/"pack")
    p.add_argument("--output", type=Path, default=ROOT/"preview")
    a = p.parse_args()
    if a.output.exists():
        p.error("output exists; choose a new directory")
    a.output.mkdir(parents=True)
    started = time.monotonic()
    model = mujoco.MjModel.from_xml_path(str(a.pack/"scene.xml"))
    data = mujoco.MjData(model)
    mujoco.mj_forward(model, data)
    model.vis.global_.offwidth, model.vis.global_.offheight = 1280, 720
    renderer = mujoco.Renderer(model, height=720, width=1280, max_geom=max(10000, model.ngeom+100))
    vendor = GL.glGetString(GL.GL_VENDOR).decode()
    gpu = GL.glGetString(GL.GL_RENDERER).decode()
    if "NVIDIA" not in vendor.upper():
        raise RuntimeError(f"NVIDIA EGL required: {vendor}/{gpu}")
    print("GPU", gpu, "geoms", model.ngeom, "mesh faces", model.nmeshface, flush=True)
    option = mujoco.MjvOption()
    option.geomgroup[3] = 0  # Same collision in all profiles; show source CAD.
    font = ImageFont.truetype("/usr/share/fonts/opentype/noto/NotoSansCJK-Regular.ttc", 26)
    small = ImageFont.truetype("/usr/share/fonts/opentype/noto/NotoSansCJK-Regular.ttc", 20)
    def draw_frame(azimuth, elevation, distance, lookat=(0, 0, .2), label=""):
        cam = mujoco.MjvCamera()
        cam.type = mujoco.mjtCamera.mjCAMERA_FREE
        cam.lookat[:] = lookat
        cam.distance, cam.azimuth, cam.elevation = distance, azimuth, elevation
        renderer.update_scene(data, camera=cam, scene_option=option)
        image = Image.fromarray(renderer.render())
        draw = ImageDraw.Draw(image)
        draw.rectangle((0, 0, 1280, 80), fill="#141C28")
        draw.text((20, 10), "RMUC 2025 · 官方场地 CAD → MuJoCo", font=font, fill="#E5EBF4")
        draw.text((20, 48), "复活赛／全国赛 V2.0.0 | 28 × 15 m | 静态地形"+ (" | "+label if label else ""), font=small, fill="#9BB3CB")
        return np.asarray(image)
    snapshots = (("overview", 135, -42, 35, (0, 0, .2), "整体"),
                 ("top", 90, -89.8, 26.5, (0, 0, .2), "俯视"),
                 ("central", 140, -32, 13, (0, 0, .3), "中央高地与隧道"),
                 ("road", 90, -25, 14, (5, 3, .3), "道路与飞坡"))
    try:
        for name, az, el, dist, at, label in snapshots:
            Image.fromarray(draw_frame(az, el, dist, at, label)).save(a.output/(name+".png"))
        video = a.output/"rmuc2025_orbit.mp4"
        with imageio.get_writer(video, fps=30, codec="libx264", pixelformat="yuv420p", macro_block_size=8,
                                ffmpeg_params=["-crf", "21", "-preset", "fast", "-threads", "2", "-movflags", "+faststart"]) as writer:
            for i in range(300):
                writer.append_data(draw_frame(135+i*.65, -38, 35))
                if i % 60 == 0:
                    print("rendered", i, "/ 300 frames", flush=True)
                    available = int(next(x.split()[1] for x in Path("/proc/meminfo").read_text().splitlines() if x.startswith("MemAvailable:")))/1024
                    if available < 2048:
                        raise RuntimeError("RAM guard reached")
        report = {"completed": True, "opengl_vendor": vendor, "opengl_renderer": gpu,
                  "ngeom": model.ngeom, "nmeshface": model.nmeshface, "video_frames": 300,
                  "video_seconds": 10, "video_sha256": hashlib.sha256(video.read_bytes()).hexdigest(),
                  "wall_seconds": time.monotonic()-started,
                  "peak_rss_mb": resource.getrusage(resource.RUSAGE_SELF).ru_maxrss/1024}
        (a.output/"render_report.json").write_text(json.dumps(report, indent=2)+"\n")
        print(video, report, flush=True)
    finally:
        renderer.close()


if __name__ == "__main__":
    main()
