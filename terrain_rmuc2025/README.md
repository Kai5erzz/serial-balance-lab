# RMUC 2025 MuJoCo 场地

采用官方 **RMUC 2025 复活赛／全国赛 V2.0.0 STEP 装配图**转换，保留 CAD 的零件位置、中央高地、隧道、道路、坡道、台阶和周围设施。主地板按官方规则的深色 PVC 调整配色；其余沿用 CAD 配色。此目录独立于 RL-lab 的控制器和默认场景。

## 场景入口

- `pack/scene.xml`：保留场地轮廓的精简 CAD 外观与物理碰撞，适合查看和渲染。
- `pack/scene_collision.xml`：相同物理碰撞，省去完整 CAD 外观，适合批量物理试验。
- `pack/scene_bounded.xml` / `pack/scene_collision_bounded.xml`：额外包含简化围挡的对应场景。
- `pack/scene_cad_reference.xml`：原始 CAD 外观与相同碰撞，供独立几何校验；完整版外观渲染占用内存较大。
- `pack/field.xml` / `pack/field_collision.xml`：用于其他 MJCF 场景的 include。
- `pack/manifest.json`：来源、坐标变换、碰撞生成参数、零件审计和资源 SHA-256。
- `preview/`：GPU 预览图和环绕视频；`checks/`：射线、落地接触和机器人组合检查结果。
- `rmuc2025_mujoco.zip`：可移植运行包，包含场景、实际引用的网格、预览、检查记录及接入脚本。原始 STEP 和转换依赖保留在本地工作区，运行包内没有这些大型重建输入。包内 MuJoCo XML 使用相对资源路径；工具脚本中的 Python / heavy_run 路径需按使用环境调整。

所有长尺寸沿 MuJoCo **X**，短尺寸沿 **Y**，高度沿 **Z**，单位为米。原点在场地水平中心，CAD 底层地板基准为 `z=0`，测试平地的可行驶贴面约在 `z=0.003`；出生高度应使用表面查询结果。官方核心场地 28×15 m，CAD 中外围安装结构略超出这个范围。可选围挡按规则的 2.4 m 高度和 28×15 m 内部矩形建模，厚度假定 50 mm，门洞未建模；为便于查看，围挡外观半透明。原始 CAD 场景保持图纸提供的边界设施。

## 接到 RL-lab

现有操作窗口可以直接选择赛场（机器人、控制器和地图源文件均无需改动）：

```bash
cd /home/kaiser/robomaster/rl-lab
/home/kaiser/.local/bin/mujoco-gpu --validation \
  /home/kaiser/miniconda3/envs/rl-lab/bin/python run_operator.py --variant rmuc2025
```

这会加载操控用轻量 CAD 外观和完整碰撞、仿真专用 46 cm 档，以及原有已识别机器人参数。
外观省略部分 30 mm 以下的装饰碎片，并按空间分块裁剪视野外的显示；原始 CAD 文件保留。
物理线程保持原来的 1 ms 控制和仿真步长，显示线程读取独立状态副本；显示模型不参与反馈、碰撞或控制。
地图保留自身材料参数；这些摩擦参数尚未按实地标定。出生点为 `(-8.25, -3.75)`，
贴地高度取 `checks/spawns.json` 的碰撞查询结果；R 重置会回到同一出生点。
首次组合约需 1–2 分钟，之后使用 `out/models/` 下的编译缓存。
P 运行/暂停，W/S 前后，Shift 满速、Ctrl 慢速，Space 跳跃；右键按住拖动云台，
Tab 切换第一/第三人称，M 切换全场俯视。全场视角只切换相机，不改变控制指令。
界面默认显示纯文本键位说明。H 隐藏全部文字、保留中央靶心，再按 H 恢复；
F11 切换全屏，Esc 暂停并退出全屏。可加 `--fullscreen` 直接全屏打开。
窗口大小可调整，显示保持比例；云台灵敏度用 [ / ] 调整。
第三人称左键按住拖动旋转视角，滚轮缩放；镜头跟随车辆位置，方向保持在世界坐标，
不随云台转动。第一人称仍跟随云台。切换视角会保留选定的第三人称方向。
可加 `--fps 30`，将画面目标提高至 30 FPS。实测 960×540 操控窗口约 27 FPS、1.00× 仿真速度，
具体帧率会随视角、运动和桌面负载变化。优化前的同分辨率画面单帧约需 36 ms；
轻量分块显示的站立检查约 12 ms，另有约 4 ms 的界面绘制。
8 s 固定输入对照中，每秒采样的物理状态与优化前完全一致。
证据见 `../out/native/rmuc_performance_20261015/`。
此启动方式沿用用户授权的独立验证锁和运行中 2500 MiB 内存保留门槛。

使用已有机器人 XML 生成新的组合场景，原文件无需改动：

```bash
/home/kaiser/miniconda3/envs/rl-lab/bin/python terrain_rmuc2025/compose.py \
  --robot model/whole_body_lqr_fast.xml \
  --output terrain_rmuc2025/my_robot_scene.xml \
  --profile collision --spawn -8 -4 0.6
```

`compose.py` 会移除 RL-lab 的 `ground` / `ground_outer` 平面，保留机器人关节、电机和控制参数，并将资源路径写为绝对路径。加 `--perimeter` 可包含上述简化围挡。它支持当前 RL-lab 直接定义资源的 XML；其他项目包含嵌套 include 时，应先处理相应路径。请先查看 `checks/spawns.json` 中经过平整度和落地检查的位置，再设置机器人出生坐标。出生高度还需考虑所用机器人的姿态和腿长。

也可直接由 Python 加载独立场地：

```python
import mujoco
model = mujoco.MjModel.from_xml_path("terrain_rmuc2025/pack/scene_collision.xml")
data = mujoco.MjData(model)
```

**上述 MuJoCo 加载、仿真和渲染操作也必须放在 heavy_run 内执行**，例如：

```bash
/home/kaiser/robomaster/rl-serial-balance-jointpd/scripts/heavy_run.sh \
  6144 /home/kaiser/miniconda3/envs/rl-lab/bin/python your_simulation.py
```

`run_heavy.sh` 已调用同一个 heavy_run，共用互斥锁。退出码 9 表示已有重任务，8 表示可用内存不足；不要绕过锁另起一个任务。

## 重建和检查

转换依赖单独安装在 `.caddeps78`（OpenCascade 7.8.1.1）和 `.caddeps`（CoACD 1.0.14、fast-simplification 0.2.0、TetGen 0.8.3），未修改 RL-lab 的 Python 环境。原始 CAD 完整版外观约 720 万三角形；默认外观约 347 万三角形，碰撞体保持相同。原始外观在 `reference_meshes` 中保留用于校验。构建及凸分解分别设置内存上限，凸分解逐件串行运行并保存检查点。

```bash
terrain_rmuc2025/run_heavy.sh cad --output terrain_rmuc2025/build/new_cad
terrain_rmuc2025/run_heavy.sh build --cad terrain_rmuc2025/build/new_cad \
  --output terrain_rmuc2025/new_pack --visual-detail cad
# 构建达到单次时限时，使用同一组路径续建：
terrain_rmuc2025/run_heavy.sh build --cad terrain_rmuc2025/build/new_cad \
  --output terrain_rmuc2025/new_pack --visual-detail cad --resume
# 完整版生成后，将它保留为参考并生成常规渲染外观：
terrain_rmuc2025/run_heavy.sh build --cad terrain_rmuc2025/build/new_cad \
  --output terrain_rmuc2025/new_pack --visual-detail compact --resume
terrain_rmuc2025/run_heavy.sh check --pack terrain_rmuc2025/pack \
  --output terrain_rmuc2025/new_checks
terrain_rmuc2025/run_heavy.sh preview --pack terrain_rmuc2025/pack \
  --output terrain_rmuc2025/new_preview
```

## 物理模型范围

MuJoCo 对单个 mesh 使用凸碰撞。本项目先按 CAD 独立实体拆分，再对凹零件做凸分解；薄平板优先使用保留孔洞的三角棱柱及相邻凸多边形合并。道路的两个复杂实体使用保留 CAD 边界和内部空腔的约束四面体分解，只在相邻块的并集为凸体时合并，并核对体积。较小零件的碰撞复杂度受到限制，不把整张地图合成一个凸包或高度场。细小紧固件省略，其余普通零件采用近似分解。

本机 MuJoCo 3.12.0 检查结果见 `checks/report.json`：

| 检查 | 结果 |
| --- | --- |
| 0.5 m 网格上的 CAD / 碰撞高度查询 | 1,680 / 1,680 点同时命中 |
| 采样高度最大绝对偏差 | 7.680 mm |
| 95% 采样高度绝对偏差 | 0.00842 mm |
| 超过 12 mm 的采样点 | 0 |
| 58 mm 半径探针，平地 / 约 0.2 m / 约 0.6 m 高台 | 全部稳定落地，接触软化造成约 0.321 mm 下沉 |
| 当前 RL-lab 机器人组合 | 成功加载，7 个执行器，10 拍被动仿真无数值异常 |

这些是离散几何查询和基础接触检查，尚未验证整车越障、平衡和控制器行为。约 29,651 个静态碰撞实例仍较复杂，进行批量试验时推荐 `scene_collision.xml`，并遵守 heavy_run 互斥及内存门槛。检查程序使用完整 CAD 参考，启动需要至少 8 GiB 可用内存。

摩擦 `1.2 0.02 0.001` 使用 RL-lab 的初始基准值，**没有用比赛场地实测数据标定**。调用方可按地板、坡道和金属表面继续分组标定。

这是官方装配图对应的静态状态；机关运动、裁判系统、伤害、灯光状态和比赛规则逻辑未实现。CAD 没有给出的贴图、照明和实际施工误差也不由模型保证。复现与重分发应遵循官方图纸使用条款。

## 机器人接入内存评估（2026-10-15）

本机训练保持运行时，以用户另行授权的 `mujoco-gpu --validation` 独立锁进行串行资源测量，
每 100 ms 检查系统可用内存，低于 2500 MiB 仅停止本次验证进程组。
这项局部验证授权不改变上方重建脚本的默认 heavy_run 行为。

960×540 NVIDIA 画面下，当前机器人平地场景峰值约 0.64 GiB；
机器人与整张碰撞地图组合约 1.32 GiB，验证期间系统可用内存最低 3331 MiB。
后者包含全部 29,651 个场地碰撞实例，显示碰撞几何；未加载完整 CAD 外观。
从源 XML 加载约 85 秒，短时被动物理每拍约 0.31 ms，GPU 每帧中位约 16.7 ms。
测量只有 200 拍被动物理和 10 帧渲染，不包含 Tk 操作窗口及整车 C 控制循环，
不能据此声称已完成实时导航、平衡或越障验收。交互版暂按 1.5–1.8 GiB 规划预算。

完整 CAD 外观地图单独渲染的既有峰值为 3.75 GiB，机器人组合尚未实测，
初步估计 4–5 GiB；计入 2500 MiB 保留量，建议启动前有 7–8 GiB 可用内存。
原始 CAD 参考与多模型几何检查的约 5.76 GiB 峰值属于另一项检查，不能当作普通操作窗口用量。
证据、组合 XML 与日志在 `../out/native/rmuc_memory_mouse_20261015/`，汇总为 `assessment.json`。

## 整车坡道接触修正（2026-10-15）

整车实测发现两处平地近似会提前顶住坡面：`guide_wheel_contact` 的隐藏平板覆盖
440×240 mm，明显超出 CAD 底部导轮的支撑范围；快速车身的轴对齐包围盒也会用其
前角代替真实车壳接触斜坡。RMUC 组合场景现改用完整机器人模型已有的 178 个车身
CAD 碰撞分块，并将辅助平板按 CAD 最低 8 mm 的导轮范围拟合为约 144×159 mm。
辅助支撑仍固定在车身上，保持原高度、厚度和高摩擦假设。质量、惯量、地图碰撞、
驱动轮、控制器、`code_914` 源码均未在本次修正中调整。普通 fast/full/platform 场景保持原有模型。

验证结果保存在 `../out/native/rmuc_access_20261015/`：

- 从地面经侧面缓坡驶上约 0.203 m 台面并停稳。
- 从 0.203 m 道路经中央缓坡驶上 0.303 m 台面，停留后倒车下坡；双轮在台面上的
  实际底部高度为 0.3025–0.3039 m。提供 `central_ramp_up_down.mp4` GPU 回放。
- 自起身、前后移动、视角裁剪及重置通过；15 项操作界面回归通过。
- 实际 960×540 操作窗口约 26 FPS、0.99988× 实时，暂停和重置通过。
- 118 个原始 C 源文件指纹一致。验证全程使用独立锁和 2500 MiB 可用内存保护。

中央高地的竖直挡边与 CAD 一致。约 21° 和 42° 的陡入口仍有真实车体接触和托底，
当前未通过整车通行验收，不能据上述缓坡结果宣称所有高地均可直接开上去。
当前简洁界面以场景为主，默认显示键位说明与一行状态，H 可隐藏全部文字。

## 来源

- [官方 2025 场地 STEP 图纸](https://bbs.robomaster.com/article/372035)，文件 `RMUC 2025复活赛&全国赛_asm.stp`。
- [官方规则手册发布页](https://www.robomaster.com/en-US/resource/pages/announcement/1774)，V2.0.0（2025-07-03）。

核验下载文件 276,996,994 字节，MD5 `a5c6caa0247feea8f734e105a178ef8e`；SHA-256 `2655d4c2caad62d35cae989b6240ce7bba0172bcb675a9af3866100ace384e1c`。原始图纸在 `research/`，转换用装配信息在 `build/cad78/assembly.json`。官方发布页面标注的版本与 STEP 内部生成日期分别保存在研究记录中。
