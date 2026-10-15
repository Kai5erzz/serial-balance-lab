# RMUC 2027 MuJoCo 静态场地

从 RoboMaster 官方 **RMUC2027_V1.0.0.stp** 转换，依据 **2026-10-15 发布的 RMUC 2027 比赛规则 V1.0.0**。场地核心尺寸 28 × 15 m，包含公路、起伏路段、隧道、高地、制高点、基地及其他设施的 CAD 外观。

场地本身是静态地形，已接入 RL-lab 的整车操控窗口。机关、风机和其他活动部件固定在 CAD 姿态；裁判逻辑、机器人控制和能量单元交互不由地图包实现。地图转换不修改现有控制器、2025/2026 场地或原始实车固件。

## 直接克隆并驾驶

地图运行资源 **已经随仓库提交**，不是指向本机磁盘的链接，也不需要 Git LFS 或 CAD 转换工具。

```bash
git clone https://github.com/Kai5erzz/serial-balance-lab.git
cd serial-balance-lab
python -m pip install -r requirements.txt
python run_operator.py --variant rmuc2027 --fullscreen --fps 30
```

Linux 桌面需要 Tk、GCC、`patch` 和 NVIDIA OpenGL/EGL 驱动；中文字体推荐 Noto Sans CJK。启动器在本机有 `mujoco-gpu` 时自动使用它，其他机器使用仓库内的独立任务锁和内存监测器，要求至少 2500 MiB 可用内存，运行中低于此值只停止自己的任务。

本机启动命令（保持独立验证锁）：

```bash
cd /home/kaiser/robomaster/rl-lab
mujoco-gpu --validation /home/kaiser/miniconda3/envs/rl-lab/bin/python run_operator.py --variant rmuc2027 --fullscreen --fps 30
```

`assets/runtime.zip` 为 39,336,187 bytes，SHA-256：`051b1b4e31ebed427a12da33847d4db3eab58af9f123a4e6746a2d824b98ec39`。另附 `assets/operator_visual.zip`（13,057,834 bytes），仅供操控窗口显示。首次启动自动校验并解压运行地图到 `~/.cache/rl-lab/rmuc2027/`；物理模型和分块显示模型在该位置生成缓存，后续直接读取。显示资源先在独立子进程编译并退出，再编译/加载完整物理场景，避免编译临时内存叠加。首次编译显示模型峰值实测约 **2.21 GiB RSS**，建议启动前预留约 **5 GiB 可用内存**；2500 MiB 是运行保护底线，并非保证所有首次编译都能成功的内存额度。原始 STEP、CAD 工具和数百 MB 的编译缓存不必进入 Git。需要其他存储盘时，在启动前设置 `RL_LAB_RMUC2027_STORAGE=/path/to/cache`。本机已有的 `pack` 目录会被复用。压缩包内的旧脚本和说明不参与自动安装，以仓库现有文档和代码为准。

默认使用仿真专用 **jump46**（单髋 40 N·m、总请求 160 N·m）、`coherent_candidate` 方向映射、未标定的 yaw 阻尼 0。`--controller-profile source` 可改用未打仿真补丁的 C 镜像。这些参数不代表实车验收。

出生点为地板 `(-6.25, -4.75)`，采样地面 Z 为 `0.012843875 m`。默认暂停、打开键位说明，并保留外围围挡。

| 按键 | 操作 |
| --- | --- |
| P / R | 运行暂停 / 重置出生点并暂停 |
| W / S | 前进 / 后退 |
| A / D | 转动云台，底盘 Follow 跟随 |
| Shift / Ctrl | 全速 / 慢速 |
| 右键拖动 | 操控云台，松开释放鼠标 |
| 左键拖动 | 第三人称绕车观察，方向不跟随云台 |
| Tab / M | 第一、第三人称切换 / 全场俯视 |
| [ / ] | 降低 / 提高鼠标灵敏度 |
| Z / X | LOW / MID 腿长 |
| C / B | 磕台阶请求；Z/X 退出 |
| Space | 跳跃请求，仍由 C 控制器判断就绪与冷却 |
| E / Q | 小陀螺 / Follow |
| H | 显示或隐藏所有文字；隐藏后保留中央准星 |
| F11 / Esc | 切换全屏 / 退出全屏并暂停 |
| Backspace / Enter | 放松 / 使能 |

说明是纯文本，默认显示。失去窗口焦点会暂停并清空输入。首次生成地图物理缓存可能需要数分钟，窗口在加载完成后出现。地图设施内部简化和台阶通行限制见下文；加载成功不等同于全场可通行。

## 使用

工作区入口：`/home/kaiser/robomaster/rl-lab/terrain_rmuc2027`。

| 文件 | 用途 |
| --- | --- |
| `pack/scene.xml` | 完整视觉与碰撞，独立加载入口 |
| `pack/scene_collision.xml` | 同一套地形碰撞，省去视觉网格 |
| `pack/scene_visual.xml` | 仅显示，供低内存 GPU 预览；不能做碰撞试验 |
| `pack/scene_bounded.xml` | 完整场景，加可选外围围挡 |
| `pack/field.xml` | 可装配进机器人 MJCF 的视觉与碰撞资源 |
| `pack/field_collision.xml` | 可装配的碰撞资源 |
| `checks/report.json` | 几何、通道、接触和机器人装配检查结果 |
| `checks/spawns.json` | 检查过的出生点及地面高度 |
| `checks/passages.json` | CAD 悬空区域与碰撞顶板/地面射线比较 |
| `preview/rmuc2027_orbit.mp4` | NVIDIA GPU 环绕视频 |

独立查看器（下列命令用于本机构建工作区）：

```bash
cd /home/kaiser/robomaster/rl-lab
./terrain_rmuc2027/run_heavy.sh view
```

**R** 重置仿真和全景镜头，其余鼠标操作遵循 MuJoCo。查看器持有共享重任务锁，关闭窗口后释放。完整地图首次从 XML 编译可能需要数分钟；看外观可直接播放视频。

只检查物理碰撞形状、减少视觉内存时，可使用 `./terrain_rmuc2027/run_heavy.sh view --scene terrain_rmuc2027/pack/scene_collision.xml`，查看器会显示碰撞组。

地图可被 `compose.py` 装配到当前机器人场景。必须根据 `checks/spawns.json` 选取出生点，机体初始 Z 取地面高度加 0.6 m；输出为一份新文件，不修改机器人源 XML。装配后的场景保留机器人执行器，但独立查看器不会自动接入机器人控制回调。

本机示例（此出生点已检查）：

```bash
./terrain_rmuc2027/run_heavy.sh compose \
  --robot model/whole_body_lqr_fast.xml \
  --output '/media/kaiser/D盘2/rl-lab-terrain/rmuc2027/my_robot_scene.xml' \
  --spawn -6.25 -4.75 0.612844
```

仓库中的可搬运包 `assets/runtime.zip`（本机原归档名 `rmuc2027_mujoco.zip`） 的 `pack/` 只用相对资源路径。解压并完整保留目录后，用 MuJoCo 3.12 打开 `pack/scene.xml` 即可，无需原始 STEP 或 CAD 转换依赖。构建/查看包装器和机器人装配脚本中的外部路径属于本机环境；它们不是独立运行包的依赖。

## 整车操控验证（2026-10-15）

在 NVIDIA RTX 5060 Laptop、MuJoCo 3.12.0 上，完整碰撞模型有 49,965 个 geom（含机器人和外围围挡）。从默认出生点执行起身、慢速 W 前进和松开操作，8,000 个真实控制 tick 无起身/跳跃故障，沿车头方向净前移 0.314 m；R 返回相同出生点。三种视角的可见性裁剪与不裁剪画面对比通过。该短程试验不代表已验证所有道路、设施或高速台阶。

完整物理加 GPU 显示的该次检查峰值约 **1.30 GiB RSS**。1920×1080 全屏显示（内部渲染 960×540）实测约 **26.9–27.1 FPS**、物理时间比 **0.992–1.001×**。默认显示说明、H 隐藏后仅剩中央准星、F11 恢复窗口、暂停和重置均通过。资源安装、启动选择、参数应用、C 接口及输入相关回归 **29 项通过**。数值是本机短程测量，不是帧率保证。

## 坐标和物理

单位为米，X 为长边、Y 为短边、Z 向上。XY 原点取 CAD 总装水平包围盒中心；Z 原点取大面积向上表面的主高度。官方地面具有倾斜，不能把所有位置的地面高度都当成零。精确旋转和平移记录在 `pack/manifest.json`。

视觉使用官方 CAD 配色、标线和文字几何；按材质分组后简化，保留原始 CAD 和未简化的独立核对网格。视觉简化不改变碰撞采样参考。

整车操控窗口另用预生成的显示 LOD：205.2 万面减少至 **167.3 万面**，保留每个连通部件及开放边界。简化候选按每个部件的包围盒偏差 3 mm 门限验收，失败部件保留原网格；这不是整个表面误差上界。完整视觉仍在 `assets/runtime.zip`，碰撞几何始终使用完整原包。预生成资源及指纹见 `operator_visual_receipt.json`，普通克隆不需要安装简化工具。显示模型还省去不可见的机器人碰撞网格；物理模型不受影响。

地形 `contype=4`、`conaffinity=3`、接触组 3；视觉不参与接触。摩擦 `1.2 0.02 0.001` 沿用 RL-lab 基线，尚未实测官方地胶。`solref=.008 1`、`solimp=.92 .99 .002` 属于仿真设置。

可选围挡内边界 28 × 15 m，顶部 2.4 m，厚度 50 mm 是建模假设，门洞未复刻。默认 `scene.xml` 保留不带额外围挡的 CAD 场景；`scene_bounded.xml` 与 `compose.py` 默认装配会添加围挡。

## 几何处理与验证

源文件为 653,443,279 bytes。流式索引后，保留原始总装关系、变换和材质，拆成 5 块 STEP，在独立子进程中顺序导出。未直接把整个 STEP 交给 CAD 内核。

凹实体按源部件处理：凸体用凸包，薄板保留孔洞，适合的闭合件使用约束四面体剖分或有资源上限的凸分解。不使用整图凸包或单值高度场。

2027 的两大片公路以 STEP 曲面壳交付，三角网格实际上闭合。它们单独提升为地形，采用向内 1 mm 的边界棱柱，保留上下表面和隧道空间；不归类为设施代理。公路碰撞网格先受控简化：所有顶点加采样面中心的双向距离上限为 1.5 mm，保持闭合性和欧拉特征，并检查体积及包围盒。这是采样表面距离检查，不是整个表面的距离上界证明。整块倾斜地板和大型复杂凹地形也保留边界；部分 CAD 三角网格有非流形接缝，其体积一致性记录在审计中。此类实体内部未全部填满，不能用于研究物体深度嵌入地形的行为。

剩余开放曲面的设施外壳采用保守凸包，过小五金件省略接触。代理设施在 manifest 和验证结果中单列，不纳入地形精度通过指标，不能据此判断设施内部可通行。所有代理误差均保留报告。

已知限制：两端设施附近的 22 个悬空间隙采样与保守外壳不一致（主要涉及实体 222、343、401 以及开放外壳）。这部分内部空间没有精确还原，未通过的采样仍保留 `passed=false`；不能据此进行设备内部穿行训练。它们与道路、隧道的地形检查分开统计。

<!-- VALIDATION_START -->
已在 MuJoCo 3.12.0 完成验证：

| 检查 | 结果 |
| --- | --- |
| 文件指纹 | 46,010 项全部一致 |
| CAD / 碰撞高度 | 1,680 点中 1,666 点属地形，最大误差 **9.337 mm**；无单侧漏检 |
| 覆盖间隙 | 6,720 个 XY 网格位置发现 96 处间隙；其中 **74 个地形间隙通过**，顶板/地面最大误差 **4.311 mm**，中点无碰撞体侵入；奇数交点拒绝数为 0 |
| 设施外壳限制 | 14 个高度样本、22 个间隙样本单列；高度偏差最大 **824.8 mm**、间隙边界偏差最大 **383.8 mm**，不代表设施内部真实空间 |
| 58 mm 半径接触探针 | 倾斜地板、公路及中央高地各一处，仿真 2 秒；局部稳定高度误差不超过 **0.321 mm**，无数值异常 |
| RL-lab 机器人 | 完整地图与 7 执行器机器人成功装配，10 个无控制回调的仿真步无数值异常；不等同于控制器通行测试 |

最终碰撞包含 49,747 个实例，视觉约 205 万三角面。检测器的本次缓存重跑内存峰值约 1.69 GiB；首次 XML 编译约需 6 分钟。编译耗时取自本机实测，不能当作其他机器的时限保证。

10 秒、1280 × 720、30 fps 的环绕视频由 NVIDIA RTX 5060 Laptop 的 EGL 渲染生成，逐帧编码，渲染进程内存峰值约 2.20 GiB。视频使用纯视觉场景，物理验证使用同一地图的完整碰撞场景。
<!-- VALIDATION_END -->

除垂直高度射线外，检查器还在 CAD 实体之间找悬空区域，从间隙内分别向上、向下测量，核对顶板和地面，并用半径 1 mm 的探针检查间隙中点是否被碰撞体侵占。此检查能发现凸包填洞，但不能证明任意机器人姿态都能通过，也不替代整车通行测试。网格的奇数次交点会被拒绝并计数，不猜测实体内外。

## 来源与复现

- [官方场地图纸发布页](https://bbs.robomaster.com/article/1970956?source=4)
- [官方 RMUC 2027 规则页](https://bbs.robomaster.com/wiki/20204847/1969149?source=7)
- STEP SHA-256：`a5ad3ede3b7674fc98f2e73b71717216383cb2b4b5dcdbe900c2038918bea8e0`
- 下载长度、HTTP 元数据和本地指纹见 `research/source_receipt.json`；HTTP ETag 为分段上传标识，没有把它当作整文件 MD5。
- 规则 PDF 指纹与页码见 `research/rules_source.json`。
- 官方开源帖标注 CC BY-NC-SA 4.0，使用官方资源时应保留来源和上游声明。官方 CAD 是参考模型，若与规则标注或现场不一致，以规则和现场为准。

本机 CAD 构建工作区及缓存位于 `/media/kaiser/D盘2/rl-lab-terrain/rmuc2027`，通过链接访问；下列复现命令需要该工作区及转换依赖。普通克隆仅加载已提交的运行地图，不依赖这个路径。

```bash
./terrain_rmuc2027/run_heavy.sh cad
./terrain_rmuc2027/run_heavy.sh build --resume --visual-only
./terrain_rmuc2027/run_heavy.sh build --resume
./terrain_rmuc2027/run_heavy.sh check --resume
./terrain_rmuc2027/run_heavy.sh preview --output '/media/kaiser/D盘2/rl-lab-terrain/rmuc2027/preview_new'
./terrain_rmuc2027/run_heavy.sh package
```

包装器调用 `rl-serial-balance-jointpd/scripts/heavy_run.sh` 的原始共享锁，不使用独立验证锁。CAD 导出地址空间上限 4.5 GiB，构建 4 GiB，凸分解子进程 2.5 GiB；渲染逐帧写入视频。构建需要本机保留的隔离 CAD/几何依赖，直接加载 `pack/` 不需要这些工具。
