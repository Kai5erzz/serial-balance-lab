# HNU serial balance lab

基于 **MuJoCo** 的 RoboMaster 轮腿机器人仿真项目。将轮腿底盘与云台放入 RMUC 比赛场地，通过键盘和鼠标驾驶，用于观察平衡、转向、跳跃、磕台阶和地形接触表现。

机器人运行原生 C 控制器，MuJoCo 负责动力学与碰撞。仓库包含控制器镜像、机器人模型和 **RMUC2027 地图运行资源**，克隆后无需另行下载 CAD 或实车工程。

## 功能

- RMUC 赛场中的轮腿机器人驾驶与地形接触仿真。
- 起身、平衡、前后行驶、云台跟随和小陀螺模式。
- 腿长切换、跳跃与磕台阶动作调试。
- 第一人称、第三人称和全场俯视视角。
- 右键拖动云台；第三人称可用左键独立转动视角，观察方向不随云台旋转。
- 全屏、暂停、重置，以及可隐藏的中文键位说明。

## 地图

| 场景 | 参数 | 资源情况 |
| --- | --- | --- |
| RMUC2027 | `--variant rmuc2027` | 运行地图已随仓库提供，首次启动自动安装 |
| RMUC2026 | `--variant rmuc2026` | 保留适配代码，需另备对应运行地图 |
| RMUC2025 | `--variant rmuc2025` | 保留适配代码，需另备对应运行地图 |
| 平地 | `--variant full` | 随仓库提供，用于基础动作调试 |
| 台阶测试场 | `--variant platform` | 随仓库提供，用于地形接触调试 |

RMUC2027 场地根据官方 CAD 转换，核心尺寸 **28 × 15 m**，包含公路、起伏路段、隧道、高地、制高点和基地等设施，并在整车操控场景中添加外围围挡。

地图详情：[RMUC2027](terrain_rmuc2027/README.md) · [RMUC2026](terrain_rmuc2026/README.md) · [RMUC2025](terrain_rmuc2025/README.md)

## 安装与启动

运行环境：**Linux 桌面、Python 3.11+、NVIDIA 显卡及可用的 OpenGL/EGL 驱动**。系统还需安装 Tk、GCC 和 `patch`，中文显示建议安装 Noto Sans CJK 字体。

```bash
git clone https://github.com/Kai5erzz/serial-balance-lab.git
cd serial-balance-lab

python3 -m venv .venv
source .venv/bin/activate
python -m pip install -r requirements.txt

python run_operator.py --variant rmuc2027 --fullscreen --fps 30
```

启动后默认暂停并显示键位说明。按 **P** 开始起身和运行，按 **R** 回到出生点并暂停。

首次启动会校验并解压地图，随后编译碰撞和显示缓存，可能需要数分钟；完成后才会出现操控窗口。以后启动会复用缓存。首次编译建议预留约 **5 GiB 可用内存**。

RMUC2027 的两个资源包合计约 **52 MB**，直接随 Git 下载，无需 Git LFS。缓存默认保存到 `~/.cache/rl-lab/rmuc2027/`。可通过环境变量 `RL_LAB_RMUC2027_STORAGE` 指定其他缓存目录。

不使用全屏时，省略 `--fullscreen`：

```bash
python run_operator.py --variant rmuc2027 --fps 30
```

切换到平地调试：

```bash
python run_operator.py --variant full --fps 30
```

## 操控方式

| 按键或鼠标 | 操作 |
| --- | --- |
| W / S | 前进 / 后退 |
| A / D | 转动云台，底盘在跟随模式下跟随云台 |
| Shift / Ctrl | 全速 / 慢速 |
| 右键拖动 | 操控云台，松开释放鼠标 |
| 左键拖动 | 转动第三人称观察视角 |
| [ / ] | 降低 / 提高鼠标灵敏度 |
| E / Q | 小陀螺 / 返回跟随模式 |
| Z / X | LOW / MID 腿长 |
| C / B | 请求磕台阶；按 Z 或 X 退出 |
| Space | 请求跳跃 |
| Tab / M | 第一、第三人称切换 / 全场俯视 |
| P / R | 运行或暂停 / 重置并暂停 |
| H | 显示或隐藏全部文字，始终保留中央准星 |
| F11 / Esc | 切换全屏 / 退出全屏并暂停 |
| Backspace / Enter | 放松 / 使能 |

窗口失去焦点时自动暂停并清空输入。两轮底盘不支持横向平移；跳跃和磕台阶请求仍受控制器的就绪条件、状态与冷却限制。

## 仿真范围与限制

- **当前为静态赛场。** 活动机关固定在地图姿态，尚未实现裁判系统、能量机关交互和射击判定。
- **云台仅包含 yaw 轴控制。** 鼠标上下移动用于调整第一人称观察角度，不代表真实俯仰机构动作。
- **控制器共用，物理模型仍在验证。** 仿真结果不能直接视为实车效果或实车参数验收。
- **部分设施内部采用简化碰撞。** 道路与隧道的地形检查已记录，设施内部空隙不保证可通行，具体误差见地图说明。
- **复杂动作仍有局限。** 高速跳跃落地与台阶通行尚未完成全面验证，不能保证所有入射角、速度或 20 cm 台阶均能通过。

默认操控配置为仿真专用 `jump46`，单髋力矩上限 40 N·m、总请求上限 160 N·m。可使用 `--controller-profile source` 对照未应用该仿真补丁的控制器镜像；不同地形上的实际跳跃高度需要单独测量。

## 项目结构

| 路径 | 内容 |
| --- | --- |
| `run_operator.py` | 整车操控窗口入口 |
| `balance_lab/firmware/` | C 控制器运行接口、操控界面与记录工具 |
| `balance_lab/plant/` | 机器人与地图场景加载 |
| `firmware/` | 控制器镜像、桌面接口、校准与仿真配置 |
| `model/` | 机器人模型和基础测试场景 |
| `terrain_rmuc2025/`、`terrain_rmuc2026/`、`terrain_rmuc2027/` | 各年度地图资源与适配 |
| `tests/`、`tools/`、`docs/` | 回归检查、开发工具和技术记录 |

开发资料：[控制器集成与验证](docs/SHARED_VEHICLE_CONTROLLER.md) · [项目架构](docs/ARCHITECTURE.md) · [物理参数辨识](docs/SYSID.md) · [跳跃控制记录](docs/JUMP_CONTROL_NOTES.md)

## 地图来源

RMUC 地图由 RoboMaster 官方场地 CAD 转换。各年度的来源、版本、几何处理和验证结果见对应地图说明。RMUC2027 官方资源标注 **CC BY-NC-SA 4.0**，使用或分发时请保留来源与上游声明。
