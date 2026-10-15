# 共用实车 C 控制器的极性排查

2026-10-14 排查发现并修正了一处实车 C 的轴向错误：跳跃 roll 差分阻尼误用 `ins.gyro[0]`，该通道属于 pitch；按原始 INS 发布约定，直立附近 roll 角速度应使用 `-ins.gyro[1]`。修正已进入 code_914 工作区并同步到 RL-lab 的 C 镜像，STM32 编译通过，未烧录。

其余接口的 37 个方向用例和 4 组模型／姿态几何检查通过。这些检查验证所假定映射下的协议、局部运动和几何方向，不能证明 LQR 或 Follow 整车闭环方向正确。后续 [轮子接管与 yaw 排查](NATIVE_WHEEL_HANDOFF_AUDIT_20261014.md) 发现：两轮力矩与反馈同时反号可显著改善接管后的 pitch，但该对照的 Follow 机械角外环存在正反馈。当前默认自起身仍未通过物理站立；跳跃分支修正不参与失败基线的自起身路径。

## 发现、复现与修正

`src/task/ins/ins_task.c` 发布 `pitch=QEKF.Roll`、`roll=-QEKF.Pitch`，三个 gyro 的符号为 `[-rawX, +rawY, -rawZ]`。原始 INS 的正反转输入检查确认，raw X 对应 pitch，raw Y 对应 roll，raw Z 对应 yaw。因此直立附近有 `pitch_dot=-gyro[0]`、`roll_dot=-gyro[1]`。

`src/task/chassis/control/chassis_control.c` 的 `Leg_FN_Calculation` 原先计算：

```c
jump_roll_diff_force = JUMP_ROLL_DIFF_KP * roll_error
                     + JUMP_ROLL_DIFF_KD * ins.gyro[0];
```

直接提取生产函数内的这段 C 编译复现：roll=0、gyro=(1,0,0) rad/s、左右各 50 N 支撑力时，旧代码产生 2 N 差分，变成左 48 N、右 52 N。此时只有 pitch 角速度，预期 roll 差分应为 0；修正前回归明确失败。

修正仅将阻尼输入换成 `-ins.gyro[1]`。修正后纯 pitch 或 yaw 角速度不产生 roll 差分；gyro Y=-1 rad/s 对应正 roll 角速度，差分为 +2 N，反向为 -2 N。回归同时检查 0.5° 死区、8 N/deg 比例项、2 N·s/rad 阻尼项、±25 N 限幅、跳跃／落地门控及左右总支撑力守恒，均通过。没有调整增益、限幅、状态机判据或物理 profile。

该变化是控制行为修复。用户此前确认的车上普通 Flash Program 固件基准早于本次修正；最新工作区及移植包不能标为已经烧录的固件。

## 接口方向检查

| 范围 | 用例数 | 结果与证据 |
| --- | ---: | --- |
| 四髋 DM 正反反馈 | 8 | 原始 DM 驱动与 Page47 保留角度、速度和力矩符号 |
| 两轮 DJI 正反反馈 | 4 | 原始驱动滤波收敛后，RPM 与电流保持正反号；首拍滤波衰减符合原驱动 |
| 原始髋回调与发包 | 1 | 40 个实际轮询包与 C VMC 前后腿映射一致，最大量化误差 0.01681 N·m，小于单个力矩量化步长 0.01710 N·m |
| 原始 INS 三轴正反转 | 6 | 角度方向与发布约定一致，最大角误差约 0.00790° |
| 前向正反加速度 | 2 | 原始 C 运动加速度的前向符号正确 |
| 两轮与 yaw 力矩脉冲 | 6 | CAD 关节角、角速度和输入力矩同号 |
| 左右腿伸缩／摆动脉冲 | 8 | 正支撑力使腿伸长，正 Tp 使 phi0 增大，负向相反；C 与 CAD 的变化方向一致 |
| 原始 yaw 双环正反命令 | 2 | 正手动输入使目标 yaw 减小、CAD yaw 增大、IMU 世界 yaw 减小；负向相反，原始回中就绪成立 |

另外检查 fast／full 各模型的 stand／encoder 两个初始姿态，共 4 组。传感器坐标系行列式为 +1，两轮轴的地面前向速度符号与 C 的左负右正一致。原始 C VMC 雅可比经过实车回调中的左侧反号和前后交换后，与编码器有限差分方向一致。

四髋仍使用 jointpd 精确零点的 `E=Q-Znew`，全部不额外反号。CAD J1 对应后电机、J2 对应前电机；左腿回调反号、右腿回调同号已经由原始 C 处理，硬件接口不能再次反号。云台 `GIMBAL_YAW_DRIVE_SIGN=-1` 同样由原始 C 处理。

悬空方向实验仅在独立诊断模型中关闭重力动力学、接触、关节阻力和气弹簧外力，保留物理惯量与执行器增益。力矩脉冲为 40 ms，yaw 双环实验为 500 ms；IMU 实验规定旋转或平移并运行原始 INS。它们用于判断方向，不能代表整车站立通过。实际硬件安装方向仍需要对应的实车输入证据；当前结论覆盖 CAD、传感器接口和所运行的 C 源码。

CAD 轮心相对两髋中点的平面投影角与 C phi0 相差约 0.39–0.52°，没有 180° 镜像。两种腿长定义也存在差异：前者为三维 CAD 点的平面投影，后者为固件二维五杆定义。尚未证明二者应逐值相等，因此没有据此修改零点或将差异直接认定为实车腿长误差。

## 自起身对照与检查结果

修正后重新运行默认 fast 模型 5 s 自起身：末段 pitch 峰值 15.3476°、roll 峰值 0.9179°，双轮同时接地比例 0.4%，非轮接触峰值 1023.1781 N，站立确认比例 0%。末帧仍处于 LQR_HANDOFF，轮电流指令为 +8191/-8191。

与修正前摩擦排查的默认组相比，100 Hz 轨迹 CSV 逐字节一致，最终状态和状态切换时刻相同。指标比较只规范化此前更名的电流阈值字段，不改变数值。该结果支持“此修正没有改变这条自起身轨迹”，不能将当前自起身失败归因于已修正的跳跃 roll 轴。

14 项共用控制接口测试、6 组极性回归、实车跳跃 roll 生产代码回归与普通 roll 范围检查通过。STM32 Debug 编译成功，FLASH 290416 B、RAM_D1 242264 B；未执行烧录。reset 另外核对了物理状态与 C 静态状态：重置后的第一拍与全新实例逐项一致。

当前固件源码指纹为 `55413877777f02705311333d2d2460e510625bdd805c3dffe6621af41b8a3103`，主机 C 构建指纹为 `389cd541c7e57f65f18fc2f5b1711fb1d2f2356af00e073197a360687b9cb401`。修正前源码指纹 `220ac1563ff6d279e0717dbeaa9c08101ea3ffdb9086a0795da6b6157fbb5e0f` 的移植包另存为 `out/vehicle_controller_before_jump_roll_polarity_20261014.zip`，旧摩擦报告身份保持原样。

原始证据保存在 `out/native/polarity_audit/report.json`、`jump_roll_fix.json` 和 `recovery_after_fix/`。GPU 自起身视频由 `tools/render_native_recovery.py` 通过用户指定的 jointpd `scripts/heavy_run.sh` 串行启动；使用 NVIDIA EGL，逐帧编码，包含侧面／正面视角、恢复状态、轮电流指令与接触力。视频及逐拍输入记录放在 `out/native/recovery_video_20261014_v2/`。

## 复现

```bash
python tools/sync_firmware.py --check
python tests/test_native_firmware.py
python tests/test_native_polarity.py
python tools/probes/native_polarity_audit.py --output out/native/polarity_audit_rerun
python tools/probes/native_friction_audit.py --cases baseline \
  --output out/native/recovery_after_polarity_rerun

# 在 code_914 根目录核对实际生产代码。
python -B tests/host/test_jump_roll_polarity_production.py
cmake --build build

# 从 rl-lab 根目录生成新的、有时限的 GPU 视频；输出目录必须是新目录。
/home/kaiser/robomaster/rl-serial-balance-jointpd/scripts/heavy_run.sh 4096 \
  timeout --signal=TERM 180s env MUJOCO_GL=egl MUJOCO_EGL_DEVICE_ID=0 \
  OMP_NUM_THREADS=1 OPENBLAS_NUM_THREADS=1 python -u tools/render_native_recovery.py \
  --output out/native/recovery_video_rerun
```

局部接口方向检查完成，整车闭环的极性仍需按后续接管与 yaw 证据核对；第一阶段的物理 1:1 尚未完成。保留用户确认的高阻力底部接触条件，不通过降低摩擦掩盖方向问题。
