# 轮子接管与 yaw 跟随方向排查

当前证据支持两个独立的方向问题。原接口下，自起身在 LQR_HANDOFF 后失败；两轮输出力矩和反馈同时反号后，pitch 与接地明显改善。这个对照进入正常 Follow 后，又出现机械跟随角外环的正反馈。IMU yaw 角速度与其角度变化同向，没有发现 IMU 发布方向单独反号。

这些结论覆盖当前 CAD、桌面传感器约定和所运行的实车 C。实车 C、默认轮接口和物理参数均未在这批排查中修改。仅反号位置的条件用于隔离外环；补充的完整 yaw 齿轮接口候选也通过了短时正反跟随检查，但还未完成实车与整车行为验证。

## LQR 接管之前和之后

快速模型首次 handoff 在 1.181 s，完整碰撞模型在 1.201 s。原始 C 在此之前发出的两轮电流均为零。快速模型的六个 5 s 对照在首次 handoff 之前，物理 qpos/qvel 序列 SHA256 完全一致：`dd08b833f0d2b1ad9a53604eb43a1d161798d15fdffd51b773801b4dedfe29d7`。

各组保持原始零位、髋输出、IMU、气弹簧、质量、摩擦、调度、遥控和控制参数。下表只比较最后 1 s 的姿态和接地，原工具的 `physically_balanced` 门限不包含 yaw，不能把该字段等同于整车闭环通过。

| 快速模型，5 s | pitch 峰值 ° | roll 峰值 ° | 双轮同时接地比例 | 非轮接触峰值 N | 站立确认比例 |
| --- | ---: | ---: | ---: | ---: | ---: |
| 原接口 | 15.3476 | 0.9179 | 0.4% | 1023.1781 | 0% |
| 只反号轮力矩 | 31.5770 | 13.1478 | 3.9% | 1440.1410 | 44.2% |
| 只反号轮反馈 | 23.8454 | 0.4051 | 100% | 78.9498 | 100% |
| handoff 起反号力矩和反馈 | 1.2505 | 2.9017 | 98.6% | 0 | 100% |
| 全程反号力矩和反馈 | 1.2417 | 2.9128 | 98.2% | 0 | 100% |
| 两轮输出置零 | 9.9494 | 0.1163 | 100% | 63.6308 | 100% |

完整碰撞模型的全程双反号对照末段 pitch 峰值为 1.4554°、roll 为 2.8076°，双轮接地和站立确认均为 100%，非轮接触为零；原接口的 pitch/roll 峰值分别为 16.5091°/75.6999°。因此这不是仅由快速碰撞近似产生的现象。

然而，两轮反号对照在 5 s 已有约 ±130° 的云台相对角。延长到 20 s 后，pitch 仍小于 1.93°、roll 小于 4.23°、双轮接地比例为 99.7%、非轮接触为零，但 yaw 反复达到约 ±6.5 rad/s。左右轮力矩和反馈整体交换的 20 s 对照也未通过姿态与接地门限。

原始报告位于 `out/native/wheel_handoff_audit/fast/`、`full/`、`followup_fast/`。最初 fast/full CSV 的四个 LQR 列标题误将 ABI 顺序标为 Lwheel、Rwheel、Lhip、Rhip；实际数据始终是 Lwheel、Lhip、Rwheel、Rhip。旧文件保留，后续 20 s 工具已更正标题；更正说明在 `out/native/yaw_follow_audit_20261014/interpretation.json`。

## IMU 与转向内环

两轮全程反号的 GPU 8 s 记录共 8000 拍。将逐拍完整输入交给一份全新的原始 C，输出及原来记录的诊断页逐项重放一致。额外读取原始 Page50，得到跟随角、目标/实际 yaw 速度、yaw 输出、左右混控和安全状态。

原始 INS 发布 `yaw=-QEKF.Yaw`、`gyro[2]=-rawZ`。本记录 rawZ 与发布 gyroZ 的相加误差逐拍为零。1.5 s 之后，100 ms 窗口内发布 yaw 角的导数与 gyroZ 的相关系数为 0.999986，差值 RMS 为 0.0241 rad/s。角度和角速度的方向一致，剩余小差值包含姿态欧拉角与机体系角速度的耦合、姿态估计校正和采样相位。

在同一个 1.6 s、云台回中完成、Follow、站立确认的物理状态，保持髋/云台命令、气弹簧和接触不变，只给左右轮叠加 ±0.1 N·m 的 yaw 输入，积分 10 ms。按原始 C `yaw_cmd_L=yaw_cmd_R=-yaw_output` 定义输入，yaw 角加速度对虚拟 yaw 命令的增益为：

| 轮输出映射 | yaw 加速度增益 rad/s² per N·m | 方向 |
| --- | ---: | --- |
| 原接口 | −15.6131 | 正 yaw 命令产生负发布 yaw 响应 |
| 两轮力矩反号 | +15.6131 | 正 yaw 命令产生正发布 yaw 响应 |

这是同状态的局部物理方向检查，不能代表原接口完整闭环轨迹。它说明不能只看到 rawZ 的负号就修改 IMU；需要同时考察轮混控到实际转向的方向。在两轮反号条件下，内环实际 yaw 速度能够跟踪原 C 的目标，问题继续出现在机械角外环。

## Follow 外环为何加速

当前仿真中的发布机械相对角满足：

```text
relative_yaw ≈ gimbal_world_yaw − chassis_yaw
relative_yaw ≈ −CAD_yaw_motor_angle
```

用 8 s 数据解开角度跨周后，两个关系的拟合斜率分别为 1.00185 和 1.00126。云台保持世界航向时，正底盘 yaw 会使这个 relative_yaw 减小。

原 C 使用 `YAW_FOLLOW_ANGLE_SIGN=-1`、`YAW_FOLLOW_ANGLE_KP=5`，即目标速度为 `-5*relative_yaw`，并限幅、限制变化斜率。这要求的机械角方向与上述仿真关系相反：负 relative 触发正底盘转向，转向又让 relative 更负，构成正反馈。

| 物理时间 s | relative ° | 目标 yaw rad/s | IMU yaw rad/s |
| --- | ---: | ---: | ---: |
| 1.50 | −1.52 | +0.133 | +0.267 |
| 2.00 | −18.60 | +1.624 | +1.276 |
| 2.25 | −43.69 | +3.812 | +3.178 |
| 2.50 | −104.08 | +6.400 | +6.299 |

机械角穿过 ±180° 后，wrapped 误差跳到另一侧，跟随目标反向，实际速度经过斜率限制和惯性后反向。因此观察到的是反复加速和反转。20 s 的 5–10、10–15、15–20 s 窗口内，绝对速度均值分别为 3.734、3.767、3.748 rad/s，没有持续上升；各段峰值维持约 6.4–6.5 rad/s。

源码中关于实车 Page50 的注释写的是“正底盘 yaw 增大发布机械相对角”，恰好与本模型关系相反。现有实车记录不足以独立复核那次 Page50 的完整输入。当前证据首先确定 C 的期望与仿真对应关系不一致，不能单凭这个诊断就把实车增益符号改掉。

## 隔离外环的正反对照

保持两轮全程双反号、原始 IMU、C 源码、PID 参数、物理参数和启动历史一致。只在上一拍已经是 Follow 且云台回中就绪时，反号 yaw 电机位置输入；该时刻 gimbal GYRO 控制使用世界 IMU 角度和角速度，回中阶段仍使用原始真实电机位置。这种只改位置的条件用于隔离外环，电机速度和力矩没有跟着改号，明确不是可以部署的完整接口。

另一组把同一位置输入置零，使 Follow 只保留原 C 的转向阻尼。两组均运行 8 s，无数值错误；末 1 s 指标如下。

| 条件 | yaw 速度峰值 rad/s | 跟随角绝对峰值 ° | pitch 峰值 ° | roll 峰值 ° | 双轮接地 | 非轮接触 N |
| --- | ---: | ---: | ---: | ---: | ---: | ---: |
| 仅反号跟随角 | 0.005764 | 0.203239 | 0.371479 | 0.206973 | 100% | 0 |
| 跟随角置零，只验证阻尼 | 0.006411 | 0.005491 | 0.374255 | 0.195050 | 100% | 0 |

置零组的跟随角数值是人为置零后的反馈，不能称为机械对中精度。反号组的电机角真实变化收敛到小角度。两组都支持：这条自旋的主要触发来自 Follow 机械角外环的方向，而不是 IMU yaw 内环随机漂移或摩擦不足。

![yaw 与 Follow 方向对照](../out/native/yaw_follow_audit_20261014/yaw_follow_comparison.png)

报告与 100 Hz Page50 轨迹在 `out/native/yaw_follow_audit_20261014/`。两个固定航向的 NVIDIA EGL 视频在 `out/native/yaw_follow_video_20261014/spinning/` 和 `angle_neg/`，有 yaw 速度与跟随角显示，0.5 倍速、各 8 s 物理时长。视频逐帧编码，均通过 jointpd `heavy_run.sh` 串行启动；默认适配器和实车 C 不变。

## 完整 yaw 齿轮接口候选

`code_914/src/rm_config.h` 明确注明 `GEAR_RATIO=1`，电机与输出为 1:1，`GIMBAL_YAW_DRIVE_SIGN=-1` 表示齿轮传动相对原同步带反向。CAD 的 `gimbal_yaw` 关节表示输出轴；原仿真接口直接将其当成电机轴，没有表达这一层反向传动。

补充候选保持两轮全程双反号、底盘 BMI/INS 和原始 C 不变，对 yaw 全程使用一致的电机—输出轴变换：

```text
motor_angle = −CAD_output_angle
motor_speed = −CAD_output_speed
motor_torque_feedback = −CAD_output_torque
CAD_output_torque_command = −motor_torque_command
gimbal_IMU_yaw/rate = +world_heading/angular_rate
```

角度、速度、反馈力矩和输出力矩同时变换；电机侧与输出侧的功率符号一致。云台 IMU 在这个候选中按正世界 yaw 输入其原始 CAN 解码，底盘 INS 仍按原 C 发布负世界 yaw。两套 IMU 的发布约定不同，Follow 的机械角期望由整个齿轮与传感器组合确定。

该完整接口候选让正底盘发布 yaw 在云台世界航向保持时增大机械 relative，符合原 C 关于 Page50 的方向期望，因此原 `YAW_FOLLOW_ANGLE_SIGN=-1` 可以形成负反馈。没有改 Follow、PID 或状态机。

快速模型三组 8 s 试验均完成恢复，并在末 7–8 s 双轮全接地、非轮接触为零、站立确认 100%。正负命令组在 5–5.4 s 向原始 DR16 输入 yaw 通道 ±100，然后回零。

| 完整齿轮候选条件 | yaw 速度峰值 rad/s，末段 | relative 峰值 °，末段 | pitch/roll 峰值 °，末段 | 云台世界 yaw 变化 ° |
| --- | ---: | ---: | --- | ---: |
| 零输入 | 0.005726 | 0.247198 | 0.383087 / 0.211255 | 约 0 |
| 正云台命令 | 0.007535 | 0.368038 | 0.275666 / 0.228975 | −18.526 |
| 负云台命令 | 0.003552 | 0.159307 | 0.260265 / 0.205444 | +18.506 |

正负命令下底盘发布 yaw 的积分变化分别为 +0.3146/−0.3309 rad，按其发布约定换到世界方向后，与云台实际转向相同，并最终机械回中。完整证据在 `out/native/yaw_follow_coherent_20261014/report.json` 与 `command_verification.json`；对应 GPU 视频在 `out/native/yaw_follow_video_20261014/coherent_gimbal/`。

![完整齿轮候选与原跟随接口对照](../out/native/yaw_follow_coherent_20261014/yaw_follow_coherent_comparison.png)

这使问题从“可能需要反号实车 Follow PID”进一步缩小为“仿真中电机轴、齿轮输出轴和云台 IMU 的组合对应不符合 C 的期望”。实车 IMU 安装方向仍缺少独立输入证据；这三组快速模型结果还不能替代完整碰撞、行驶、其他转向模式和实车验证，因此候选尚未提升为默认接口。

## 后续与复现

下一步需要核对完整候选的实车对应关系，并继续做完整碰撞、正反向行驶、直接 yaw 和 Spin 验收。不能把单一跟随角反号留成仿真专用补丁，也不能把保持站立和正反 Follow 的短时诊断称为物理 1:1 已完成。

本批次 C 源码指纹为 `55413877777f02705311333d2d2460e510625bdd805c3dffe6621af41b8a3103`，主机 C 构建指纹为 `389cd541c7e57f65f18fc2f5b1711fb1d2f2356af00e073197a360687b9cb401`。各报告保留执行时的工具 SHA256 与原始输入身份。

```bash
# 在 rl-lab 根目录运行；输出目录必须是新目录。
/home/kaiser/robomaster/rl-serial-balance-jointpd/scripts/heavy_run.sh 4096 \
  timeout 90s env OPENBLAS_NUM_THREADS=1 OMP_NUM_THREADS=1 python -u \
  tools/probes/native_yaw_follow_audit.py --output out/native/yaw_follow_rerun

/home/kaiser/robomaster/rl-serial-balance-jointpd/scripts/heavy_run.sh 4096 \
  timeout 90s env OPENBLAS_NUM_THREADS=1 OMP_NUM_THREADS=1 python -u \
  tools/probes/native_yaw_follow_audit.py --cases coherent_gimbal --gimbal-check \
  --output out/native/yaw_follow_coherent_rerun

/home/kaiser/robomaster/rl-serial-balance-jointpd/scripts/heavy_run.sh 4096 \
  timeout --signal=TERM 180s env MUJOCO_GL=egl MUJOCO_EGL_DEVICE_ID=0 \
  OPENBLAS_NUM_THREADS=1 OMP_NUM_THREADS=1 python -u tools/render_native_recovery.py \
  --wheel-case both_neg_from_start --follow-case coherent_gimbal --seconds 8 \
  --camera-yaw fixed --output out/native/yaw_follow_video_rerun
```
