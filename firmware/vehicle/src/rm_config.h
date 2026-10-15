 /*
 * Change Logs:
 * Date            Author          Notes
 * 2023-08-23      ChuShicheng     first version
 * 2026-03-08      SuperChen       second version
 */
#ifndef _RM_CONFIG_H
#define _RM_CONFIG_H

#define CPU_FREQUENCY 168     /* CPU主频(mHZ) */


#define BSP_CHASSIS_LEG_MODE
/* --------------------------- 调试输出模式选择 --------------------------- */
/* 开关配置位于 sysview_switch.h 中，支持 VOFA / OCTO / SYSTEMVIEW 自由切换 */
#include "sysview_switch.h"

/* VOFA 调试页：3=机械Follow与Spin无扰切换，8=倒置/身下 recovery，
 * 12=theta 腿长标定，14=双侧腿长 PID，
 * 15=横滚 PID，16=LQR 单位与输出，17=腿长切换振荡诊断，
 * 18=recovery 连续性诊断，19=左腿振荡与差动力诊断，
 * 28=摆腿差模阻尼，29=轮端混控，30=整车 Roll 增益调度，
 * 31=转向卡滞与局部轮端 LQR 回注，32=髋关节差模与转向对称性，
 * 33=Roll 静差、高腿劈叉保护与中腿颤抖，
 * 34=腿长切换与 7~8 Hz 颤抖，35=theta 补偿与零指令漂移，
 * 36=停车状态机与 theta 静态学习，37=低腿残差隔离与腿长切换，
 * 38=腿长卡滞触发与 PID 验证，39=分档速度、yaw、roll 与腿长联调，
 * 40=OPEN_LOOP yaw-Roll 颤抖与静态 trim 余量验证，
 * 41=方向 yaw 补偿、Roll 劈叉保护与左腿长度环复核，
 * 42=动态支持力前馈输入、微分、修正量与最终输出，
 * 43=静态坡度腿差、动态离心推力与中腿差模联调。 */
/* Gimbal diagnostics: 44=communication health, 45=pitch chain, 46=yaw chain,
 * 49=combined pitch/yaw tuning chain (32 channels, safe-rate output),
 * 50=Follow Gimbal chassis chain (32 channels),
 * 51=left-front hip shake control/motor chain (32 channels),
 * 52=conservative jump state/leg/airborne/landing chain (32 channels),
 * 53=Spin 入口腿长档位锁存、固定 yaw 速度与变腿长切换链 (32 channels),
 * 54=BUMP 磕台阶专用诊断页 (32 channels)：I2=bump_state, I3=触发请求,
 *    I4=active, I6=off_ground, I8/I9=腿长, I10/I11=theta, I12/I13=轮实测力矩,
 *    I16/I17=hip_tp/target_length, I18-I21=接触锁存与计时, I22=状态已持续时间。
 *    移植自 code_611；该页在 611 被 runtime 页遮蔽（死代码），914 无遮蔽，可正常使用。
 * 64=离地检测源/位置rate/计时状态 (32 channels)：I2=flags，I4=最大髋反馈age，
 *    I5=四髋source dt，I6/I7=legacy dL，I8-I15=实测速率/加速度，
 *    I16/I17=检测Fn，I20/I21=最终VMC轴向命令，I22/I23=最后轮回调电流指令码。
 * 57=jump state/leg/stroke-release diagnostics v4 (32 channels).
 * 58=jump local timestamp-rate feedback diagnostics v7 (32 channels).
 * Chassis diagnostics: 47=four DM8009P + two M3508 execution/health chain
 * (32 channels), 48=recovery state-machine transition capture (16 channels). */
#define CHASSIS_DEBUG_PAGE 54U

#ifndef JUMP_LOCAL_RATE_FEEDBACK_ENABLE
#define JUMP_LOCAL_RATE_FEEDBACK_ENABLE 1U
#endif

/* 使用带真实时间戳的滑动微分器生成正常控制车速；不改变 recovery 导数。 */
#define CHASSIS_KINEMATIC_RATE_OBSERVER_ENABLE 1

#if (CHASSIS_DEBUG_USE_VOFA + CHASSIS_DEBUG_USE_OCTO + CHASSIS_DEBUG_USE_SYSTEMVIEW) != 1
#error "Select exactly one chassis debug output: VOFA, OCTO, or SYSTEMVIEW in sysview_switch.h"
#endif

#include "stm32h723xx.h" // 使用的芯片
#include "cmsis_os.h" // 使用的 OS 头文件
#ifdef _CMSIS_OS_H
#define user_malloc pvPortMalloc
#define user_free vPortFree
#else
#define user_malloc malloc
#define user_malloc free
#endif


/* 底盘和云台分别对应的 can 总线 */
#define CAN_GIMBAL     hfdcan2
#define CAN_WHEEL_MOTOR hfdcan2
#define CAN_ID_WHEEL_MOTOR 2
#define CAN_ID_GIMBAL_MOTOR 2
#define CAN_ID_CHASSIS_MOTOR 1
#define CAN_ID_FIRE_MOTOR 2


/* ---------------------------------- 遥控器相关 --------------------------------- */
#define RC_DBUS_MAX_VALUE      660.0f  /* DBUS遥控器通道最大值 */
#define RC_RATIO          0.0009f
#define KB_RATIO          0.010f

 /* 鼠标键盘模式下的底盘最大速度限制 */
/* 底盘平移速度 */
#define CHASSIS_PC_MOVE_RATIO_X 1.0f
/* 底盘前进速度 */
#define CHASSIS_PC_MOVE_RATIO_Y 1.0f
/* 底盘旋转速度，只在底盘开环模式下使用 */
#define CHASSIS_PC_MOVE_RATIO_R 3.5f//5.0f

/* 鼠标键盘模式下的云台速度限制 */
/* 云台pitch轴速度 */
#define GIMBAL_PC_MOVE_RATIO_PIT 0.1f
/* 云台yaw轴速度 */
#define GIMBAL_PC_MOVE_RATIO_YAW 0.25f


/* ---------------------------------- 底盘相关 ---------------------------------- */
/*chassis_can_allocation*/
#define CHASSIS_JOINT_LEFT_FRONT     4
#define CHASSIS_JOINT_RIGHT_FRONT    3
#define CHASSIS_JOINT_LEFT_BACK      1
#define CHASSIS_JOINT_RIGHT_BACK     2
#define CHASSIS_JOINT_AND_YAW_RX_ID  0X10

#define CHASSIS_WHEEL_LEFT_ID    0X201
#define CHASSIS_WHEEL_RIGHT_ID   0X202

/*没有装导轮,底盘与地面直接接触的腿部摆杆角度*/
///*腿部角度phi1和phi2计算时涉及*/
//#define DM_ZERO_OFFSET_LF  62.17f   /*对应phi1*/
//#define DM_ZERO_OFFSET_LB  35.22f   /*对应phi2*/
//#define DM_ZERO_OFFSET_RF  62.17f
//#define DM_ZERO_OFFSET_RB  35.22f

/*装导轮,导轮与地面直接接触的腿部摆杆角度*/
/*腿部角度phi1和phi2计算时涉及*/
#define DM_ZERO_OFFSET_LF  67.19f   /*对应phi1*/
#define DM_ZERO_OFFSET_LB  30.20f   /*对应phi2*/
#define DM_ZERO_OFFSET_RF  67.19f
#define DM_ZERO_OFFSET_RB  30.20f

#define LEG_SAFE_AREA      20.0f  /*能接受的腿部theta角运动范围扩展角度*/

/*查3508资料和xroll减速箱淘宝详情得到*/
#define M3508_TOR_TO_CUR  2220  //扭矩电流系数
#define M3508_TOR_MAX  3.69  //堵转扭矩
#define M3508_READUCTION_RATIO_R   15.7647        //15.77/1.12 = 14.08  //268/17 貌似淘宝店给的参数不对  //右轮3508改装xroll减速箱减速比
#define M3508_READUCTION_RATIO_L   15.7647        //26.34/2.1 = 12.543   //268/17   //右轮3508改装xroll减速箱减速比

/*底盘机体相关*/
#define WHEEL_RADIUS  0.058f      //轮子半径/m
#define m_b_whole  12.801f //装上云台后的重量
#define m_b  6.120f
#define g  9.81f
/* Complete robot with gimbal. The LQR coefficients use the same physical profile. */
#define CHASSIS_SUPPORT_MASS m_b_whole
/* Ruled physical support mass (2026-09-21: 66.6718 N/leg), from the
 * rl-lab sim alignment.  NOT APPLIED since 2026-09-22: the +/-7.5 N
 * feedforward step at jump entry/exit perturbed the length loop on
 * hardware; standing feedforward stays on CHASSIS_SUPPORT_MASS until
 * the step is measured acceptable on VOFA (I22/I23 vs I6/I7). */
#define VMC_SUPPORT_MASS 13.592626f /* NOT APPLIED since 2026-09-22 revert: see Leg_FN_Calculation */
#define Rl 0.214f    //轮间距



/* 未安装云台时的标定初值，仅保留作历史对照。 */
#define THETA_COMP_CAL_INIT_LOW_LEGACY_NO_GIMBAL  0.0551f
#define THETA_COMP_CAL_INIT_MID_LEGACY_NO_GIMBAL -0.000195f
#define THETA_COMP_CAL_INIT_HIG_LEGACY_NO_GIMBAL -0.0003016f

/* 安装云台后的新拟合完成前，继续以旧值作为安全起点，由慢速积分器修正。 */
/* Gimbal calibration warm starts. LOW includes the Page 37 residual;
 * MID/HIG use the current polynomial near this group's target lengths. */
#define THETA_COMP_CAL_INIT_LOW   +0.0290f
#define THETA_COMP_CAL_INIT_MID   +0.0078f
#define THETA_COMP_CAL_INIT_HIG   +0.0280f


#define THETA_COMP_CAL_LEARN_LIMIT 0.2000f

/* Current three-point theta calibration targets. The fitted data always use
 * measured average length from VOFA rather than these requested values. */
#define THETA_COMP_CAL_TARGET_LOW  0.17f
#define THETA_COMP_CAL_TARGET_MID  0.24f
#define THETA_COMP_CAL_TARGET_HIG  0.30f

/* 腿长相关 theta 补偿的标定流程：
 * 1：使用 LOW/MID/HIG 三个写死样本学习，积分器限幅较大。
 * 0：使用下方拟合多项式作为基值，并保留小范围慢速积分残差。
 * 只有 THETA_COMP_POLY_* 已替换为拟合结果后，才允许切换为 0。 */

/* 未安装云台时的历史残余补偿系数，仅保留作对照，禁止与云台数据混用。 */
#define THETA_COMP_POLY_C0_LEGACY_NO_GIMBAL +0.015176364f
#define THETA_COMP_POLY_C1_LEGACY_NO_GIMBAL -0.029052680f
#define THETA_COMP_POLY_C2_LEGACY_NO_GIMBAL +0.010519117f

/* 加装云台后的 Page 12 标定结果：
 * - 39 个经复核采样点；拟合横坐标使用实际平均腿长 I2；
 * - 有效采样范围 0.130958~0.298922 m，稳健残差尺度 0.011516 rad；
 * - 多项式只表示 DX_OFFSET=0.0182 m 几何项之外的残余补偿；
 * - 慢速积分器继续修正装配、地面和电池状态带来的小残差。 */
#define THETA_LENGTH_CALIBRATION_ENABLE 0
#define THETA_COMP_POLY_L0_MIN    0.1300000f
#define THETA_COMP_POLY_L0_MAX    0.3000000f
#define THETA_COMP_POLY_L0_CENTER 0.2150000f
#define THETA_COMP_POLY_L0_SCALE  0.0850000f
#define THETA_COMP_POLY_C0        +0.010312509f
#define THETA_COMP_POLY_C1        -0.009055911f
#define THETA_COMP_POLY_C2        +0.001545024f
#define THETA_COMP_POLY_TRIM_LIMIT 0.0300000f

/* Off-ground detection remains a two-leg AND decision.  The reconstructed
 * contact-force bias grows near the high-leg singular geometry, so only the
 * threshold and confirmation time are scheduled by the commanded leg level. */
#define OFFGROUND_DETECTION_ENABLE 1
#define OFFGROUND_TAKEOFF_FN_LOW        30.0f
#define OFFGROUND_TAKEOFF_FN_MID        30.0f
#define OFFGROUND_TAKEOFF_FN_HIGH       40.0f
/* 2026-09-23 (jump11.csv forensics): the jump takeoff decision never fired in
 * ANY measured jump.  The reconstructed contact force is honest while the
 * boost window is open (measured 225-230 N mean, matching the 120 N/leg floor
 * plus the 70 N boost), but after the boost window closes the estimator reads
 * the gas spring, and at 0.30-0.36 m extension that spring alone is ~88-103 N
 * per leg (F_Spring 300 N x 0.199 m/0.155 m moment arm).  The estimator
 * therefore reported ~100 N mean through the whole pre-tuck instead of falling
 * through the old 45 N gate, so `chassis_off_ground` stayed 0 for the entire
 * flight: the wheels kept full ground-LQR authority (page 56 recorded +-8191
 * counter-rotating commands in mid-air), the airborne reaction-wheel channel
 * never engaged, and the hip common axis never switched to the air law.
 *
 * These gates only apply once the legs have already passed 0.26 m (see
 * Offground_EnterCondition), so they discriminate "legs extended but still
 * loaded" from "legs extended and unloaded".  Measured: grounded extension
 * 190-230 N mean / 178-250 N per side; unloaded pre-tuck 92-106 N mean /
 * 80-115 N per side; confirmed flight 0-55 N mean.  70/90 sits between the
 * unloaded and loaded populations with margin on both sides. */
#define OFFGROUND_JUMP_TAKEOFF_SIDE_MAX 90.0f
#define OFFGROUND_JUMP_TAKEOFF_MEAN_MAX 70.0f
#define OFFGROUND_JUMP_TAKEOFF_CONFIRM   3U
#define OFFGROUND_TAKEOFF_CONFIRM_LOW    5U
#define OFFGROUND_TAKEOFF_CONFIRM_MID    5U
#define OFFGROUND_TAKEOFF_CONFIRM_HIGH  15U
#define OFFGROUND_LANDING_RELEASE_MARGIN_N            5.0f
#define OFFGROUND_CONFIRM_DROPOUT_MS                  3.0f
#define OFFGROUND_CONFIRM_WINDOW_MS                  30.0f
#define OFFGROUND_CONFIRM_MAX_SAMPLE_DT_MS            3.0f
#define OFFGROUND_POST_LANDING_REARM_WINDOW_MS       30.0f
#define OFFGROUND_POST_LANDING_WEAK_CONFIRM_MS       12.0f
#define OFFGROUND_POST_LANDING_STRONG_FN_MAX_N       10.0f

/* Landing still requires both wheels to carry real load. */
#define OFFGROUND_LANDING_FN_LOW        45.0f
#define OFFGROUND_LANDING_FN_MID        45.0f
#define OFFGROUND_LANDING_FN_HIGH       50.0f
#define OFFGROUND_LANDING_CONFIRM_LOW    8U
#define OFFGROUND_LANDING_CONFIRM_MID   10U
#define OFFGROUND_LANDING_CONFIRM_HIGH  12U

/* Landing gear and buffer:
 * LOW deploys to 0.27 m after takeoff so impact has real compression travel.
 * On contact LOW uses a downward-only ratchet reference: compression keeps a
 * small braking margin, but rebound cannot restore the captured high target.
 * It then returns to the stored operator target; MID/HIGH keep the proven
 * 0.21 m absorption target. */
#define LANDING_AIR_DEPLOY_LENGTH        0.27f
#define LANDING_AIR_DEPLOY_REF_RATE      0.55f
#define LANDING_AIR_DEPLOY_KP           350.0f
#define LANDING_AIR_DEPLOY_KD            20.0f
#define LANDING_AIR_DEPLOY_FORCE_LIMIT   60.0f
/* jump105 compact profile: retract toward 0.13 m at 1.8 m/s, then require
 * measured two-leg compactness and 80 ms of continuous hold. The 0.25 s cap
 * starts the deploy reference by that elapsed time if actual tracking fails. */
#define JUMP_AIR_TUCK_LENGTH              0.13f
/* 2026-09-23: airborne reaction wheel RE-ENABLED.
 *
 * It was disabled on the theory that two M3508 rotors at the 3.69 Nm limit are
 * worth only ~1.7 deg of body pitch rate and cannot hold attitude.  That
 * conclusion was drawn while the leg-angle (theta) loop had an INVERTED sign,
 * i.e. while the legs were being actively driven round; the wheel was losing a
 * fight it should never have been in.  With the sign fixed, the 1:1 simulation
 * (/home/kaiser/robomaster/rl-lab) measures the opposite:
 *
 *   theta +80/+12, wheel ON  -> max|theta| 0.123 rad, max|pitch|  7.1 deg
 *   theta +80/+12, wheel OFF -> max|theta| 0.123 rad, max|pitch| 29.8 deg
 *
 * The theta loop holds the LEG ANGLE; it cannot control absolute body pitch.
 * The wheel is what takes pitch from 30 deg down to 7 deg, and 30 deg sits only
 * 5 deg below DANGER_SOFT_PITCH_DEG (35) and 10 deg below the jump abort gate
 * (40), so leaving it off would keep the abort guard on a hair trigger.  Both
 * channels are needed: theta holds the rods, the wheel holds the body.
 * The landing-buffer wheel damper (JUMP_LANDING_WHEEL_ENABLE) is separate. */
#define JUMP_AIR_REACTION_WHEEL_ENABLE       1U
#define JUMP_AIR_TUCK_TIME                0.18f /* retained by the fixed 0.22 s touch/impact inhibit */
#define JUMP_AIR_TUCK_HOLD_TIME           0.04f /* keeps the fixed contact inhibit at 0.22 s */
#define JUMP_AIR_TUCK_MIN_HOLD_TIME       0.02f /* legacy schedule API; actual dwell is never descent-shortened */
#define JUMP_AIR_TUCK_REF_RATE            1.80f
#define JUMP_AIR_ACTUAL_COMPACT_LENGTH    0.145f
#define JUMP_AIR_ACTUAL_COMPACT_REF_TOL   0.002f
#define JUMP_AIR_ACTUAL_COMPACT_HOLD      0.080f
#define JUMP_AIR_COMPACT_HARD_MAX         0.250f
#define JUMP_AIR_LANDING_LENGTH           0.27f
/* 2026-09-23: airborne leg-angle (theta) reference and the rate it is driven
 * from the captured takeoff angle toward that target.  All three reference
 * implementations hold the leg angle explicitly instead of leaving it free:
 *   - climber v3   : airborne joint torque = k[1][0]*theta + k[1][1]*d_theta
 *                    with ref.leg_state.theta fixed at 0 (pure spring+damping)
 *   - chassis.7z   : theta target `pitchToBadyBalance`, 0.25 rad during the jump
 *                    phases, with joint gain K[1] ~ +80 and wheel row K[0] ~ -20
 *   - 2026Climber  : Tp = kp_phi*(target_Phi0 - Phi0) + kd_phi*(-dPhi0), kp 60
 * The previous code drove the pre-tuck reference toward 0.0 (vertical rods) and
 * left the airborne reference hard-coded at 0, so the legs were free to fold and
 * the body reaction had nothing to push against. */
/* 2026-09-23: airborne leg-angle (theta) reference and the rate it is driven
 * from the captured takeoff angle toward that target.
 *
 * WHAT IT IS: theta is the leg rod's angle relative to the BODY's vertical
 *   theta = pi/2 - phi0 + pitch            (drv_leg.c:71)
 * so theta = 0 means the rods are vertical, theta > 0 means they lean BACK
 * relative to the body, theta < 0 means they lean FORWARD.  This constant is
 * the angle the airborne joint controller holds during flight (the control
 * chain captures the real takeoff angle and slews it here at
 * JUMP_AIR_THETA_REF_SLEW_RATE).  It is used twice: as the airborne hold
 * reference, and as the target the pre-tuck reference is driven to.
 *
 * WHY 0: it matches the 1:1 simulation, whose theta loop targets 0 and which is
 * the only configuration actually validated end to end:
 *   theta +80/+12, wheel ON -> max|theta| 0.123 rad, max|pitch| 7.1 deg, no fault
 * The previous 0.25 rad (14.3 deg) was copied from a third-party open-source
 * chassis and never validated on our model; it parked the real robot's flight
 * equilibrium 14 deg forward of the verified case. */
#define JUMP_AIR_THETA_REF_RAD            0.00f
#define JUMP_AIR_THETA_REF_SLEW_RATE      1.00f
/* 2026-09-23 (jump18.csv): bound the leg-angle tracking error before the gain.
 * theta is computed as pi/2 - phi0 + pitch and therefore WRAPS at +-pi, so an
 * unclamped error can command tens of Nm in the wrong direction once a rod
 * passes vertical.  jump18 is the measured failure: the right rod ran
 * phi0 149 -> 172 -> -179 deg (folded over the top of the body) and theta_R
 * wrapped from -0.79 to +5.39 rad inside a single 8 ms sample, while the joint
 * kept being driven around -- the operator saw the leg "spin once in the air".
 * With Kp 80 a 0.20 rad band already reaches ~16 Nm, so clamping here keeps the
 * loop in its linear region and makes it safe against wrap.
 * Shared by the grounded thrust channel (chassis_control.c) and the airborne
 * channel (chassis_safety.c). */
#define JUMP_THETA_ERR_CLAMP              0.20f
/* 2026-09-22 F3: retained legacy descent diagnostic threshold. The current
 * flight profile deploys after 80 ms of measured two-leg compact dwell or at
 * the 250 ms hard deadline; descent does not shorten the dwell. The independent
 * touch/impact inhibit remains fixed at 0.22 s. */
#define JUMP_AIR_DEPLOY_DESCENT_VZ        (-0.20f)
#define JUMP_AIR_CAPTURE_LENGTH_MAX       0.37f
#define JUMP_AIR_DEPLOY_REF_RATE          1.50f
#define JUMP_AIR_DEPLOY_KP              1000.0f
#define JUMP_AIR_DEPLOY_KD                26.0f
#define JUMP_AIR_DEPLOY_FORCE_LIMIT      150.0f
#define JUMP_AIR_RETRACT_FORCE_LIMIT     100.0f /* 2026-09-22 fix #2: back to实车 pair with PEAK 180 */
#define JUMP_AIR_RETRACT_FORCE_PEAK      140.0f /* 2026-09-22 fix #2: the -230N impulse escaped via this final clamp; cut to实车 180 */
/* Shared by the timed pre-tuck and the confirmed-airborne controller so the
 * takeoff edge cannot change the left/right length law. */
#define JUMP_AIR_LENGTH_SYNC_KP           380.0f
#define JUMP_AIR_LENGTH_SYNC_KD            24.0f
#define JUMP_AIR_LENGTH_SYNC_LIMIT        50.0f
/* Active JUMP only; BUMP keeps the shared legacy air settings above.
 * Gas compensation sits outside the -100 N active common retract limit.
 * Final motor packets still obey 35 Nm each and the 80 Nm total budget. */
#define JUMP_FLIGHT_TUCK_KP              2600.0f
#define JUMP_FLIGHT_LENGTH_KD              40.0f
#define JUMP_FLIGHT_DEPLOY_KP            1800.0f
#define JUMP_FLIGHT_DEPLOY_KD              26.0f
#define JUMP_FLIGHT_DEPLOY_REF_RATE         3.0f
#define JUMP_FLIGHT_RETRACT_PEAK          200.0f
#define JUMP_FLIGHT_LENGTH_SYNC_KP       1200.0f
#define JUMP_FLIGHT_LENGTH_SYNC_KD         60.0f
#define JUMP_FLIGHT_LENGTH_SYNC_LIMIT      70.0f
#define JUMP_FLIGHT_ROLL_KP                20.0f /* N/deg */
#define JUMP_FLIGHT_ROLL_KD               180.0f /* N/(rad/s) */
#define JUMP_FLIGHT_ROLL_LIMIT             80.0f
#define JUMP_FLIGHT_COMPACT_HOLD            0.084f /* >=80 ms with 2 ms feedback */
#define JUMP_FLIGHT_COMPACT_HARD_MAX        0.400f
#define JUMP_TOUCH_SPECIFIC_FORCE_MIN       6.0f
#define JUMP_TOUCH_COMPRESSION_RATE_MAX    (-0.02f)
#define LANDING_BUFFER_MID_LENGTH        0.21f
#define LANDING_LENGTH_REF_MIN            0.10f
#define LANDING_LENGTH_REF_MAX            0.34f
#define LANDING_COMPRESS_REF_RATE        0.60f
#define LANDING_COMPRESS_MAX_TIME        0.20f
#define LANDING_HOLD_MIN_TIME            0.18f
#define LANDING_HOLD_MAX_TIME            0.45f
#define LANDING_SETTLE_LENGTH_ERROR      0.015f
#define LANDING_SETTLE_LENGTH_SPEED      0.10f
#define LANDING_RETURN_REF_RATE          0.35f
#define LANDING_RETURN_REF_RATE_LOW      0.80f
#define LANDING_LOW_RATCHET_MARGIN       0.006f
#define LANDING_LOW_RATCHET_REF_RATE     1.20f
/* At first contact, allow a small mechanical split but do not carry a large
 * one-sided touchdown reference through the whole buffer sequence. */
#define LANDING_LOW_CONTACT_SPLIT_MAX    0.008f
#define LANDING_PID_RETURN_TIME          0.30f
#define LANDING_EXIT_STABLE_TIME         0.18f
#define LANDING_EXIT_LENGTH_SPEED        0.12f
#define LANDING_EXIT_ATTITUDE_PITCH_MAX_DEG 10.0f
#define LANDING_EXIT_ATTITUDE_ROLL_MAX_DEG  10.0f
#define LANDING_EXIT_ATTITUDE_RATE_MAX_RAD_S 1.00f
#define LANDING_SUPPORT_BLEND_TIME       0.04f
#define LANDING_LENGTH_KP               650.0f
#define LANDING_LENGTH_KI                 0.0f
#define LANDING_LENGTH_KD                40.0f
#define LANDING_LENGTH_OUTPUT_LIMIT     100.0f
#define LANDING_LENGTH_KD_LOW            45.0f
#define LANDING_LENGTH_KD_LOW_REBOUND   110.0f
#define LANDING_LENGTH_OUTPUT_LIMIT_LOW 120.0f

/* A one-wheel impact starts axial cushioning before the conservative
 * two-wheel landing decision.  Wheel output remains shut until both wheels
 * confirm contact, so this path cannot falsely re-enable a suspended wheel. */
#define LANDING_IMPACT_LENGTH_SPEED     (-0.35f)
#define LANDING_IMPACT_LENGTH_SPEED_LOW (-0.18f)
#define LANDING_IMPACT_CONFIRM_CNT        2U
#define LANDING_PRECONTACT_TIMEOUT        0.25f
#define LANDING_IMPACT_REARM_TIME         0.05f
/* Keep airborne attitude/leg-angle authority when the compact tuck is
 * reached before the filtered ground detector reports touchdown. */
#define LANDING_PRECONTACT_LATCH_TIME     0.22f
#define LANDING_PRECONTACT_COMPACT_LENGTH 0.25f
#define LANDING_PRECONTACT_COMPACT_RATE_MAX (-0.02f)
#define LANDING_PRECONTACT_CONFIRM_CNT    2U

/* Motor landing cushion (2026-10-04, jump100).  The gas spring alone cannot
 * absorb a jump touchdown: the legs arrive fully extended (0.35-0.38 m), the
 * two independent length loops commanded opposite signs on first contact
 * (+156/-27 N, -114/+115 N), and the rods were already at 3-9 rad/s.  This adds
 * one SYMMETRIC braking force on the axial (support-force) channel,
 * proportional to the common compression rate and applied to BOTH legs, so it
 * cannot inject a left/right split or a body roll moment:
 *
 *   F = clamp(KD * max(0, -v_common - deadband), 0, FORCE_MAX)      [N]
 *
 * KD is N per (m/s): at 1.0 m/s of compression it contributes 150 N per leg on
 * top of the length loop.  FORCE_MAX stays below FORCE_LIMIT (200 N) so the
 * cushion cannot become the saturating term.  The window starts at confirmed
 * contact and is independent of the landing buffer, because the fast-exit
 * handoff resets that buffer while the legs are still moving. */
#define LANDING_CUSHION_ENABLE            1
/* 1 = only landings whose buffer was opened in CHASSIS_JUMP.  Set to 0 to give
 * every landing the same motor cushion. */
#define LANDING_CUSHION_JUMP_ORIGIN_ONLY  1
#define LANDING_CUSHION_KD              150.0f  /* N per (m/s) */
#define LANDING_CUSHION_FORCE_MAX       150.0f  /* N, per leg (same both sides) */
/* v18 (jump101 #2): cap the SUM of the landing length loop and the cushion.
 * The v17 cushion stacked on top of the loop and pinned support_force at the
 * 200 N FORCE_LIMIT for several cycles, which is what launched the robot into
 * the second flight.  The cushion now only fills the headroom up to this
 * total; if the length loop already commands more, the cushion adds nothing. */
#define LANDING_CUSHION_TOTAL_LIMIT     150.0f  /* N, per leg, loop + cushion */
#define LANDING_CUSHION_RATE_DEADBAND     0.05f /* m/s */
/* v19 (jump102): the fast FALL_RATE is what keeps a bounce from being pumped
 * (the force is gone ~5 ms after the compression ends).  v18 additionally made
 * the cushion one-shot per landing, which starved the deeper second collapse
 * and produced the "slammed down" report -- do not reintroduce a latch. */
#define LANDING_CUSHION_RISE_RATE     12000.0f  /* N/s -> 150 N in ~13 ms */
#define LANDING_CUSHION_FALL_RATE     30000.0f  /* N/s -> released in ~5 ms */
#define LANDING_CUSHION_WINDOW_S          0.400f

/* Post-buffer landing support reference (v19, jump102).  The v15 fast-exit
 * releases the landing buffer as soon as both legs are below 0.28 m, which also
 * dropped the ratcheted landing reference; the leg target then snapped straight
 * back to the operator's 0.13 m while the robot was still compressing, so the
 * length loop actively pulled the leg IN (-83 N measured) during the collapse
 * and the robot free-fell onto the fold stop.  For this window after a
 * jump-origin touchdown the released buffer hands its one-way ratchet
 * reference to a bounded post-buffer hold, then fades back to the operator
 * target at a bounded rate so nothing steps. */
#define LANDING_POSTBUFFER_REF_ENABLE     1
#define LANDING_POSTBUFFER_REF_HOLD_S     0.400f
#define LANDING_POSTBUFFER_FADE_RATE      0.80f /* m/s back to the operator target */
/* v20 (jump103): the post-buffer window also keeps the LANDING length-PID
 * tuning (DoM + Kp 650 / Kd 45-110 + 120 N limit) instead of dropping straight
 * back to the standing gains (Kp 450 / Kd 18).  jump103 measured the
 * consequence of the drop: a 10 Hz, 20-40 mm peak-to-peak leg-length ring for
 * ~0.8 s after every touchdown -- the same tremor v6 fixed inside the landing
 * buffer, which the v15 fast-exit now ends 50-70 ms after contact.  The gains
 * blend back to the standing values over this time as the reference fades. */
#define LANDING_POSTBUFFER_GAIN_FADE_S    0.30f

/* Post-landing airborne guard (v18, jump101 #2).  After a jump-origin landing
 * the robot can be thrown back into the air by the leg gas spring (measured:
 * 380 ms of flight starting 330 ms after touchdown).  By then the jump state
 * machine has finished, so the reaction-wheel channel (gated on
 * jump_state == FLIGHT) and the stiff airborne leg-angle hold (gated on
 * ctrl_mode == CHASSIS_JUMP) were both unavailable: Pitch ran +7 -> +59 deg
 * with no authority and the robot flipped on the way down.  For this window
 * after confirmed jump-origin contact, an off-ground robot keeps the
 * jump-grade attitude authority (reaction wheel + 80/12 theta hold). */
#define LANDING_AIRBORNE_GUARD_ENABLE     1
#define LANDING_AIRBORNE_GUARD_WINDOW_S   0.800f

/******** 底盘最大速度设置 *******/
/* 底盘移动最大速度，单位是米每秒 */
#define MAX_CHASSIS_VX_SPEED 4.0
#define MAX_CHASSIS_VY_SPEED 2.0 // 对应平步底盘，该值为roll轴倾斜
#define KEY_ACCEL_VX_RATIO 0.01F /*按键控制的加速度*/
#define KEY_ACCEL_VY_RATIO 0.01F /*按键控制的加速度*/
/* ---------------------------------- 云台相关 ---------------------------------- */
#define DM_IMU_TX_ID 0x10
#define DM_IMU_RX_ID 0x11  /*即master_id*/

#define GM6020_TOR_TO_CUR  7372.5f /*扭矩电流系数 TODO 待查表计算*/
#define GM6020_TOR_MAX   1.2f
 /* [0]为yaw，[1]为pitch */
#define YAW 0
#define PITCH 1
 /*gimbal_can allocation*/
#define YAW_MOTOR_ID     6
#define PITCH_MOTOR_ID   0x206

#define CENTER_YAW_ANGLE   0.0f        //云台yaw归中角度设置
#define CENTER_PITCH_ANGLE 0.0f        //云台pitch归中角度设置

#define GEAR_RATIO 1.0f             //云台yaw轴齿轮传动比：电机:输出 = 1:1
#define GIMBAL_YAW_DRIVE_SIGN (-1.0f) //齿轮传动相对原同步带机构反向

/* 云台控制周期 (ms) */
#define GIMBAL_PERIOD 1
/* 云台回中初始化时间 (ms) */
#define BACK_CENTER_TIME 40
#define INIT_TIMEOUT 600  // 单位: ms，超时后放宽归中角度容差
#define GIMBAL_INIT_FORCE_TIMEOUT_MS 3000U // 归中满 3s 强制交回正常控制

/* 正常归中需满足角度和速度条件并持续 BACK_CENTER_TIME；3s 强制超时除外。 */
#define GIMBAL_INIT_PITCH_ANGLE_STRICT_RAD  0.020f
#define GIMBAL_INIT_YAW_ANGLE_STRICT_RAD    0.015f
#define GIMBAL_INIT_PITCH_ANGLE_TIMEOUT_RAD 0.080f
#define GIMBAL_INIT_YAW_ANGLE_TIMEOUT_RAD   0.160f
#define GIMBAL_INIT_PITCH_RATE_MAX_RADPS    0.50f
#define GIMBAL_INIT_YAW_RATE_MAX_RADPS      0.80f

/* pitch轴最大仰角 */
#define PITCH_ANGLE_MAX        20.0f
/* pitch轴最大俯角 */
#define PITCH_ANGLE_MIN        -15.0f

/* ------------------------------------------------------- 发射相关 --------------------------------------------------- */
// TODO: 实际值待整定
#define RIGHT_FRICTION_MOTOR_ID     0x203
#define LEFT_FRICTION_MOTOR_ID      0x204
#define TRIGGER_MOTOR_ID            0x205

#define FRICTION_SPEED_CONTINUE      6000

#define SHOOT_TRIGGER_REVERSE_SPEED 5000/*堵弹反转*/
/** COUNTINUE模式参数 **/
#define DBUS_TRIGGER_SPEED_L 3500
#define DBUS_TRIGGER_SPEED_H 6000

#define TRIGGER_MOTOR_51_TO_ANGLE 51.47f
/*M2006的减速比为36:1，因此转轴旋转45度，要在转子的基础上乘36倍*/
#define TRIGGER_MOTOR_45_TO_ANGLE 45 * 36
/* -------------------------------- 发射电机PID参数 ------------------------------- */
// TODO: 速度期望应改为变量应对速度切换。初次参数调整已完成
/* 右摩擦轮M3508电机PID参数 */
/* 速度环 */
#define RIGHT_KP_V             18
#define RIGHT_KI_V             0.0f
#define RIGHT_KD_V             0.001f
#define RIGHT_INTEGRAL_V       0
#define RIGHT_MAX_V            16384

/* 左摩擦轮M3508电机PID参数 */
/* 速度环 */
#define LEFT_KP_V           18
#define LEFT_KI_V           0.0f
#define LEFT_KD_V           0.001f
#define LEFT_INTEGRAL_V     0
#define LEFT_MAX_V          16384

// TODO：PID参数初次微调已完成，期待后续微调
/* 拨弹电机M3508电机PID参数 */
/* 速度环 */
#define TRIGGER_KP_V           10
#define TRIGGER_KI_V           5
#define TRIGGER_KD_V           0.01f
#define TRIGGER_INTEGRAL_V     1500
#define TRIGGER_MAX_V          16384
/* 角度环 */
#define TRIGGER_KP_A           5
#define TRIGGER_KI_A           0.01
#define TRIGGER_KD_A           0.003
#define TRIGGER_INTEGRAL_A     5.0f
#define TRIGGER_MAX_A          10000


/* -------------------------------- 云台电机PID参数 ------------------------------- */
/* 云台yaw轴电机PID参数 */
/* imu速度环 */
#define YAW_KP_V_IMU             0.9f
#define YAW_KI_V_IMU             0.25f
#define YAW_KD_V_IMU             0.0f
#define YAW_INTEGRAL_V_IMU       0.25f
#define YAW_MAX_V_IMU            2.4f
/* imu角度环 */
#define YAW_KP_A_IMU             5.0f
#define YAW_KI_A_IMU             0.2f
#define YAW_KD_A_IMU             0.0f
#define YAW_INTEGRAL_A_IMU       0.10f
#define YAW_MAX_A_IMU            5.8f
/* 归中专用环：1:1 齿轮机构采用较高的赛场归中增益；正常 GYRO 环保持不变。 */
#define YAW_KP_V_INIT            0.75f
#define YAW_KI_V_INIT            0.12f
#define YAW_KD_V_INIT            0.0f
#define YAW_INTEGRAL_V_INIT      0.25f
#define YAW_MAX_V_INIT           3.5f
#define YAW_KP_A_INIT            5.5f
#define YAW_KI_A_INIT            0.0f
#define YAW_KD_A_INIT            0.0f
#define YAW_INTEGRAL_A_INIT      0.0f
#define YAW_MAX_A_INIT           6.0f
/* auto速度环 */
#define YAW_KP_V_AUTO            0.9f
#define YAW_KI_V_AUTO            0.25f
#define YAW_KD_V_AUTO            0.0f
#define YAW_INTEGRAL_V_AUTO      0.25f
#define YAW_MAX_V_AUTO           1.8f
/* auto角度环 */
#define YAW_KP_A_AUTO            5.0f
#define YAW_KI_A_AUTO            0.2f
#define YAW_KD_A_AUTO            0.0f
#define YAW_INTEGRAL_A_AUTO      0.10f
#define YAW_MAX_A_AUTO           2.8f


/* 云台PITCH轴电机PID参数 */
/* 归中专用环：快速回零，积分仅修正重力前馈误差。 */
#define PITCH_KP_V_INIT          6000.0f
#define PITCH_KI_V_INIT          300.0f
#define PITCH_KD_V_INIT          0.0f
#define PITCH_INTEGRAL_V_INIT    800.0f
#define PITCH_MAX_V_INIT         15000.0f
#define PITCH_KP_A_INIT          6.5f
#define PITCH_KI_A_INIT          0.0f
#define PITCH_KD_A_INIT          0.04f
#define PITCH_INTEGRAL_A_INIT    0.0f
#define PITCH_MAX_A_INIT         2.8f

/* imu速度环 */
#define PITCH_KP_V_IMU           4500.0f
#define PITCH_KI_V_IMU           500.0f
#define PITCH_KD_V_IMU           0.0f
#define PITCH_INTEGRAL_V_IMU     1500.0f
#define PITCH_MAX_V_IMU          13000.0f

/* imu角度环 */
#define PITCH_KP_A_IMU           5.0f
#define PITCH_KI_A_IMU           0.3f
#define PITCH_KD_A_IMU           0.05f
#define PITCH_INTEGRAL_A_IMU     0.2f
#define PITCH_MAX_A_IMU          2.0f

/* 重力前馈系数：Pitch 轴重力矩补偿，按 cos(IMU pitch) 前馈到电流。
 * 保守初值 5000，按 gimbal4 采样中 -8.6° 处约 5500 电流维持平衡估算，
 * 待拿到云台重心/质量后按 m*g*L 标定。 */
#define PITCH_GRAVITY_FF         5000.0f

/* auto速度环 */
#define PITCH_KP_V_AUTO          4500.0f
#define PITCH_KI_V_AUTO          500.0f
#define PITCH_KD_V_AUTO          0.0f
#define PITCH_INTEGRAL_V_AUTO    1500.0f
#define PITCH_MAX_V_AUTO         13000.0f
/* auto角度环 */
#define PITCH_KP_A_AUTO          5.0f
#define PITCH_KI_A_AUTO          0.3f
#define PITCH_KD_A_AUTO          0.05f
#define PITCH_INTEGRAL_A_AUTO    0.2f
#define PITCH_MAX_A_AUTO         2.0f
/***********chassis部分关于length/theta/yaw/roll的pid参数**********************/

/////*位置*/
//#define l_length_Kp 300.0
//#define l_length_Ki 50.00
//#define l_length_Kd 0.0001
//#define l_length_InteVal 150
//#define l_length_MaxVal 300
//
//
///*位置*/
//#define r_length_Kp 300.0
//#define r_length_Ki 50.00
//#define r_length_Kd 0.0001
//#define r_length_InteVal 150
//#define r_length_MaxVal 300

///*位置*/
#define l_length_Kp 450.0   /* reverted 2026-09-22: per-leg 2039/16 table overshot on hardware */
#define l_length_Ki 100.00
#define l_length_Kd 18.0
#define l_length_InteVal 35
#define l_length_MaxVal 150


/*位置*/
#define r_length_Kp 450.0
#define r_length_Ki 100.00
#define r_length_Kd 18.0
#define r_length_InteVal 35
#define r_length_MaxVal 150

/* Length integral separation: integrate fully below CoefB, taper between
 * CoefB and CoefA+CoefB, and stop integrating above that range. */
#define length_Integral_CoefA 0.040f
#define length_Integral_CoefB 0.004f

/* 腿长微分使用实际周期计算，必须滤掉机构解算与编码器噪声。 */
#define length_Derivative_LPF_RC 0.012f

/*theta相关*/
#define theta_Kp 15
#define theta_Ki 0
#define theta_Kd 0.00001
#define theta_InteVal 0
#define theta_MaxVal 50

/*yaw相关,转向采用pd控制*/
#define yaw_Kp 1.0
#define yaw_Ki 0.2
#define yaw_Kd 0.2
#define yaw_InteVal 0
#define yaw_MaxVal 20.0
//
/*roll相关*/
#define roll_Kp 5.5f
#define roll_Ki 0
#define roll_Kd 0.18f
#define roll_InteVal 0.0f
/* roll 角度微分的低通时间常数，单位 s。 */
#define roll_Derivative_LPF_RC 0.020f
/* 低腿长保留已实测稳定的裸车增益；随实际平均腿长连续过渡到整车质量
 * 对应的增益。这样切换档位不会产生 Kp/Kd 阶跃，也不会扰动 recovery。 */
#define ROLL_GAIN_SCHEDULE_L0_START 0.140f
#define ROLL_GAIN_SCHEDULE_L0_END   0.220f
#define ROLL_WHOLE_BODY_GAIN_SCALE  (1.12f * m_b_whole / m_b)
/* The full-body mass is about twice the bare chassis mass. Keep the small-error
 * gains unchanged, but allow enough differential support force to arrest a
 * large roll excursion during a moving turn. */
#define roll_MaxVal 60

/*****************************function_open******************************************/
/****************************底盘部分**************************************/
/*要使用时打开宏定义!!!*/
/*
 * @brief设置髋关节damiao电机的零点
 * @warning零点设置注意位置,若是位置不对会导致phi的角度都不对会疯车
 * @warning 不小心开启并烧录的话需要重新校准零点,不然一定疯车
 * */
// #define DM_8009_SET_ZERO_POSITION
/*云台电机yaw轴设置零点*/
// #define YAW_MOTOR_SET_ZERO_POSITION
//#define BSP_BMI088_CALI
/*使用imu校准(不用每次都校准,一段时间校准即可,温度在40摄氏度左右再进行校
/*使用位于云台的达妙imu*/
#define  BSP_USING_DM_IMU
/*使用damiao板imu加热*/
#define  BSP_USING_IMU_HEAT

/*使用3508*/
#define BSP_USING_DJI_MOTOR

/*使用dm电机,后续可细分电机种类*/
#define BSP_USING_DM_MOTOR

/*使用lk电机*/
//#define  BSP_USING_LK_MOTOR

/*使用ht电机*/
//#define BSP_USING_HT_MOTOR

/*3508输入置零*/
// #define M3508_SET_ZERO
/*8009输入置零*/
// #define DM8009P_SET_ZERO
/***********************************************************************************/

/*******************************云台部分***********************************************/
/*云台失能*/
// #define GIMBAL_RELEX
// #define GM6020_SET_ZERO
/*4310输入置零*/
// #define DM4310_SET_ZERO

/*********************************状态机部分*********************************************/
#define BSP_USING_RC_DBUS_KEYBOARD /*使用键盘控制*/
//#define BSP_USING_SUPERCAP/*超电控制*/
// #define CHASSIS_FOLLOW_GIMBAL_CLOSE
// #define FRICTION_CLOSE
#define GIMBAL_AUTO_CLOSE
/**************************************************************************************/
/************************************************************************************/


/* -------------------------------------------------------------------------- */
/*                            remote_controler                                  */
/* -------------------------------------------------------------------------- */
#define BSP_USING_RC_DBUS
#define BSP_USING_RC_KEYBOARD


#endif /* _RM_CONFIG_H */
