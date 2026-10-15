#include "cmd_task.h"
#include "rm_module.h"
#include "rm_algorithm.h"
#include "robot.h"
#include "dr16_control.h"



/* ------------------------------- ipc 线程间通讯相关 ------------------------------ */
// 订阅
MCN_DECLARE(chassis_fdb);
static McnNode_t chassis_fdb_node;
static struct chassis_fdb_msg chassis_fdb;
MCN_DECLARE(gimbal_fdb);
static McnNode_t gimbal_fdb_node;
static struct gimbal_fdb_msg gim_fdb;
MCN_DECLARE(shoot_fdb);
static McnNode_t shoot_fdb_node;
static struct shoot_fdb_msg sht_fdb;
MCN_DECLARE(transmission_fdb);
static McnNode_t trans_fdb_node;
static struct trans_fdb_msg trans_fdb;
MCN_DECLARE(ins_topic);
static McnNode_t ins_topic_node;
static struct ins_msg ins;
// 发布
MCN_DECLARE(chassis_cmd);
static struct chassis_cmd_msg chassis_cmd_data;
MCN_DECLARE(gimbal_cmd);
static struct gimbal_cmd_msg gimbal_cmd_data;
MCN_DECLARE(shoot_cmd);
static struct shoot_cmd_msg shoot_cmd_data;

static void cmd_pub_push(void);
static void cmd_sub_init(void);
static void cmd_sub_pull(void);

/* --------------------------------------------------- 遥控器相�?---------------------------------------------------- */
#define RC_UP 1u
#define RC_MI 3u
#define RC_DN 2u
#ifdef BSP_USING_RC_DBUS
static Remote_Info_Typedef *remote_ctrl_now;
static Remote_Info_Typedef remote_ctrl_last;
#else
extern sbus_data_t sbus_data_fdb;
#endif

/*发射停止标志�?*/
static int trigger_flag=0;
/*堵转电流反转记次*/
static int reverse_cnt;
/*堵弹反转计数*/
static int reverse_remove_cnt;
/* 外部变量声明 */
/*自瞄鼠标累计操作�?*/
static float mouse_accumulate_x=0;
static float mouse_accumulate_y=0;
/*储存鼠标坐标数据*/
First_Order_Filter_t mouse_y_lpf,mouse_x_lpf;
/*自瞄继承角度*/
kb_control_t *keyboard_ctrl_now;

/*继承角度*/
static float gyro_yaw_inherit;
static float gyro_pitch_inherit;
/* ------------------------------- 遥控数据转换为控制指�?------------------------------ */
static void remote_to_cmd_pc_DT7(void);/*键鼠控制||遥控器控�?都可�?*/
//TODO: 添加图传链路的自定义控制器控制方式和键鼠控制方式

/* -------------------------------- cmd 线程主体 -------------------------------- */
void cmd_task_init(void)
{
    remote_ctrl_now = &remote_ctrl;
    keyboard_ctrl_now = &key_board_ctrl;
    BSP_USART_Init();
    cmd_sub_init();

}

void cmd_control_task(void)
{
    cmd_sub_pull();

#ifdef BSP_USING_RC_KEYBOARD
    PC_Handle_kb();//处理PC端键鼠控�?
    remote_to_cmd_pc_DT7();
#else
    remote_to_cmd();
#endif

    cmd_pub_push();
}
static float jump_flag=0;
static float flag=0;

static float jump_count=0;

/* -------------------------------------------- 将遥控器数据转换为控制指�?----------------------------------------------- */
#ifdef BSP_USING_RC_DBUS_KEYBOARD
static void remote_to_cmd_pc_DT7(void)
{
    dr16_control_update(remote_ctrl_now,
                        &chassis_fdb,
                        &gim_fdb,
                        &sht_fdb,
                        &chassis_cmd_data,
                        &gimbal_cmd_data,
                        &shoot_cmd_data);
    remote_ctrl_last = *remote_ctrl_now;
    return;

    /* Legacy keyboard/remote mapping retained below for reference. */


    float fx=First_Order_Filter_Calculate(&mouse_x_lpf,remote_ctrl_now->mouse.x);
    float fy=First_Order_Filter_Calculate(&mouse_y_lpf,remote_ctrl_now->mouse.y);

// TODO: 目前状态机转换较为简单，有很多优化和改进空间
    /*底盘命令*/
    chassis_cmd_data.vx =  (float)remote_ctrl_now->rc.ch[1] + keyboard_ctrl_now->vx * CHASSIS_PC_MOVE_RATIO_X;
    slope_following(&chassis_cmd_data.vx,&chassis_cmd_data.vx_set,10.0f);
    chassis_cmd_data.vy =  (float)remote_ctrl_now->rc.ch[0] + keyboard_ctrl_now->vy * CHASSIS_PC_MOVE_RATIO_Y;
    slope_following(&chassis_cmd_data.vx,&chassis_cmd_data.vy_set,0.8f);
    chassis_cmd_data.vw_relative =  (float)remote_ctrl_now->rc.ch[2] + remote_ctrl_now->mouse.x * CHASSIS_PC_MOVE_RATIO_R;
    slope_following(&chassis_cmd_data.vw_relative,&chassis_cmd_data.vw_relative_set,50.0f);



    // 右拨杆s[0]]为上时，底盘和云台均REALX；为中时，云台为GYRO，地盘为OPEN；为下时，云台为AUTO�?
    // 左拨杆s[1]为上�?
    switch (remote_ctrl_now->rc.s[0])
    {
        case RC_UP:
            chassis_cmd_data.ctrl_mode = CHASSIS_RELAX;
            gimbal_cmd_data.ctrl_mode = GIMBAL_RELAX;
            shoot_cmd_data.ctrl_mode=SHOOT_STOP;
            break;

        case RC_MI:

            if(gimbal_cmd_data.last_mode == GIMBAL_RELAX)
            {/* 判断上次状态是否为RELAX，是则先归中 */
                gimbal_cmd_data.ctrl_mode = GIMBAL_INIT;
            }

            if((chassis_fdb.stand_state != CHASSIS_IS_DANGER))
            {
                if(chassis_cmd_data.last_mode == CHASSIS_INIT || chassis_cmd_data.last_mode == CHASSIS_RELAX)
                {
                    chassis_cmd_data.ctrl_mode = CHASSIS_RECOVERY;//完成倒地自起
                }
                else if(chassis_cmd_data.last_mode == CHASSIS_RECOVERY || chassis_cmd_data.last_mode == CHASSIS_AUTO || chassis_cmd_data.last_mode == CHASSIS_JUMP)
                {
                    if(chassis_fdb.stand_state == CHASSIS_IS_STAND)
                    {
                        chassis_cmd_data.ctrl_mode = CHASSIS_OPEN_LOOP;

                    }
                    else if(chassis_fdb.stand_state == CHASSIS_IS_DANGER)
                    {
                        chassis_cmd_data.ctrl_mode = CHASSIS_RELAX;
                    }

                    else if(chassis_fdb.stand_state == CHASSIS_IS_JUMPING || chassis_fdb.stand_state == CHASSIS_IS_JUMPOK){/*跳跃过程中就算切换状态机也要优先确保跳跃全过程结�?*/
                        if(chassis_fdb.stand_state == CHASSIS_IS_JUMPOK){
                            chassis_cmd_data.ctrl_mode = CHASSIS_FOLLOW_GIMBAL;
                        }else{
                            chassis_cmd_data.ctrl_mode = CHASSIS_JUMP;
                        }
                    }
                    else{
                        chassis_cmd_data.ctrl_mode = CHASSIS_RECOVERY;
                    }
                }

            }
            else{
                chassis_cmd_data.ctrl_mode = CHASSIS_RECOVERY;
            }

            if(gim_fdb.back_mode == BACK_IS_OK)
            {
                gimbal_cmd_data.ctrl_mode = GIMBAL_GYRO;/*gimbal进入手动控制状�?*/
#if (defined(CHASSIS_FOLLOW_GIMBAL_CLOSE) || defined(GIMBAL_RELEX) )

#else
                if(chassis_cmd_data.ctrl_mode == CHASSIS_OPEN_LOOP){
                    chassis_cmd_data.ctrl_mode = CHASSIS_FOLLOW_GIMBAL ;/*正常进行跟随云台运动,不开云台时不需�?*/
                }
#endif
            }

            break;

        case RC_DN:

            if((chassis_cmd_data.last_mode == CHASSIS_OPEN_LOOP) && (gim_fdb.back_mode == BACK_IS_OK)){
#if (defined(CHASSIS_FOLLOW_GIMBAL_CLOSE) || defined(GIMBAL_RELEX) )

#else
                chassis_cmd_data.ctrl_mode = CHASSIS_FOLLOW_GIMBAL;/*不开云台时不需�?*/
#endif
            }

            if(gimbal_cmd_data.last_mode == GIMBAL_RELAX)
            {/* 判断上次状态是否为RELAX，是则先归中 */
                gimbal_cmd_data.ctrl_mode = GIMBAL_INIT;
            }
            else
            {
                if(gim_fdb.back_mode == BACK_IS_OK)
                {/* 判断归中是否完成 */
#ifdef GIMBAL_AUTO_CLOSE

#else
                    gimbal_cmd_data.ctrl_mode = GIMBAL_AUTO;
#endif
                }else{
                    gimbal_cmd_data.ctrl_mode = GIMBAL_INIT;
                }
            }

            break;
    }
    /************************************************自瞄部分*****************************************************************/
    if ((remote_ctrl_now->mouse.press_r==1||remote_ctrl_last.rc.s[1]==RC_DN) && (gim_fdb.back_mode == BACK_IS_OK && chassis_cmd_data.ctrl_mode == CHASSIS_OPEN_LOOP)) /*!如果鼠标按下右键或者遥控器选择自瞄模式*/
    {
/*TODO 测试手动瞄准时需要关�?避免疯车*/
#ifdef GIMBAL_AUTO_CLOSE

#else
        gimbal_cmd_data.ctrl_mode = GIMBAL_AUTO;
#endif
    }
    /***********************************************小陀�?******************************************************************/
    /*TODO:小陀�?*/
    //开小陀�?
    if ( keyboard_ctrl_now->E_sta==KEY_PRESS_ONCE ||remote_ctrl_now->rc.s[1]==RC_DN)
    {
        if (gim_fdb.back_mode==BACK_IS_OK && (chassis_cmd_data.last_mode == CHASSIS_FOLLOW_GIMBAL)/*底盘稳定时才可以开始小陀�?*/)
        {
            chassis_cmd_data.ctrl_mode=CHASSIS_SPIN;
        }
        else
        {
            gimbal_cmd_data.ctrl_mode=GIMBAL_INIT;/*云台先归中在进行小陀�?*/
        }
    }
    if (chassis_cmd_data.ctrl_mode==CHASSIS_SPIN)
    {
        chassis_cmd_data.vw_relative=0.0f;/*TODO 小陀螺平�?*/
    }

    //关小陀�?
    if(chassis_cmd_data.ctrl_mode==CHASSIS_SPIN && keyboard_ctrl_now->Q_sta==KEY_PRESS_ONCE )
    {
#if (defined(CHASSIS_FOLLOW_GIMBAL_CLOSE) || defined(GIMBAL_RELEX) )

#else
        chassis_cmd_data.ctrl_mode = CHASSIS_FOLLOW_GIMBAL;
#endif
    }
    /*TODO:--------------------------------------------------发射模块状态机--------------------------------------------------------------*/
    /*!-----------------------------------------开关摩擦轮--------------------------------------------*/
    if((keyboard_ctrl_now->F_sta==KEY_PRESS_ONCE || remote_ctrl_now->rc.s[1]==RC_MI) && (remote_ctrl_now->rc.s[0]!=RC_UP))
    {
#ifdef FRICTION_CLOSE
        shoot_cmd_data.friction_status=0;/*TODO 测试腿长时用*/
#else
        shoot_cmd_data.friction_status=1;
#endif

    }
    else{
        shoot_cmd_data.friction_status = 0;
    }
    //关摩擦轮
    if((shoot_cmd_data.friction_status==1) && (keyboard_ctrl_now->G_sta==KEY_PRESS_ONCE || remote_ctrl_now->rc.s[1]!=RC_MI))
    {
        shoot_cmd_data.friction_status = 0;
    }
    /***********************************************射击模式*******************************************/
    if(shoot_cmd_data.ctrl_mode != SHOOT_STOP){
        if(keyboard_ctrl_now->V_sta == KEY_PRESS_ONCE){/*旨在通过按键v操作实现单发和连发切�?*/
            shoot_cmd_data.ctrl_mode = SHOOT_ONE;
        }
        else if(shoot_cmd_data.ctrl_mode == SHOOT_ONE && keyboard_ctrl_now->V_sta == KEY_PRESS_ONCE){
            shoot_cmd_data.ctrl_mode = SHOOT_COUNTINUE;
        }
        else{
            shoot_cmd_data.ctrl_mode = SHOOT_COUNTINUE;
        }
    }
    if(remote_ctrl_now->rc.ch[4]>200 && chassis_cmd_data.ctrl_mode != CHASSIS_RELAX){/*TODO 测试弹链�?*/
        shoot_cmd_data.ctrl_mode = SHOOT_COUNTINUE;
    }else{
        shoot_cmd_data.ctrl_mode = SHOOT_STOP;
    }
    /***********************************************腿长的切�?******************************************/
    if(chassis_cmd_data.ctrl_mode != CHASSIS_RELAX && chassis_cmd_data.ctrl_mode != CHASSIS_INIT && chassis_cmd_data.ctrl_mode != CHASSIS_RECOVERY){/*稳定状态下切换腿长*/
        if(keyboard_ctrl_now->Z_sta == KEY_PRESS_ONCE || remote_ctrl_now->rc.s[1] == RC_UP){
            chassis_cmd_data.leg_level = LEG_LOW;
        }
        if(keyboard_ctrl_now->X_sta == KEY_PRESS_ONCE || remote_ctrl_now->rc.s[1] == RC_MI){
            chassis_cmd_data.leg_level = LEG_MID;
        }
        if(keyboard_ctrl_now->C_sta == KEY_PRESS_ONCE || remote_ctrl_now->rc.s[1] == RC_DN){
            chassis_cmd_data.leg_level = LEG_HIG;
        }
    }
    else{
        chassis_cmd_data.leg_level = LEG_LOW;
    }

    /* -------------------------------------初始化ui按键B-------------------------------------------*/
//    if(keyboard_ctrl_now.B_sta == KEY_PRESS_DOWN){
//
//        ui_cmd.ui_init = 1;//B键被按下满足ui初始化条�?
//
//    }else{
//
//        ui_cmd.ui_init = 0;
//
//    }
    /*--------------------------------------------底盘跟随-------------------------------------------*/
    if(chassis_cmd_data.ctrl_mode == CHASSIS_FOLLOW_GIMBAL){/*推导�?md文档里有*/
        chassis_cmd_data.vw_fllow_gimbal_temp = gim_fdb.gimbal_yaw_refer - gim_fdb.yaw_delta;
        chassis_cmd_data.vw_fllow_gimbal_set = 2.0f * gim_fdb.fllow_gimbal_yaw_total_position  - chassis_cmd_data.vw_fllow_gimbal_temp ;
    }

    /*云台命令*/
    if (gimbal_cmd_data.ctrl_mode == GIMBAL_GYRO)
    {
        /*需要注意数据量�?!!*/
        /*云台控制*/
        gimbal_cmd_data.vw_relative_set = (float)remote_ctrl_now->rc.ch[2];
        gimbal_cmd_data.pitch_relative_set = (float)remote_ctrl_now->rc.ch[3] ;

        gyro_yaw_inherit =gim_fdb.gimbal_yaw_refer;
        gyro_pitch_inherit =gim_fdb.gimbal_pitch_refer;
        mouse_accumulate_x=0;
        mouse_accumulate_y=0;
    }
    if (gimbal_cmd_data.ctrl_mode==GIMBAL_AUTO)
    {
        /*需要注意数据量�?!!*/
        mouse_accumulate_x-=fx * KB_RATIO * GIMBAL_PC_MOVE_RATIO_PIT;
        mouse_accumulate_y-=fy * KB_RATIO * GIMBAL_PC_MOVE_RATIO_PIT;
        gimbal_cmd_data.yaw_auto_set = trans_fdb.yaw_target+gyro_yaw_inherit + remote_ctrl_now->mouse.x /* + mouse_accumulate_x*/;/* + 150 * remote_ctrl_now->ch3 * RC_RATIO * GIMBAL_RC_MOVE_RATIO_YAW*/;//上位机自�?
        gimbal_cmd_data.pitch_auto_set = trans_fdb.pitch_target + gyro_pitch_inherit + remote_ctrl_now->mouse.y/* +100 * remote_ctrl_now->ch4 * RC_RATIO * GIMBAL_RC_MOVE_RATIO_PIT */;//上位机自�?
    }

    switch (shoot_cmd_data.ctrl_mode)
    {
        case SHOOT_ONE:
            if (keyboard_ctrl_now->lk_sta == KEY_PRESS_ONCE && shoot_cmd_data.friction_status == 1)
            {
                if (sht_fdb.trigger_status==SHOOT_WAITING&&trigger_flag==0)
                {
                    shoot_cmd_data.trigger_status=TRIGGER_ON;
                    trigger_flag=1;
                }
                else if(sht_fdb.trigger_status==SHOOT_OK)
                {
                    shoot_cmd_data.trigger_status=TRIGGER_ING;
                }

            }
            else
            {
                shoot_cmd_data.trigger_status=TRIGGER_OFF;
            }
            if (remote_ctrl_now->mouse.press_l==KEY_RELEASE)
            {
                trigger_flag=0;
            }

            break;

        case SHOOT_COUNTINUE:

            if(
                (
                ((keyboard_ctrl_now->lk_sta==KEY_PRESS_ONCE||remote_ctrl_now->rc.ch[4]>=200)
                    // && trans_fdb.roll==1
                    && gimbal_cmd_data.ctrl_mode == GIMBAL_AUTO)
                ||((keyboard_ctrl_now->lk_sta==KEY_PRESS_ONCE||remote_ctrl_now->rc.ch[4]>=200)&&gimbal_cmd_data.ctrl_mode == GIMBAL_GYRO)
               )
               &&shoot_cmd_data.friction_status==1
            )
            {

                shoot_cmd_data.shoot_trigger_freq = DBUS_TRIGGER_SPEED_H;
            }
            else
            {
                shoot_cmd_data.shoot_trigger_freq=0;
            }
            break;

    }

    /*************************************拨弹测试(只需要dt7波轮达到要求后启动拨弹电�?***********************************************************/


    /*-------------------------------------------------------------堵弹反转检�?-----------------------------------------------------------*/
    if (sht_fdb.trigger_motor_current>=10000||reverse_cnt!=0)/*M2006电机的堵转电流是10000*/
    {
        shoot_cmd_data.ctrl_mode=SHOOT_REVERSE;
        if (reverse_cnt<300)
            reverse_cnt++;
        else{
            reverse_cnt=0;
            shoot_cmd_data.ctrl_mode=SHOOT_COUNTINUE;
        }
    }

    /*----------------------------------------------------------------使能判断---------------------------------------------------------------*/


#ifdef DM_8009_SET_ZERO_POSITION
    chassis_cmd_data.ctrl_mode = CHASSIS_INIT;
#endif

    //TODO:使能判断放最后，防止抽风
    if (remote_ctrl_now->rc.s[0] == RC_UP)/*放在最后确保使�?*/
    {
        gimbal_cmd_data.ctrl_mode = GIMBAL_RELAX;
        chassis_cmd_data.ctrl_mode = CHASSIS_RELAX;
        shoot_cmd_data.ctrl_mode=SHOOT_STOP;
        /*放开状态下，gim不接收�?*/
        gimbal_cmd_data.pitch_relative_set=0;
        gimbal_cmd_data.vw_relative_set=0;

    }
    /* 保存上一次数�?*/
    gimbal_cmd_data.last_mode = gimbal_cmd_data.ctrl_mode;
    chassis_cmd_data.last_mode = chassis_cmd_data.ctrl_mode;
    shoot_cmd_data.last_mode=shoot_cmd_data.ctrl_mode;
    remote_ctrl_last = *remote_ctrl_now;
}
#endif

/*********************************************subcription and publication***************************************************************************/
/**
 * @brief cmd 线程中所有发布者推送更新话�?
 */
static void cmd_pub_push(void)
{
    // data_content my_data = ;
    mcn_publish(MCN_HUB(chassis_cmd), &chassis_cmd_data);
    mcn_publish(MCN_HUB(gimbal_cmd), &gimbal_cmd_data);
    mcn_publish(MCN_HUB(shoot_cmd), &shoot_cmd_data);
}

/**
 * @brief cmd 线程中所有订阅者初始化
 */
static void cmd_sub_init(void)
{
    chassis_fdb_node = mcn_subscribe(MCN_HUB(chassis_fdb), NULL, NULL);/*注意先订阅节�?再去mcn_poll否则会因为找不到订阅节点导致程序卡死*/
    gimbal_fdb_node = mcn_subscribe(MCN_HUB(gimbal_fdb), NULL, NULL);
    shoot_fdb_node = mcn_subscribe(MCN_HUB(shoot_fdb), NULL, NULL);
    trans_fdb_node = mcn_subscribe(MCN_HUB(transmission_fdb), NULL, NULL);
    ins_topic_node = mcn_subscribe(MCN_HUB(ins_topic), NULL, NULL);
}


/**
 * @brief cmd 线程中所有订阅者获取更新话�?
 */
static void cmd_sub_pull(void)
{
    if (mcn_poll(chassis_fdb_node))
    {
        mcn_copy(MCN_HUB(chassis_fdb), chassis_fdb_node, &chassis_fdb);
    }
    if (mcn_poll(gimbal_fdb_node))
    {
        mcn_copy(MCN_HUB(gimbal_fdb), gimbal_fdb_node, &gim_fdb);
    }
    if (mcn_poll(shoot_fdb_node))
    {
        mcn_copy(MCN_HUB(shoot_fdb), shoot_fdb_node, &sht_fdb);
    }
    if (mcn_poll(trans_fdb_node))
    {
        mcn_copy(MCN_HUB(transmission_fdb), trans_fdb_node, &trans_fdb);
    }
    if (mcn_poll(ins_topic_node))
    {
        mcn_copy(MCN_HUB(ins_topic), ins_topic_node, &ins);
    }
}


