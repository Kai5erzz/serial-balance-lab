//
// Created by SuperChen on 2025/11/15.
//
#include "DM_motor.h"
#include "string.h"
#include "user_lib.h"

extern FDCAN_HandleTypeDef hfdcan1;
extern FDCAN_HandleTypeDef hfdcan2;

#define DM_MOTOR_CNT 5
static dm_motor_object_t dm_motor_obj[DM_MOTOR_CNT];
static uint8_t idx=0;

static void pack_contol_para(dm_motor_object_t dm_motor_obiect,dm_motor_para_t para, uint8_t *buf);

uint8_t Data_Enable[8]={0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFC};		//电机使能命令
uint8_t Data_Failure[8]={0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFD};		//电机失能命令
uint8_t Data_Save_zero[8]={0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFE};	//电机保存零点命令



/**
 * @brief  采用浮点数据等比例转换成整数
 * @param  x_int     	要转换的无符号整数
 * @param  x_min      目标浮点数的最小值
 * @param  x_max    	目标浮点数的最大值
 * @param  bits      	无符号整数的位数
 */
static float uint_to_float(int x_int, float x_min, float x_max, int bits){
/// converts unsigned int to float, given range and number of bits ///
    float span = x_max - x_min;
    float offset = x_min;
    return ((float)x_int)*span/((float)((1<<bits)-1)) + offset;
}

/**
 * @brief  将浮点数转换为无符号整数
 * @param  x     			要转换的浮点数
 * @param  x_min      浮点数的最小值
 * @param  x_max    	浮点数的最大值
 * @param  bits      	无符号整数的位数
 */

static uint16_t float_to_uint(float x, float x_min, float x_max, int bits){
    /// Converts a float to an unsigned int, given range and number of bits///
    float span = x_max - x_min;
    float offset = x_min;
    return (int) ((x-offset)*((float)((1<<bits)-1))/span);
}


/**
 * @brief 根据返回的can_object对反馈报文进行解析
 *
 * @param object 收到数据的object,通过遍历与所有电机进行对比以选择正确的实例
 */
static void motor_decode(dm_motor_object_t *motor, uint8_t *data)
{

    uint16_t value_temp;   // 解析数据的中间变量
    uint8_t *rxbuff = data;
    dm_motor_measure_t *measure = &motor->measure; // measure要多次使用,保存指针减小访存开销
    memcpy(measure->raw_data, rxbuff, sizeof(measure->raw_data));

    // 解析数据并对电流和速度进行滤波,电机的反馈报文具体格式见电机说明手册

    measure->id = (rxbuff[0])&0x0f;
    measure->ERR = (rxbuff[0])>>4;

    value_temp=(uint16_t)(rxbuff[1]<<8)|rxbuff[2];
    measure->angle = uint_to_float(value_temp, DM_P_MIN, DM_P_MAX, 16);

    value_temp=(uint16_t)(rxbuff[3]<<4)|(rxbuff[4]>>4);
    measure->speed_rads = uint_to_float(value_temp, DM_V_MIN, DM_V_MAX, 12);

    value_temp=(uint16_t)((rxbuff[4]&0x0F)<<8)|rxbuff[5];
    measure->torque = uint_to_float(value_temp, DM_T_MIN, DM_T_MAX, 12);

    measure->temperature_MOS   = (float)(rxbuff[6]);
    measure->temperature_Rotor = (float)(rxbuff[7]);
    measure->rx_count++;
    measure->last_rx_ms = HAL_GetTick();

    measure->angle_abs = fmodf(DM_P_MAX + measure->angle, DM_P_MAX); //float取余操作

    /*关于套圈的情况*/
    if(((measure->angle_abs - measure->last_angle_abs) > PI)){/*计算圈数,后续可能会用到*/
        measure->circle_cnt --;
    }
    if(((measure->angle_abs - measure->last_angle_abs) < -PI)){
        measure->circle_cnt ++;
    }
    measure->total_angle = measure->circle_cnt * PI2 + measure->angle_abs;/*电机绝对位置累加,适用于云台yaw传动带齿轮比不为1:1的情况*/
    measure->yaw_angle_round = PI2 * GEAR_RATIO;
    measure->yaw_angle = fmodf(measure->total_angle,measure->yaw_angle_round);

    measure->last_angle_abs = measure->angle_abs;
}

/**
 * @brief 电机反馈报文接收回调函数,该函数被can_rx_call调用
 *
 * @param dev 接收到报文的CAN设备
 * @param id 接收到的报文的id
 * @param data 接收到的报文的数据
 */
int dm_motor_rx_callback(uint32_t id, uint8_t *data){
    // 找到对应的实例后再调用motor_decode进行解析
    for (size_t i = 0; i < idx; i++)
    {   /* 电机手册反馈报文 */
        if (dm_motor_obj[i].tx_id == (data[0]&0x0f))
        {
            motor_decode(&dm_motor_obj[i], data);
            return 0;
        }
    }

    return -1; // 未找到对应的电机实例
}




static uint8_t data_buf[8] = {0};
// 运算电机实例的控制器,发送控制报文
static void dm_motor_control(void const *parameter)
{
    dm_motor_object_t *motor = (dm_motor_object_t *)parameter;
    dm_motor_measure_t measure = motor->measure;
    dm_motor_para_t set; // 电机控制器计算得到的控制参数
    dm_motor_para_t control_get; // 电机控制器计算得到的控制参数

    control_get = motor->control(measure); // 调用对接的电机控制,保证控制频率

    /* 首先检查是否需要切换模式 */
    if (motor->to_mode != motor->ctrl_mode)
    {
        memset(data_buf, 0xff, 7);  // 发送电机指令的时候前面7bytes都是0xff
        data_buf[7] = (uint8_t)motor->to_mode; // 最后一位是命令id
        CAN_send(motor->fdcan, motor->tx_id, data_buf);  // 发送报文
        motor->ctrl_mode = motor->to_mode;  // 切换模式成功
        osSemaphoreRelease(motor->turn_complete);
    }
    else /* 不需要切换模式情况，发送控制值 */
    {
        if (motor->ctrl_mode == DM_CMD_MOTOR_MODE) {
            set = control_get;
            pack_contol_para(*motor,set, data_buf); // 将控制参数打包成报文数据帧
            CAN_send(motor->fdcan, motor->tx_id, data_buf);  // 发送报文

        }
        else if(motor->to_mode == DM_CMD_RESET_MODE){ //确保失能
            memset(data_buf, 0xff, 7);  // 发送电机指令的时候前面7bytes都是0xff
            data_buf[7] = (uint8_t)motor->ctrl_mode; // 最后一位是命令id
            CAN_send(motor->fdcan, motor->tx_id, data_buf);  // 发送报文
            motor->ctrl_mode = motor->to_mode;  // 切换模式成功
        }
    }

}

static uint8_t i;
static volatile uint8_t batch_poll_enabled;

void dm_motor_set_batch_poll(uint8_t enabled)
{
    batch_poll_enabled = enabled ? 1U : 0U;
}

void dm_controll_all_poll(void)
{
    if (idx == 0U) return;
    /* TIM4 remains at 2 ms. A jump batch polls every registered motor once,
     * through the same callbacks, wire packing and torque budget as driving. */
    const uint8_t count = batch_poll_enabled ? idx : 1U;
    for (uint8_t n = 0U; n < count; ++n) {
        dm_motor_control(&dm_motor_obj[i]);
        i = (i + 1) % idx;
    }
}

/**
 * @brief 设置电机模式,报文内容[0xff,0xff,0xff,0xff,0xff,0xff,0xff,cmd]
 *
 * @param cmd
 * @param motor
 */
static void motor_set_mode(dm_motor_object_t *motor, dm_motor_mode_e cmd)
{
    if(motor->ctrl_mode == cmd) return; // 电机已经处于该模式,直接返回

    motor->to_mode = cmd;
    osSemaphoreWait(motor->turn_complete, 20);
}


/**
 * @brief 电机设置启停模式
 */
void dm_motor_set_type(dm_motor_object_t *motor, motor_working_type_e type)
{
    motor->stop_flag = type;
}

void dm_motor_enable_all()
{
    for (size_t i = 0; i < idx; i++)
    {
        motor_set_mode(&dm_motor_obj[i], DM_CMD_MOTOR_MODE);
    }
}

void dm_motor_disable_all()
{
    for (size_t i = 0; i < idx - 1/*云台的4310不能被失能*/; i++)
    {
        motor_set_mode(&dm_motor_obj[i], DM_CMD_RESET_MODE);
    }
}

void dm_motor_yaw_relax()
{
    motor_set_mode(&dm_motor_obj[idx-1], DM_CMD_RESET_MODE);
}
/**
 * @brief 电机初始化,返回一个电机实例
 * @param config 电机配置
 * @return dm_motor_object_t* 电机实例指针
 */
dm_motor_object_t *dm_motor_register(motor_config_t *config, void *control)
{
    // 对接用户配置的 motor_config
    dm_motor_obj[idx].motor_type = config->motor_type;             // dm8009p
    dm_motor_obj[idx].rx_id = config->rx_id;                       // 接收报文的ID(主收)
    dm_motor_obj[idx].tx_id = config->tx_id;                       // 发送报文的ID(主发)
    dm_motor_obj[idx].control = control;                           // 电机控制器执行
    dm_motor_obj[idx].set_mode = motor_set_mode;                   // 对接电机设置参数方法
    dm_motor_obj[idx].measure.circle_cnt = 0;                      //电机圈数初始为零
    // 电机挂载CAN总线
    switch (config->can_id)
    {
        case 1:
            dm_motor_obj[idx].fdcan = &hfdcan1;
            break;
        case 2:
            dm_motor_obj[idx].fdcan = &hfdcan2;
            break;
        case 3:
            dm_motor_obj[idx].fdcan = &hfdcan3;
            break;
        default:
            break;
    }

    osSemaphoreDef(turn_Sem);
    dm_motor_obj[idx].turn_complete = osSemaphoreCreate(osSemaphore(turn_Sem), 1);  // 初始化信号量
    // 电机离线检测定时器相关

    dm_motor_obj[idx].ctrl_mode = DM_CMD_RESET_MODE;
    dm_motor_obj[idx].to_mode = DM_CMD_RESET_MODE;
    motor_set_mode(&dm_motor_obj[idx], DM_CMD_RESET_MODE);   // 初始化为 RESET 模式

    switch (dm_motor_obj[idx].tx_id & 0xF00)/*判断电机模式*/
    {
        case 0x100:
            dm_motor_obj[idx].set_control_mode = POSITION_SPEED;
            break;
        case 0x200:
            dm_motor_obj[idx].set_control_mode = SPEED;
            break;
        case 0x300:
            dm_motor_obj[idx].set_control_mode = PVT;/*力位混控用不到,发送数据打包没写*/
            break;
        default:
            dm_motor_obj[idx].set_control_mode = MIT;
            break;
    }

    return &dm_motor_obj[idx++];
}

/**
  * @brief  封装一帧参数控制报文的数据帧
  * @param  para: 电机控制参数
  * @param  buf:  CAN数据帧
  * @retval
  */
uint8_t *pbuf,*vbuf;
static void pack_contol_para(dm_motor_object_t dm_motor_obiect,dm_motor_para_t para, uint8_t *buf)
{
    uint16_t p, v, kp, kd, t;

    switch (dm_motor_obiect.set_control_mode) {

        case MIT:

            /* 输入参数限幅 */
            LIMIT_MIN_MAX(para.p,  DM_P_MIN,  DM_P_MAX);
            LIMIT_MIN_MAX(para.v,  DM_V_MIN,  DM_V_MAX);
            LIMIT_MIN_MAX(para.kp, DM_KP_MIN, DM_KP_MAX);
            LIMIT_MIN_MAX(para.kd, DM_KD_MIN, DM_KD_MAX);
            LIMIT_MIN_MAX(para.t,  DM_T_MIN,  DM_T_MAX);


            /* 转换float参数 */
            p = float_to_uint(para.p,     DM_P_MIN,  DM_P_MAX,  16);
            v = float_to_uint(para.v,     DM_V_MIN,  DM_V_MAX,  12);
            kp = float_to_uint(para.kp,   DM_KP_MIN, DM_KP_MAX, 12);
            kd = float_to_uint(para.kd,   DM_KD_MIN, DM_KD_MAX, 12);
            t = float_to_uint(para.t,     DM_T_MIN,  DM_T_MAX,  12);

            /* 将参数存入CAN数据帧 */
            buf[0] = (p >> 8);
            buf[1] = p&0xFF;
            buf[2] = (v >> 4);
            buf[3] = ((v&0xF)<<4)|(kp>>8);
            buf[4] = kp&0xFF;
            buf[5] = (kd >> 4);
            buf[6] = ((kd&0xF)<<4)|(t>>8);
            buf[7] = t&0xFF;

            break;

        case POSITION_SPEED:

            pbuf=(uint8_t*)&para.p;
            vbuf=(uint8_t*)&para.v;

            buf[0] = *pbuf;;
            buf[1] = *(pbuf+1);
            buf[2] = *(pbuf+2);
            buf[3] = *(pbuf+3);
            buf[4] = *vbuf;
            buf[5] = *(vbuf+1);
            buf[6] = *(vbuf+2);
            buf[7] = *(vbuf+3);

            break;
        case SPEED:

            vbuf=(uint8_t*)&para.v;

            buf[0] = *vbuf;
            buf[1] = *(vbuf+1);
            buf[2] = *(vbuf+2);
            buf[3] = *(vbuf+3);

            break;

        default:
            break;


    }

}
void dm_motor_relax(dm_motor_object_t *motor)
{
    motor->stop_flag = MOTOR_STOP;
}

void dm_motor_enable(dm_motor_object_t *motor)
{
    motor->stop_flag = MOTOR_ENALBED;
}

/* 预留命令接口，可用于调试 */
static void enable(void){
    static dm_motor_para_t set; // 电机控制器计算得到的控制参数m
    for (size_t i = 0; i < idx; i++)
    {
        motor_set_mode(&dm_motor_obj[i], DM_CMD_MOTOR_MODE);
    }
}
//MSH_CMD_EXPORT(enable, enter motor_mode);

static void relax(void){
    for (size_t i = 0; i < idx; i++)
    {
        motor_set_mode(&dm_motor_obj[i], DM_CMD_RESET_MODE);
    }
}
//MSH_CMD_EXPORT(relax, out motor_mode);

static void zero(void){
    for (size_t i = 0; i < idx; i++)
    {
        motor_set_mode(&dm_motor_obj[i], DM_CMD_ZERO_POSITION);
    }
}
//MSH_CMD_EXPORT(zero, set motor zero);
