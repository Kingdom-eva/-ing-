/**
  ****************************(C) COPYRIGHT 2019 DJI****************************
  * @file       can_receive.c/h
  * @brief      there is CAN interrupt function  to receive motor data,
  *             and CAN send function to send motor current to control motor.
  *             这里是CAN中断接收函数，接收电机数据,CAN发送函数发送电机电流控制电机.
  * @note       
  * @history
  *  Version    Date            Author          Modification
  *  V1.0.0     Dec-26-2018     RM              1. done
  *
  @verbatim
  ==============================================================================

  ==============================================================================
  @endverbatim
  ****************************(C) COPYRIGHT 2019 DJI****************************
  */

#include "CAN_receive.h"
#include "main.h"




// DM4310 反馈结构体（手册反馈帧格式）
typedef struct
{
    uint8_t  id;          // 电机ID
    uint8_t  err;          // 状态/错误码
    int16_t  pos;          // 位置（16位）
    int16_t  vel;          // 速度（12位）
    int16_t  torque;       // 扭矩（12位）
    uint8_t  t_mos;        // 驱动温度
    uint8_t  t_rotor;      // 电机温度
} dm4310_measure_t;

typedef struct
{
	float pos_deg_d;
	float pos_rad_d;
	float vel_d;
	float torgue_d;
}deal_dm4310;

// 全局实例
dm4310_measure_t dm4310;

deal_dm4310 deal_dm4310_d;


extern CAN_HandleTypeDef hcan1;
extern CAN_HandleTypeDef hcan2;
//motor data read
#define get_motor_measure(ptr, data)                                    \
    {                                                                   \
        (ptr)->last_ecd = (ptr)->ecd;                                   \
        (ptr)->ecd = (uint16_t)((data)[0] << 8 | (data)[1]);            \
        (ptr)->speed_rpm = (uint16_t)((data)[2] << 8 | (data)[3]);      \
        (ptr)->given_current = (uint16_t)((data)[4] << 8 | (data)[5]);  \
        (ptr)->temperate = (data)[6];                                   \
    }
/*
motor data,  0:chassis motor1 3508;1:chassis motor3 3508;2:chassis motor3 3508;3:chassis motor4 3508;
4:yaw gimbal motor 6020;5:pitch gimbal motor 6020;6:trigger motor 2006;
电机数据, 0:底盘电机1 3508电机,  1:底盘电机2 3508电机,2:底盘电机3 3508电机,3:底盘电机4 3508电机;
4:yaw云台电机 6020电机; 5:pitch云台电机 6020电机; 6:拨弹电机 2006电机*/
motor_measure_t motor_chassis[8];

static CAN_TxHeaderTypeDef  gimbal_tx_message;
static uint8_t              gimbal_can_send_data[8];
static CAN_TxHeaderTypeDef  chassis_tx_message;
static uint8_t              chassis_can_send_data[8];

/**
  * @brief          hal CAN fifo call back, receive motor data
  * @param[in]      hcan, the point to CAN handle
  * @retval         none
  */
/**
  * @brief          hal库CAN回调函数,接收电机数据
  * @param[in]      hcan:CAN句柄指针
  * @retval         none
  */
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
    CAN_RxHeaderTypeDef rx_header;
    uint8_t rx_data[8];

    HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &rx_header, rx_data);

    switch (rx_header.StdId)
    {
        case CAN_3508_M1_ID:
        case CAN_3508_M2_ID:
        case CAN_3508_M3_ID:
        case CAN_3508_M4_ID:
        case CAN_YAW_MOTOR_ID:
        case CAN_PIT_MOTOR_ID:
        case CAN_TRIGGER_MOTOR_ID:
        {
            static uint8_t i = 0;
            //get motor id
            i = rx_header.StdId - CAN_3508_M1_ID;
            get_motor_measure(&motor_chassis[i], rx_data);
            break;
        }
		
		
		//DM4310
		case DM4310_FB_ID:
        {
            dm4310.id      = rx_data[0]&0x0f;
            dm4310.err     = (rx_data[0] >> 4)&0x0F;
            dm4310.pos     = (int16_t)((rx_data[1] << 8) | rx_data[2]);
			deal_dm4310_d.pos_deg_d=(dm4310.pos/65535.0f)*360.0f*2;
			deal_dm4310_d.pos_rad_d=(dm4310.pos/65535.0f)*2.0f*3.14159f;
            dm4310.vel     = ((rx_data[3] << 4) |( (rx_data[4]>>4)&0x0F ));
			int16_t vel_mid;
			vel_mid=(int16_t)dm4310.vel-0x07FF;
			deal_dm4310_d.vel_d =vel_mid *0.01f;//rad/s
			deal_dm4310_d.vel_d=deal_dm4310_d.vel_d*60.0f/(2*3.14159);
            dm4310.torque  = ((rx_data[4] & 0x0F) << 8) | rx_data[5];
            dm4310.t_mos   = rx_data[6];
			dm4310.t_rotor = rx_data[7];
			
            // rx_data[8] 不存在，手册最后一字节是T_Rotor，上面已经解析完
            break;
        }
		
//		case CAN_TRIGGER_MOTOR_ID:
//        {
//            static uint8_t i = 0;
//            //get motor id
//            i = rx_header.StdId - CAN_3508_M1_ID;
//            get_motor_measure(&motor_chassis[i], rx_data);
//            break;
//        }
//		
	
        default:
        {
            break;
        }
    }
}

void DM4310_Enable(void)
{
    uint32_t tx_mailbox;
    CAN_TxHeaderTypeDef tx_header;
    uint8_t data[8] = {0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFC};

    tx_header.StdId = DM4310_CTRL_ID;
    tx_header.IDE = CAN_ID_STD;
    tx_header.RTR = CAN_RTR_DATA;
    tx_header.DLC = 8;

    HAL_CAN_AddTxMessage(&hcan1, &tx_header, data, &tx_mailbox);
}

// DM4310 失能
void DM4310_Disable(void)
{
    uint32_t tx_mailbox;
    CAN_TxHeaderTypeDef tx_header;
    uint8_t data[8] = {0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFD};

    tx_header.StdId = DM4310_CTRL_ID;
    tx_header.IDE = CAN_ID_STD;
    tx_header.RTR = CAN_RTR_DATA;
    tx_header.DLC = 8;

    HAL_CAN_AddTxMessage(&hcan1, &tx_header, data, &tx_mailbox);
}

// mode: 1=MIT,2=位置速度,3=速度,4=力位混控
void DM4310_SetMode(uint8_t mode)
{
    uint32_t tx_mailbox;
    CAN_TxHeaderTypeDef tx_header;
    // 写参数帧：ID=0x7FF, 格式见手册
    uint8_t data[8] = {
        DM4310_ESC_ID & 0xFF, (DM4310_ESC_ID>>8)&0xFF,
        0x55, 0x0A, // 0x55=写, 0x0A=模式寄存器
        mode,0,0,0
    };

    tx_header.StdId = 0x7FF;
    tx_header.IDE = CAN_ID_STD;
    tx_header.RTR = CAN_RTR_DATA;
    tx_header.DLC = 8;

    HAL_CAN_AddTxMessage(&hcan1, &tx_header, data, &tx_mailbox);
}

void DM4310_SetCANID(uint8_t old_id, uint8_t new_id)
{
    uint32_t tx_mailbox;
    CAN_TxHeaderTypeDef tx_header;
    uint8_t data[8] = {0};

    // 写参数帧格式：ID=0x7FF
    data[0] = old_id & 0xFF;         // 旧ID低字节
    data[1] = (old_id >> 8) & 0xFF;  // 旧ID高字节
    data[2] = 0x55;                    // 写参数标志
    data[3] = DM4310_REG_ESC_ID;      // 目标寄存器：ESC_ID
    data[4] = new_id & 0xFF;          // 新ID低字节
    data[5] = (new_id >> 8) & 0xFF;   // 新ID高字节
    data[6] = 0x00;
    data[7] = 0x00;

    tx_header.StdId = 0x7FF;
    tx_header.IDE = CAN_ID_STD;
    tx_header.RTR = CAN_RTR_DATA;
    tx_header.DLC = 8;

    HAL_CAN_AddTxMessage(&hcan1, &tx_header, data, &tx_mailbox);
}

void DM4310_SaveParams(uint8_t can_id)
{
    uint32_t tx_mailbox;
    CAN_TxHeaderTypeDef tx_header;
    uint8_t data[8] = {0};

    data[0] = can_id & 0xFF;
    data[1] = (can_id >> 8) & 0xFF;
    data[2] = 0xAA;  // 保存参数标志
    data[3] = DM4310_REG_SAVE;
    data[4] = 0x00;
    data[5] = 0x00;
    data[6] = 0x00;
    data[7] = 0x00;

    tx_header.StdId = 0x7FF;
    tx_header.IDE = CAN_ID_STD;
    tx_header.RTR = CAN_RTR_DATA;
    tx_header.DLC = 8;

    HAL_CAN_AddTxMessage(&hcan1, &tx_header, data, &tx_mailbox);
}

// 修改电机上报的反馈ID（解决调试里一直显示0x12的问题）
void DM4310_SetMSTID(uint8_t old_mst_id, uint8_t new_mst_id)
{
    uint32_t tx_mailbox;
    CAN_TxHeaderTypeDef tx_header;
    uint8_t data[8] = {0};

    data[0] = old_mst_id & 0xFF;
    data[1] = (old_mst_id >> 8) & 0xFF;
    data[2] = 0x55;                    // 写参数标志
    data[3] = DM4310_REG_MST_ID;       // 目标寄存器：MST_ID
    data[4] = new_mst_id & 0xFF;
    data[5] = (new_mst_id >> 8) & 0xFF;
    data[6] = 0x00;
    data[7] = 0x00;

    tx_header.StdId = 0x7FF;
    tx_header.IDE = CAN_ID_STD;
    tx_header.RTR = CAN_RTR_DATA;
    tx_header.DLC = 8;

    HAL_CAN_AddTxMessage(&hcan1, &tx_header, data, &tx_mailbox);
}

/**
  * @brief          send control current of motor (0x205, 0x206, 0x207, 0x208)
  * @param[in]      yaw: (0x205) 6020 motor control current, range [-30000,30000] 
  * @param[in]      pitch: (0x206) 6020 motor control current, range [-30000,30000]
  * @param[in]      shoot: (0x207) 2006 motor control current, range [-10000,10000]
  * @param[in]      rev: (0x208) reserve motor control current
  * @retval         none
  */
/**
  * @brief          发送电机控制电流(0x205,0x206,0x207,0x208)
  * @param[in]      yaw: (0x205) 6020电机控制电流, 范围 [-30000,30000]
  * @param[in]      pitch: (0x206) 6020电机控制电流, 范围 [-30000,30000]
  * @param[in]      shoot: (0x207) 2006电机控制电流, 范围 [-10000,10000]
  * @param[in]      rev: (0x208) 保留，电机控制电流
  * @retval         none
  */
void CAN_cmd_gimbal(int16_t yaw, int16_t pitch, int16_t shoot, int16_t rev)
{
    uint32_t send_mail_box;
    gimbal_tx_message.StdId = CAN_GIMBAL_ALL_ID;
    gimbal_tx_message.IDE = CAN_ID_STD;
    gimbal_tx_message.RTR = CAN_RTR_DATA;
    gimbal_tx_message.DLC = 0x08;
    gimbal_can_send_data[0] = (yaw >> 8);
    gimbal_can_send_data[1] = yaw;
    gimbal_can_send_data[2] = (pitch >> 8);
    gimbal_can_send_data[3] = pitch;
    gimbal_can_send_data[4] = (shoot >> 8);
    gimbal_can_send_data[5] = shoot;
    gimbal_can_send_data[6] = (rev >> 8);
    gimbal_can_send_data[7] = rev;
    HAL_CAN_AddTxMessage(&GIMBAL_CAN, &gimbal_tx_message, gimbal_can_send_data, &send_mail_box);
}

/**
  * @brief          send CAN packet of ID 0x700, it will set chassis motor 3508 to quick ID setting
  * @param[in]      none
  * @retval         none
  */
/**
  * @brief          发送ID为0x700的CAN包,它会设置3508电机进入快速设置ID
  * @param[in]      none
  * @retval         none
  */
void CAN_cmd_chassis_reset_ID(void)
{
    uint32_t send_mail_box;
    chassis_tx_message.StdId = 0x700;
    chassis_tx_message.IDE = CAN_ID_STD;
    chassis_tx_message.RTR = CAN_RTR_DATA;
    chassis_tx_message.DLC = 0x08;
    chassis_can_send_data[0] = 0;
    chassis_can_send_data[1] = 0;
    chassis_can_send_data[2] = 0;
    chassis_can_send_data[3] = 0;
    chassis_can_send_data[4] = 0;
    chassis_can_send_data[5] = 0;
    chassis_can_send_data[6] = 0;
    chassis_can_send_data[7] = 0;

    HAL_CAN_AddTxMessage(&CHASSIS_CAN, &chassis_tx_message, chassis_can_send_data, &send_mail_box);
}


/**
  * @brief          send control current of motor (0x201, 0x202, 0x203, 0x204)
  * @param[in]      motor1: (0x201) 3508 motor control current, range [-16384,16384] 
  * @param[in]      motor2: (0x202) 3508 motor control current, range [-16384,16384] 
  * @param[in]      motor3: (0x203) 3508 motor control current, range [-16384,16384] 
  * @param[in]      motor4: (0x204) 3508 motor control current, range [-16384,16384] 
  * @retval         none
  */
/**
  * @brief          发送电机控制电流(0x201,0x202,0x203,0x204)
  * @param[in]      motor1: (0x201) 3508电机控制电流, 范围 [-16384,16384]
  * @param[in]      motor2: (0x202) 3508电机控制电流, 范围 [-16384,16384]
  * @param[in]      motor3: (0x203) 3508电机控制电流, 范围 [-16384,16384]
  * @param[in]      motor4: (0x204) 3508电机控制电流, 范围 [-16384,16384]
  * @retval         none
  */
void CAN_cmd_chassis(int16_t motor1, int16_t motor2, int16_t motor3, int16_t motor4)
{
    uint32_t send_mail_box;
    chassis_tx_message.StdId = CAN_CHASSIS_ALL_ID;
    chassis_tx_message.IDE = CAN_ID_STD;
    chassis_tx_message.RTR = CAN_RTR_DATA;
    chassis_tx_message.DLC = 0x08;
    chassis_can_send_data[0] = motor1 >> 8;
    chassis_can_send_data[1] = motor1;
    chassis_can_send_data[2] = motor2 >> 8;
    chassis_can_send_data[3] = motor2;
    chassis_can_send_data[4] = motor3 >> 8;
    chassis_can_send_data[5] = motor3;
    chassis_can_send_data[6] = motor4 >> 8;
    chassis_can_send_data[7] = motor4;

    HAL_CAN_AddTxMessage(&CHASSIS_CAN, &chassis_tx_message, chassis_can_send_data, &send_mail_box);
}

/**
  * @brief          return the yaw 6020 motor data point
  * @param[in]      none
  * @retval         motor data point
  */
/**
  * @brief          返回yaw 6020电机数据指针
  * @param[in]      none
  * @retval         电机数据指针
  */
const motor_measure_t *get_yaw_gimbal_motor_measure_point(void)
{
    return &motor_chassis[4];
}

/**
  * @brief          return the pitch 6020 motor data point
  * @param[in]      none
  * @retval         motor data point
  */
/**
  * @brief          返回pitch 6020电机数据指针
  * @param[in]      none
  * @retval         电机数据指针
  */
const motor_measure_t *get_pitch_gimbal_motor_measure_point(void)
{
    return &motor_chassis[5];
}


/**
  * @brief          return the trigger 2006 motor data point
  * @param[in]      none
  * @retval         motor data point
  */
/**
  * @brief          返回拨弹电机 2006电机数据指针
  * @param[in]      none
  * @retval         电机数据指针
  */
const motor_measure_t *get_trigger_motor_measure_point(void)
{
    return &motor_chassis[6];
}


/**
  * @brief          return the chassis 3508 motor data point
  * @param[in]      i: motor number,range [0,3]
  * @retval         motor data point
  */
/**
  * @brief          返回底盘电机 3508电机数据指针
  * @param[in]      i: 电机编号,范围[0,3]
  * @retval         电机数据指针
  */
const motor_measure_t *get_chassis_motor_measure_point(uint8_t i)
{
    return &motor_chassis[(i & 0x03)];
}
