#include "bsp_can.h"
#include "CAN_receive.h"
//滤波器初始化
void can_filter_init(void)
{
		CAN_FilterTypeDef can_filter_st;
    can_filter_st.FilterActivation = ENABLE;
    can_filter_st.FilterMode = CAN_FILTERMODE_IDMASK;
    can_filter_st.FilterScale = CAN_FILTERSCALE_32BIT;
    can_filter_st.FilterIdHigh = 0x0000;
    can_filter_st.FilterIdLow = 0x0000;
    can_filter_st.FilterMaskIdHigh = 0x0000;
    can_filter_st.FilterMaskIdLow = 0x0000; 
    can_filter_st.FilterBank = 0;
    can_filter_st.FilterFIFOAssignment = CAN_RX_FIFO0;
    HAL_CAN_ConfigFilter(&hcan1, &can_filter_st);
    HAL_CAN_Start(&hcan1);
    HAL_CAN_ActivateNotification(&hcan1, CAN_IT_RX_FIFO0_MSG_PENDING);

    can_filter_st.SlaveStartFilterBank = 14;
    can_filter_st.FilterBank = 14;
    HAL_CAN_ConfigFilter(&hcan2, &can_filter_st);
    HAL_CAN_Start(&hcan2);
    HAL_CAN_ActivateNotification(&hcan2, CAN_IT_RX_FIFO0_MSG_PENDING);
}

//void can_filter_init(void)
//{

//    CAN_FilterTypeDef can_filter_st;
//    can_filter_st.FilterActivation = ENABLE;
//    can_filter_st.FilterMode = CAN_FILTERMODE_IDMASK;
//    can_filter_st.FilterScale = CAN_FILTERSCALE_32BIT;
//    can_filter_st.FilterIdHigh = 0x0000;
//    can_filter_st.FilterIdLow = 0x0000;
//    can_filter_st.FilterMaskIdHigh = 0x0000;
//    can_filter_st.FilterMaskIdLow = 0x0000;
//    can_filter_st.FilterBank = 0;
//    can_filter_st.FilterFIFOAssignment = CAN_RX_FIFO0;
//    HAL_CAN_ConfigFilter(&hcan1, &can_filter_st);
//    HAL_CAN_Start(&hcan1);
//    HAL_CAN_ActivateNotification(&hcan1, CAN_IT_RX_FIFO0_MSG_PENDING);


//    can_filter_st.SlaveStartFilterBank = 14;
//    can_filter_st.FilterBank = 14;
//    HAL_CAN_ConfigFilter(&hcan2, &can_filter_st);
//    HAL_CAN_Start(&hcan2);
//    HAL_CAN_ActivateNotification(&hcan2, CAN_IT_RX_FIFO0_MSG_PENDING);



//}

/*can发送控制4个同一类id电机*/
void CAN_CMD_BASE(CAN_HandleTypeDef* hcan,uint32_t id, int16_t motor1, int16_t motor2, int16_t motor3, int16_t motor4)
{
	uint32_t send_mail_box;
	CAN_TxHeaderTypeDef  chassis_tx_message ={0};
	uint8_t              chassis_can_send_data[8];
	chassis_tx_message.StdId = id;
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
  
	HAL_CAN_AddTxMessage(hcan, &chassis_tx_message, chassis_can_send_data, &send_mail_box);
}

void CAN_CMD_f32(CAN_HandleTypeDef* hcan,uint32_t id, fp32 data1,fp32 data2)
{
    uint32_t send_mail_box;
    CAN_TxHeaderTypeDef  chassis_tx_message ={0};
    uint8_t              chassis_can_send_data[8];
    chassis_tx_message.StdId = id;
    chassis_tx_message.IDE = CAN_ID_STD;
    chassis_tx_message.RTR = CAN_RTR_DATA;
    chassis_tx_message.DLC = 0x08;
    memcpy(chassis_can_send_data, (uint8_t*)&data1 ,4);
    memcpy(chassis_can_send_data+4, (uint8_t*)&data2 ,4);
  
    HAL_CAN_AddTxMessage(hcan, &chassis_tx_message, chassis_can_send_data, &send_mail_box);
}

//CAN地址为0x202（可以改变）
void CAN_MT_Sendcmd(CAN_HandleTypeDef* hcan,uint32_t id,float p_des,float v_des,float kp,float kd,float t_ff)
{
	uint32_t tx_mailbox;
	CAN_TxHeaderTypeDef MT_sendMessage={0};
	uint8_t MT_senddata[8];
	
	uint16_t u_p =  p_des_clc(p_des);
    uint16_t u_v =  v_des_clc(v_des);
	uint16_t u_kp = kp_clc(kp);
	uint16_t u_kd = kd_clc(kd);
	uint16_t u_t = t_ff_clc(t_ff);//映射
	
	MT_sendMessage.StdId = id;
	MT_sendMessage.IDE = CAN_ID_STD;
	MT_sendMessage.RTR = CAN_RTR_DATA;
	MT_sendMessage.DLC = 0X08;
	MT_senddata[0]=u_p >> 8;
	MT_senddata[1]=u_p;
	MT_senddata[2]=u_v >> 4;
	MT_senddata[3]=((u_v & 0x0F) << 4)|((u_kp >> 8) & 0x0F);
	MT_senddata[4]=u_kp;
	MT_senddata[5]=u_kd >> 4;
	MT_senddata[6]=((u_kd & 0x0F) << 4)|((u_t >> 8) & 0x0F);
	MT_senddata[7]=u_t;
	
	HAL_CAN_AddTxMessage(hcan,&MT_sendMessage,MT_senddata,&tx_mailbox);
	}//p_des范围（p_des为-12.5~12.5单位rad，v_des为-30~30单位rad/s，kp范围是0~500，kd是0~5，t_ff为-7~7N·m）
void DM4310_SET_zero(void)
{
    uint32_t tx_mailbox;
    CAN_TxHeaderTypeDef tx_header;
    uint8_t data[8] = {0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFE};

    tx_header.StdId = DM4310_CTRL_ID;
    tx_header.IDE = CAN_ID_STD;
    tx_header.RTR = CAN_RTR_DATA;
    tx_header.DLC = 8;

    HAL_CAN_AddTxMessage(&hcan1, &tx_header, data, &tx_mailbox);
}

uint16_t p_des_clc(float p_des)
{
	if(p_des< -12.5f) p_des=-12.5f;
	if(p_des> 12.5f) p_des=12.5f;
	return (uint16_t)((p_des+12.5f)/25.0f*65535.0f);
}

uint16_t v_des_clc(float v_des)
{
	if(v_des<-30.0f) v_des=-30.0f;
	if(v_des>30.0f) v_des=30.0f;
	return (uint16_t)((v_des+30.0f)/60.0f*4095.0f);
}

uint16_t kp_clc(float kp)
{
	if(kp<0.0f) kp=0.0f;
	if(kp>500.0f) kp=500.0f;
	return (uint16_t)((kp-0.0f)/500.0f*4095.0f);
}

uint16_t kd_clc(float kd)
{
	if(kd<0) kd=0.0f;
	if(kd>5.0f) kd=5.0f;
	return (uint16_t)((kd-0.0f)/5.0f*4095.0f);
}

uint16_t t_ff_clc(float t_ff)
{
	if(t_ff< -7.0f) t_ff=7.0f;
	if(t_ff> 7.0f) t_ff=7.0f;
	return (uint16_t)((t_ff+7.0f)/14.0f*4095.0f);
}














void CAN_Speed_SendCmd(CAN_HandleTypeDef* hcan,uint32_t id,float v_set)
{
	uint32_t tx_mailbox;
	CAN_TxHeaderTypeDef Speed_sendMessage={0};
	uint8_t Speed_senddata[4];
	
	Speed_sendMessage.StdId=0x200+id;
	Speed_sendMessage.IDE=CAN_ID_STD;
	Speed_sendMessage.RTR=CAN_RTR_DATA;
	Speed_sendMessage.DLC = 4;
	
	if(v_set> 20.0f) v_set=20.0f;
	if(v_set<-20.0f) v_set=-20.0f;
	
	uint32_t v_deal=(uint32_t)v_set;
	
	memcpy(Speed_senddata,&v_set,4);
	
	HAL_CAN_AddTxMessage(hcan, &Speed_sendMessage,Speed_senddata,&tx_mailbox);
}
//速度模式下的控制帧模式，v_set单位rad/s，范围-20~20.


