#ifndef BSP_CAN_H
#define BSP_CAN_H
#include "can.h"
#include "struct_typedef.h"
#include "arm_math.h"

extern void can_filter_init(void);
extern void CAN_CMD_BASE(CAN_HandleTypeDef* hcan,uint32_t id, int16_t motor1, int16_t motor2, int16_t motor3, int16_t motor4);
extern void CAN_CMD_f32(CAN_HandleTypeDef* hcan,uint32_t id, fp32 data1,fp32 data2);

extern void CAN_MT_Sendcmd(CAN_HandleTypeDef* hcan,uint32_t id,float p_des,float v_des,float kp,float kd,float t_ff);
extern void float_to_bytes(float f,uint8_t *buf);
extern void DM4310_SET_zero(void);
extern uint16_t kd_clc(float kd);
extern uint16_t kp_clc(float kp);
extern uint16_t t_ff_clc(float t_ff);
extern uint16_t p_des_clc(float p_des);
extern uint16_t v_des_clc(float kd);
extern void CAN_Speed_SendCmd(CAN_HandleTypeDef* hcan,uint32_t id,float v_set);
#endif
