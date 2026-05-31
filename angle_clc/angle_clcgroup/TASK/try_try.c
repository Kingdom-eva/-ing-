#include "main.h"
#include "INS_Task.h"
#include "try_try.h"
#include "FreeRTos.h"
#include "cmsis_os.h"
#include "bmi088driver.h"
#include "ist8310driver.h"
#include "AHRS.h"

#include "bsp_can.h"
#include "CAN_receive.h"
#include "pid.h"
#include "try_try.h"
//extern osThreadId try_tryHandle;
#define PI_C   3.14159

fp32 torque;
extern fp32 INS_angle[3];//用来获取角度 单位：
extern motor_measure_t motor_chassis[8];//用来获取速度 单位：rpm


float kp_s_pitch=13.6f,ki_s_pitch=0.15f,kd_s_pitch=1.2f;//电机一pid速度环
float kp_p_pitch=4.78f,ki_p_pitch=0.1f,kd_p_pitch=1.0f;//角度环

float kp_s_yaw=4.0f,ki_s_yaw=0.0f,kd_s_yaw=0.0f;//电机二pid
float kp_p_yaw=6.5f,ki_p_yaw=0.1f,kd_p_yaw=1.5f;//角度环

float  degree_pitch,degree_yaw;
float  speed_pitch,speed_yaw;

pid_type_def motor_s_pitch,motor_p_pitch;
pid_type_def motor_s_yaw,motor_p_yaw;

float degree_set_pitch,degree_set_yaw;//设定目标角度
float speed_set_pitch,speed_set_yaw;//计算设定角度
float speed_set_yaw_s;
int16_t delta_pitch_torque,delta_yaw_torgue;//输出扭矩电流
uint8_t mode;
int16_t x1=-6000,x2=-6000,x3=-6000,x4=-6000;
uint16_t time;
float rd_yaw_set_degree;
uint16_t TIMER=0;

uint8_t exit_flag = 0;//MODE标志按钮
uint8_t rising_falling_flag;

uint8_t check_songshou=0;

void Try_Try(void const * argument){		

	can_filter_init();
//	DM4310_Disable();
//	HAL_Delay(50);

//	// 2. 修改CAN ID：旧ID=1，新ID=2
//	DM4310_SetCANID(1, 2);
//	HAL_Delay(100);

//	// 3. 保存参数到Flash（掉电不丢）
//	DM4310_SaveParams(2);
//	HAL_Delay(200);  // 保存耗时最长30ms
//	// 初始化CAN后
	HAL_Delay(500);
	DM4310_Enable();  // 使能 
	
	// 1. 先失能电机（必须！）
//DM4310_Disable();
//HAL_Delay(50);
//// mode: 1=MIT,2=位置速度,3=速度,4=力位混控
HAL_Delay(500);
DM4310_SetMode(3);
//// 3. 修改反馈上报ID（MST_ID）为2，解决调试里显示0x12的问题
////DM4310_SetMSTID(0x12, 2);  // 旧的出厂MST_ID是18（0x12）
////HAL_Delay(100);

//// 4. 保存参数到Flash
//DM4310_SaveParams(2U);
//HAL_Delay(200);

// 5. 断电重启电机！（这一步必须做，否则不生效）
//	
	float PID_s_pitch[3]={kp_s_pitch,ki_s_pitch,kd_s_pitch};
	float PID_p_pitch[3]={kp_p_pitch,ki_p_pitch,kd_p_pitch};
	
	float PID_p_yaw[3]={kp_p_yaw,ki_p_yaw,kd_p_yaw};
	float PID_s_yaw[3]={kp_s_yaw,ki_s_yaw,kd_s_yaw};
	PID_init(&motor_s_pitch,PID_POSITION,PID_s_pitch,750,100);
	PID_init(&motor_p_pitch,PID_POSITION,PID_p_pitch,450,80);
	
	PID_init(&motor_s_yaw,PID_POSITION,PID_s_pitch,200,80);
	PID_init(&motor_p_yaw,PID_POSITION,PID_p_yaw,450,40);
	
//	DM4310_SET_zero();
	while(1)
	{
	
		degree_yaw=INS_angle[0]*360.0f/(2.0f*PI_C);
		degree_pitch=INS_angle[1]*360.0f/(2.0f*PI_C);
		
		
		speed_pitch=(bmi088_real_data.gyro[0])*60/(2*PI_C);
		speed_yaw=(bmi088_real_data.gyro[2])*60/(2*PI_C);
		
		rd_yaw_set_degree=degree_set_yaw*(2.0f*PI_C)/360.0f;
		
		
		//speed_yaw=motor.chassis[i].speed_rpm;
		if(mode==0)//失能
		{

			CAN_CMD_BASE(&hcan1,0x200, 0,x2, x3,x4);
			DM4310_Disable();
			time=0;
			vTaskDelay(50);			
		}
		if(mode==1)
		{
			x1=-6500;
			x2=-6000;
			x3=-6000;
			x4=-6000;
			if(time==0) {DM4310_Enable(); time++;} // 使能 
			if(degree_set_pitch<-15.0f)
				degree_set_pitch=-15.0;
			if(degree_set_pitch>20.0f)
				degree_set_pitch=20.0f;
			if(degree_set_yaw>90.0f)
				degree_set_yaw=90.0f;
			if(degree_set_yaw<-90.0f)
				degree_set_yaw=-90.0f;
			
			
			PID_calc(&motor_p_pitch,degree_pitch,degree_set_pitch);
			PID_calc(&motor_p_yaw,degree_yaw,degree_set_yaw);
			
			
			
//			if((degree_set_pitch-degree_pitch)<0.2f&&(degree_set_pitch-degree_pitch)>-0.2f)
//			{
//				speed_set_pitch=0;
//				
//			}
//			
			if((degree_set_yaw-degree_yaw)<0.2f&&(degree_set_yaw-degree_yaw)>-0.2f)
			{
				speed_set_yaw=0;
			}
			
			speed_set_pitch=motor_p_pitch.out;
			speed_set_yaw=motor_p_yaw.out;
			
			PID_calc(&motor_s_pitch,speed_pitch,speed_set_pitch);
			PID_calc(&motor_s_yaw,speed_yaw,speed_set_yaw);

//			delta_pitch_torque=small_pitch_enable(motor_s_pitch.out);
			CAN_CMD_BASE(&hcan1,0x200,(motor_s_pitch.out*15),0,0,0);
			
			

			speed_set_yaw_s=(motor_p_yaw.out*0.4f*2.0f*PI_C)/60.0f;//speed_set_yaw:rpm   speed_set_yaw_s:rad/s
			if(motor_p_yaw.out<1.2f&&motor_p_yaw.out>-1.2f)
				speed_set_yaw_s=0;
			
			CAN_Speed_SendCmd(&hcan1,2U,speed_set_yaw_s);
		}
		if(mode==2)
		{
			
			TIMER+=2;
			degree_set_pitch=sinf((TIMER*PI_C)/180.0f)*10.0f;
			degree_set_yaw=sinf((TIMER*PI_C)/180.0f)*20.0f;
			if(time==0) {DM4310_Enable(); time++;} // 使能 
			if(degree_set_pitch<-15.0f)
				degree_set_pitch=-15.0;
			if(degree_set_pitch>20.0f)
				degree_set_pitch=20.0f;
			if(degree_set_yaw>90.0f)
				degree_set_yaw=90.0f;
			if(degree_set_yaw<-90.0f)
				degree_set_yaw=-90.0f;
			
			
			PID_calc(&motor_p_pitch,degree_pitch,degree_set_pitch);
			PID_calc(&motor_p_yaw,degree_yaw,degree_set_yaw);
			
			
			if((degree_set_yaw-degree_yaw)<0.2f&&(degree_set_yaw-degree_yaw)>-0.2f)
			{
				speed_set_yaw=0;
			}
			
			speed_set_pitch=motor_p_pitch.out;
			speed_set_yaw=motor_p_yaw.out;
			
			PID_calc(&motor_s_pitch,speed_pitch,speed_set_pitch);
			PID_calc(&motor_s_yaw,speed_yaw,speed_set_yaw);
			
			CAN_CMD_BASE(&hcan1,0x200,(motor_s_pitch.out*15),0,0,0);
			
			

			speed_set_yaw_s=(motor_p_yaw.out*0.4f*2.0f*PI_C)/60.0f;//speed_set_yaw:rpm   speed_set_yaw_s:rad/s
			if(motor_p_yaw.out<1.2f&&motor_p_yaw.out>-1.2f)
				speed_set_yaw_s=0;
			
			CAN_Speed_SendCmd(&hcan1,2U,speed_set_yaw_s);
			
		}
		if(mode>=3)
		{
			mode=0;
		}
		if(exit_flag == 1)
        {
            exit_flag = 2;
            if(rising_falling_flag == GPIO_PIN_RESET)
            {
                //debouce
                //消抖
                HAL_Delay(20);
                if(HAL_GPIO_ReadPin(KEY_GPIO_Port, KEY_Pin) == GPIO_PIN_RESET)
                {
					if(check_songshou==1)
					{
						mode++;
						check_songshou=0;
					}
                    exit_flag = 0;
                }
                else
                {
                    exit_flag = 0;
                }
            }
            else if(rising_falling_flag == GPIO_PIN_SET)
            {
                //debouce
                //消抖
                HAL_Delay(20);
                if(HAL_GPIO_ReadPin(KEY_GPIO_Port, KEY_Pin) == GPIO_PIN_SET)
                {
					check_songshou=1;
                    exit_flag = 0;
                }
                else
                {
                    exit_flag = 0;
                }
            }

        }
		vTaskDelay(5);
		
	}
}


int16_t small_pitch_enable(float motor_s_pitchs)
{
	if(motor_s_pitchs>250.0f||motor_s_pitchs<-300.0f)
		return (int16_t)(motor_s_pitchs*30);
	else if(motor_s_pitchs<=250.0f&&motor_s_pitchs>=100.0f)
		return (uint16_t)(motor_s_pitchs*30)*0.8f+1500.0f;
	else if(motor_s_pitchs<=-100.0f&&motor_s_pitchs>=-250.0f)
		return (int16_t)(motor_s_pitchs*30)*0.8f-1500.0f-2000.0f;
	else if(motor_s_pitchs>30.0f&&motor_s_pitchs<100.0f)
		return (int16_t)(motor_s_pitchs*30)*0.4f+2700.0f;
	else if(motor_s_pitchs>-100.0f&&motor_s_pitchs<-30.0f)
		return (int16_t)(motor_s_pitchs*30)*0.4f-2700.0f-2000.0f;
	else
		return 0;
}
