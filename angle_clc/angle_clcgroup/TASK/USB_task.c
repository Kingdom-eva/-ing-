#include "main.h"
#include "cmsis_os.h"
#include "dma.h"
#include "spi.h"
#include "tim.h"
#include "usb_device.h"
#include "gpio.h"

#include "CAN_receive.h"
#include "usbd_cdc_if.h"
#include "try_try.h"
#include "INS_Task.h"
#include "pid.h"

extern osThreadId USB_taskHandle;
extern fp32 INS_angle_deg[3];
extern float degree_set_pitch,degree_set_yaw;
extern uint8_t mode;
extern  bmi088_real_data_t bmi088_real_data;
extern  pid_type_def motor_s_pitch,motor_p_pitch;

#define CH_COUNT 6

uint8_t checkout=0;
struct Frame
{
    float fdata[CH_COUNT];
    unsigned char tail[4];
};
struct Frame sine_frame = {
    .fdata = {0},
    .tail = {0x00, 0x00, 0x80, 0x7f}
};

struct gimbal_and_config_data_t 
{
	 uint8_t header;
	 uint8_t length;
	uint8_t id;
	 uint8_t mode; // 工作模式
	 float roll; // 横滚角（度）
	 float pitch; // 俯仰角（度）
	 float yaw; // 偏航角（度）};
	uint8_t crc8;
	 
};

struct aim_data_t {
	uint8_t header;
	uint8_t length;
	uint8_t id;
 uint8_t success; // 开火许可
 float pitch; // 目标俯仰角（度）
 float yaw; // 目标偏航角（度）
 float distance; // 目标距离（保留字段，当前未使用） float w_pitch; // 俯仰角速度前馈（度/秒）
 float w_yaw; // 偏航角速度前馈（度/秒）
 float jmp_time; // 保留字段（当前未赋值）
 uint8_t target_rate; // 目标弹频（当前硬编码 20）
 uint8_t target_number; // 目标装甲板编号
	uint8_t crc8;
	
};

uint8_t check_check(uint8_t header,uint8_t length,uint8_t id,uint8_t crc8)
{
	if(header!=0xFF)
		return 0;
	if(length!=sizeof(struct aim_data_t))
		return 0;
	if (crc8!=0x0D)
		return 0;
	if(id!=0x81)
		return 0;
	return 1;
}


struct gimbal_and_config_data_t sendmessage=
{
	.header=0xFF,
	.length=sizeof(struct gimbal_and_config_data_t),
	.id=0x14,
	.crc8=0x0D
};
struct aim_data_t receivemessage;




void USB_Task(void const * argument)
{
	while(1)
	{
	sine_frame.fdata[0]=degree_set_pitch;
	sine_frame.fdata[1]=INS_angle_deg[1];//pitch轴
	sine_frame.fdata[2]=degree_set_yaw;
	sine_frame.fdata[3]=INS_angle_deg[0];//yaw轴
	sine_frame.fdata[4]=bmi088_real_data.gyro[0];
	sine_frame.fdata[5]=motor_p_pitch.out;
		
	sendmessage.roll=INS_angle_deg[2];
	sendmessage.pitch=INS_angle_deg[1];
	sendmessage.yaw=INS_angle_deg[0];
	sendmessage.mode=mode;

		
	CDC_Transmit_FS((uint8_t*)&sine_frame,sizeof(struct Frame));
	CDC_Transmit_FS((uint8_t*)&sendmessage,sizeof(struct gimbal_and_config_data_t ));
	CDC_Transmit_FS((uint8_t*)&receivemessage,sizeof(struct aim_data_t));
	
		
	vTaskDelay(10);
	}
}
