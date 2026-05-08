#include "main.h"
#include "cmsis_os.h"
#include "dma.h"
#include "spi.h"
#include "tim.h"
#include "usb_device.h"
#include "gpio.h"

#include "FreeRTOS.h"
#include "task.h"
#include "main.h"
#include "cmsis_os.h"

#include "BMI088driver.h"
#include "BMI088reg.h"
#include "BMI088Middleware.h"
#include "usbd_cdc_if.h"
extern osThreadId Usb_taskHandle;

#define CH_COUNT 3

struct Frame
{
    float fdata[CH_COUNT];
    unsigned char tail[4];
};
struct Frame sine_frame = {
    .fdata = {0},
    .tail = {0x00, 0x00, 0x80, 0x7f}
};


fp32 gyro[3], accel[3], temp;
uint8_t tx_buf[12];
void float_to_bytes(float f,uint8_t *buf)//将1float转化为4bit
{
	union
	{
		float f;
		uint8_t b[4];
	}u;
	u.f=f;
	buf[0]=u.b[0];
	buf[1]=u.b[1];
	buf[2]=u.b[2];
	buf[3]=u.b[3];
}//联合
void usb_task(void const * argument)
{
	while(BMI088_init())
{
	;
}
	while(1)
	{
	 BMI088_read(gyro, accel, &temp);
	float_to_bytes(gyro[0],&tx_buf[0]);
	float_to_bytes(gyro[1],&tx_buf[4]);
	float_to_bytes(gyro[2],&tx_buf[8]);
	sine_frame.fdata[0]=gyro[0];
		sine_frame.fdata[1]=gyro[1];
		sine_frame.fdata[2]=gyro[2];
		CDC_Transmit_FS((uint8_t*)&sine_frame,sizeof(struct Frame));
	vTaskDelay(10);
	}
}