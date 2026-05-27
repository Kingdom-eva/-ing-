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

extern osThreadId USB_taskHandle;
extern fp32 INS_angle_deg[3];
extern float degree_set_pitch,degree_set_yaw;

#define CH_COUNT 4

struct Frame
{
    float fdata[CH_COUNT];
    unsigned char tail[4];
};
struct Frame sine_frame = {
    .fdata = {0},
    .tail = {0x00, 0x00, 0x80, 0x7f}
};

void USB_Task(void const * argument)
{
	while(1)
	{
	sine_frame.fdata[0]=degree_set_pitch;
	sine_frame.fdata[1]=INS_angle_deg[1];//pitch÷·
	sine_frame.fdata[2]=degree_set_yaw;
	sine_frame.fdata[3]=INS_angle_deg[0];//yaw÷·
	CDC_Transmit_FS((uint8_t*)&sine_frame,sizeof(struct Frame));
	vTaskDelay(10);
	}
}
