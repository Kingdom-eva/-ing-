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

fp32 gyro[3], accel[3], temp;
void usb_task(void const * argument)
{
	while(BMI088_init())
{
	;
}
	while(1)
	{
	 BMI088_read(gyro, accel, &temp);
	vTaskDelay(1);
	}
}