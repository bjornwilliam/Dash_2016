#include <sam.h>

#include "same70-base_16/FreeRTOS/Source/include/FreeRTOS.h"
#include "same70-base_16/FreeRTOS/Source/include/task.h"
#include "same70-base_16/FreeRTOS/Source/include/semphr.h"
#include "same70-base_16/FreeRTOS/Source/include/queue.h"



#include "hardwareInit.h"

#include "Task_Menu.h"
#include "Task_DataLogger.h"
#include "Task_RemoteControl.h"
#include "Task_DeviceStatus.h"
#include "Dash_drivers/IO_DashInit.h"

#include "Dash_drivers/spi_wrapper.h"



int main(void)
{
	
	hardwareInit();

	file_access_mutex			= xSemaphoreCreateMutex();
	//xButtonStruct				= xSemaphoreCreateMutex();
	
    spi_handlerIsDoneSempahore	= xSemaphoreCreateBinary();
    spi_mutex					= xSemaphoreCreateMutex();
	
	BaseType_t status;
	uint32_t bytesremaining;
	xDataloggerCommandQueue		= xQueueCreate(5,sizeof(uint8_t));
	//xPresetQueue				= xQueueCreate(2,sizeof(struct presetParameterStruct));	
	
	
	status = xTaskCreate(dashTask,"dashTask",500, NULL,  tskIDLE_PRIORITY + 3, NULL);
	bytesremaining = xPortGetFreeHeapSize();
	//status = xTaskCreate(usbMscTask,"MscTask",1000, NULL, tskIDLE_PRIORITY + 1, &mscTaskHandle);
	bytesremaining = xPortGetFreeHeapSize();
	status = xTaskCreate(dataLoggerTask,"Datalogger",2500,NULL,tskIDLE_PRIORITY +3, &dataLoggerHandle);
	bytesremaining = xPortGetFreeHeapSize();
	//status = xTaskCreate(Task_remoteControl,"remote",500, NULL, tskIDLE_PRIORITY + 2,NULL);
	bytesremaining = xPortGetFreeHeapSize();
	//xTaskCreate(deviceStatusTask,"device",500,NULL,tskIDLE_PRIORITY + 4,NULL);
	//xTaskCreate(Task_ButtonInput, "buttonTask", configMINIMAL_STACK_SIZE, NULL,  tskIDLE_PRIORITY + 1, NULL);
	//xTaskCreate(Task_RotaryEncoder,"rotary", configMINIMAL_STACK_SIZE,NULL, tskIDLE_PRIORITY + 2, NULL);
	vTaskStartScheduler();
	
	
	
    while (1) 
    {
    }
}
