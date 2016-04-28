#ifndef MCANFREERTOSWRAPPER_H_
#define MCANFREERTOSWRAPPER_H_

#include "../same70-base_16/FreeRTOS/Source/include/FreeRTOS.h"
#include "../same70-base_16/FreeRTOS/Source/include/queue.h"
#include "../same70-base_16/FreeRTOS/Source/include/semphr.h"


#include "../same70-base_16/RevolveDrivers/CAN/can.h"

extern QueueHandle_t xDeviceStatusQueue;
extern QueueHandle_t xRemoteControlQueue;
extern QueueHandle_t xDataLoggerQueue;
extern QueueHandle_t xDashQueue;

extern uint32_t can_send_to_datalogger_queue_failed;


extern QueueHandle_t Queue_CAN_to_Safety;
extern QueueHandle_t Queue_CAN_to_TV;

enum status_code mcan_freeRTOSSendMessage(Mcan *mcan, struct Can_message_t message);

void mcan_freeRTOSSetup();

#endif /* MCANFREERTOSWRAPPER_H_ */