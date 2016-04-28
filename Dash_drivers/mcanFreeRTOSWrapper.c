#include "mcanFreeRTOSWrapper.h"
#include "../same70-base_16/RevolveDrivers/CAN/mcan.h"

#include "../revolve_can_definitions_16/revolve_can_definitions.h"


#include "../Dash_drivers/canMessages.h" // JUST FOR TEST ; REMOVE



SemaphoreHandle_t	can_mutex_0				= NULL;
SemaphoreHandle_t	can_mutex_1				= NULL;

QueueHandle_t		Queue_CAN_to_Safety		= NULL;
QueueHandle_t		Queue_CAN_to_TV			= NULL;

#define CAN_SEND_RETRY_LIMIT			10000
static uint32_t can_retry_counter_mcan_0 = 0;
static uint32_t can_retry_counter_mcan_1 = 0;

enum status_code mcan_freeRTOSSendMessage(Mcan *mcan, struct Can_message_t message) {
	enum status_code can_status;
	if (mcan == MCAN0) {
		xSemaphoreTake(can_mutex_0, portMAX_DELAY); 
		can_retry_counter_mcan_0 = 0;
		while ( (can_sendMessage(mcan,&message) != STATUS_OK) && (can_retry_counter_mcan_0 < CAN_SEND_RETRY_LIMIT)) {
			can_retry_counter_mcan_0 += 1;
		}	
		//can_status = can_sendMessage(mcan,&message);		
		xSemaphoreGive(can_mutex_0);
	}
	else if (mcan == MCAN1) {
		xSemaphoreTake(can_mutex_1, portMAX_DELAY);
		can_retry_counter_mcan_1 = 0;
		while ( (can_sendMessage(mcan,&message) != STATUS_OK) && (can_retry_counter_mcan_1 < CAN_SEND_RETRY_LIMIT)) {
			can_retry_counter_mcan_1 += 1;
		}
		//can_status = can_sendMessage(mcan,&message);
		
		xSemaphoreGive(can_mutex_1);
	}
	//return can_status;
}
QueueHandle_t xDeviceStatusQueue = NULL;
QueueHandle_t xRemoteControlQueue = NULL;
QueueHandle_t xDataLoggerQueue = NULL;
QueueHandle_t xDashQueue = NULL;
uint32_t can_send_to_datalogger_queue_failed = 0;


#define DATALOGGER_QUEUE_SIZE 250

struct SensorMessage {
	struct Can_message_t CanMsg;
	uint32_t time_stamp;
};
struct SensorMessage SensorPacket;


void mcan0_CallbackFunction(uint32_t index) {
	struct Can_message_t message;
	can_getMessage(MCAN0, &message, index);
	SensorPacket.CanMsg = message;
	SensorPacket.time_stamp = 0;
	if (xQueueSendToBackFromISR(xDataLoggerQueue,&SensorPacket,NULL) != pdTRUE) {
		can_send_to_datalogger_queue_failed += 1;
	}
	//xQueueSendToBackFromISR(Queue_CAN_to_Safety, &message, NULL);
	//xQueueSendToBackFromISR(Queue_CAN_to_Safety, &message, NULL);
	//xQueueSendToBackFromISR(Queue_CAN_to_TV, &message, NULL);
}

void mcan1_CallbackFunction(uint32_t index) {
	struct Can_message_t message;
	can_getMessage(MCAN1, &message, index);
	SensorPacket.CanMsg = message;
	SensorPacket.time_stamp = 0;
	if (xQueueSendToBackFromISR(xDataLoggerQueue,&SensorPacket,NULL) != pdTRUE) {
		can_send_to_datalogger_queue_failed += 1;
	}	
}



// Hack to read a fraction of the inverter messages, since they are sent way too often.
#define INVERTER_MESSAGE_DELAY_COUNTER 80
static uint32_t inverter_time_counter_status = 0;
static uint32_t inverter_time_counter_voltage = 0;





static uint8_t torque_alive = 0;

void mcan_freeRTOSSetup() {
		SensorPacket.CanMsg.data.u64  = 0;
		SensorPacket.CanMsg.dataLength = 0;
		SensorPacket.CanMsg.messageID = 0;
		SensorPacket.time_stamp = 0;
	
	//Queue_CAN_to_Safety			= xQueueCreate(20,sizeof(struct Can_message_t));
	//Queue_CAN_to_TV				= xQueueCreate(20,sizeof(struct Can_message_t));
	xDataLoggerQueue	= xQueueCreate(DATALOGGER_QUEUE_SIZE,sizeof(SensorPacket));
	xDashQueue			= xQueueCreate(20,sizeof(struct Can_message_t));
	// Only needs one byte to identify the different buttons
	xRemoteControlQueue = xQueueCreate(20,sizeof(uint8_t));
	xDeviceStatusQueue	= xQueueCreate(20,sizeof(uint8_t));
	
	
	can_mutex_0					= xSemaphoreCreateMutex();
	can_mutex_1					= xSemaphoreCreateMutex();
	mcan_init(MCAN0, CAN_BPS_1000K);
	mcan_enable_interrupt(MCAN0, mcan0_CallbackFunction, 5);
	uint32_t can0_standardFilterList[5] = {0};
	uint32_t can0_extendedFilterList[5] = {0};
	mcan_setStandardFilter(MCAN0, can0_standardFilterList,0);
	
	mcan_setExtendedFilter(MCAN0, can0_extendedFilterList,0);
	
	//***********************************************************//
	mcan_init(MCAN1, CAN_BPS_1000K);
	mcan_enable_interrupt(MCAN1, mcan1_CallbackFunction, 5); 
	// 	uint32_t can0_filterList[5] = {CAN_ID_GAS, CAN_ID_BRAKE, CAN_ID_STEERING, CAN_ID_VX,CAN_ID_VY,
	// 								   CAN_ID_FZ_FL,CAN_ID_FZ_FR,CAN_ID_FZ_RL,CAN_ID_FZ_RR, CAN_ID_YAW_RATE,
	// 								   CAN_ID_AY, CAN_ID_AX};
	uint32_t can1_standardFilterList[5] = {0};
	uint32_t can1_extendedFilterList[5] = {0};
	mcan_setStandardFilter(MCAN1, can1_standardFilterList,0);
	mcan_setExtendedFilter(MCAN1, can1_extendedFilterList,0);
	
	
	
}



