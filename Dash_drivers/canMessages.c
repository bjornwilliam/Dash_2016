#include "canMessages.h"
#include "../revolve_can_definitions_16/revolve_can_definitions.h"
#include "../canID_definitions.h"

// Identifier for Modules on FAN card
#define RADIATOR_FAN_ID		1
#define BATTERY_FAN_ID		2
#define MONO_FAN_ID			3
#define PUMP_ID				4
#define TOGGLE_ALL_FANS_ID	7





struct Can_message_t IAmAlive = {
	.idType = STANDARD_CAN_ID,
	.dataLength = 1,
	.data.u8[0] = ALIVE_DASH,
	.messageID = ID_ALIVE
};

struct Can_message_t TorquePedalCalibrationMax = {
	.idType = STANDARD_CAN_ID,
	.data.u8[0] = 0xF0,
	.dataLength = 1,
	.messageID = ID_TORQUE_PEDAL_CALIBRATION
};
struct Can_message_t TorquePedalCalibrationMin = {
	.idType = STANDARD_CAN_ID,
	.data.u8[0] = 0x0F,
	.dataLength = 1,
	.messageID = ID_TORQUE_PEDAL_CALIBRATION
};
struct Can_message_t SteeringCalibrationLeft = {
	.idType = STANDARD_CAN_ID,
	.data.u8[0] = 0xF0,
	.dataLength = 1,
	.messageID = ID_STEERING_CALIBRATION
};
struct Can_message_t SteeringCalibrationRight = {
	.idType = STANDARD_CAN_ID,
	.data.u8[0] = 0x0F,
	.dataLength = 1,
	.messageID = ID_STEERING_CALIBRATION
};

struct Can_message_t Acknowledge = {
	.idType = STANDARD_CAN_ID,
	.messageID = ID_DASH_ACKNOWLEDGE,
	.dataLength = 0
};

struct Can_message_t FinishedRTDS = {
	.idType = STANDARD_CAN_ID,
	.messageID = ID_FINISHED_RTDS,
	.dataLength = 0
};
struct Can_message_t RequestDriveDisable = {
	.idType = STANDARD_CAN_ID,
	.messageID = ID_ECU_DRIVE,
	.dataLength = 1,
	.data.u8[0] = 0
};
struct Can_message_t RequestDriveEnable = {
	.idType = STANDARD_CAN_ID,
	.messageID = ID_ECU_DRIVE,
	.dataLength = 1,
	.data.u8[0] = 1
};

struct Can_message_t RequestLCInit = {
	.idType = STANDARD_CAN_ID,
	.messageID = ID_ECU_LC,
	.dataLength = 1,
	.data.u8[0] = 1
};

struct Can_message_t RequestLCArmed = {
	.idType = STANDARD_CAN_ID,
	.messageID = ID_ECU_LC,
	.dataLength = 1,
	.data.u8[0] = 2
};

struct Can_message_t RequestLCDisable = {
	.idType = STANDARD_CAN_ID,
	.messageID = ID_ECU_LC,
	.dataLength = 1,
	.data.u8[0] = 0x03
};

struct Can_message_t TurnOffPump = {
	.idType = STANDARD_CAN_ID,
	.messageID = ID_FAN_CONTROL,
	.dataLength = 3,
	.data.u8[0] = PUMP_ID,
	.data.u8[1] = 0,
	.data.u8[2] = 0
};
struct Can_message_t TurnOnPump = {
	.idType = STANDARD_CAN_ID,
	.messageID = ID_FAN_CONTROL,
	.dataLength = 3,
	.data.u8[0] = PUMP_ID,
	.data.u8[1] = 0,
	.data.u8[2] = 1
};
struct Can_message_t TurnOffAllFans = {
	.idType = STANDARD_CAN_ID,
	.messageID = ID_FAN_CONTROL,
	.dataLength = 3,
	.data.u8[0] = TOGGLE_ALL_FANS_ID,
	.data.u8[1] = 0,
	.data.u8[2] = 0
};
struct Can_message_t TurnOnAllFans = {
	.idType = STANDARD_CAN_ID,
	.messageID = ID_FAN_CONTROL,
	.dataLength = 3,
	.data.u8[0] = TOGGLE_ALL_FANS_ID,
	.data.u8[1] = 0,
	.data.u8[2] = 1
};
struct Can_message_t AdjustDutyCycleRadiatorFan = {
	.idType = STANDARD_CAN_ID,
	.messageID = ID_FAN_CONTROL,
	.dataLength = 3,
	.data.u8[0] = RADIATOR_FAN_ID,
	.data.u8[1] = 0
};
struct Can_message_t AdjustDutyCycleBatteryFan = {
	.idType = STANDARD_CAN_ID,
	.messageID = ID_FAN_CONTROL,
	.dataLength = 3,
	.data.u8[0] = BATTERY_FAN_ID,
	.data.u8[1] = 0
};
struct Can_message_t AdjustDutyCycleMonoFan = {
	.idType = STANDARD_CAN_ID,
	.messageID = ID_FAN_CONTROL,
	.dataLength = 3,
	.data.u8[0] = MONO_FAN_ID,
	.data.u8[1] = 0
};
// 0-100
// radiator 1
// batter 2
// mono 3
// sette byte [2] til 0-100


struct Can_message_t EcuParametersFromFile = {
	// Byte 1 describes what parameter is being set in the ECU , 0x01 for Torque, 0x02 for Kers amount, 0x03 for KERS ON/OFF, 0x04 ..
	// Byte 2 is for Torque percentage
	// Byte 3 is for Kers Amount
	// Byte 4 is for Kers ON/OFF
	// Byte 5 ...
	.idType = STANDARD_CAN_ID,
	.messageID		= ID_ECU_PARAMETERS,
	.dataLength		= 8
};

struct Can_message_t EcuTractionControl = {
	.idType = STANDARD_CAN_ID,
	.messageID = ID_ECU_TRACTION_CONTROL,
	.dataLength = 1
};

