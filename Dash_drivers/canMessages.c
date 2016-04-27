#include "canMessages.h"
#include "../revolve_can_definitions_16/revolve_can_definitions.h"



struct Can_message_t EcuStatusMessage = {
	.idType = STANDARD_CAN_ID,
	.messageID = ECU_STATUS_MESSAGE_ID,
	.dataLength = 4
	};

//*******************************************************************************
//-------------------------------------------------------------------------------
//                          ECU <-> DASH CAN MESSAGES
//-------------------------------------------------------------------------------
//*******************************************************************************
struct Can_message_t iAmAliveMessage = {
	.idType = STANDARD_CAN_ID,
	.dataLength = 0,
	.messageID = CANR_ALIVE_MSG_ID
};

struct Can_message_t playRTDS = {
	.idType = STANDARD_CAN_ID,
	.messageID = ECU_PLAY_RTDS_ID,
	};
	
struct Can_message_t driveEnabled = {
	.idType = STANDARD_CAN_ID,
	.messageID = ECU_DRIVE_ENABLE_ID,
	.dataLength = 0
	};
	
struct Can_message_t driveDisabled = {
	.idType = STANDARD_CAN_ID,
	.messageID = ECU_DRIVE_DISABLE_ID,
	.dataLength = 0
};



//*******************************************************************************
//-------------------------------------------------------------------------------
//                         CAN MESSAGES TO ANALYZE
//-------------------------------------------------------------------------------
//*******************************************************************************
struct Can_message_t slipRatios ={
	.idType = STANDARD_CAN_ID,
	.dataLength = 8,
	.messageID = ECU_SLIP_RATIO_ID
	};
	
struct Can_message_t FxdividedByFz = {
	.idType = STANDARD_CAN_ID,
	.dataLength = 8,
	.messageID = ECU_FX_DIV_FZ_ID
};

struct Can_message_t FzDamperEst  = {
	.idType = STANDARD_CAN_ID,
	.dataLength = 8,
	.messageID = ECU_FZ_DAMPER_EST_ID
};
struct Can_message_t FzLoadTransferEst = {
	.idType = STANDARD_CAN_ID,
	.dataLength = 8,
	.messageID = ECU_FZ_LOAD_TRANSFER_ID
};

struct Can_message_t controlSystemToTelemetri = {
	.idType = STANDARD_CAN_ID,
	.dataLength = 4,
	.messageID = ECU_CONTROL_SYSTEM_VALUES_ID
	};
	
	

//*******************************************************************************
//-------------------------------------------------------------------------------
//                         CAN MESSAGES FOR TESTING
//-------------------------------------------------------------------------------
//*******************************************************************************




//*******************************************************************************
//-------------------------------------------------------------------------------
//                          AMK INVERTER CAN MESSAGES
//-------------------------------------------------------------------------------
//*******************************************************************************


struct Can_message_t InvSetpoints_FL = {
	.idType = STANDARD_CAN_ID,
	.messageID = ECU_SET_POINT_AMK_FL_ID,
	.dataLength = 6
};

struct Can_message_t InvSetpoints_FR = {
	.idType = STANDARD_CAN_ID,
	.messageID = ECU_SET_POINT_AMK_FR_ID,
	.dataLength = 6
};

struct Can_message_t InvSetpoints_RL = {
	.idType = STANDARD_CAN_ID,
	.messageID = ECU_SET_POINT_AMK_RL_ID,
	.dataLength = 6
};

struct Can_message_t InvSetpoints_RR = {
	.idType = STANDARD_CAN_ID,
	.messageID = ECU_SET_POINT_AMK_RR_ID,
	.dataLength = 6
};







struct Can_message_t steeringAngleTest = {
	.idType = STANDARD_CAN_ID,
	.dataLength = 4,
	.messageID = 1050

};
	
//*******************************************************************************
//-------------------------------------------------------------------------------
//                INS and maybe optical sensor CAN messages from Safety task to CAN
//-------------------------------------------------------------------------------
//*******************************************************************************

struct Can_message_t safetyYawData = {
	.idType = STANDARD_CAN_ID,
	.messageID = ECU_INS_YAW_RATE_ACC_ID,
	.dataLength = 4	
	};

struct Can_message_t safety_ax_ay_vx_vy  = {
	.idType = STANDARD_CAN_ID,
	.messageID = ECU_INS_AX_AY_VX_VY_ID,
	.dataLength = 8
};

struct Can_message_t safetyLongitudeAndLatitude = {
	.idType = STANDARD_CAN_ID,
	.messageID = ECU_GPS_LONGITUDE_LATITUDE_ID,
	.dataLength = 8
	};
struct Can_message_t safety_gpsFix_nrSats = {
	.idType = STANDARD_CAN_ID,
	.messageID = ECU_GPS_FIX_AND_NRSATS_ID,
	.dataLength = 2
};


struct Can_message_t safetyINSStatuses = {
	.idType = STANDARD_CAN_ID,
	.messageID = ECU_INS_STATUSES_ID,
	.dataLength = 3
};

