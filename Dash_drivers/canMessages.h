#ifndef CANMESSAGES_H_
#define CANMESSAGES_H_

#include "../same70-base_16/RevolveDrivers/CAN/can.h"


extern struct Can_message_t EcuStatusMessage;



extern struct Can_message_t InvSetpoints_FL;
extern struct Can_message_t InvSetpoints_FR;
extern struct Can_message_t InvSetpoints_RL;
extern struct Can_message_t InvSetpoints_RR;

extern struct Can_message_t InvSetpoints_bDcOn_FL;
extern struct Can_message_t InvSetpoints_bDcOn_FR;
extern struct Can_message_t InvSetpoints_bDcOn_RL;
extern struct Can_message_t InvSetpoints_bDcOn_RR;

extern struct Can_message_t InvSetpoints_bEnable_bInverterOn_FL;
extern struct Can_message_t InvSetpoints_bEnable_bInverterOn_FR;
extern struct Can_message_t InvSetpoints_bEnable_bInverterOn_RL;
extern struct Can_message_t InvSetpoints_bEnable_bInverterOn_RR;

extern struct Can_message_t InvSetpoints_bEnableOff_bInverterOff_FL;
extern struct Can_message_t InvSetpoints_bEnableOff_bInverterOff_FR;
extern struct Can_message_t InvSetpoints_bEnableOff_bInverterOff_RL;
extern struct Can_message_t InvSetpoints_bEnableOff_bInverterOff_RR;

extern struct Can_message_t InvSetpoints_bDcOff_FL;
extern struct Can_message_t InvSetpoints_bDcOff_FR;
extern struct Can_message_t InvSetpoints_bDcOff_RL;
extern struct Can_message_t InvSetpoints_bDcOff_RR;

extern struct Can_message_t InvSetpoints_ErrorReset_FL;
extern struct Can_message_t InvSetpoints_ErrorReset_FR;
extern struct Can_message_t InvSetpoints_ErrorReset_RL;
extern struct Can_message_t InvSetpoints_ErrorReset_RR;


extern struct Can_message_t playRTDS;
extern struct Can_message_t driveEnabled;
extern struct Can_message_t driveDisabled;

extern struct Can_message_t TimeControlSystem;
extern struct Can_message_t TimeFzEstimation;
extern struct Can_message_t TimeSpeedEstimation;

extern struct Can_message_t startTime;
extern struct Can_message_t stopTime;

extern struct Can_message_t testTimestamp;

extern struct Can_message_t testExtended;


// MESSAGES RELATED TO OUTPUT TO ANALYZE 
extern struct Can_message_t slipRatios;
extern struct Can_message_t FxdividedByFz;
extern struct Can_message_t FzDamperEst;
extern struct Can_message_t FzLoadTransferEst;

extern struct Can_message_t controlSystemToTelemetri;

extern struct Can_message_t iAmAliveMessage;


// TESTING DERIVATIVE IN SAFETY

extern struct Can_message_t yawRateAndAcc;

extern struct Can_message_t steeringAngleTest;

//*******************************************************************************
//-------------------------------------------------------------------------------
//                INS and maybe optical sensor CAN messages from Safety task to CAN
//-------------------------------------------------------------------------------
//*******************************************************************************

extern struct Can_message_t safetyYawData;

extern struct Can_message_t safety_ax_ay_vx_vy;

extern struct Can_message_t safetyLongitudeAndLatitude;

extern struct Can_message_t safetyINSStatuses;

extern struct Can_message_t safety_gpsFix_nrSats;

// 
// #define ECU_CHANGE_TRACTION_CONTROL_STATE_ID	200
// #define ECU_CHANGE_YAW_RATE_CONTROL_STATE_ID	201
// #define ECU_CHANGE_TORQUE_ALLOCATION_STATE_ID	202
// 
// // LOCAL CAN MESSAGE DECODE INFO
// 
// #define TURN_OFF_TRACTION_CONTROL					0
// #define TURN_ON_TRACTION_CONTROL					1
// 
// #define YAW_RATE_CONTROL_TURN_OFF					1
// #define YAW_RATE_CONTROL_PI_ENABLE					2
// #define YAW_RATE_CONTROL_PID_ENABLE					3
// 
// #define TORQUE_ALLOCATION_STATIC_ENABLE				1
// #define TORQUE_ALLOCATION_AWD_NEG_TORQUE_ENABLE		2
// #define TORQUE_ALLOCATION_AWD_POS_TORQUE_ENABLE		3
// #define TORQUE_ALLOCATION_AWD_STEERING_ENABLE		4


#endif /* CANMESSAGES_H_ */