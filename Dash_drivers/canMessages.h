#ifndef CANMESSAGES_H_
#define CANMESSAGES_H_

#include "../same70-base_16/RevolveDrivers/CAN/can.h"




extern struct Can_message_t IAmAlive;
extern struct Can_message_t TorquePedalCalibrationMax ;
extern struct Can_message_t TorquePedalCalibrationMin ;
extern struct Can_message_t SteeringCalibrationLeft ;
extern struct Can_message_t SteeringCalibrationRight;
extern struct Can_message_t Acknowledge ;

extern struct Can_message_t FinishedRTDS ;

extern struct Can_message_t RequestDriveDisable ;

extern struct Can_message_t RequestDriveEnable ;


extern struct Can_message_t RequestLCInit ;


extern struct Can_message_t RequestLCArmed ;


extern struct Can_message_t RequestLCDisable ;


extern struct Can_message_t TurnOffPump ;

extern struct Can_message_t TurnOnPump ;

extern struct Can_message_t TurnOffAllFans ;

extern struct Can_message_t TurnOnAllFans;

extern struct Can_message_t AdjustDutyCycleRadiatorFan ;

extern struct Can_message_t AdjustDutyCycleBatteryFan;

extern struct Can_message_t AdjustDutyCycleMonoFan ;

// 0-100
// radiator 1
// batter 2
// mono 3
// sette byte [2] til 0-100


extern struct Can_message_t EcuParametersFromFile;


extern struct Can_message_t EcuTractionControl;




#endif /* CANMESSAGES_H_ */