#ifndef HARDWAREINIT_H_
#define HARDWAREINIT_H_

#include "Dash_drivers/tc.h"

extern enum tc_timer TV_hardware_timer;
extern enum tc_timer Safety_sendINSCAN_timer;
extern enum tc_timer TV_to_analyze_timer;

void hardwareInit();


#endif /* HARDWAREINIT_H_ */