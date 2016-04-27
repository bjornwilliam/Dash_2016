/*
 * tc.h
 *
 * Created: 16.01.2016 18:21:54
 *  Author: Ørjan
 */ 


#ifndef TC_H_
#define TC_H_
#include "sam.h"
#include <stdbool.h>

enum tc_timer {
	TIMER0,
	TIMER1,
	TIMER2
};

struct tc_timestamp {
	uint32_t us;
	uint32_t ms;
	uint32_t sec;
	uint64_t total_time_in_us;
};

void	 tc_initTimestampTimer (void);
void	 tc_getTimestamp       (struct tc_timestamp * timestamp); 
uint64_t tc_getElapsedTime     (struct tc_timestamp * last_timestamp);  // New. How about making it a bool which checks
																		// if the elapsed time is bigger than some time value?

void	 tc_initTimer		   (enum tc_timer timer, uint32_t overflow_frequency);
uint32_t tc_enableOverflowInterrupt(enum tc_timer timer);
uint32_t tc_getTimerValue      (enum tc_timer timer);
void	 tc_resetTimer		   (enum tc_timer timer);
bool	 tc_overflowOccurred   (enum tc_timer timer);

#endif /* TC_H_ */