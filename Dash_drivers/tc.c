/*
 * tc.c
 *
 * Created: 16.01.2016 18:21:38
 *  Author: Ørjan
 */ 
#include "tc.h"
#include "../same70-base_16/RevolveDrivers/pmc.h"

#include "../board.h"


#define ENABLE_WRITE_PROTECTION  (TC_WPMR_WPKEY_PASSWD | TC_WPMR_WPEN)
#define DISABLE_WRITE_PROTECTION  TC_WPMR_WPKEY_PASSWD
#define PCK6 TC_CMR_TCCLKS_TIMER_CLOCK1
#define BASE_FREQUENCY_PCK6 1000000
#if (PCB_VERSION == PRODUCTION_CARD)
	#define PMC_DIV		12	// 
#elif (PCB_VERSION == PROTOTYPE_CARD)
	#define PMC_DIV		16	// 16 MHz oscillator. Needs div = 16 to achieve 1 mhz frequency
#else
	#define PMC_DIV		16	// 16 MHz oscillator. Needs div = 16 to achieve 1 mhz frequency
#endif
#define REGISTER_A 0
#define REGISTER_B 1
#define REGISTER_C 2

// Timestamp timer definitions
#define BASECOUNTER       0
#define FASTCOUNTER       1
#define SLOWCOUNTER       2
#define FAST_MULT      1000
#define SLOW_MULT   1000000
#define BASEMODE (TC_CMR_ACPC_SET | TC_CMR_ACPA_CLEAR | TC_CMR_WAVE | TC_CMR_WAVSEL_UP_RC | PCK6)
#define FASTMODE (TC_CMR_ACPC_SET | TC_CMR_ACPA_CLEAR | TC_CMR_WAVE | TC_CMR_WAVSEL_UP_RC | TC_CMR_TCCLKS_XC1)
#define SLOWMODE (TC_CMR_WAVE | TC_CMR_TCCLKS_XC2)
#define TOP 1000
#define MID  500

#define TIMER0_CHANNEL 0
#define TIMER1_CHANNEL 1
#define TIMER2_CHANNEL 2

static void tc_selectMode(Tc * tc, uint8_t channel, uint32_t mode){
	tc->TC_CHANNEL[channel].TC_CMR = mode;
}
static void tc_enableTimer(Tc * tc, uint8_t channel){
	tc->TC_CHANNEL[channel].TC_CCR = TC_CCR_CLKEN;
}
static void tc_startClock(Tc * tc, uint8_t channel){
	tc->TC_CHANNEL[channel].TC_CCR = TC_CCR_SWTRG;
}

static void tc_setRegisterValue(Tc * tc, uint8_t channel, uint8_t register_x, uint32_t register_value){
	switch(register_x){
		case REGISTER_A:
		tc->TC_CHANNEL[channel].TC_RA = register_value;
		break;
		case REGISTER_B:
		tc->TC_CHANNEL[channel].TC_RB = register_value;
		break;
		case REGISTER_C:
		tc->TC_CHANNEL[channel].TC_RC = register_value;
		break;
	}
}

static void tc_enableWriteProtection(Tc * tc){
	tc->TC_WPMR = ENABLE_WRITE_PROTECTION;
}
static void tc_disableWriteProtection(Tc * tc){
	tc->TC_WPMR = DISABLE_WRITE_PROTECTION;
}

static void tc_syncronizedStartTimerModule(Tc * tc)
{
	tc->TC_BCR |= TC_BCR_SYNC;
}

static void tc_chainTimerModule(Tc * tc)
{
	TC0->TC_BMR = TC_BMR_TC1XC1S_TIOA0 | TC_BMR_TC2XC2S_TIOA1;
}

void tc_initTimestampTimer(){

	if (!pmc_is_programmable_clk_enabled(PMC_PCK_6)){
		pmc_switch_programmable_clk_to_mainck(PMC_PCK_6, PMC_DIV); // ASSUMING MAINCLOCK = 12 MHz
		pmc_enable_programmable_clk(PMC_PCK_6);
	}
	pmc_enable_periph_clk(TC0_IRQn);
	pmc_enable_periph_clk(TC1_IRQn);
	pmc_enable_periph_clk(TC2_IRQn);
	
	tc_disableWriteProtection(TC0);
	
	tc_chainTimerModule(TC0);
	tc_selectMode(TC0, BASECOUNTER, BASEMODE);
	tc_selectMode(TC0, FASTCOUNTER, FASTMODE);
	tc_selectMode(TC0, SLOWCOUNTER, SLOWMODE);
	tc_setRegisterValue(TC0, BASECOUNTER, REGISTER_A, MID);
	tc_setRegisterValue(TC0, BASECOUNTER, REGISTER_C, TOP);
	tc_setRegisterValue(TC0, FASTCOUNTER, REGISTER_A, MID);
	tc_setRegisterValue(TC0, FASTCOUNTER, REGISTER_C, TOP);
	tc_enableTimer(TC0, BASECOUNTER);
	tc_enableTimer(TC0, FASTCOUNTER);
	tc_enableTimer(TC0, SLOWCOUNTER);
	tc_syncronizedStartTimerModule(TC0);
	
	// wait until timestamp timer has synchronized to real time (takes about 1 sec).
	while (TC0->TC_CHANNEL[FASTCOUNTER].TC_SR & TC_SR_CPCS);

	tc_enableWriteProtection(TC0);
}

void tc_getTimestamp(struct tc_timestamp * timestamp)
{
	uint32_t base_time = (TC0->TC_CHANNEL[BASECOUNTER].TC_CV);
	uint32_t fast_time = (TC0->TC_CHANNEL[FASTCOUNTER].TC_CV);
	uint32_t slow_time = (TC0->TC_CHANNEL[SLOWCOUNTER].TC_CV);
	
	timestamp->total_time_in_us	= base_time + fast_time*FAST_MULT + slow_time*SLOW_MULT;
	timestamp->us				= base_time;
	timestamp->ms				= fast_time;
	timestamp->sec				= slow_time;
}

uint64_t tc_getElapsedTime(struct tc_timestamp * last_timestamp)
{
	struct tc_timestamp new_timestamp;
	tc_getTimestamp(&new_timestamp);
	
	uint64_t elapsed_time = new_timestamp.total_time_in_us - last_timestamp->total_time_in_us;
	
	return elapsed_time;
}

void tc_initTimer(enum tc_timer timer, uint32_t overflow_frequency)
{
	if (!pmc_is_programmable_clk_enabled(PMC_PCK_6)){
		pmc_switch_programmable_clk_to_mainck(PMC_PCK_6, PMC_DIV); // ASSUMING MAINCLOCK = 12 MHz
		pmc_enable_programmable_clk(PMC_PCK_6);
	}
	
	uint8_t channel;

	switch(timer)
	{
		case TIMER0:
		pmc_enable_periph_clk(TC9_IRQn);
		channel = TIMER0_CHANNEL;
		break;
		
		case TIMER1:
		pmc_enable_periph_clk(TC10_IRQn); 
		channel = TIMER1_CHANNEL;
		break;
		
		case TIMER2:
		pmc_enable_periph_clk(TC11_IRQn);
		channel = TIMER2_CHANNEL;
		break;
		default:
			channel = TIMER0;
		break;
	}
	
	uint32_t mode   = TC_CMR_WAVE | TC_CMR_WAVSEL_UP_RC | PCK6;
	uint32_t register_value = BASE_FREQUENCY_PCK6/overflow_frequency;
	
	tc_disableWriteProtection(TC3);
	
	tc_selectMode(TC3, channel, mode);
	tc_setRegisterValue(TC3, channel, REGISTER_C, register_value);
	tc_enableTimer(TC3, channel);
	tc_startClock(TC3, channel);
	
	tc_enableWriteProtection(TC3);
}

uint32_t tc_enableOverflowInterrupt(enum tc_timer timer) {
	// TIMER0 in TC3 has tc_handler9
	// TIMER1 in TC3 has tc_handler10
	// TIMER2 in TC3 has tc_handler11
	switch(timer)
	{
		case TIMER0:
		return TC3->TC_CHANNEL[TIMER0_CHANNEL].TC_IER = TC_IER_COVFS;
		break;
		
		case TIMER1:
		return TC3->TC_CHANNEL[TIMER1_CHANNEL].TC_IER = TC_IER_COVFS;
		break;
		
		case TIMER2:
		return TC3->TC_CHANNEL[TIMER2_CHANNEL].TC_IER = TC_IER_COVFS;
		break;
	}
	return 0xFFFFFFFF;
}

uint32_t tc_getTimerValue(enum tc_timer timer)
{
	switch(timer)
	{
		case TIMER0:
		return TC3->TC_CHANNEL[TIMER0_CHANNEL].TC_CV;
		break;
		
		case TIMER1:
		return TC3->TC_CHANNEL[TIMER1_CHANNEL].TC_CV;
		break;
		
		case TIMER2:
		return TC3->TC_CHANNEL[TIMER2_CHANNEL].TC_CV;
		break;
	}
	return 0xFFFFFFFF;
}

bool tc_overflowOccurred(enum tc_timer timer)
{
	switch(timer)
	{
		case TIMER0:
		return (TC3->TC_CHANNEL[TIMER0_CHANNEL].TC_SR & TC_SR_CPCS);
		break;
		
		case TIMER1:
		return (TC3->TC_CHANNEL[TIMER1_CHANNEL].TC_SR & TC_SR_CPCS);
		break;
		
		case TIMER2:
		return (TC3->TC_CHANNEL[TIMER2_CHANNEL].TC_SR & TC_SR_CPCS);
		break;
	}
	return false;
}


void tc_resetTimer(enum tc_timer timer) 
{
	switch(timer)
	{
		case TIMER0:
		TC3->TC_CHANNEL[TIMER0_CHANNEL].TC_CCR = TC_CCR_SWTRG;
		break;
		
		case TIMER1:
		TC3->TC_CHANNEL[TIMER1_CHANNEL].TC_CCR = TC_CCR_SWTRG;
		break;
		
		case TIMER2:
		TC3->TC_CHANNEL[TIMER2_CHANNEL].TC_CCR = TC_CCR_SWTRG;
		break;
	}
}