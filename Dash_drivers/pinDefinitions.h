/*
 * pinDefinitions.h
 *
 * Created: 29-Feb-16 2:52:13 PM
 *  Author: Walid
 */ 




#ifndef PINDEFINITIONS_H_
#define PINDEFINITIONS_H_

/*	debug led definitions		 */
#define debugLed0_PIO PIOA
#define debugLed0_PIN 20
#define debugLed1_PIO PIOB
#define debugLed1_PIN 1

/*	leds on dashboard	*/

#define LED_AMS_PIO PIOA
#define LED_AMS_PIN 12
#define LED_IMD_PIO PIOD	
#define LED_IMD_PIN 1
#define LED_HIGHT_PIO PIOD
#define LED_HIGHT_PIN 4
#define LED_LDV_PIO PIOD
#define LED_LDV_PIN 5
#define LED_TS_PIO PIOD
#define LED_TS_PIN 19
#define LED_SBSPD_PIO PIOA
#define LED_SBSPD_PIN 0
#define LED_KERS_PIO PIOD
#define LED_KERS_PIN 19

/*	buttons on dashboard		*/
#define BUTTON_SELECT_PIO PIOD
#define BUTTON_SELECT_PIN 17
#define BUTTON_START_PIO PIOD
#define BUTTON_START_PIN 7
#define BUTTON_KERS_PIO PIOD
#define BUTTON_KERS_PIN 14
#define BUTTON_LAUNCH_PIO PIOD
#define BUTTON_LAUNCH_PIN 6
#define BUTTON_UP_PIO PIOD
#define BUTTON_UP_PIN 16
#define BUTTON_DOWN_PIO PIOA
#define BUTTON_DOWN_PIN 5
#define BUTTON_LEFT_PIO PIOA
#define BUTTON_LEFT_PIN	4
#define BUTTON_RIGHT_PIO PIOA
#define BUTTON_RIGHT_PIN 9

/*	Rotary encoder				*/
#define ROT_BUTTON_PIO PIOD
#define ROT_BUTTON_PIN 11
#define ROT_OUTPUTA_PIO PIOD
#define ROT_OUTPUTA_PIN 10
#define ROT_OUTPUTB_PIO PIOA
#define ROT_OUTPUTB_PIN 1

/*	FT800						*/
#define FT800_PD_PIO PIOD
#define FT800_PD_PIN 30
/* USB							*/
/*
#define USB_DETECT_PIO PIOA
#define USB_DETECT_PIN 22
*/


#endif /* PINDEFINITIONS_H_ */