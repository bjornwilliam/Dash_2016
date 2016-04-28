

#ifndef DASH_IO_H_
#define DASH_IO_H_

#include "../same70-base_16/RevolveDrivers/pio.h"
#include "../same70-base_16/RevolveDrivers/pmc.h"





/*	leds on dashboard	*/

#define AMS_LED_PIO PIOA
#define AMS_LED_PIN 12

#define IMD_LED_PIO PIOD
#define IMD_LED_PIN 1

#define ECU_LED_PIO PIOD
#define ECU_LED_PIN 4

#define LED_LDV_PIO PIOD
#define LED_LDV_PIN 5

#define TS_LED_PIO PIOD
#define TS_LED_PIN 19

#define VOLT_LED_PIO PIOA
#define VOLT_LED_PIN 0

#define TEMP_LED_PIO PIOD
#define TEMP_LED_PIN 19



// #define LC_LED_PIO PIOD
// #define LC_LED_PIN 27
// 
// #define DEVICE_LED_PIO PIOC
// #define DEVICE_LED_PIN 7

/*	Rotary encoder				*/
#define ROT_PUSH3_PIO PIOD
#define ROT_PUSH3_PIN 11

#define ROT_A_PIO PIOD
#define ROT_A_PIN 10

#define ROT_B_PIO PIOA
#define ROT_B_PIN 1


/*	buttons on dashboard		*/
#define NAV_ACK_PIO PIOD
#define NAV_ACK_PIN 17

#define START_PIO PIOD
#define START_PIN 7

#define BUTTON_KERS_PIO PIOD
#define BUTTON_KERS_PIN 14

#define LC_PIO PIOD
#define LC_PIN 6


#define NAVIGATION_U_PIO PIOD
#define NAVIGATION_U_PIN 16
#define NAVIGATION_D_PIO PIOA
#define NAVIGATION_D_PIN 5
#define NAVIGATION_L_PIO PIOA
#define NAVIGATION_L_PIN	4
#define NAVIGATION_R_PIO PIOA
#define NAVIGATION_R_PIN 9


/*	FT800						*/
#define BUZZER_PIO PIOD
#define BUZZER_PIN 30

/*	FT800						*/
#define FT800_POWERDOWN_PIO PIOD
#define FT800_POWERDOWN_PIN 30


#define DETECT_USB_PIO PIOB
#define DETECT_USB_PIN 13


void dash_io_init();

void pio_inputDebounce(Pio *pio, uint8_t pin, uint16_t div, enum FilterType filter_type);
void pio_inputPulldown(Pio *pio, uint8_t pin,enum PullType pull_type);
void pio_outputPulldownLow(Pio *pio, uint8_t pin);

#endif /* DASH_IO_H_ */