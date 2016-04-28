#include "hardwareInit.h"
#include <sam.h>
#include <stdbool.h>

#include "Dash_drivers/mcanFreeRTOSWrapper.h"
#include "same70-base_16/RevolveDrivers/eefc.h"
#include "same70-base_16/RevolveDrivers/pio.h"
#include "same70-base_16/RevolveDrivers/pmc.h"

#include "same70-base_16/RevolveDrivers/fpu.h"
#include "same70-base_16/RevolveDrivers/spi.h"
#include "same70-base_16/RevolveDrivers/uart.h"
#include "same70-base_16/RevolveDrivers/delay.h"
#include "same70-base_16/RevolveDrivers/spi.h"

#include "Dash_drivers/sd_mmc/ctrl_access.h"
#include "Dash_drivers/sd_mmc/sd_mmc.h"
#include "Dash_drivers/tc.h"
#include "board.h"


enum tc_timer TV_hardware_timer = TIMER0;
enum tc_timer Safety_sendINSCAN_timer = TIMER1;

enum tc_timer TV_to_analyze_timer = TIMER2;

void PMCInit() {

	system_init_flash(configCPU_CLOCK_HZ); // MUST BE CALLED BEFORE ENABLING CLOCK !!!!!

	if (PCB_VERSION == PRODUCTION_CARD) {
		struct PmcInit pmcInit = {
			.main_freq = EXT_12MHZ,
			.css = PLLA_CLOCK,
			.pres = CLK_2,
			.divide = 1,
			.multiply = 49 // This number is the actual number the clock will be multiplied with. Range: [1,62]
		};
		pmc_init_clocks(pmcInit);
	}
	else if (PCB_VERSION == PROTOTYPE_CARD) {
		struct PmcInit pmcInit = {
			.main_freq = EXT_16MHZ,
			.css = PLLA_CLOCK,
			.pres = CLK_1,
			.divide = 2,
			.multiply = 37 // This number is the actual number the clock will be multiplied with. Range: [1,62]
		};
		pmc_init_clocks(pmcInit);
	}
	else {
		struct PmcInit pmcInit = {
			.main_freq = EXT_16MHZ,
			.css = PLLA_CLOCK,
			.pres = CLK_1,
			.divide = 2,
			.multiply = 37 // This number is the actual number the clock will be multiplied with. Range: [1,62]
		};
		pmc_init_clocks(pmcInit);
	}	
}


static void sdInit(){
	pio_setMux(PIOA,25,D);	// MCCK
	pio_setMux(PIOA,28,C);	// MCCDA
	pio_setMux(PIOA,30,C);	// MCDA0
	pio_setMux(PIOA,31,C);	// MCDA1
	pio_setMux(PIOA,26,C);	// MCDA2
	pio_setMux(PIOA,27,C);	// MCDA3
	
	sd_mmc_init();
	Ctrl_status status;
	//Wait for card present and ready:
	do {
		status = sd_mmc_test_unit_ready(0);
		if (CTRL_FAIL == status) {
			while (CTRL_NO_PRESENT != sd_mmc_check(0)) {
			}
		}
	} while (CTRL_GOOD != status);
}

void hardwareInit() {
	//NVIC_SetPriorityGrouping(0U);
	fpu_enable();
	WDT->WDT_MR |= 1<<15; // Disable Watch dog timer
	PMCInit();
	pio_init();
	
// **************************************************************************************
// ************************************ TIMER COUNTER ***********************************
// **************************************************************************************	
	tc_initTimestampTimer();
	tc_initTimer(TV_hardware_timer,100); // TV
	//delay_ms(5);
	tc_initTimer(Safety_sendINSCAN_timer, 100); // Estimations Block
	tc_initTimer(TV_to_analyze_timer, 100); // Estimations Block
	
// SD CARD
	
	sdInit();

// **************************************************************************************
// ************************************ SPI *********************************************
// **************************************************************************************	
	// CHIP SELECT FRAM = PIN 32 = SPI0_NPCS3

	struct SpiMasterSettings masterSettings = {
		.cs_0 = none0,
		.cs_1 = none1,
		.cs_2 = none2,
		.cs_3 = PD27,
		.CSDelay_ns = 0
	};
	struct SpiSlaveSettings slave_ft800_settings = {
		.spi_mode = MODE_0,
		.chip_select = NPCS3,
		.raise_CS_every_transfer = false,
		.spi_baudRate_Hz = 5000000,
		.data_width = SPI_8BIT_DATA
	};
		 
 	spi_master_init(SPI0,masterSettings);
 	spi_chip_select_init(SPI0,slave_ft800_settings);


// **************************************************************************************
// ************************************ MCAN  *******************************************
// **************************************************************************************	
	mcan_freeRTOSSetup();


	
}
