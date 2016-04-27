#include "hardwareInit.h"

#include "ECUDrivers/mcanFreeRTOSWrapper.h"
#include "same70-base_16/RevolveDrivers/eefc.h"
#include "same70-base_16/RevolveDrivers/pio.h"
#include "same70-base_16/RevolveDrivers/pmc.h"

#include "same70-base_16/RevolveDrivers/fpu.h"
#include "same70-base_16/RevolveDrivers/spi.h"
#include "same70-base_16/RevolveDrivers/uart.h"
#include "same70-base_16/RevolveDrivers/delay.h"

#include "ECUDrivers/uart_dma_INS.h"
#include "ECUDrivers/VN_200_INS.h"
#include "ECUDrivers/tc.h"

#include <sam.h>
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
	
	


// **************************************************************************************
// ************************************ SPI *********************************************
// **************************************************************************************	
	// CHIP SELECT FRAM = PIN 32 = SPI0_NPCS3
	struct SpiMuxSettings spi_mux_settings = {
		.cs_0 = none0,
		.cs_1 = none1,
		.cs_2 = none2,
		.cs_3 = PD27
	};
	struct SpiChipSelectSettings spi_cs3_settings = {
		.spi_mode = MODE_0,
		.chip_select = 3,
		.peripheral_clock_Hz = configCPU_CLOCK_HZ/2,
		.spi_baudRate_Hz = 10000000
		};
	spi_master_init(SPI0,spi_mux_settings);
	spi_chip_select_init(SPI0,spi_cs3_settings);
// **************************************************************************************
// ************************************ UART ********************************************
// **************************************************************************************	
	struct uart_config uart_settings = {
		// UARTX1 = PIN55 = PA4
		// UARTRX1 = PIN52 = PA5
		.uartID = UART1,
		.TX_pin = UART1_TX_PIN_PA4,
		.RX_pin = UART1_RX_PIN_PA5,
		.parity = NO_PARITY,
		.mode = NORMAL_MODE,
		.baudrate = 115200
		};
	uart_init(uart_settings);
	//uart_setCompareRegisterFlagOnly(UART1, 0xfa,0xfa);
	//uart_enableCompareInterrupt(UART1);
	uart_enableRxReadyInterrupt(UART1);
	NVIC_ClearPendingIRQ(UART1_IRQn);
	NVIC_SetPriority(UART1_IRQn, 5);	
	
	// Enable cmp interrupt, load cmp register with newline

// **************************************************************************************
// ************************************ MCAN  *******************************************
// **************************************************************************************	
	mcan_freeRTOSSetup();
// **************************************************************************************
// ************************************ INS *********************************************
// **************************************************************************************

	// WAIT UNTIL INS IS READY TO RECEIVE SETTINGS
	delay_ms(1000);
	VN200_UART_stopAsyncOutput();
	
	VN200_configureIMUFilter(8,4); // 800 hz / 8 filter width -> 100 Hz eq 
	VN200_setBaudRate(460800);
	uart_settings.baudrate = (459000); // to get the baud rate div to be 20 instead of 19, more accurate.
	uart_init(uart_settings);
	delay_ms(1000);
	// Define output format of message from INS
	struct INSOutput fastSensorDataMessage = {
		.outputRegister			= REG_BINARY_OUTPUT_1,
		.output_frequency_Hz	= 200,
		.output_group			= COMMON_GROUP | ATTITUDE_GROUP | INS_GROUP,
		.group_field_1			= (COMMON_ANGULAR_RATES | COMMON_YAW_PITCH_ROLL),
		.group_field_2			= ATTITUDE_ACCEL_LIN,
		.group_field_3			= INS_VEL_BODY,
		.payload_length			= (COMMON_ANGULAR_RATES_SIZE + COMMON_YAW_PITCH_ROLL_SIZE +
								  ATTITUDE_ACCEL_LIN_SIZE + INS_VEL_BODY_SIZE)
	};
	struct INSOutput slowDataMessage = {
		.outputRegister			= REG_BINARY_OUTPUT_2,
		.output_frequency_Hz	= 5,
		.output_group			= GPS_GROUP | ATTITUDE_GROUP |  INS_GROUP,
		.group_field_1			= GPS_NUM_TRACKED_SATELLITE | GPS_FIX,
		.group_field_2			= ATTITUDE_STATUS,
		.group_field_3			= INS_STATUS | INS_LLA,
		.payload_length			= ( GPS_NUM_TRACKED_SATTELITE_SIZE + GPS_FIX_SIZE + ATTITUDE_STATUS_SIZE  + INS_STATUS_SIZE + INS_LLA_SIZE)
	}; 
	VN200_activateFastOutputGroup(fastSensorDataMessage);
	VN200_activateSlowOutputGroup(slowDataMessage);
//***************************************************************************************** //
	uart_resetStatusRegister(UART1);
	NVIC_ClearPendingIRQ(UART1_IRQn);
	
}
