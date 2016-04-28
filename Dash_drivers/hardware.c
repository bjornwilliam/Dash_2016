// /*
//  * hardware.c
//  *
//  * Created: 26-Feb-16 3:16:49 PM
//  *  Author: Walid
//  */ 
// #include "pmc.h"
// #include "spi.h"
// #include "pio.h"
// #include "CAN/can.h"
// #include "sd_mmc.h"
// #include "sd_handler.h"
// #include "../../canHandler.h"
// void static pmcInit();
// static void spi_init(uint32_t systemclock);
// void static canInit();
// void static sdInit();
// static void can_callback(uint32_t index);
// static void can_callback2(uint32_t index);
// static void fat_writeCanData(uint32_t index);
// 
// /*
// uint8_t byte = 0;
// 	
// #if defined(EXT12MHZ)
// 	struct PmcInit pmc_settings = {.main_freq = EXT_12MHZ, .css = PLLA_CLOCK, .pres = CLK_1, .divide = 1, .multiply = 25};
// 	#define SYSTEM_CLOCK_FREQ 300000000
// 	
// 	
// #else
// 	/ *struct PmcInit pmc_settings = {.main_freq = EXT_16MHZ, .css = PLLA_CLOCK, .pres = CLK_1,.divide = 2,.multiply = 37};
// 	#define SYSTEM_CLOCK_FREQ 296000000* /
// 	struct PmcInit pmc_settings = {.main_freq = EXT_12MHZ, .css = PLLA_CLOCK, .pres = CLK_1, .divide = 1, .multiply = 25};
// 	#define SYSTEM_CLOCK_FREQ 300000000
// #endif
// */
// 
// #define SYSTEM_CLOCK_FREQ 300000000
// #include "interrupt/interrupt_sam_nvic.h"
// #define WATCHDOG_DOWN_COUNTER 769
// void hardware_init(){
// 	WDT->WDT_MR |= 1<<15;
// 	//WDT->WDT_MR |= WATCHDOG_DOWN_COUNTER;	
// 	irq_initialize_vectors();
// 	cpu_irq_enable();
// 	
// 	
// 	
// 	struct PmcInit pmc_settings = {.main_freq = EXT_12MHZ, .css = PLLA_CLOCK, .pres = CLK_1, .divide = 1, .multiply = 25};
// 	/*struct PmcInit pmc_settings = {.main_freq = EXT_16MHZ, .css = PLLA_CLOCK, .pres = CLK_1,.divide = 2,.multiply = 37};*/
// 
// 	uint32_t system_core_clock = pmc_calcCoreClock(pmc_settings);
// 	system_init_flash(system_core_clock);
// 	pmc_init_clocks(pmc_settings);
// 
// 	NVIC_SetPriorityGrouping(0U);	//Lars said simen added this*/
// 	WDT->WDT_MR |= 1<<15; // Disable Watch dog timer
// 
// //	system_init_flash(SYSTEM_CLOCK_FREQ);
// 
// 	pio_init();
// 	spi_init(system_core_clock); // enable spi peripheral mainly for ft800 with 10 MHz speed.
// 	
// 	sdInit();
// 	init_Canhandler();
// //	canInit();
// }
// enum NCPS0pin cs_0;
// enum NCPS1pin cs_1;
// enum NCPS2pin cs_2;
// enum NCPS3pin cs_3;
// static void spi_init(uint32_t systemclock){
// 	Spi *spi = SPI0;
// 	struct SpiMasterSettings spi_master_settings = {
// 		.cs_0 = none0,
// 		.cs_1 = none1,
// 		.cs_2 = none2,
// 		.cs_3 = PD27,
// 		.CSDelay = 0
// 	};
// 	struct SpiSlaveSettings spi_slave_settings = {
// 		.spi_mode = MODE_0,
// 		.chip_select= NPCS3,
// 		.spi_baudRate_Hz = 7500000, // must not exceed 11 mhz before clock is ready.
// 		.data_width = SPI_8BIT_DATA,
// 		.raise_CS_every_transfer = false
// 	};
// 	
// 	spi_master_init(SPI0,spi_master_settings);
// 	spi_chip_select_init(SPI0,spi_slave_settings);
// 	//spi_setBaudRateHz(systemclock/2,spi_slave_settings.spi_baudRate_Hz,spi_slave_settings.chip_select);
// }
// 
// void spi_init2(){
// 	Spi *spi = SPI0;
// 	struct SpiMasterSettings spi_master_settings = {
// 		.cs_0 = none0,
// 		.cs_1 = none1,
// 		.cs_2 = none2,
// 		.cs_3 = PD27,
// 		.CSDelay = 0
// 	};
// 	struct SpiSlaveSettings spi_slave_settings = {
// 		.spi_mode = MODE_0,
// 		.chip_select= NPCS3,
// 		.spi_baudRate_Hz = 25000000, // must not exceed 11 mhz before clock is ready.
// 		.data_width = SPI_8BIT_DATA,
// 		.raise_CS_every_transfer = false
// 	};
// 	
// 	spi_master_init(SPI0,spi_master_settings);
// 	spi_chip_select_init(SPI0,spi_slave_settings);
// 	//spi_setBaudRateHz(systemclock/2,spi_slave_settings.spi_baudRate_Hz,spi_slave_settings.chip_select);
// }
// 
// 
// 
// /*
// void static spiInit(uint32_t systemclock){
// 	Spi *spi = SPI0;
// 	struct SpiMuxSettings spi_master_settings = {
// 		.cs_0 = none0,
// 		.cs_1 = none1,
// 		.cs_2 = none2,
// 		.cs_3 = PD27
// 		};
// 	struct SpiChipSelectSettings spi_slave_settings = {
// 		.spi_mode = MODE_0,
// 		.chip_select = 3,
// 		.peripheral_clock_Hz = systemclock,
// 		.spi_baudRate_Hz = 10000000//maximum 11 MHz during initialization phase for FT800
// 		};
// 
// 	spi_master_init(SPI0,spi_master_settings);
// 	spi_chip_select_init(SPI0, spi_slave_settings);
// 	spi_setBaudRateHz(systemclock,spi_slave_settings.spi_baudRate_Hz, spi_slave_settings.chip_select);
// }
// */
// 
// 
// void canInit(){
// 	
// 		uint32_t accepted_standard_ids[3] = {0xAA, 0x12, 0x34};	// Example
// 		uint32_t accepted_extended_ids[2] = {0x1BCDEF12, 0x53};	// Example
// 		
// 	
// 		can_init(MCAN0,CAN_BPS_1000K,accepted_standard_ids,3,accepted_extended_ids,2,can_callback2,5);
// 		can_init(MCAN1,CAN_BPS_1000K,accepted_standard_ids,3,accepted_extended_ids,2,can_callback,5);
// }
// 
// //future referencce
// /*
// static void can_callback(uint32_t index){
// 	struct Can_message_t rx_msg;
// 	can_getMessage(MCAN1, &rx_msg, index);
// 	PIOA->PIO_SODR |=(1 << 20);
// 	can_sendMessage(MCAN1,&rx_msg);
// 	//can_got_message = true;
// }
// 
// static void can_callback2(uint32_t index){
// 	struct Can_message_t rx_msg;
// 	can_getMessage(MCAN0, &rx_msg, index);
// 	PIOA->PIO_SODR |=(1 << 20);
// 	can_sendMessage(MCAN0,&rx_msg);
// 	//can_got_message = true;
// }
// 
// / *
// static void fat_writeCanData(uint32_t index){
// 	//
// 	struct Can_message_t msg_rx;
// 	can_getMessage(MCAN1,&msg_rx,index);
// 	byte = msg_rx.data.u8[0];* /
// 	/ *FATFS fs;
// 	FIL fileobject;
// 	FRESULT res;
// 	char filename[] = "TextCanMe.txt";
// 	f_mount(LUN_ID_SD_MMC_0_MEM,&fs);
// 	f_open(&fileobject,filename,FA_OPEN_ALWAYS | FA_WRITE)	;
// 	f_lseek(&fileobject,f_size(&fileobject));
// 	//f_printf(&fileobject, "CAN ID: test DATA: 0x%02X	 \n   ",254);
// 	f_puts("I just wanna let it go", &fileobject);
// 	f_close(&fileobject);	* /
// / *
// 	char *str = "CAN ID: %i DATA: 0x%02X	 \n   ";
// 	struct Can_message_t rx_msg;
// 	can_getMessage(MCAN1, &rx_msg, index);
// 	if(rx_msg.data.u8[0] == 0xAA)
// 		pio_setOutput(PIOA,20,PIN_HIGH);
// 	if(rx_msg.data.u8[0] == 0xFB)
// 		pio_setOutput(PIOA,20,PIN_LOW);
// 
// 	f_mount(LUN_ID_SD_MMC_0_MEM,&fs);
// 
// 	char file[]= "pcantest.txt";	
// 	f_open(&fileobject,file, FA_OPEN_ALWAYS | FA_WRITE);
// 	//f_lseek(&fileobject,f_size(&fileobject));
// 	//f_puts("s ", &fileobject);
// 	//f_lseek(&fileobject,f_size(&fileobject));
// 	f_printf(&fileobject,str , rx_msg.messageID,rx_msg.data.u8[0]);
// 	f_close(&fileobject);
// 
// 	can_sendMessage(MCAN1,&rx_msg);	//pio_setOutput(PIOA,20,PIN_LOW);
// 	* /
// }*/
// 
// 
// static void sdInit(){
// 	pio_setMux(PIOA,25,D);	// MCCK
// 	pio_setMux(PIOA,28,C);	// MCCDA
// 	pio_setMux(PIOA,30,C);	// MCDA0
// 	pio_setMux(PIOA,31,C);	// MCDA1
// 	pio_setMux(PIOA,26,C);	// MCDA2
// 	pio_setMux(PIOA,27,C);	// MCDA3
// 	
// 	sd_mmc_init();
// 	Ctrl_status status;
// 	//Wait for card present and ready:
// 		do {
// 			status = sd_mmc_test_unit_ready(0);
// 			if (CTRL_FAIL == status) {
// 				while (CTRL_NO_PRESENT != sd_mmc_check(0)) {
// 				}
// 				}
// 			} while (CTRL_GOOD != status);
// }