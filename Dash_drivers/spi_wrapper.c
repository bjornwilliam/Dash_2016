#include "spi_wrapper.h"
//#include "../same70-base_16/RevolveDrivers/xdmac.h"
#include "../same70-base_16/RevolveDrivers/xdmac_wrapper.h"
#include <stdlib.h>


SemaphoreHandle_t spi_handlerIsDoneSempahore = NULL;
SemaphoreHandle_t spi_mutex = NULL;
static void (*callBackFunctionPointer)(void);

#define SPI_CHANNEL_DMA 4
#define SPI_BUFFER_SIZE 100
static uint32_t spi_dma_buffer[100] = {0};

void spi_dma_callback(uint32_t status) {
	SPI0->SPI_CR |= SPI_CR_LASTXFER;
}

Xdma_transfer_opts_t spi_dma_opts = {
	.channel_number = SPI_CHANNEL_DMA,
	.element_size = XDMAC_SIZE_32,
	.array_size   = SPI_BUFFER_SIZE,
	.num_transfers = 0,
	.peripheral_transfer_dir = XDMAC_DIR_MEM2PER,
	.source_adress = (uint32_t *)spi_dma_buffer,
	.destination_adress = &(SPI0->SPI_TDR),
	.trigger = XDMAC_EVENT_HWID_SPI0_TX,
	.interrupt_mask = XDMAC_CIE_BIE,
	.interrupt_priority = 5,
	.source_bus  = XDMAC_BUS_0,
	.destination_bus = XDMAC_BUS_1,
	.callback	= NULL
};


// Xdma_transfer_opts_t dmaUart1RxConfig_slowSensorData_tr = {
// 	.array_size				= RX_SLOW_MESSAGE_LENGTH,
// 	.channel_number			= XDMAC_RX_CH_SLOW_MESSAGE,
// 	.destination_adress		= (uint32_t *)slow_rx_buffer,// RX BUFFER
// 	.source_adress			= &(UART1->UART_RHR),  // UART Read register
// 	.element_size			= XDMAC_SIZE_8,
// 	.num_transfers			= 0,
// 	.peripheral_transfer_dir = XDMAC_DIR_PER2MEM,
// 	.trigger				= XDMAC_EVENT_HWID_UART1_RX,
// 	.interrupt_mask			= XDMAC_CIE_BIE,
// 	.interrupt_priority		= 6,
// 	.callback				= slow_callback
// };

void spi_freeRTOSTranceive(uint16_t *tx_buffer,  uint16_t *rx_buffer, uint32_t buffer_size, enum SpiChipSelect chip_select) {
	//Aquire the spi resource
	xSemaphoreTake(spi_mutex,portMAX_DELAY);

	//callBackFunctionPointer = callBackFunc;
	bool is_last_transfer;
	for (uint32_t i = 0; i < buffer_size; i++) {
		
		if (i == (buffer_size-1)) {
			is_last_transfer = true;
		}
		else {
			is_last_transfer = false;
		}
		
		spi_dma_buffer[i] = spi_generateControlWord(is_last_transfer, chip_select, tx_buffer[i]);
	
	}
	
	spi_dma_opts.array_size = buffer_size;
	xdma_setup_peripheral_transfer(XDMAC, &spi_dma_opts);
	xdma_start_transfer(XDMAC, &spi_dma_opts);
	NVIC_EnableIRQ(SPI0_IRQn);
	//xdma_stop_transfer(XDMAC,&spi_dma_opts);
	// Wait for the SPI_Handler to run ,and signal that the transfer is complete
	xSemaphoreTake(spi_handlerIsDoneSempahore, portMAX_DELAY);
	
	// Release spi resource
	xSemaphoreGive(spi_mutex);
}

void SPI0_Handler(void) {
	NVIC_DisableIRQ(SPI0_IRQn);
	long lHigherPriorityTaskWoken = pdFALSE;
	SPI0->SPI_SR; // MUST READ SR TO CLEAR NSSR
	if (callBackFunctionPointer != NULL) {
		callBackFunctionPointer();
	}
	if (spi_handlerIsDoneSempahore != NULL) {
		xSemaphoreGiveFromISR(spi_handlerIsDoneSempahore,&lHigherPriorityTaskWoken);
	}
	portEND_SWITCHING_ISR(lHigherPriorityTaskWoken);
}

// Med CSAAT = 1 , så må lastxfer i SPI CR !! settes etter siste data er overført
void spi_wrapper_master_init(Spi *spi, struct SpiMasterSettings spi_settings) {
	NVIC_DisableIRQ(SPI0_IRQn);
	NVIC_ClearPendingIRQ(SPI0_IRQn);
	NVIC_SetPriority(SPI0_IRQn,6);
	NVIC_EnableIRQ(SPI0_IRQn);
	
	spi_master_init(spi, spi_settings);
	//SPI0->SPI_IER |=  SPI_IER_NSSR;
	//SPI0->SPI_IER |=  SPI_IER_NSSR;
	
	//SPI0->SPI_CSR[3] |= SPI_CSR_CSAAT;
	SPI0->SPI_IER |=  SPI_IER_TXEMPTY;
}


spi_status_t spi_transceive_tdre(uint16_t *tx_buffer,  uint16_t *rx_buffer, uint32_t buffer_size, enum SpiChipSelect chip_select) {
	uint32_t timeout = SPI_TIMEOUT;
	bool is_last_transfer;
	uint32_t i = 0;
	for (i = 0; i < buffer_size; i++) {
		
		if (i == (buffer_size-1)) {
			is_last_transfer = true;
		}
		else {
			is_last_transfer = false;
		}
		
		SPI0->SPI_TDR = spi_generateControlWord(is_last_transfer, chip_select, tx_buffer[i]);
		while (!(SPI0->SPI_SR & SPI_SR_TDRE)) {
			if (!timeout--) {
				return SPI_ERROR_TIMEOUT;
			}
		}
		if(rx_buffer != NULL){
			rx_buffer[i] = (SPI0->SPI_RDR & 0xFFFF);
		}
	}
	return SPI_OK;
}

