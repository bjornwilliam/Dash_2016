
#ifndef SPI_WRAPPER_H_
#define SPI_WRAPPER_H_
#include "../same70-base_16/FreeRTOS/Source/include/FreeRTOS.h"
#include "../same70-base_16/FreeRTOS/Source/include/semphr.h"
#include "../same70-base_16/RevolveDrivers/spi.h"

extern SemaphoreHandle_t spi_handlerIsDoneSempahore;
extern SemaphoreHandle_t spi_mutex;

void spi_wrapper_master_init(Spi *spi, struct SpiMasterSettings spi_settings);
void spi_freeRTOSTranceive(uint16_t *tx_buffer,  uint16_t *rx_buffer, uint32_t buffer_size, enum SpiChipSelect chip_select);

spi_status_t spi_transceive_tdre(uint16_t *tx_buffer,  uint16_t *rx_buffer, uint32_t buffer_size, enum SpiChipSelect chip_select);


#endif /* SPI_WRAPPER_H_ */