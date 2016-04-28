#ifndef XDMAC_H_INCLUDED
#define XDMAC_H_INCLUDED


#include "sam.h"

/** DMA channel hardware interface number */
#define XDMAC_CHANNEL_HWID_HSMCI       0
#define XDMAC_CHANNEL_HWID_SPI0_TX     1
#define XDMAC_CHANNEL_HWID_SPI0_RX     2
#define XDMAC_CHANNEL_HWID_SPI1_TX     3
#define XDMAC_CHANNEL_HWID_SPI1_RX     4
#define XDMAC_CHANNEL_HWID_QSPI_TX     5
#define XDMAC_CHANNEL_HWID_QSPI_RX     6
#define XDMAC_CHANNEL_HWID_USART0_TX   7
#define XDMAC_CHANNEL_HWID_USART0_RX   8
#define XDMAC_CHANNEL_HWID_USART1_TX   9
#define XDMAC_CHANNEL_HWID_USART1_RX   10
#define XDMAC_CHANNEL_HWID_USART2_TX   11
#define XDMAC_CHANNEL_HWID_USART2_RX   12
#define XDMAC_CHANNEL_HWID_PWM0        13
#define XDMAC_CHANNEL_HWID_TWIHS0_TX   14
#define XDMAC_CHANNEL_HWID_TWIHS0_RX   15
#define XDMAC_CHANNEL_HWID_TWIHS1_TX   16
#define XDMAC_CHANNEL_HWID_TWIHS1_RX   17
#define XDMAC_CHANNEL_HWID_TWIHS2_TX   18
#define XDMAC_CHANNEL_HWID_TWIHS2_RX   19
#define XDMAC_CHANNEL_HWID_UART0_TX    20
#define XDMAC_CHANNEL_HWID_UART0_RX    21
#define XDMAC_CHANNEL_HWID_UART1_TX    22
#define XDMAC_CHANNEL_HWID_UART1_RX    23
#define XDMAC_CHANNEL_HWID_UART2_TX    24
#define XDMAC_CHANNEL_HWID_UART2_RX    25
#define XDMAC_CHANNEL_HWID_UART3_TX    26
#define XDMAC_CHANNEL_HWID_UART3_RX    27
#define XDMAC_CHANNEL_HWID_UART4_TX    28
#define XDMAC_CHANNEL_HWID_UART4_RX    29
#define XDMAC_CHANNEL_HWID_DAC         30
#define XDMAC_CHANNEL_HWID_SSC_TX      32
#define XDMAC_CHANNEL_HWID_SSC_RX      33
#define XDMAC_CHANNEL_HWID_PIOA        34
#define XDMAC_CHANNEL_HWID_AFEC0       35
#define XDMAC_CHANNEL_HWID_AFEC1       36
#define XDMAC_CHANNEL_HWID_AES_TX      37
#define XDMAC_CHANNEL_HWID_AES_RX      38
#define XDMAC_CHANNEL_HWID_PWM1        39
#define XDMAC_CHANNEL_HWID_TC0         40
#define XDMAC_CHANNEL_HWID_TC1         41
#define XDMAC_CHANNEL_HWID_TC2         42
#define XDMAC_CHANNEL_HWID_TC3         43

/* XDMA_MBR_UBC */
#define   XDMAC_UBC_NDE            (0x1u << 24)
#define   XDMAC_UBC_NDE_FETCH_DIS  (0x0u << 24)
#define   XDMAC_UBC_NDE_FETCH_EN   (0x1u << 24)
#define   XDMAC_UBC_NSEN           (0x1u << 25)
#define   XDMAC_UBC_NSEN_UNCHANGED (0x0u << 25)
#define   XDMAC_UBC_NSEN_UPDATED   (0x1u << 25)
#define   XDMAC_UBC_NDEN           (0x1u << 26)
#define   XDMAC_UBC_NDEN_UNCHANGED (0x0u << 26)
#define   XDMAC_UBC_NDEN_UPDATED   (0x1u << 26)
#define   XDMAC_UBC_NVIEW_Pos      27
#define   XDMAC_UBC_NVIEW_Msk      (0x3u << XDMAC_UBC_NVIEW_Pos)
#define   XDMAC_UBC_NVIEW_NDV0     (0x0u << XDMAC_UBC_NVIEW_Pos)
#define   XDMAC_UBC_NVIEW_NDV1     (0x1u << XDMAC_UBC_NVIEW_Pos)
#define   XDMAC_UBC_NVIEW_NDV2     (0x2u << XDMAC_UBC_NVIEW_Pos)
#define   XDMAC_UBC_NVIEW_NDV3     (0x3u << XDMAC_UBC_NVIEW_Pos)
#define   XDMAC_UBC_UBLEN_Pos 0
#define   XDMAC_UBC_UBLEN_Msk (0xffffffu << XDMAC_UBC_UBLEN_Pos)
#define   XDMAC_UBC_UBLEN(value) ((XDMAC_UBC_UBLEN_Msk & ((value) << XDMAC_UBC_UBLEN_Pos)))



struct XdmacChannelConfig {
	uint32_t receive_channel;
	uint32_t microblock_length;
	uint32_t source_address; //
	uint32_t destination_address; //
	uint32_t config_register; // 
	uint32_t block_length;
	uint32_t mempattern_datastride;
	uint32_t source_microblock_datastride; //
	uint32_t destination_microblock_datastride; // 
	
	};

/**
 * \brief Structure for storing parameters for DMA view0 that can be
 * performed by the DMA Master transfer.
 */
typedef struct {
	/** Next Descriptor Address number. */
	uint32_t mbr_nda;
	/** Microblock Control Member. */
	uint32_t microblock_length;
	/** Destination Address Member. */
	uint32_t destination_address;
} lld_view0;

/**
 * \brief Structure for storing parameters for DMA view1 that can be
 * performed by the DMA Master transfer.
 */
typedef struct {
	/** Next Descriptor Address number. */
	uint32_t mbr_nda;
	/** Microblock Control Member. */
	uint32_t microblock_length;
	/** Source Address Member. */
	uint32_t source_address;
	/** Destination Address Member. */
	uint32_t destination_address;
} lld_view1;

/**
 * \brief Structure for storing parameters for DMA view2 that can be
 * performed by the DMA Master transfer.
 */
typedef struct {
	/** Next Descriptor Address number. */
	uint32_t mbr_nda;
	/** Microblock Control Member. */
	uint32_t microblock_length;
	/** Source Address Member. */
	uint32_t source_address;
	/** Destination Address Member. */
	uint32_t destination_address;
	/** Configuration Register. */
	uint32_t config_register;
} lld_view2;

/**
 * \brief Structure for storing parameters for DMA view3 that can be
 * performed by the DMA Master transfer.
 */
typedef struct {
	/** Next Descriptor Address number. */
	uint32_t mbr_nda;
	/** Microblock Control Member. */
	uint32_t microblock_length;
	/** Source Address Member. */
	uint32_t source_address;
	/** Destination Address Member. */
	uint32_t destination_address;
	/** Configuration Register. */
	uint32_t config_register;
	/** Block Control Member. */
	uint32_t block_length;
	/** Data Stride Member. */
	uint32_t mempattern_datastride;
	/** Source Microblock Stride Member. */
	uint32_t source_microblock_datastride;
	/** Destination Microblock Stride Member. */
	uint32_t destination_microblock_datastride;
} lld_view3;

/**
 * \brief Get XDMAC global type.
 *
 * \param[out] xdmac Module hardware register base address pointer.
 */
static inline uint32_t xdmac_get_type( Xdmac *xdmac)
{
	return xdmac->XDMAC_GTYPE;
}

/**
 * \brief Get XDMAC global configuration.
 *
 * \param[out] xdmac Module hardware register base address pointer.
 */
static inline uint32_t xdmac_get_config(Xdmac *xdmac)
{
	
	return xdmac->XDMAC_GCFG;
}

/**
 * \brief Get XDMAC global weighted arbiter configuration.
 *
 * \param[out] xdmac Module hardware register base address pointer.
 */
static inline uint32_t xdmac_get_arbiter(Xdmac *xdmac)
{
	
	return xdmac->XDMAC_GWAC;
}

/**
 * \brief Enables XDMAC global interrupt.
 *
 * \param[out] xdmac Module hardware register base address pointer.
 * \param[in] mask A bitmask of channels to be enabled interrupt.
 */
static inline void xdmac_enable_interrupt(Xdmac *xdmac, uint32_t mask)
{
	
	xdmac->XDMAC_GIE = ( XDMAC_GIE_IE0 << mask) ;
}

/**
 * \brief Disables XDMAC global interrupt
 *
 * \param[out] xdmac Module hardware register base address pointer.
 * \param[in] mask A bitmask of channels to be disabled interrupt.
 */
static inline void xdmac_disable_interrupt(Xdmac *xdmac, uint32_t mask)
{
	
	xdmac->XDMAC_GID = (XDMAC_GID_ID0 << mask);
}

/**
 * \brief Get XDMAC global interrupt mask.
 *
 * \param[out] xdmac Module hardware register base address pointer.
 */
static inline uint32_t xdmac_get_interrupt_mask(Xdmac *xdmac)
{
	
	return (xdmac->XDMAC_GIM);
}

/**
 * \brief Get XDMAC global interrupt status.
 *
 * \param[out] xdmac Module hardware register base address pointer.
 */
static inline uint32_t xdmac_get_interrupt_status(Xdmac *xdmac)
{
	
	return (xdmac->XDMAC_GIS);
}

/**
 * \brief enables the relevant channel of given XDMAC.
 *
 * \param[out] xdmac Module hardware register base address pointer.
 * \param[in]  channel_num  XDMA Channel number (range 0 to 23)
 */
static inline void xdmac_channel_enable(Xdmac *xdmac, uint32_t channel_num)
{
	
	xdmac->XDMAC_GE = (XDMAC_GE_EN0 << channel_num);
}

/**
 * \brief Disables the relevant channel of given XDMAC.
 *
 * \param[out] xdmac Module hardware register base address pointer.
 * \param[in]  channel_num  XDMA Channel number (range 0 to 23)
 */
static inline void xdmac_channel_disable(Xdmac *xdmac, uint32_t channel_num)
{
	
	xdmac->XDMAC_GD =(XDMAC_GD_DI0 << channel_num);
}

/**
 * \brief Get Global channel status of given XDMAC.
 * \note: When set to 1, this bit indicates that the channel x is enabled.
          If a channel disable request is issued, this bit remains asserted
          until pending transaction is completed.
 * \param[out] xdmac Module hardware register base address pointer.
 */
static inline uint32_t xdmac_channel_get_status(Xdmac *xdmac)
{
	
	return xdmac->XDMAC_GS;
}

/**
 * \brief Suspend the relevant channel's read.
 *
 * \param[out] xdmac Module hardware register base address pointer.
 * \param[in] channel_num  XDMA Channel number (range 0 to 23).
 */
static inline void xdmac_channel_read_suspend(Xdmac *xdmac, uint32_t channel_num)
{
	xdmac->XDMAC_GRS |= XDMAC_GRS_RS0 << channel_num;
}

/**
 * \brief Suspend the relevant channel's write.
 *
 * \param[out] xdmac Module hardware register base address pointer.
 * \param[in] channel_num  XDMA Channel number (range 0 to 23).
 */
static inline void xdmac_channel_write_suspend(Xdmac *xdmac, uint32_t channel_num)
{
	
	xdmac->XDMAC_GWS |= XDMAC_GWS_WS0 << channel_num;
}

/**
 * \brief Suspend the relevant channel's read & write.
 *
 * \param[out] xdmac Module hardware register base address pointer.
 * \param[in] channel_num  XDMA Channel number (range 0 to 23).
 */
static inline void xdmac_channel_readwrite_suspend(Xdmac *xdmac, uint32_t channel_num)
{
	
	xdmac->XDMAC_GRWS = (XDMAC_GRWS_RWS0 << channel_num);
}

/**
 * \brief Resume the relevant channel's read & write.
 *
 * \param[out] xdmac Module hardware register base address pointer.
 * \param[in] channel_num  XDMA Channel number (range 0 to 23).
 */
static inline void xdmac_channel_readwrite_resume(Xdmac *xdmac, uint32_t channel_num)
{
	xdmac->XDMAC_GRWR = (XDMAC_GRWR_RWR0 << channel_num);
}

/**
 * \brief Set software transfer request on the relevant channel.
 *
 * \param[out] xdmac Module hardware register base address pointer.
 * \param[in] channel_num  XDMA Channel number (range 0 to 23).
 */
static inline void xdmac_channel_software_request(Xdmac *xdmac, uint32_t channel_num)
{

	xdmac->XDMAC_GSWR = (XDMAC_GSWR_SWREQ0 << channel_num);
}

/**
 * \brief Get software transfer status of the relevant channel.
 *
 * \param[out] xdmac Module hardware register base address pointer.
 */
static inline uint32_t xdmac_get_software_request_status(Xdmac *xdmac)
{
	
	return xdmac->XDMAC_GSWS;
}

/**
 * \brief Enable interrupt with mask on the relevant channel of given XDMA.
 *
 * \param[out] xdmac Module hardware register base address pointer.
 * \param[in] channel_num  XDMA Channel number (range 0 to 23).
 * \param[in] mask Interrupt mask.
 */
static inline void xdmac_channel_enable_interrupt(Xdmac *xdmac, uint32_t channel_num, uint32_t mask)
{
	
	xdmac->XDMAC_CHID[channel_num].XDMAC_CIE = mask;
}

/**
 * \brief Disable interrupt with mask on the relevant channel of given XDMA.
 *
 * \param[out] xdmac Module hardware register base address pointer.
 * \param[in] channel_num  XDMA Channel number (range 0 to 23).
 * \param[in] mask Interrupt mask.
 */
static inline void xdmac_channel_disable_interrupt(Xdmac *xdmac, uint32_t channel_num, uint32_t mask)
{
	
	xdmac->XDMAC_CHID[channel_num].XDMAC_CID = mask;
}

/**
 * \brief Get interrupt mask for the relevant channel of given XDMA.
 *
 * \param[out] xdmac Module hardware register base address pointer.
 * \param[in] channel_num  XDMA Channel number (range 0 to 23).
 */
static inline uint32_t xdmac_channel_get_interrupt_mask(Xdmac *xdmac, uint32_t channel_num)
{
	return xdmac->XDMAC_CHID[channel_num].XDMAC_CIM;
}

/**
 * \brief Get interrupt status for the relevant channel of given XDMA.
 *
 * \param[out] xdmac Module hardware register base address pointer.
 * \param[in] channel_num  XDMA Channel number (range 0 to 23).
 */
static inline uint32_t xdmac_channel_get_interrupt_status(Xdmac *xdmac, uint32_t channel_num)
{

	return xdmac->XDMAC_CHID[channel_num].XDMAC_CIS;
}

/**
 * \brief Set software flush request on the relevant channel.
 *
 * \param[out] xdmac Module hardware register base address pointer.
 * \param[in] channel_num  XDMA Channel number (range 0 to 23).
 */
static inline void xdmac_channel_software_flush_request(Xdmac *xdmac, uint32_t channel_num)
{

	xdmac->XDMAC_GSWF = (XDMAC_GSWF_SWF0 << channel_num);
	//while( !(xdmac_channel_get_interrupt_status(xdmac, channel_num) & XDMAC_CIS_FIS) );
}

/**
 * \brief Set source address for the relevant channel of given XDMA.
 *
 * \param[out] xdmac Module hardware register base address pointer
 * \param[in] channel_num  DMA Channel number (range 0 to 23)
 * \param[in] src_addr Source address
 */
static inline void xdmac_channel_set_source_addr(Xdmac *xdmac, uint32_t channel_num, uint32_t src_addr)
{

	xdmac->XDMAC_CHID[channel_num].XDMAC_CSA = src_addr;
}

/**
 * \brief Set destination address for the relevant channel of given XDMA.
 *
 * \param[out] xdmac Module hardware register base address pointer
 * \param[in] channel_num  DMA Channel number (range 0 to 23)
 * \param[in] dst_addr Destination address
 */
static inline void xdmac_channel_set_destination_addr(Xdmac *xdmac, uint32_t channel_num, uint32_t dst_addr)
{

	xdmac->XDMAC_CHID[channel_num].XDMAC_CDA = dst_addr;
}

/**
 * \brief Set next descriptor's address & interface for the relevant channel of given XDMA.
 *
 * \param[out] xdmac Module hardware register base address pointer
 * \param[in] channel_num  DMA Channel number (range 0 to 23)
 * \param[in] desc_addr Address of next descriptor.
 * \param[in] ndaif Interface of next descriptor.
 */
static inline void xdmac_channel_set_descriptor_addr(Xdmac *xdmac, uint32_t channel_num,
		uint32_t desc_addr, uint8_t ndaif)
{

	xdmac->XDMAC_CHID[channel_num].XDMAC_CNDA = (desc_addr & 0xFFFFFFFC) | ndaif;
}

/**
 * \brief Set next descriptor's configuration for the relevant channel of given XDMA.
 *
 * \param[out] xdmac Module hardware register base address pointer
 * \param[in] channel_num  DMA Channel number (range 0 to 23)
 * \param[in] config Configuration of next descriptor.
 */
static inline void xdmac_channel_set_descriptor_control(Xdmac *xdmac, uint32_t channel_num, uint32_t config)
{

	xdmac->XDMAC_CHID[channel_num].XDMAC_CNDC = config;
}

/**
 * \brief Set microblock length for the relevant channel of given XDMA.
 *
 * \param[out] xdmac Module hardware register base address pointer
 * \param[in] channel_num  DMA Channel number (range 0 to 23)
 * \param[in] ublen Microblock length.
 */
static inline void xdmac_channel_set_microblock_control(Xdmac *xdmac, uint32_t channel_num, uint32_t ublen)
{

	xdmac->XDMAC_CHID[channel_num].XDMAC_CUBC = XDMAC_CUBC_UBLEN(ublen);
}

/**
 * \brief Set block length for the relevant channel of given XDMA.
 *
 * \param[out] xdmac Module hardware register base address pointer
 * \param[in] channel_num  XDMA Channel number (range 0 to 23)
 * \param[in] blen Block length.
 */
static inline void xdmac_channel_set_block_control(Xdmac *xdmac, uint32_t channel_num, uint32_t blen)
{

	xdmac->XDMAC_CHID[channel_num].XDMAC_CBC = XDMAC_CBC_BLEN(blen);
}

/**
 * \brief Set configuration for the relevant channel of given XDMA.
 *
 * \param[out] xdmac Module hardware register base address pointer
 * \param[in] channel_num  XDMA Channel number (range 0 to 23)
 * \param[in] config Channel configuration.
 */
static inline void xdmac_channel_set_config(Xdmac *xdmac, uint32_t channel_num, uint32_t config)
{

	xdmac->XDMAC_CHID[channel_num].XDMAC_CC = config;
}

/**
 * \brief Set the relevant channel's data stride memory pattern of given XDMA.
 *
 * \param[out] xdmac Module hardware register base address pointer
 * \param[in] channel_num  XDMA Channel number (range 0 to 23)
 * \param[in] dds_msp Data stride memory pattern.
 */
static inline void xdmac_channel_set_datastride_mempattern(Xdmac *xdmac, uint32_t channel_num, uint32_t dds_msp)
{
	xdmac->XDMAC_CHID[channel_num].XDMAC_CDS_MSP = dds_msp;
}

/**
 * \brief Set the relevant channel's source microblock stride of given XDMA.
 *
 * \param[out] xdmac Module hardware register base address pointer
 * \param[in] channel_num  XDMA Channel number (range 0 to 23)
 * \param[in] subs Source microblock stride.
 */
static inline void xdmac_channel_set_source_microblock_stride(Xdmac *xdmac,
		uint32_t channel_num, uint32_t subs)
{
	xdmac->XDMAC_CHID[channel_num].XDMAC_CSUS = XDMAC_CSUS_SUBS(subs);
}

/**
 * \brief Set the relevant channel's destination microblock stride of given XDMA.
 *
 * \param[out] xdmac Module hardware register base address pointer
 * \param[in] channel_num  XDMA Channel number (range 0 to 23)
 * \param[in] dubs Destination microblock stride.
 */
static inline void xdmac_channel_set_destination_microblock_stride(Xdmac *xdmac,
		uint32_t channel_num, uint32_t dubs)
{
	xdmac->XDMAC_CHID[channel_num].XDMAC_CDUS = XDMAC_CDUS_DUBS(dubs);
}

void xdmac_configure_transfer(Xdmac *xdmac, uint32_t channel_num,
		volatile struct XdmacChannelConfig *p_cfg);



#endif /* XDMAC_H_INCLUDED */
