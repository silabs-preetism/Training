#include "SPI.h"
#include "UDMA.h"
#include "rsi_rom_udma_wrapper.h" 
#include "rsi_ccp_user_config.h"

extern RSI_UDMA_HANDLE_T udmaHandle0;    //check
extern uint32_t dma_rom_buff0[30],dma_rom_buff1[30];     //we can keep wrapeers

#define CONTROL_STRUCT0   (UDMA_NUMBER_OF_CHANNELS * 2)
#define CONTROL_STRUCT1   (ULP_UDMA_NUMBER_OF_CHANNELS * 2)

/* IAR support */
#if defined(__ICCARM__)
#pragma location = UDMA0_SRAM_BASE
extern RSI_UDMA_DESC_T UDMA0_Table[CONTROL_STRUCT0];
#pragma location = UDMA1_SRAM_BASE
extern RSI_UDMA_DESC_T UDMA1_Table[CONTROL_STRUCT1];
#endif

/* DMA descriptors must be aligned to 16 bytes */
#if defined(__CC_ARM)
extern RSI_UDMA_DESC_T UDMA0_Table[CONTROL_STRUCT0] ;
extern RSI_UDMA_DESC_T UDMA1_Table[CONTROL_STRUCT1] ;
#endif /* defined (__CC_ARM) */

#if defined( __GNUC__ )
extern RSI_UDMA_DESC_T __attribute__ ((section(".udma_addr0"))) UDMA0_Table[CONTROL_STRUCT0];
extern RSI_UDMA_DESC_T __attribute__ ((section(".udma_addr1"))) UDMA1_Table[CONTROL_STRUCT1];
#endif /* defined (__GNUC__) */

extern UDMA_Channel_Info udma0_chnl_info[32] ;
extern UDMA_Channel_Info udma1_chnl_info[12] ;

/* UDMA0 Resources */
extern UDMA_RESOURCES UDMA0_Resources ;
/* UDMA1 Resources */
extern UDMA_RESOURCES UDMA1_Resources;

#define ARM_SPI_DRV_VERSION    ARM_DRIVER_VERSION_MAJOR_MINOR(2, 0) /* driver version */

extern RSI_UDMA_HANDLE_T udmaHandle0,udmaHandle1;

#define ARM_SPI_DRV_VERSION    ARM_DRIVER_VERSION_MAJOR_MINOR(2, 0) /* driver version */

/* Driver Version */
static const ARM_DRIVER_VERSION DriverVersion = {
		ARM_SPI_API_VERSION,
		ARM_SPI_DRV_VERSION
};

/* Driver Capabilities */
static const ARM_SPI_CAPABILITIES DriverCapabilities = {
		0, /* Simplex Mode (Master and Slave) */
		1, /* TI Synchronous Serial Interface */
		1, /* Microwire Interface */
		1  /* Signal Mode Fault event: \ref SPI_EVENT_MODE_FAULT */
};


#ifdef SSI_MASTER

#define  SSI_MASTER_IRQHandler     IRQ047_Handler
// SSI_MASTER Run-Time Information
static SPI_INFO          SSI_MASTER_Info         = { 0U };
static SPI_TRANSFER_INFO SSI_MASTER_TransferInfo = { 0U };

#ifdef SSI_MASTER_MOSI_SEL
static SPI_PIN SSI_MASTER_mosi = {SSI_MASTER_MOSI_PORT, SSI_MASTER_MOSI_PIN, SSI_MASTER_MOSI_MODE, SSI_MASTER_MOSI_PADSEL};
#endif
#ifdef SSI_MASTER_MISO_SEL
static SPI_PIN SSI_MASTER_miso = {SSI_MASTER_MISO_PORT, SSI_MASTER_MISO_PIN, SSI_MASTER_MISO_MODE, SSI_MASTER_MISO_PADSEL};
#endif
#ifdef SSI_MASTER_SCK_SEL
static SPI_PIN SSI_MASTER_sck  = {SSI_MASTER_SCK_PORT,  SSI_MASTER_SCK_PIN, SSI_MASTER_SCK_MODE, SSI_MASTER_SCK_PADSEL};
#endif
#ifdef SSI_MASTER_CS0_SEL
static SPI_PIN SSI_MASTER_cs  =  {SSI_MASTER_CS0_PORT,  SSI_MASTER_CS0_PIN, SSI_MASTER_CS0_MODE, SSI_MASTER_CS0_PADSEL};
#endif

#if (SSI_MASTER_TX_DMA_Instance == 1)
void SSI_MASTER_UDMA_Tx_Event (uint32_t event ,uint8_t dmaCh);
static SPI_DMA SSI_MASTER_UDMA_TX_CHNL = {
		{
				0,
				0,
				0,
				0,
				1,
				SSIMASTER_ACK,
				SSI_MASTER_TX_DMA_Channel
		},
		SSI_MASTER_TX_DMA_Channel,
		SSI_MASTER_UDMA_Tx_Event
};
#endif
#if (SSI_MASTER_RX_DMA_Instance == 1)
void SSI_MASTER_UDMA_Rx_Event (uint32_t event ,uint8_t dmaCh);
static SPI_DMA SSI_MASTER_UDMA_RX_CHNL = {
		{
				0,
				0,
				0,
				0,
				1,
				SSIMASTER_ACK,
				SSI_MASTER_RX_DMA_Channel
		},
		SSI_MASTER_RX_DMA_Channel,
		SSI_MASTER_UDMA_Rx_Event
};
#endif

// SSI_MASTER Resources
static const SPI_RESOURCES SSI_MASTER_Resources = {
		SSI0,
		RSI_CLK_GetBaseClock,

		// PINS
		{
#ifdef SSI_MASTER_MOSI_SEL
				&SSI_MASTER_mosi,
#else
				NULL,
#endif
#ifdef SSI_MASTER_MISO_SEL
				&SSI_MASTER_miso,
#else
				NULL,
#endif
#ifdef SSI_MASTER_SCK_SEL
				&SSI_MASTER_sck,
#else
				NULL,
#endif
#ifdef SSI_MASTER_CS0_SEL
				&SSI_MASTER_cs,
#else
				NULL,
#endif
		},

		SSI0_IRQn,

#ifdef SSI_MASTER_RX_DMA_Instance
		&SSI_MASTER_UDMA_RX_CHNL,
#else
		NULL,
#endif

#ifdef SSI_MASTER_TX_DMA_Instance
		&SSI_MASTER_UDMA_TX_CHNL,
#else
		NULL,
#endif
		&SSI_MASTER_Info,
		&SSI_MASTER_TransferInfo,
		SPI_MASTER_MODE
};
#endif /* SSI_Master */
#ifdef SSI_SLAVE

#define  SSI_SLAVE_IRQHandler     IRQ044_Handler
// SSI_SLAVE Run-Time Information
static SPI_INFO          SSI_SLAVE_Info         = { 0U };
static SPI_TRANSFER_INFO SSI_SLAVE_TransferInfo = { 0U };

#ifdef SSI_SLAVE_MOSI_SEL
static SPI_PIN SSI_SLAVE_mosi = {SSI_SLAVE_MOSI_PORT, SSI_SLAVE_MOSI_PIN, SSI_SLAVE_MOSI_MODE, SSI_SLAVE_MOSI_PADSEL};
#endif
#ifdef SSI_SLAVE_MISO_SEL
static SPI_PIN SSI_SLAVE_miso = {SSI_SLAVE_MISO_PORT, SSI_SLAVE_MISO_PIN, SSI_SLAVE_MISO_MODE, SSI_SLAVE_MISO_PADSEL};
#endif
#ifdef SSI_SLAVE_SCK_SEL
static SPI_PIN SSI_SLAVE_sck  = {SSI_SLAVE_SCK_PORT,  SSI_SLAVE_SCK_PIN, SSI_SLAVE_SCK_MODE, SSI_SLAVE_SCK_PADSEL};
#endif
#ifdef SSI_SLAVE_CS0_SEL
static SPI_PIN SSI_SLAVE_cs  = {SSI_SLAVE_CS0_PORT,  SSI_SLAVE_CS0_PIN, SSI_SLAVE_CS0_MODE, SSI_SLAVE_CS0_PADSEL};
#endif

#if (SSI_SLAVE_TX_DMA_Instance == 1)
void SSI_SLAVE_UDMA_Tx_Event (uint32_t event ,uint8_t dmaCh);
static SPI_DMA SSI_SLAVE_UDMA_TX_CHNL = {
		{
				0,
				0,
				1,
				0,
				1,
				SSISLAVE_ACK,
				SSI_SLAVE_TX_DMA_Channel
		},
		SSI_SLAVE_TX_DMA_Channel,
		SSI_SLAVE_UDMA_Tx_Event
};
#endif
#if (SSI_SLAVE_RX_DMA_Instance == 1)
void SSI_SLAVE_UDMA_Rx_Event (uint32_t event ,uint8_t dmaCh);
static SPI_DMA SSI_SLAVE_UDMA_RX_CHNL = {
		{
				0,
				0,
				1,
				0,
				1,
				SSISLAVE_ACK,
				SSI_SLAVE_RX_DMA_Channel
		},
		SSI_SLAVE_RX_DMA_Channel,
		SSI_SLAVE_UDMA_Rx_Event
};
#endif

// SSI_SLAVE Resources
static const SPI_RESOURCES SSI_SLAVE_Resources = {
		SSISlave,
		RSI_CLK_GetBaseClock,

		// PINS
		{
#ifdef SSI_SLAVE_MOSI_SEL
				&SSI_SLAVE_mosi,
#else
				NULL,
#endif
#ifdef SSI_SLAVE_MISO_SEL
				&SSI_SLAVE_miso,
#else
				NULL,
#endif
#ifdef SSI_SLAVE_SCK_SEL
				&SSI_SLAVE_sck,
#else
				NULL,
#endif
#ifdef SSI_SLAVE_CS0_SEL
				&SSI_SLAVE_cs,
#else
				NULL,
#endif
		},

		SSI1_IRQn,

#ifdef SSI_SLAVE_RX_DMA_Instance
		&SSI_SLAVE_UDMA_RX_CHNL,
#else
		NULL,
#endif

#ifdef SSI_SLAVE_TX_DMA_Instance
		&SSI_SLAVE_UDMA_TX_CHNL,
#else
		NULL,
#endif

		&SSI_SLAVE_Info,
		&SSI_SLAVE_TransferInfo,
		SPI_SLAVE_MODE
};
#endif /* SSI_SLAVE */

#ifdef SSI_ULP_MASTER

#define  SSI_ULP_MASTER_IRQHandler     IRQ016_Handler
// SSI_ULP_MASTER Run-Time Information
static SPI_INFO          SSI_ULP_MASTER_Info         = { 0U };
static SPI_TRANSFER_INFO SSI_ULP_MASTER_TransferInfo = { 0U };

#ifdef SSI_ULP_MASTER_MOSI_SEL
static SPI_PIN SSI_ULP_MASTER_mosi = {SSI_ULP_MASTER_MOSI_PORT, SSI_ULP_MASTER_MOSI_PIN, SSI_ULP_MASTER_MOSI_MODE};
#endif
#ifdef SSI_ULP_MASTER_MISO_SEL
static SPI_PIN SSI_ULP_MASTER_miso = {SSI_ULP_MASTER_MISO_PORT, SSI_ULP_MASTER_MISO_PIN, SSI_ULP_MASTER_MISO_MODE};
#endif
#ifdef SSI_ULP_MASTER_SCK_SEL
static SPI_PIN SSI_ULP_MASTER_sck  = {SSI_ULP_MASTER_SCK_PORT,  SSI_ULP_MASTER_SCK_PIN, SSI_ULP_MASTER_SCK_MODE};
#endif
#ifdef SSI_ULP_MASTER_CS0_SEL
static SPI_PIN SSI_ULP_MASTER_cs  = {SSI_ULP_MASTER_CS0_PORT,  SSI_ULP_MASTER_CS0_PIN, SSI_ULP_MASTER_CS0_MODE};
#endif

#if (SSI_ULP_MASTER_TX_DMA_Instance == 1)
void SSI_ULP_MASTER_UDMA_Tx_Event (uint32_t event ,uint8_t dmaCh);
static SPI_DMA SSI_ULP_MASTER_UDMA_TX_CHNL = {
		{
				0,
				0,
				1,
				0,
				1,
				0,
				SSI_ULP_MASTER_TX_DMA_Channel
		},
		SSI_ULP_MASTER_TX_DMA_Channel,
		SSI_ULP_MASTER_UDMA_Tx_Event
};
#endif
#if (SSI_ULP_MASTER_RX_DMA_Instance == 1)
void SSI_ULP_MASTER_UDMA_Rx_Event (uint32_t event ,uint8_t dmaCh);
static SPI_DMA SSI_ULP_MASTER_UDMA_RX_CHNL = {
		{
				0,
				0,
				1,
				0,
				1,
				0,
				SSI_ULP_MASTER_RX_DMA_Channel
		},
		SSI_ULP_MASTER_RX_DMA_Channel,
		SSI_ULP_MASTER_UDMA_Rx_Event
};
#endif

// SSI_ULP_MASTER Resources
static const SPI_RESOURCES SSI_ULP_MASTER_Resources = {
		SSI2,
		RSI_CLK_GetBaseClock,

		// PINS
		{
#ifdef SSI_ULP_MASTER_MOSI_SEL
				&SSI_ULP_MASTER_mosi,
#else
				NULL,
#endif
#ifdef SSI_ULP_MASTER_MISO_SEL
				&SSI_ULP_MASTER_miso,
#else
				NULL,
#endif
#ifdef SSI_ULP_MASTER_SCK_SEL
				&SSI_ULP_MASTER_sck,
#else
				NULL,
#endif
#ifdef SSI_ULP_MASTER_CS0_SEL
				&SSI_ULP_MASTER_cs,
#else
				NULL,
#endif
		},

		SSI2_IRQn,

#ifdef SSI_ULP_MASTER_RX_DMA_Instance
		&SSI_ULP_MASTER_UDMA_RX_CHNL,
#else
		NULL,
#endif

#ifdef SSI_ULP_MASTER_TX_DMA_Instance
		&SSI_ULP_MASTER_UDMA_TX_CHNL,
#else
		NULL,
#endif

		&SSI_ULP_MASTER_Info,
		&SSI_ULP_MASTER_TransferInfo,
		SPI_ULP_MASTER_MODE
};
#endif /* SSI_ULP_MASTER */

/**
 * @fn           ARM_DRIVER_VERSION SPI_GetVersion(void)
 * @brief        Get Driver Version.
 * @param[in]    none
 * @return 		   ARM DRIVER VERSION
 */
ARM_DRIVER_VERSION SPI_GetVersion(void)
{
	return DriverVersion;
}

/**
 * @fn           ARM_SPI_CAPABILITIES SPI_GetCapabilities(void)
 * @brief        Get Driver capabilities.
 * @param[in]    none
 * @return 		   ARM SPI VERSION
 */
ARM_SPI_CAPABILITIES SPI_GetCapabilities(void)
{
	return DriverCapabilities;
}

/**
 * @fn           int32_t SPI_Initialize(ARM_SPI_SignalEvent_t cb_event, const SPI_RESOURCES *spi)
 * @brief        Initialize SPI Interface.
 * @param[in]    cb_event  : Pointer to the ARM_SPI_SignalEvent
 * @param[in]    spi       : Pointer to the spi resources
 * @return 		   excecution status 
 */
int32_t SPI_Initialize(ARM_SPI_SignalEvent_t cb_event, const SPI_RESOURCES *spi)
{
	if (spi->info->state & SPI_INITIALIZED)
	{
		return ARM_DRIVER_OK;
	}

	// Initialize SPI Run-Time Resources
	spi->info->cb_event = cb_event;
	spi->info->status.busy       = 0U;
	spi->info->status.data_lost  = 0U;
	spi->info->status.mode_fault = 0U;

	// Clear transfer information
	memset(spi->xfer, 0, sizeof(SPI_TRANSFER_INFO));
	if((spi->instance_mode == SPI_MASTER_MODE) || (spi->instance_mode == SPI_SLAVE_MODE)){

		/*Power up the ssi*/
		#if defined(CHIP_9117)
		RSI_PS_M4ssPeriPowerUp(M4SS_PWRGATE_ULP_EFUSE_PERI);
		#else
		RSI_PS_M4ssPeriPowerUp(M4SS_PWRGATE_ULP_PERI1);
		#endif

	//CLK
	if(spi->io.sck->pin > 64){
		RSI_EGPIO_UlpPadReceiverEnable(spi->io.sck->pin - 64);
		RSI_EGPIO_SetPinMux(EGPIO1 ,0 ,(spi->io.sck->pin - 64) ,EGPIO_PIN_MUX_MODE6);
	}
	RSI_EGPIO_SetPinMux(EGPIO, spi->io.sck->port, spi->io.sck->pin, spi->io.sck->mode);
	if(spi->io.sck->pad_sel !=0)
	{
		RSI_EGPIO_PadSelectionEnable(spi->io.sck->pad_sel);
	}
	RSI_EGPIO_PadReceiverEnable(spi->io.sck->pin);
	if(spi->io.sck->pin >= 25 && spi->io.sck->pin <= 30)
		RSI_EGPIO_HostPadsGpioModeEnable(spi->io.sck->pin);

	//CS0
	if(spi->io.cs->pin > 64){
		RSI_EGPIO_UlpPadReceiverEnable(spi->io.cs->pin - 64);
		RSI_EGPIO_SetPinMux(EGPIO1 ,0 ,(spi->io.cs->pin - 64) ,EGPIO_PIN_MUX_MODE6);
	}
	RSI_EGPIO_SetPinMux(EGPIO, spi->io.cs->port, spi->io.cs->pin, spi->io.cs->mode);
	if(spi->io.cs->pad_sel !=0)
	{
		RSI_EGPIO_PadSelectionEnable(spi->io.cs->pad_sel);
	}
	RSI_EGPIO_PadReceiverEnable(spi->io.cs->pin);
	if(spi->io.cs->pin >= 25 && spi->io.cs->pin <= 30)
		RSI_EGPIO_HostPadsGpioModeEnable(spi->io.cs->pin);

	//MOSI
	if(spi->io.cs->pin > 64){
		RSI_EGPIO_UlpPadReceiverEnable(spi->io.mosi->pin - 64);
		RSI_EGPIO_SetPinMux(EGPIO1 ,0 ,(spi->io.mosi->pin - 64) ,EGPIO_PIN_MUX_MODE6);
	}
	RSI_EGPIO_SetPinMux(EGPIO, spi->io.mosi->port, spi->io.mosi->pin, spi->io.mosi->mode);
	if(spi->io.mosi->pad_sel !=0)
	{
		RSI_EGPIO_PadSelectionEnable(spi->io.mosi->pad_sel);
	}
	RSI_EGPIO_PadReceiverEnable(spi->io.mosi->pin);
	if(spi->io.mosi->pin >= 25 && spi->io.mosi->pin <= 30)
		RSI_EGPIO_HostPadsGpioModeEnable(spi->io.mosi->pin);

		//MISO
		if(spi->io.miso->pin > 64){
			RSI_EGPIO_UlpPadReceiverEnable(spi->io.miso->pin - 64);
			RSI_EGPIO_SetPinMux(EGPIO1 ,0 ,(spi->io.miso->pin - 64) ,EGPIO_PIN_MUX_MODE6);
		}
		RSI_EGPIO_SetPinMux(EGPIO, spi->io.miso->port, spi->io.miso->pin, spi->io.miso->mode);
		if(spi->io.miso->pad_sel !=0)
		{
			RSI_EGPIO_PadSelectionEnable(spi->io.miso->pad_sel);
		}
		RSI_EGPIO_PadReceiverEnable(spi->io.miso->pin);
		if(spi->io.miso->pin >= 25 && spi->io.miso->pin <= 30)
			RSI_EGPIO_HostPadsGpioModeEnable(spi->io.miso->pin);
	}
	if(spi->instance_mode == SPI_ULP_MASTER_MODE){

		/*power up the ssi peripheral*/
		RSI_PS_UlpssPeriPowerUp(ULPSS_PWRGATE_ULP_SSI);

		RSI_EGPIO_UlpPadReceiverEnable(spi->io.sck->pin);
		RSI_EGPIO_SetPinMux(EGPIO1 ,spi->io.sck->port , spi->io.sck->pin ,spi->io.sck->mode);

		RSI_EGPIO_UlpPadReceiverEnable(spi->io.cs->pin);
		RSI_EGPIO_SetPinMux(EGPIO1 ,spi->io.cs->port , spi->io.cs->pin ,spi->io.cs->mode);

		RSI_EGPIO_UlpPadReceiverEnable(spi->io.mosi->pin);
		RSI_EGPIO_SetPinMux(EGPIO1 ,spi->io.mosi->port , spi->io.mosi->pin ,spi->io.mosi->mode);

		RSI_EGPIO_UlpPadReceiverEnable(spi->io.miso->pin);
		RSI_EGPIO_SetPinMux(EGPIO1 ,spi->io.miso->port , spi->io.miso->pin ,spi->io.miso->mode);
	}
#ifdef __SPI_DMA
	if ((spi->rx_dma != NULL) || (spi->tx_dma != NULL)) {

		// Enable DMA instance
		if ((spi->reg == SSI0) || (spi->reg == SSISlave)) {
			// DMA0 used for SSI_MASTER and SSI_SLAVE
			UDMAx_Initialize(&UDMA0_Resources,UDMA0_Table,udmaHandle0,dma_rom_buff0);
		} else {
			// DMA1 used for SSI_ULP_MASTER
			UDMAx_Initialize(&UDMA1_Resources,UDMA1_Table,udmaHandle1,dma_rom_buff1);
		}
	}
#endif
	spi->info->state = SPI_INITIALIZED;

	return ARM_DRIVER_OK;
}

/**
 * @fn           int32_t SPI_Uninitialize(const SPI_RESOURCES *spi)
 * @brief        DeInitialize SPI Interface
 * @param[in]    spi       : Pointer to the spi resources
 * @return 		   excecution status 
 */
int32_t SPI_Uninitialize(const SPI_RESOURCES *spi)
{
	spi->reg->SSIENR = SSI_DISABLE ;
	// Clear SPI state
	spi->info->state = 0U;
	if ((spi->reg == SSI0) || (spi->reg == SSISlave))
	{
		/*Power down the ssi*/
		#if defined(CHIP_9117)
		RSI_PS_M4ssPeriPowerDown(M4SS_PWRGATE_ULP_EFUSE_PERI);
		#else
		RSI_PS_M4ssPeriPowerDown(M4SS_PWRGATE_ULP_PERI1);
		#endif
	}
	else
	{
		/*power up the ssi peripheral*/
		RSI_PS_UlpssPeriPowerDown(ULPSS_PWRGATE_ULP_SSI);
	}
#ifdef __SPI_DMA
	if ((spi->rx_dma != NULL) || (spi->tx_dma != NULL))
	{
		// Diasable DMA instance
		if ((spi->reg == SSI0) || (spi->reg == SSISlave))
		{
			UDMAx_Uninitialize(&UDMA0_Resources);
		}
		else
		{
			UDMAx_Uninitialize(&UDMA1_Resources);
		}
	}
#endif
	return ARM_DRIVER_OK;
}

/**
 * @fn           int32_t SPI_PowerControl(ARM_POWER_STATE state, const SPI_RESOURCES *spi)
 * @brief        Control SPI Interface Power.
 * @param[in]    state     : Power state
 * @param[in]    spi       : Pointer to the spi resources
 * @return 		   excecution status 
 */
int32_t SPI_PowerControl(ARM_POWER_STATE state, const SPI_RESOURCES *spi)
{
	switch (state)
	{
	case ARM_POWER_OFF:

		NVIC_DisableIRQ (spi->irq_num);   // Disable SPI IRQ in NVIC
		spi->reg->SSIENR = SSI_DISABLE ;

		// Reset SPI Run-Time Resources
		spi->info->status.busy       = 0U;
		spi->info->status.data_lost  = 0U;
		spi->info->status.mode_fault = 0U;

		// Clear pending SPI interrupts in NVIC
		NVIC_ClearPendingIRQ(spi->irq_num);

		// Clear transfer information
		memset(spi->xfer, 0, sizeof(SPI_TRANSFER_INFO));

		spi->info->state &= ~SPI_POWERED; // SPI is not powered

		break;

	case ARM_POWER_FULL:

		if ((spi->info->state & SPI_INITIALIZED) ==0U) { return ARM_DRIVER_ERROR; }
		if ((spi->info->state & SPI_POWERED)     !=0U) { return ARM_DRIVER_OK; }

		// Connect SPI BASE clock
		/* Static Clock gating is Enabled for M4 SOC Other Clock*/
		if(spi->instance_mode == SPI_MASTER_MODE){
			RSI_CLK_SsiMstClkConfig(M4CLK ,ENABLE_STATIC_CLK,RTE_SSI_MASTER_INPUT_CLOCK,0);
		}
		else if(spi->instance_mode == SPI_SLAVE_MODE){
			RSI_CLK_PeripheralClkEnable(M4CLK,RTE_SSI_SLAVE_INPUT_CLOCK,ENABLE_STATIC_CLK);
		}
		else if(spi->instance_mode == SPI_ULP_MASTER_MODE){
			RSI_ULPSS_UlpSsiClkConfig(ULPCLK , ENABLE_STATIC_CLK, RTE_SSI_ULP_MASTER_INPUT_CLOCK, 0);
		}

		// Reset SPI Run-Time Resources
		spi->info->status.busy       = 0U;
		spi->info->status.data_lost  = 0U;
		spi->info->status.mode_fault = 0U;
		spi->info->state |= SPI_POWERED;  // SPI is powered

		NVIC_ClearPendingIRQ (spi->irq_num);
		NVIC_EnableIRQ (spi->irq_num);    // Enable SPI IRQ in NVIC
		break;

	case ARM_POWER_LOW:
	default:
		return ARM_DRIVER_ERROR_UNSUPPORTED;
	}
	return ARM_DRIVER_OK;
}

/**
 * @fn           int32_t SPI_Send(const void *data, uint32_t num, const SPI_RESOURCES *spi)
 * @brief        Start sending data to SPI transmitter.
 * @param[in]    data     : Pointer to buffer with data to send to SPI transmitter
 * @param[in]    num      : Number of data items to send
 * @param[in]    spi       : Pointer to the spi resources
 * @return 		   excecution status 
 */
int32_t SPI_Send(const void *data, uint32_t num, const SPI_RESOURCES *spi)
{
	uint16_t data_width;
	RSI_UDMA_CHA_CONFIG_DATA_T control ={0};
	volatile int32_t stat;

	volatile uint32_t dummy_data=0;

	spi->info->status.busy       = 0U;

	if ((data == NULL) || (num == 0U))
	{
		return ARM_DRIVER_ERROR_PARAMETER;
	}
	if (!(spi->info->state & SPI_CONFIGURED))
	{
		return ARM_DRIVER_ERROR;
	}
	if (  spi->info->status.busy)
	{
		return ARM_DRIVER_ERROR_BUSY;
	}
	spi->info->status.busy       = 1U;
	spi->info->status.data_lost  = 0U;
	spi->info->status.mode_fault = 0U;

	spi->xfer->tx_buf = (uint8_t *)data;
	spi->xfer->rx_buf = NULL;
	spi->xfer->num    = num;
	spi->xfer->rx_cnt = 0U;
	spi->xfer->tx_cnt = 0U;

	spi->reg->CTRLR0_b.TMOD    = TRANSMIT_AND_RECEIVE;

	spi->reg->SSIENR = SSI_ENABLE ;

	if((spi->instance_mode == SPI_MASTER_MODE) || (spi->instance_mode == SPI_ULP_MASTER_MODE))
	{
		data_width = spi->reg->CTRLR0_b.DFS_32;
	}
	else
	{
		data_width = spi->reg->CTRLR0_b.DFS;
	}
	if ((spi->rx_dma != NULL) || (spi->tx_dma != NULL))
	{
		if (spi->rx_dma != NULL)
		{
			control.transferType       = UDMA_MODE_BASIC;
			control.nextBurst          = 0;
			if(num < 1024)
			{
				control.totalNumOfDMATrans = (num-1);
			}
			else
			{
				control.totalNumOfDMATrans =0x3FF;
			}
			control.rPower             = ARBSIZE_1;
			control.srcProtCtrl        = 0x0;
			control.dstProtCtrl        = 0x0;
			if (data_width <= (8U - 1U))
			{
				// 8-bit data frame
				control.srcSize            = SRC_SIZE_8;
				control.srcInc             = SRC_INC_NONE;
				control.dstSize            = DST_SIZE_8;
				control.dstInc             = DST_INC_NONE;
			}
			else if(data_width <= (16U - 1U))
			{
				// 16-bit data frame
				control.srcSize            = SRC_SIZE_16;
				control.srcInc             = SRC_INC_NONE;
				control.dstSize            = DST_SIZE_16;
				control.dstInc             = DST_INC_NONE;
			}
			else
			{
				// 32-bit data frame
				control.srcSize            = SRC_SIZE_32;
				control.srcInc             = SRC_INC_NONE;
				control.dstSize            = DST_SIZE_32;
				control.dstInc             = DST_INC_NONE;
			}
			spi->reg->DMACR_b.RDMAE = 1;
			spi->reg->DMARDLR_b.DMARDL = 0;
			// Initialize and start SPI TX DMA Stream
			if ((spi->reg == SSI0) || (spi->reg == SSISlave))
			{
				stat = UDMAx_ChannelConfigure( &UDMA0_Resources,spi->rx_dma->channel,
						(uint32_t)&(spi->reg->DR),
						(uint32_t)&(dummy_data),
						num,
						control,
						&spi->rx_dma->chnl_cfg,
						spi->rx_dma->cb_event,udma0_chnl_info,udmaHandle0);
				if (stat == -1)
				{
					return ARM_DRIVER_ERROR;
				}
			}
			else
			{
				stat = UDMAx_ChannelConfigure(&UDMA1_Resources, spi->rx_dma->channel,
						(uint32_t)&(spi->reg->DR),
						(uint32_t)&(dummy_data),
						num,
						control,
						&spi->rx_dma->chnl_cfg,
						spi->rx_dma->cb_event,udma1_chnl_info,udmaHandle1);
				if (stat == -1)
				{
					return ARM_DRIVER_ERROR;
				}
			}
		}
		if (spi->tx_dma != NULL)
		{
			control.transferType       = UDMA_MODE_BASIC;
			control.nextBurst          = 0;
			if(num < 1024)
			{
				control.totalNumOfDMATrans = (num-1);
			}
			else
			{
				control.totalNumOfDMATrans = 0x3FF;
			}
			control.rPower             = ARBSIZE_1;
			control.srcProtCtrl        = 0x0;
			control.dstProtCtrl        = 0x0;
			if (data_width <= (8U - 1U))
			{
				// 8-bit data frame
				control.srcSize            = SRC_SIZE_8;
				control.srcInc             = SRC_INC_8;
				control.dstSize            = DST_SIZE_8;
				control.dstInc             = DST_INC_NONE;
			}
			else if(data_width <= (16U - 1U))
			{
				// 16-bit data frame
				control.srcSize            = SRC_SIZE_16;
				control.srcInc             = SRC_INC_16;
				control.dstSize            = DST_SIZE_16;
				control.dstInc             = DST_INC_NONE;
			}
			else
			{
				// 32-bit data frame
				control.srcSize            = SRC_SIZE_32;
				control.srcInc             = SRC_INC_32;
				control.dstSize            = DST_SIZE_32;
				control.dstInc             = DST_INC_NONE;
			}
			spi->reg->DMACR_b.TDMAE = 1;
			spi->reg->DMATDLR_b.DMATDL = 1;
			// Initialize and start SPI TX DMA Stream
			if ((spi->reg == SSI0) || (spi->reg == SSISlave))
			{
				stat = UDMAx_ChannelConfigure(&UDMA0_Resources,spi->tx_dma->channel,
						(uint32_t)(spi->xfer->tx_buf),
						(uint32_t)&(spi->reg->DR),
						num,
						control,
						&spi->tx_dma->chnl_cfg,
						spi->tx_dma->cb_event,udma0_chnl_info,udmaHandle0);
				if (stat == -1)
				{
					return ARM_DRIVER_ERROR;
				}
				UDMAx_ChannelEnable (spi->tx_dma->channel,&UDMA0_Resources,udmaHandle0);
				UDMAx_ChannelEnable (spi->rx_dma->channel,&UDMA0_Resources,udmaHandle0);
				UDMAx_DMAEnable(&UDMA0_Resources,udmaHandle0);
			}
			else
			{
				stat = UDMAx_ChannelConfigure(&UDMA0_Resources,spi->tx_dma->channel,
						(uint32_t)(spi->xfer->tx_buf),
						(uint32_t)&(spi->reg->DR),
						num,
						control,
						&spi->tx_dma->chnl_cfg,
						spi->tx_dma->cb_event,udma1_chnl_info,udmaHandle1);
				if (stat == -1)
				{
					return ARM_DRIVER_ERROR;
				}
				UDMAx_ChannelEnable (spi->tx_dma->channel,&UDMA1_Resources,udmaHandle1);
				UDMAx_ChannelEnable (spi->rx_dma->channel,&UDMA1_Resources,udmaHandle1);
				UDMAx_DMAEnable(&UDMA0_Resources,udmaHandle0);
			}
		}
	}
	else
	{
		spi->reg->IMR |= (TXEIM |RXFIM);
	}
	return ARM_DRIVER_OK;
}

/**
 * @fn          int32_t SPI_Receive( void *data,
                                  uint32_t num,
																	const SPI_RESOURCES *spi )
 * @brief		     Start receiving data from SPI receiver.
 * @param[in]    data        : Pointer to buffer for data to receive from SPI receiver
 * @param[in]    num         : Number of data items to receive
 * @param[in]	   spi        : Pointer to the SPI resources
 * @return 		   excecution status
 */
int32_t SPI_Receive(void *data, uint32_t num, const SPI_RESOURCES *spi)
{
	RSI_UDMA_CHA_CONFIG_DATA_T control ={0};
	uint16_t data_width;
	volatile int32_t stat;
	static uint32_t dummy_data=0;
	if ((data == NULL) || (num == 0U))        { return ARM_DRIVER_ERROR_PARAMETER; }
	if (!(spi->info->state & SPI_CONFIGURED)) { return ARM_DRIVER_ERROR; }
	if (  spi->info->status.busy)             { return ARM_DRIVER_ERROR_BUSY; }
	spi->info->status.busy       = 1U;
	spi->info->status.data_lost  = 0U;
	spi->info->status.mode_fault = 0U;

	spi->xfer->rx_buf = NULL;
	spi->xfer->num    = num;
	spi->xfer->rx_buf = (uint8_t *)data;
	spi->xfer->rx_cnt = 0U;
	spi->xfer->tx_cnt = 0U;
	spi->reg->CTRLR0_b.TMOD    = RECEIVE_ONLY;

	spi->reg->SSIENR = SSI_ENABLE ;

	if((spi->instance_mode == SPI_MASTER_MODE) || (spi->instance_mode == SPI_ULP_MASTER_MODE))
	{
		data_width = spi->reg->CTRLR0_b.DFS_32;
	}
	else
	{
		data_width = spi->reg->CTRLR0_b.DFS;
	}
#ifdef __SPI_DMA
	if ((spi->rx_dma != NULL) || (spi->tx_dma != NULL))
	{
		if (spi->rx_dma != NULL)
		{
			control.transferType       = UDMA_MODE_BASIC;
			control.nextBurst          = 0;
			if(num < 1024)
			{
				control.totalNumOfDMATrans = (num-1);
			}
			else
			{
				control.totalNumOfDMATrans = 0x3FF;
			}
			control.rPower             = ARBSIZE_1;
			control.srcProtCtrl        = 0x0;
			control.dstProtCtrl        = 0x0;
			if (data_width <= (8U - 1U))
			{
				// 8-bit data frame
				control.srcSize            = SRC_SIZE_8;
				control.srcInc             = SRC_INC_NONE;
				control.dstSize            = DST_SIZE_8;
				control.dstInc             = DST_INC_8;
			}
			else if(data_width <= (16U - 1U))
			{
				// 16-bit data frame
				control.srcSize            = SRC_SIZE_16;
				control.srcInc             = SRC_INC_NONE;
				control.dstSize            = DST_SIZE_16;
				control.dstInc             = DST_INC_16;
			}
			else
			{
				// 32-bit data frame
				control.srcSize            = SRC_SIZE_32;
				control.srcInc             = SRC_INC_NONE;
				control.dstSize            = DST_SIZE_32;
				control.dstInc             = DST_INC_32;
			}
			spi->reg->DMACR_b.RDMAE = 1;
			spi->reg->DMARDLR_b.DMARDL = 0;
			// Initialize and start SPI TX DMA Stream
			if ((spi->reg == SSI0) || (spi->reg == SSISlave))
			{
				stat = UDMAx_ChannelConfigure(&UDMA0_Resources,spi->rx_dma->channel,
						(uint32_t)&(spi->reg->DR),
						(uint32_t)(spi->xfer->rx_buf),
						num,
						control,
						&spi->rx_dma->chnl_cfg,
						spi->rx_dma->cb_event,udma0_chnl_info,udmaHandle0);
				if (stat == -1)
				{
					return ARM_DRIVER_ERROR;
				}
			}
			else
			{
				stat = UDMAx_ChannelConfigure(&UDMA1_Resources, spi->rx_dma->channel,
						(uint32_t)&(spi->reg->DR),
						(uint32_t)(spi->xfer->rx_buf),
						num,
						control,
						&spi->rx_dma->chnl_cfg,
						spi->rx_dma->cb_event,udma1_chnl_info,udmaHandle1);
				if (stat == -1)
				{
					return ARM_DRIVER_ERROR;
				}
			}
		}
		if (spi->tx_dma != NULL) {
			control.transferType       = UDMA_MODE_BASIC;
			control.nextBurst          = 0;
			if(num < 1024)
			{
				control.totalNumOfDMATrans = (num-1);
			}
			else
			{
				control.totalNumOfDMATrans = 0x3FF;
			}
			control.rPower             = ARBSIZE_1;
			control.srcProtCtrl        = 0x0;
			control.dstProtCtrl        = 0x0;
			if (data_width <= (8U - 1U))
			{
				// 8-bit data frame
				control.srcSize            = SRC_SIZE_8;
				control.srcInc             = SRC_INC_8;
				control.dstSize            = DST_SIZE_8;
				control.dstInc             = DST_INC_NONE;
			}
			else if(data_width <= (16U - 1U))
			{
				// 16-bit data frame
				control.srcSize            = SRC_SIZE_16;
				control.srcInc             = SRC_INC_16;
				control.dstSize            = DST_SIZE_16;
				control.dstInc             = DST_INC_NONE;
			}
			else
			{
				// 32-bit data frame
				control.srcSize            = SRC_SIZE_32;
				control.srcInc             = SRC_INC_32;
				control.dstSize            = DST_SIZE_32;
				control.dstInc             = DST_INC_NONE;
			}
			spi->reg->DMACR_b.TDMAE = 1;
			spi->reg->DMATDLR_b.DMATDL = 1;
			// Initialize and start SPI TX DMA Stream
			if ((spi->reg == SSI0) || (spi->reg == SSISlave))
			{
				stat = UDMAx_ChannelConfigure(&UDMA0_Resources, spi->tx_dma->channel,
						(uint32_t)&(dummy_data),
						(uint32_t)&(spi->reg->DR),
						num,
						control,
						&spi->tx_dma->chnl_cfg,
						spi->tx_dma->cb_event,udma0_chnl_info,udmaHandle0);
				if (stat == -1)
				{
					return ARM_DRIVER_ERROR;
				}
				UDMAx_ChannelEnable (spi->tx_dma->channel,&UDMA0_Resources,udmaHandle0);
				UDMAx_ChannelEnable (spi->rx_dma->channel,&UDMA0_Resources,udmaHandle0);
				UDMAx_DMAEnable(&UDMA0_Resources,udmaHandle0);
			}
			else
			{
				stat = UDMAx_ChannelConfigure(&UDMA1_Resources,spi->tx_dma->channel,
						(uint32_t)(dummy_data),
						(uint32_t)&(spi->reg->DR),
						num,
						control,
						&spi->tx_dma->chnl_cfg,
						spi->tx_dma->cb_event,udma1_chnl_info,udmaHandle1);
				if (stat == -1)
				{
					return ARM_DRIVER_ERROR;
				}
				UDMAx_ChannelEnable (spi->tx_dma->channel,&UDMA1_Resources,udmaHandle1);
				UDMAx_ChannelEnable (spi->rx_dma->channel,&UDMA1_Resources,udmaHandle1);
				UDMAx_DMAEnable(&UDMA1_Resources,udmaHandle1);
			}
		}
	}
	else
#endif
	{
		// Interrupt mode
		// RX Buffer not empty interrupt enable
		spi->reg->IMR |= (TXEIM |RXFIM);
	}
	return ARM_DRIVER_OK;
}

/**
 * @fn          int32_t SPI_Transfer(const void *data_out,
																				void *data_in,
																				uint32_t num,
																				const SPI_RESOURCES *spi )
 * @brief		     Start sending/receiving data to/from SPI transmitter/receiver.
 * @param[in]    data_out    : Pointer to buffer with data to send to SPI transmitter
 * @param[in]    data_in     : Pointer to buffer for data to receive from SPI receiver
 * @param[in]    num         : Number of data items to transfer
 * @param[in]	   spi         : Pointer to the SPI resources
 * @return 		   excecution status
 */
int32_t SPI_Transfer(const void *data_out, void *data_in, uint32_t num, const SPI_RESOURCES *spi)
{
	RSI_UDMA_CHA_CONFIG_DATA_T control ={0};
	uint16_t data_width;
	volatile int32_t stat;

	if ((data_out == NULL) || (data_in == NULL) || (num == 0U))        { return ARM_DRIVER_ERROR_PARAMETER; }
	if (!(spi->info->state & SPI_CONFIGURED)) { return ARM_DRIVER_ERROR; }
	if (  spi->info->status.busy)             { return ARM_DRIVER_ERROR_BUSY; }
	spi->info->status.busy       = 1U;
	spi->info->status.data_lost  = 0U;
	spi->info->status.mode_fault = 0U;

	spi->xfer->rx_buf = (uint8_t *)data_in;;
	spi->xfer->tx_buf = (uint8_t *)data_out;

	spi->xfer->num    = num;
	spi->xfer->rx_cnt = 0U;
	spi->xfer->tx_cnt = 0U;

	spi->reg->CTRLR0_b.TMOD    = TRANSMIT_AND_RECEIVE;

	spi->reg->SSIENR = SSI_ENABLE ;

	if((spi->instance_mode == SPI_MASTER_MODE) || (spi->instance_mode == SPI_ULP_MASTER_MODE))
	{
		data_width = spi->reg->CTRLR0_b.DFS_32;
	}
	else
	{
		data_width = spi->reg->CTRLR0_b.DFS;
	}
#ifdef __SPI_DMA
	if ((spi->rx_dma != NULL) || (spi->tx_dma != NULL))
	{
		if (spi->rx_dma != NULL)
		{
			control.transferType       = UDMA_MODE_BASIC;
			control.nextBurst          = 0;
			if(num < 1024)
			{
				control.totalNumOfDMATrans = (num-1);
			}
			else
			{
				control.totalNumOfDMATrans = 0x3FF;
			}
			control.rPower             = ARBSIZE_1;
			control.srcProtCtrl        = 0x0;
			control.dstProtCtrl        = 0x0;
			if (data_width <= (8U - 1U))
			{
				// 8-bit data frame
				control.srcSize            = SRC_SIZE_8;
				control.srcInc             = SRC_INC_NONE;
				control.dstSize            = DST_SIZE_8;
				control.dstInc             = DST_INC_8;
			}
			else if(data_width <= (16U - 1U))
			{
				// 16-bit data frame
				control.srcSize            = SRC_SIZE_16;
				control.srcInc             = SRC_INC_NONE;
				control.dstSize            = DST_SIZE_16;
				control.dstInc             = DST_INC_16;
			}
			else
			{
				// 32-bit data frame
				control.srcSize            = SRC_SIZE_32;
				control.srcInc             = SRC_INC_NONE;
				control.dstSize            = DST_SIZE_32;
				control.dstInc             = DST_INC_32;
			}
			spi->reg->DMACR_b.RDMAE = 1;
			spi->reg->DMARDLR_b.DMARDL = 0;
			// Initialize and start SPI TX DMA Stream
			if ((spi->reg == SSI0) || (spi->reg == SSISlave))
			{
				stat = UDMAx_ChannelConfigure(&UDMA0_Resources, spi->rx_dma->channel,
						(uint32_t)&(spi->reg->DR),
						(uint32_t)(spi->xfer->rx_buf),
						num,
						control,
						&spi->rx_dma->chnl_cfg,
						spi->rx_dma->cb_event,udma0_chnl_info,udmaHandle0);
				if (stat == -1)
				{
					return ARM_DRIVER_ERROR;
				}
			}
			else
			{
				stat = UDMAx_ChannelConfigure(&UDMA1_Resources, spi->rx_dma->channel,
						(uint32_t)&(spi->reg->DR),
						(uint32_t)(spi->xfer->rx_buf),
						num,
						control,
						&spi->rx_dma->chnl_cfg,
						spi->rx_dma->cb_event,udma1_chnl_info,udmaHandle1);
				if (stat == -1)
				{
					return ARM_DRIVER_ERROR;
				}
			}
		}
		if (spi->tx_dma != NULL) {

			control.transferType       = UDMA_MODE_BASIC;
			control.nextBurst          = 0;
			if(num < 1024)
			{
				control.totalNumOfDMATrans = (num-1);
			}
			else
			{
				control.totalNumOfDMATrans = 0x3FF;
			}
			control.rPower             = ARBSIZE_1;
			control.srcProtCtrl        = 0x0;
			control.dstProtCtrl        = 0x0;
			if (data_width <= (8U - 1U))
			{
				// 8-bit data frame
				control.srcSize            = SRC_SIZE_8;
				control.srcInc             = SRC_INC_8;
				control.dstSize            = DST_SIZE_8;
				control.dstInc             = DST_INC_NONE;
			}
			else if(data_width <= (16U - 1U))
			{
				// 16-bit data frame
				control.srcSize            = SRC_SIZE_16;
				control.srcInc             = SRC_INC_16;
				control.dstSize            = DST_SIZE_16;
				control.dstInc             = DST_INC_NONE;
			}
			else
			{
				// 32-bit data frame
				control.srcSize            = SRC_SIZE_32;
				control.srcInc             = SRC_INC_32;
				control.dstSize            = DST_SIZE_32;
				control.dstInc             = DST_INC_NONE;
			}
			spi->reg->DMACR_b.TDMAE = 1;
			spi->reg->DMATDLR_b.DMATDL = 1;
			// Initialize and start SPI TX DMA Stream
			if ((spi->reg == SSI0) || (spi->reg == SSISlave)) {
				stat = UDMAx_ChannelConfigure(&UDMA0_Resources,spi->tx_dma->channel,
						(uint32_t)(spi->xfer->tx_buf),
						(uint32_t)&(spi->reg->DR),
						num,
						control,
						&spi->tx_dma->chnl_cfg,
						spi->tx_dma->cb_event,udma0_chnl_info,udmaHandle0);
				if (stat == -1)
				{
					return ARM_DRIVER_ERROR;
				}
				UDMAx_ChannelEnable (spi->tx_dma->channel,&UDMA0_Resources,udmaHandle0);
				UDMAx_ChannelEnable (spi->rx_dma->channel,&UDMA0_Resources,udmaHandle0);
				UDMAx_DMAEnable(&UDMA0_Resources,udmaHandle0);
			}
			else{
				stat = UDMAx_ChannelConfigure(&UDMA1_Resources,spi->tx_dma->channel,
						(uint32_t)(spi->xfer->tx_buf),
						(uint32_t)&(spi->reg->DR),
						num,
						control,
						&spi->tx_dma->chnl_cfg,
						spi->tx_dma->cb_event,udma1_chnl_info,udmaHandle1);
				if (stat == -1)
				{
					return ARM_DRIVER_ERROR;
				}
				UDMAx_ChannelEnable (spi->tx_dma->channel,&UDMA1_Resources,udmaHandle1);
				UDMAx_ChannelEnable (spi->rx_dma->channel,&UDMA1_Resources,udmaHandle1);
				UDMAx_DMAEnable(&UDMA1_Resources,udmaHandle1);
			}
		}
	} else
#endif
	{
		// Interrupt mode
		// Buffer not empty interrupt enable
		spi->reg->IMR |= TXEIM | RXFIM;
	}
	return ARM_DRIVER_OK;
}

/**
 * @fn          uint32_t SPI_GetDataCount(const SPI_RESOURCES *spi)
 * @brief		     Get transferred data count.
 * @param[in]	   SPI        : Pointer to the SPI resources
 * @return 		   number of data items transferred
 */
uint32_t SPI_GetDataCount(const SPI_RESOURCES *spi)
{
	if (!(spi->info->state & SPI_CONFIGURED)) { return 0U; }

	return spi->xfer->rx_cnt;
}

/**
 * @fn          int32_t SPI_Control(uint32_t control, uint32_t arg, const SPI_RESOURCES *spi)
 * @brief		     Control SPI Interface
 * @param[in]    control    : Operation
 * @param[in]    arg        : Argument of operation(optional)
 * @param[in]	   spi        : Pointer to the SPI resources
 * @return 		   excecution status
 */
int32_t SPI_Control(uint32_t control, uint32_t arg, const SPI_RESOURCES *spi)
{
	uint32_t sck_divisor;
	uint32_t data_bits;
	uint32_t val=0;
	volatile uint32_t icr;
	if (!(spi->info->state & SPI_POWERED))
	{
		return ARM_DRIVER_ERROR;
	}
	if ((spi->reg == SSISlave) && ((control & ARM_SPI_CONTROL_Msk) == ARM_SPI_MODE_MASTER))
	{
		return ARM_DRIVER_ERROR_UNSUPPORTED;
	}
	if (((spi->reg == SSI0) || (spi->reg == SSI2)) && ((control & ARM_SPI_CONTROL_Msk) == ARM_SPI_MODE_SLAVE))
	{
		return ARM_DRIVER_ERROR_UNSUPPORTED;
	}
	if ((control & ARM_SPI_CONTROL_Msk) == ARM_SPI_ABORT_TRANSFER)
	{
		icr = spi->reg->ICR;           // Disable SPI interrupts
		memset(spi->xfer, 0, sizeof(SPI_TRANSFER_INFO));
		spi->info->status.busy = 0U;
		return ARM_DRIVER_OK;
	}
	if (spi->info->status.busy)
	{
		return ARM_DRIVER_ERROR_BUSY;
	}

	spi->reg->SSIENR = SSI_DISABLE ;

	switch (control & ARM_SPI_CONTROL_Msk)
	{
	case ARM_SPI_MODE_INACTIVE:
		spi->info->mode  &= ~ARM_SPI_CONTROL_Msk;
		spi->info->mode  |=  ARM_SPI_MODE_INACTIVE;
		spi->info->state &= ~SPI_CONFIGURED;
		return ARM_DRIVER_OK;

	case ARM_SPI_MODE_MASTER:
		spi->info->mode  &= ~ARM_SPI_CONTROL_Msk;
		spi->info->mode  |=  ARM_SPI_MODE_MASTER;
		spi->info->state |=  SPI_CONFIGURED;
		goto set_speed;

	case ARM_SPI_MODE_SLAVE:
		spi->info->mode  &= ~ARM_SPI_CONTROL_Msk;
		spi->info->mode  |=  ARM_SPI_MODE_SLAVE;
		spi->info->state |=  SPI_CONFIGURED;
		break;

	case ARM_SPI_MODE_MASTER_SIMPLEX:
		return ARM_SPI_ERROR_MODE;

	case ARM_SPI_MODE_SLAVE_SIMPLEX:
		return ARM_SPI_ERROR_MODE;

	case ARM_SPI_SET_BUS_SPEED:
		set_speed:
		if((spi->instance_mode ==  1) || (spi->instance_mode ==  2)) {
			sck_divisor = (spi->periph_clock(M4_SSI_MST)/arg);
		}else{
			sck_divisor = (spi->periph_clock(ULPSS_SSI)/arg);
		}
	spi->reg->BAUDR_b.SCKDV = sck_divisor;
	break;

	case ARM_SPI_GET_BUS_SPEED:             // Get Bus Speed in bps
		if((spi->instance_mode ==  1) || (spi->instance_mode ==  3)) {
			return (spi->periph_clock(M4_SSI_MST)/spi->reg->BAUDR_b.SCKDV);
		}  else {
			return (spi->periph_clock(ULPSS_SSI)/spi->reg->BAUDR_b.SCKDV);
		}
	case ARM_SPI_SET_DEFAULT_TX_VALUE:      // Set default Transmit value; arg = value
		spi->xfer->def_val = (uint16_t)(arg & 0xFFFF);
		return ARM_DRIVER_OK;

	case ARM_SPI_CONTROL_SS:                // Control Slave Select; arg = 0:inactive, 1:active
		if (((spi->info->mode & ARM_SPI_CONTROL_Msk) != ARM_SPI_MODE_MASTER) )
		{
			return ARM_DRIVER_ERROR;
		}
		val=(spi->info->mode & ARM_SPI_SS_MASTER_MODE_Msk);
		if (arg == ARM_SPI_SS_INACTIVE)
		{
			if(val==ARM_SPI_SS_MASTER_HW_OUTPUT)
			{
				if(spi->instance_mode == SPI_MASTER_MODE)
				{
					if((spi->io.cs->pin == 9)||(spi->io.cs->pin == 16)||(spi->io.cs->pin == 28)||(spi->io.cs->pin == 42)||(spi->io.cs->pin == 73))
					{
						/*cs0*/
						spi->reg->SER = 0x0<<0;
					}
					else if((spi->io.cs->pin == 10)||(spi->io.cs->pin == 17)||(spi->io.cs->pin == 29)||(spi->io.cs->pin == 43)||(spi->io.cs->pin == 64)||(spi->io.cs->pin == 70))
					{
						/*cs1*/
						spi->reg->SER = 0x0<<1;
					}
					else if((spi->io.cs->pin == 20)||(spi->io.cs->pin == 30)||(spi->io.cs->pin == 35)||(spi->io.cs->pin == 44)||(spi->io.cs->pin == 48)||(spi->io.cs->pin == 65))
					{
						/*cs2*/
						spi->reg->SER = 0x0<<2;
					}
					else if((spi->io.cs->pin == 23)||(spi->io.cs->pin == 45)||(spi->io.cs->pin == 47))
					{
						/*cs3*/
						spi->reg->SER = 0x0<<3;
					}
				}
				if(spi->instance_mode == SPI_ULP_MASTER_MODE)
				{
					if((spi->io.cs->pin == 3) || (spi->io.cs->pin == 10))
					{
						/*cs0*/
						spi->reg->SER = 0x0<<0;
					}
					if((spi->io.cs->pin == 2) || (spi->io.cs->pin == 4) || (spi->io.cs->pin == 12))
					{
						/*cs1*/
						spi->reg->SER = 0x0<<1;
					}
					if((spi->io.cs->pin == 6) || (spi->io.cs->pin == 13))
					{
						/*cs2*/
						spi->reg->SER = 0x0<<2;
					}
				}
				if(spi->instance_mode == SPI_SLAVE_MODE)
				{
					spi->reg->SER = 0<<0;
				}
			}
			else
			{

				RSI_EGPIO_SetPin(EGPIO, spi->io.cs->port, spi->io.cs->pin, 1);
			}
		}
		else
		{
			if(val==ARM_SPI_SS_MASTER_HW_OUTPUT)
			{
				if(spi->instance_mode == SPI_MASTER_MODE)
				{
					if((spi->io.cs->pin == 9)||(spi->io.cs->pin == 16)||(spi->io.cs->pin == 28)||(spi->io.cs->pin == 42)||(spi->io.cs->pin == 73))
					{
						/*cs0*/
						spi->reg->SER = 0x1<<0;
					}
					else if((spi->io.cs->pin == 10)||(spi->io.cs->pin == 17)||(spi->io.cs->pin == 29)||(spi->io.cs->pin == 43)||(spi->io.cs->pin == 64)||(spi->io.cs->pin == 70))
					{
						/*cs1*/
						spi->reg->SER = 0x1<<1;
					}
					else if((spi->io.cs->pin == 20)||(spi->io.cs->pin == 30)||(spi->io.cs->pin == 35)||(spi->io.cs->pin == 44)||(spi->io.cs->pin == 48)||(spi->io.cs->pin == 65))
					{
						/*cs2*/
						spi->reg->SER = 0x1<<2;
					}
					else if((spi->io.cs->pin == 23)||(spi->io.cs->pin == 45)||(spi->io.cs->pin == 47))
					{
						/*cs3*/
						spi->reg->SER = 0x1<<3;
					}
				}
				if(spi->instance_mode == SPI_ULP_MASTER_MODE)
				{
					if((spi->io.cs->pin == 3) || (spi->io.cs->pin == 10))
					{
						/*cs0*/
						spi->reg->SER = 0x1<<0;
					}
					if((spi->io.cs->pin == 2) || (spi->io.cs->pin == 4) || (spi->io.cs->pin == 12))
					{
						/*cs1*/
						spi->reg->SER = 0x1<<1;
					}
					if((spi->io.cs->pin == 6) || (spi->io.cs->pin == 13))
					{
						/*cs2*/
						spi->reg->SER = 0x1<<2;
					}
				}
				if(spi->instance_mode == SPI_SLAVE_MODE)
				{
					spi->reg->SER = 0x1<<0;
				}
			}
			else
			{
				RSI_EGPIO_SetPin(EGPIO, spi->io.cs->port, spi->io.cs->pin, 0);
			}
		}
		return ARM_DRIVER_OK;
	}

	if ((spi->info->mode & ARM_SPI_CONTROL_Msk) ==  ARM_SPI_MODE_MASTER) {
		switch (control & ARM_SPI_SS_MASTER_MODE_Msk) {
		case ARM_SPI_SS_MASTER_UNUSED:        // SPI Slave Select when Master: Not used (default)
			spi->reg->SER = 0;
			spi->info->mode  &= ~ARM_SPI_SS_MASTER_MODE_Msk;
			spi->info->mode  |=  ARM_SPI_SS_MASTER_UNUSED;
			break;

		case ARM_SPI_SS_MASTER_HW_INPUT:      // SPI Slave Select when Master: Hardware monitored Input
			spi->info->mode  &= ~ARM_SPI_SS_MASTER_MODE_Msk;
			return ARM_SPI_ERROR_SS_MODE;

		case ARM_SPI_SS_MASTER_SW:            // SPI Slave Select when Master: Software controlled
			spi->info->mode  &= ~ARM_SPI_SS_MASTER_MODE_Msk;
			if (spi->io.cs->pin != NULL)
			{
				if(spi->io.cs->pin > 64)
				{
					RSI_EGPIO_SetPinMux(EGPIO1 ,0 ,(spi->io.cs->pin - 64) ,EGPIO_PIN_MUX_MODE6);
				}
				RSI_EGPIO_SetPinMux(EGPIO, spi->io.cs->port, spi->io.cs->pin, 0);
				if(spi->io.cs->pad_sel != 0)
				{
					RSI_EGPIO_PadSelectionEnable(spi->io.cs->pad_sel);
				}
				RSI_EGPIO_SetPin(EGPIO, spi->io.cs->port, spi->io.cs->pin, 1);
				spi->info->mode |= ARM_SPI_SS_MASTER_SW;
			} else {
				return ARM_SPI_ERROR_SS_MODE;
			}
			break;

		case ARM_SPI_SS_MASTER_HW_OUTPUT:     // SPI Slave Select when Master: Hardware controlled Output
			spi->info->mode  &= ~ARM_SPI_SS_MASTER_MODE_Msk;
			if (spi->io.cs->pin != NULL) {
				spi->reg->SER = 1<<0;
				spi->info->mode |= ARM_SPI_SS_MASTER_HW_OUTPUT;
			} else {
				return ARM_SPI_ERROR_SS_MODE;
			}
			break;
		default:
			break;
		}
	}
	if ((spi->info->mode & ARM_SPI_CONTROL_Msk) ==  ARM_SPI_MODE_SLAVE) {
		switch (control & ARM_SPI_SS_SLAVE_MODE_Msk) {
		case ARM_SPI_SS_SLAVE_HW:             // SPI Slave Select when Slave: Hardware monitored (default)
			spi->info->mode  &= ~ARM_SPI_SS_SLAVE_MODE_Msk;
			if (spi->io.cs->pin != NULL) {
				spi->info->mode |= ARM_SPI_SS_SLAVE_HW;
			} else {
				return ARM_SPI_ERROR_SS_MODE;
			}
			break;

		case ARM_SPI_SS_SLAVE_SW:             // SPI Slave Select when Slave: Software controlled
			spi->info->mode  &= ~ARM_SPI_SS_SLAVE_MODE_Msk;
			return ARM_SPI_ERROR_SS_MODE;
		default:
			break;
		}
	}

	// Frame format:
	switch (control & ARM_SPI_FRAME_FORMAT_Msk) {
	case ARM_SPI_CPOL0_CPHA0:
		spi->reg->CTRLR0_b.SCPOL  = 0;
		spi->reg->CTRLR0_b.SCPH   = 0;
		break;

	case ARM_SPI_CPOL0_CPHA1:
		spi->reg->CTRLR0_b.SCPOL  = 0;
		spi->reg->CTRLR0_b.SCPH   = 1;
		break;

	case ARM_SPI_CPOL1_CPHA0:
		spi->reg->CTRLR0_b.SCPOL  = 1;
		spi->reg->CTRLR0_b.SCPH   = 0;
		break;

	case ARM_SPI_CPOL1_CPHA1:
		spi->reg->CTRLR0_b.SCPOL  = 1;
		spi->reg->CTRLR0_b.SCPH   = 1;
		break;

	case ARM_SPI_TI_SSI:
		spi->reg->CTRLR0_b.FRF = TEXAS_INSTRUMENTS_SSP;
		break;

	case ARM_SPI_MICROWIRE:
		spi->reg->CTRLR0_b.FRF = NATIONAL_SEMICONDUCTORS_MICROWIRE;
		break;

	default:
		return ARM_SPI_ERROR_FRAME_FORMAT;
	}

	// Configure Number of Data Bits
	data_bits = ((control & ARM_SPI_DATA_BITS_Msk) >> ARM_SPI_DATA_BITS_Pos);
	if((spi->instance_mode ==  1) || (spi->instance_mode ==  3))
	{
		if ((data_bits >=  4U) &&(data_bits <= 32U))
		{
			spi->reg->CTRLR0_b.DFS_32 = (data_bits -1U);
		}
		else
		{
			return ARM_SPI_ERROR_DATA_BITS;
		}
	}
	else
	{
		if ((data_bits >=  4U) &&(data_bits <= 16U))
		{
			spi->reg->CTRLR0_b.DFS = (data_bits -1U);
		}
		else
		{
			return ARM_SPI_ERROR_DATA_BITS;
		}
	}
	// Configure Bit Order
	if ((control & ARM_SPI_BIT_ORDER_Msk) == ARM_SPI_LSB_MSB) {
		return ARM_SPI_ERROR_BIT_ORDER;
	}

	spi->reg->CTRLR0_b.SRL = 0;
	spi->reg->CTRLR0_b.SPI_FRF = STANDARD_SPI_FORMAT;
	spi->reg->CTRLR0_b.FRF 	   = MOTOROLA_SPI;
	spi->reg->TXFTLR_b.TFT     = 0;
	spi->reg->RXFTLR_b.RFT     = 0;

	if((spi->instance_mode == 1U) || (spi->instance_mode == 3U))
		spi->reg->IMR &= ~(0x3F);           // Disable SPI interrupts
	else
		spi->reg->IMR &= ~(0x1F);           // Disable SPI interrupts

	return ARM_DRIVER_OK;
}

/**
 * @fn          ARM_SPI_STATUS SPI_GetStatus(const SPI_RESOURCES *spi)
 * @brief		    Get SPI status.
 * @param[in]	  spi        : Pointer to the SPI resources
 * @return 		  ARM_SPI_STATUS
 */
ARM_SPI_STATUS SPI_GetStatus(const SPI_RESOURCES *spi)
{
	ARM_SPI_STATUS status;

	status.busy       = spi->info->status.busy;
	status.data_lost  = spi->info->status.data_lost;
	status.mode_fault = spi->info->status.mode_fault;

	return status;
}

#ifdef __SPI_DMA_TX
static void SPI_UDMA_Tx_Event (uint32_t event,uint8_t dmaCh, SPI_RESOURCES *spi)
{
	switch (event)
	{
	case UDMA_EVENT_XFER_DONE:
		// Update TX buffer info
		spi->xfer->tx_cnt = spi->xfer->num;
		break;
	case UDMA_EVENT_ERROR:
		break;
	}
}
#endif

#ifdef __SPI_DMA_RX
static void SPI_UDMA_Rx_Event(uint32_t event, uint8_t dmaCh,SPI_RESOURCES *spi)
{
	switch (event)
	{
	case UDMA_EVENT_XFER_DONE:
		//spi->xfer->rx_cnt    = spi->xfer->num;
		spi->info->status.busy = 0U;
		break;
	case UDMA_EVENT_ERROR:
		break;
	}
	if (spi->info->cb_event != NULL) {
		spi->info->cb_event(ARM_SPI_EVENT_TRANSFER_COMPLETE);
	}
}
#endif
/**
 * @fn          void SPI_IRQHandler(const SPI_RESOURCES *spi)
 * @brief		    SPI IRQ Handler
 * @param[in]	  spi        : Pointer to the SPI resources
 * @return 		  none
 */
void SPI_IRQHandler (const SPI_RESOURCES *spi)
{
	uint8_t  data_8bit,i;
	uint16_t data_16bit;
	uint32_t data_32bit;
	uint32_t event;
	uint16_t data_width;
	uint32_t isr;
	uint32_t sr;
	volatile uint32_t ovrrunclr;
	volatile uint32_t icr;
	event = 0U;

	isr = spi->reg->ISR;
	sr  = spi->reg->SR;
	icr = spi->reg->ICR;

	if((spi->instance_mode == SPI_MASTER_MODE) || (spi->instance_mode == SPI_ULP_MASTER_MODE))
	{
		data_width = spi->reg->CTRLR0_b.DFS_32;
	}
	else
	{
		data_width = spi->reg->CTRLR0_b.DFS;
	}
	if ((isr & BIT(1))|| (isr & BIT(3)))
	{                             // Read overrun
		// Overrun flag is set
		// Clear Overrun flag
		ovrrunclr = spi->reg->TXOICR;
		ovrrunclr = spi->reg->RXOICR;
		spi->info->status.data_lost = 1U;
		event |= ARM_SPI_EVENT_DATA_LOST;
	}
	if ((((sr & BIT(3))) != 0U) && (((isr & BIT(4))) != 0U))
	{
		// Receive Buffer Not Empty
		if (spi->xfer->rx_cnt < spi->xfer->num)
		{
			if (spi->xfer->rx_buf)
			{
				if (data_width <= (8U - 1U))
				{
					// 8-bit data frame
					data_8bit = *(volatile uint8_t *)(&spi->reg->DR);
					*(spi->xfer->rx_buf++) = data_8bit;
				}
				else if(data_width <= (16U - 1U))
				{
					// 16-bit data frame
					data_16bit = *(volatile uint16_t *)(&spi->reg->DR);
					*(spi->xfer->rx_buf++) = (uint8_t) data_16bit;
					*(spi->xfer->rx_buf++) = (uint8_t)(data_16bit >> 8U);
				}
				else
				{
					// 32-bit data frame
					data_32bit = *(volatile uint32_t *)(&spi->reg->DR);
					*(spi->xfer->rx_buf++) = (uint8_t) data_32bit;
					*(spi->xfer->rx_buf++) = (uint8_t)(data_32bit >> 8U);
					*(spi->xfer->rx_buf++) = (uint8_t)(data_32bit >> 16U);
					*(spi->xfer->rx_buf++) = (uint8_t)(data_32bit >> 24U);
				}
			}
			spi->xfer->rx_cnt++;
			if (spi->xfer->rx_cnt == spi->xfer->num)
			{
				// Disable RX Buffer Not Empty Interrupt
				spi->reg->IMR &= ~RXFIM;

				// Clear busy flag
				spi->info->status.busy = 0U;

				// Transfer completed
				event |= ARM_SPI_EVENT_TRANSFER_COMPLETE;
			}
		}
		else
		{
			// Unexpected transfer, data lost
			event |= ARM_SPI_EVENT_DATA_LOST;
		}
	}
	if ((((sr & BIT(2))) != 0U) && (((isr & BIT(0))) != 0U))
	{
		if (spi->xfer->tx_cnt < spi->xfer->num)
		{
			if (data_width <= (8U - 1U))
			{
				if (spi->xfer->tx_buf)
				{
					data_8bit = *(spi->xfer->tx_buf++);
					*(volatile uint8_t *)(&spi->reg->DR) = data_8bit;
				}
				else
				{
					*(volatile uint8_t *)(&spi->reg->DR) =0x0;//dummy data
				}
			}
			else if(data_width <= (16U - 1U))
			{
				if (spi->xfer->tx_buf != NULL)
				{
					data_16bit  = *(spi->xfer->tx_buf++);
					data_16bit |= *(spi->xfer->tx_buf++) << 8U;
					*(volatile uint16_t *)(&spi->reg->DR) = data_16bit;
				}
				else
				{
					*(volatile uint16_t *)(&spi->reg->DR) =0x0;//dummy data
				}
			}
			else
			{
        if(spi->xfer->tx_buf != NULL)
        {
          data_32bit = *(spi->xfer->tx_buf++);
          data_32bit |= *(spi->xfer->tx_buf++) << 8U;
          data_32bit |= *(spi->xfer->tx_buf++) << 16U;
          data_32bit |= *(spi->xfer->tx_buf++) << 24U;
          *(volatile uint32_t *)(&spi->reg->DR) = data_32bit;
        }
        else
        {
           *(volatile uint32_t *)(&spi->reg->DR) =0x0;//dummy data
        }
			}
			do{
				sr  = spi->reg->SR;
			}while(sr&BIT(0));

			for(i=0;i<1;i++);
			spi->xfer->tx_cnt++;
			if (spi->xfer->tx_cnt == spi->xfer->num)
			{
				// All data sent, disable TX Buffer Empty Interrupt
				spi->reg->IMR &= ~TXEIM;
			}
		}
		else
		{
			// Unexpected transfer, data lost
			event |= ARM_SPI_EVENT_DATA_LOST;
		}
	}

	// Send event
	if ((event != 0U) && ((spi->info->cb_event != NULL)))
	{
		spi->info->cb_event(event);
	}
}

// SSI_MASTER
#ifdef SSI_MASTER
static int32_t        SSI_MASTER_Initialize          (ARM_SPI_SignalEvent_t pSignalEvent)                { return SPI_Initialize (pSignalEvent, &SSI_MASTER_Resources); }
static int32_t        SSI_MASTER_Uninitialize        (void)                                              { return SPI_Uninitialize (&SSI_MASTER_Resources); }
static int32_t        SSI_MASTER_PowerControl        (ARM_POWER_STATE state)                             { return SPI_PowerControl (state, &SSI_MASTER_Resources); }
static int32_t        SSI_MASTER_Send                (const void *data, uint32_t num)                    { return SPI_Send (data, num, &SSI_MASTER_Resources); }
static int32_t        SSI_MASTER_Receive             (void *data, uint32_t num)                          { return SPI_Receive (data, num, &SSI_MASTER_Resources); }
static int32_t        SSI_MASTER_Transfer            (const void *data_out, void *data_in, uint32_t num) { return SPI_Transfer (data_out, data_in, num, &SSI_MASTER_Resources); }
static uint32_t       SSI_MASTER_GetDataCount        (void)                                              { return SPI_GetDataCount (&SSI_MASTER_Resources); }
static int32_t        SSI_MASTER_Control             (uint32_t control, uint32_t arg)                    { return SPI_Control (control, arg, &SSI_MASTER_Resources); }
static ARM_SPI_STATUS SSI_MASTER_GetStatus           (void)                                              { return SPI_GetStatus (&SSI_MASTER_Resources); }
void          		  SSI_MASTER_IRQHandler          (void)                                              {        SPI_IRQHandler (&SSI_MASTER_Resources); }
#if (SSI_MASTER_TX_DMA_Instance == 1)
void 				  SSI_MASTER_UDMA_Tx_Event		 (uint32_t event, uint8_t dmaCh)					 {		  SPI_UDMA_Tx_Event (event,dmaCh, &SSI_MASTER_Resources);	}
#endif

#if (SSI_MASTER_RX_DMA_Instance == 1)
void 				  SSI_MASTER_UDMA_Rx_Event		 (uint32_t event, uint8_t dmaCh)					 {		  SPI_UDMA_Rx_Event (event,dmaCh, &SSI_MASTER_Resources);	}
#endif


// End SPI Interface

ARM_DRIVER_SPI Driver_SSI_MASTER = {
		SPI_GetVersion,
		SPI_GetCapabilities,
		SSI_MASTER_Initialize,
		SSI_MASTER_Uninitialize,
		SSI_MASTER_PowerControl,
		SSI_MASTER_Send,
		SSI_MASTER_Receive,
		SSI_MASTER_Transfer,
		SSI_MASTER_GetDataCount,
		SSI_MASTER_Control,
		SSI_MASTER_GetStatus
};
#endif

// SSI_SLAVE
#ifdef SSI_SLAVE
static int32_t        SSI_SLAVE_Initialize          (ARM_SPI_SignalEvent_t pSignalEvent)                { return SPI_Initialize (pSignalEvent, &SSI_SLAVE_Resources); }
static int32_t        SSI_SLAVE_Uninitialize        (void)                                              { return SPI_Uninitialize (&SSI_SLAVE_Resources); }
static int32_t        SSI_SLAVE_PowerControl        (ARM_POWER_STATE state)                             { return SPI_PowerControl (state, &SSI_SLAVE_Resources); }
static int32_t        SSI_SLAVE_Send                (const void *data, uint32_t num)                    { return SPI_Send (data, num, &SSI_SLAVE_Resources); }
static int32_t        SSI_SLAVE_Receive             (void *data, uint32_t num)                          { return SPI_Receive (data, num, &SSI_SLAVE_Resources); }
static int32_t        SSI_SLAVE_Transfer            (const void *data_out, void *data_in, uint32_t num) { return SPI_Transfer (data_out, data_in, num, &SSI_SLAVE_Resources); }
static uint32_t       SSI_SLAVE_GetDataCount        (void)                                              { return SPI_GetDataCount (&SSI_SLAVE_Resources); }
static int32_t        SSI_SLAVE_Control             (uint32_t control, uint32_t arg)                    { return SPI_Control (control, arg, &SSI_SLAVE_Resources); }
static ARM_SPI_STATUS SSI_SLAVE_GetStatus           (void)                                              { return SPI_GetStatus (&SSI_SLAVE_Resources); }
void           		  SSI_SLAVE_IRQHandler          (void)                                              {        SPI_IRQHandler (&SSI_SLAVE_Resources); }
#if (SSI_SLAVE_TX_DMA_Instance == 1)
void 				  SSI_SLAVE_UDMA_Tx_Event		(uint32_t event, uint8_t dmaCh)					 	{		 SPI_UDMA_Tx_Event (event,dmaCh, &SSI_SLAVE_Resources);	}
#endif

#if (SSI_SLAVE_RX_DMA_Instance == 1)
void 				  SSI_SLAVE_UDMA_Rx_Event		(uint32_t event, uint8_t dmaCh)					 	{		 SPI_UDMA_Rx_Event (event,dmaCh, &SSI_SLAVE_Resources);	}
#endif


// End SPI Interface

ARM_DRIVER_SPI Driver_SSI_SLAVE = {
		SPI_GetVersion,
		SPI_GetCapabilities,
		SSI_SLAVE_Initialize,
		SSI_SLAVE_Uninitialize,
		SSI_SLAVE_PowerControl,
		SSI_SLAVE_Send,
		SSI_SLAVE_Receive,
		SSI_SLAVE_Transfer,
		SSI_SLAVE_GetDataCount,
		SSI_SLAVE_Control,
		SSI_SLAVE_GetStatus
};
#endif

// SSI_ULP_MASTER
#ifdef SSI_ULP_MASTER
static int32_t        SSI_ULP_MASTER_Initialize          (ARM_SPI_SignalEvent_t pSignalEvent)                { return SPI_Initialize (pSignalEvent, &SSI_ULP_MASTER_Resources); }
static int32_t        SSI_ULP_MASTER_Uninitialize        (void)                                              { return SPI_Uninitialize (&SSI_ULP_MASTER_Resources); }
static int32_t        SSI_ULP_MASTER_PowerControl        (ARM_POWER_STATE state)                             { return SPI_PowerControl (state, &SSI_ULP_MASTER_Resources); }
static int32_t        SSI_ULP_MASTER_Send                (const void *data, uint32_t num)                    { return SPI_Send (data, num, &SSI_ULP_MASTER_Resources); }
static int32_t        SSI_ULP_MASTER_Receive             (void *data, uint32_t num)                          { return SPI_Receive (data, num, &SSI_ULP_MASTER_Resources); }
static int32_t        SSI_ULP_MASTER_Transfer            (const void *data_out, void *data_in, uint32_t num) { return SPI_Transfer (data_out, data_in, num, &SSI_ULP_MASTER_Resources); }
static uint32_t       SSI_ULP_MASTER_GetDataCount        (void)                                              { return SPI_GetDataCount (&SSI_ULP_MASTER_Resources); }
static int32_t        SSI_ULP_MASTER_Control             (uint32_t control, uint32_t arg)                    { return SPI_Control (control, arg, &SSI_ULP_MASTER_Resources); }
static ARM_SPI_STATUS SSI_ULP_MASTER_GetStatus           (void)                                              { return SPI_GetStatus (&SSI_ULP_MASTER_Resources); }
void          		  SSI_ULP_MASTER_IRQHandler          (void)                                              {        SPI_IRQHandler (&SSI_ULP_MASTER_Resources); }
#if (SSI_ULP_MASTER_TX_DMA_Instance == 1)
void 				  SSI_ULP_MASTER_UDMA_Tx_Event		 (uint32_t event, uint8_t dmaCh)					 {		  SPI_UDMA_Tx_Event (event,dmaCh, &SSI_ULP_MASTER_Resources);	}
#endif

#if (SSI_ULP_MASTER_RX_DMA_Instance == 1)
void 				  SSI_ULP_MASTER_UDMA_Rx_Event		 (uint32_t event, uint8_t dmaCh)					 {		  SPI_UDMA_Rx_Event (event,dmaCh, &SSI_ULP_MASTER_Resources);	}
#endif

// End SPI Interface

ARM_DRIVER_SPI Driver_SSI_ULP_MASTER = {
		SPI_GetVersion,
		SPI_GetCapabilities,
		SSI_ULP_MASTER_Initialize,
		SSI_ULP_MASTER_Uninitialize,
		SSI_ULP_MASTER_PowerControl,
		SSI_ULP_MASTER_Send,
		SSI_ULP_MASTER_Receive,
		SSI_ULP_MASTER_Transfer,
		SSI_ULP_MASTER_GetDataCount,
		SSI_ULP_MASTER_Control,
		SSI_ULP_MASTER_GetStatus
};
#endif
