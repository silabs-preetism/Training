/*******************************************************************************
* @file  rsi_sio.h
* @brief 
*******************************************************************************
* # License
* <b>Copyright 2020 Silicon Laboratories Inc. www.silabs.com</b>
*******************************************************************************
*
* The licensor of this software is Silicon Laboratories Inc. Your use of this
* software is governed by the terms of Silicon Labs Master Software License
* Agreement (MSLA) available at
* www.silabs.com/about-us/legal/master-software-license-agreement. This
* software is distributed to you in Source Code format and is governed by the
* sections of the MSLA applicable to Source Code.
*
******************************************************************************/

/**
 * Includes
 */
#ifndef __RSI_SIO_H__
#define __RSI_SIO_H__

#ifdef __cplusplus
extern "C" {
#endif

#include "base_types.h"
#include "rsi_ccp_common.h"

/**
 * \cond HIDDEN_SYMBOLS 
 */
/*Enum for SIO channels */
typedef enum en_sio_channels {
  SioCh0,
  SioCh1,
  SioCh2,
  SioCh3,
  SioCh4,
  SioCh5,
  SioCh6,
  SioCh7,
} en_sio_channels_t;

/*SIO-I2S transfer states*/
typedef enum en_sio_i2s_states {
  SioI2sIdle,
  SioI2sBusy,
  SioI2sTrasmitDone,

} en_sio_i2s_states_t;

/*SIO-I2S transfer states*/
typedef enum en_sio_uart_states {
  SioUartIdle,
  SioUartsBusy,
  SioUartTxComeplete,
  SioUartRxComeplete,
} en_sio_uart_states_t;

/*SIO-SPI transfer states */
typedef enum en_sio_spi_states { SioSpiIdle, SioSpiBusy, SioSpiTrasmitDone } en_sio_spi_states_t;

/*SIO-I2S events for call back functions */
typedef enum en_sio_i2s_events { SioI2sTxDone, SioI2sRxDone } en_sio_i2s_events_t;

/*SIO-UART events for call back functions*/
typedef enum en_sio_Uart_events {
  SioUartTxDone,
  SioUartRxDone,
  SioUartParityError,
  SioUartFramingError,
  SioUartRecvChar
} en_sio_Uart_events_t;

/*SIO-SPI events for call back functions*/
typedef enum en_sio_spi_events { SioSpiTxDone, SioSpiRxDone } en_sio_spi_events_t;

/*SIO-SPI modes configurations */
typedef enum en_sio_spi_mode {
  SioSpiMode0,
  SioSpiMode3,
} en_sio_spi_mode_t;

/*SIO-I2S Audio bit depth */
typedef enum en_i2s_audio_depth { i2sAudioDepth16Bit, i2sAudioDepth32Bit } en_i2s_audio_depth_t;

/*SIO-I2C configuration structure*/
typedef struct stc_sio_i2c_config {
  uint8_t u8SioI2cSda;
  uint8_t u8SioI2cScl;
  uint8_t u8SioI2cOen;
  uint32_t u32SampleRate;
} stc_sio_i2c_config_t;

/*call back function pointers */
typedef void (*sio_i2s_func_ptr_t)(en_sio_i2s_events_t enI2sSio);
typedef void (*sio_Spi_func_ptr_t)(en_sio_spi_events_t enSpiSio);
typedef void (*sio_Uart_func_ptr_t)(en_sio_Uart_events_t enUartSio, uint16_t u16RxChar);

/*SIO-I2S Transfer structure*/
typedef struct stc_sio_i2s_xfer {
  void *txBuff; /**< TX buffer pointer; Must be a uint16_t pointer when transfer
										     size is 16 bits and uint8_t pointer when transfer size is 8 bits (can be NULL only when *txCount is 0) */
  void *rxBuff; /**< RX buffer pointer; Must be uint16_t pointer when transfer size is 16 bits or
										     must be uint8_t pointer when transfer is 8-bits (can be NULL only when *txCount is 0) */
  int32_t
    txCount; /**< Pointer to an int32_t memory (never initialize to NULL) that has the Size of the txBuff in items (not bytes), not modified by driver */
  int32_t
    rxCount; /**< Number of items (not bytes) to send in rxBuff buffer (Never initialize to NULL), not modified by driver */
  uint8_t u8BitLen; /**< number of bits to transfer 8 , 16 , 32 */
  int32_t
    txDoneCount; /**< Total items (not bytes) transmitted (initialize to 0), modified by driver [In case of underflow txDoneCount will be greater than *txCount] */
  int32_t
    rxDoneCount; /**< Total items (not bytes) received (initialize to 0), modified by driver [In case of over flow rxDoneCount will be greater than *rxCount] */
  sio_i2s_func_ptr_t pfnCb;
  uint8_t u8Status;
} stc_sio_i2s_xfer_t;

/*SIO-I2S internal global structure member */
typedef struct stc_sio_i2s {
  /*I2S*/
  uint8_t u8I2sDataOutCh; /**< u8I2sDataOutCh is used to hold the user configured I2S channel for internal API usage */
  uint8_t u8I2sDataInCh;  /**< u8I2sDataInCh is used to hold the user configured I2S channel for internal API usage */
  uint8_t u8I2sClkCh;     /**< u8I2sClkCh is used to hold the user configured I2S channel for internal API usage */
  uint8_t u8I2sWsCh;      /**< u8I2sWsCh is used to hold the user configured I2S channel for internal API usage */
  uint8_t u8I2sValid;
  uint8_t u8I2sWsWidth;
  stc_sio_i2s_xfer_t *pstcI2sXfer;
} stc_sio_i2s_t;

/*SIO-I2C Internal global structure member*/
typedef struct stc_sio_i2c {
  /*I2c*/
  uint8_t u8I2cSdaCh; /**< u8I2cSdaCh is used to hold the user configured i2c channel for internal API usage */
  uint8_t u8I2cClkCh; /**< u8I2cClkCh is used to hold the user configured i2c channel for internal API usage */
} stc_sio_i2c_t;

/*SIO-SPI configuration structure*/
typedef struct stc_sio_spi_cfg {

  uint8_t u8SpiMosiCh;     /**< u8SpiMosiCh is used to use with SIO*/
  uint8_t u8SpiMisoCh;     /**< u8SpiMisoCh is used to use with SIO*/
  uint8_t u8SpiClkCh;      /**< u8SpiClkCh is used to use with SIO*/
  uint8_t u8SpiCsCh;       /**< u8SpiCCh is used to use with SIO*/
  uint32_t u32SpiClockFrq; /**< u32SpiClockFrq is used to use with SIO */
  uint8_t u8BitOrder;      /**< u8BitOrder is used to use with SIO */
  uint8_t u8Mode;          /**< Mode 0 , and mode 1 are supported  */
  uint8_t u8BitLen;
} stc_sio_spi_cfg_t;

/*SIO-SPI transfer structure*/
typedef struct stc_sio_spi_xfer {

  void *txBuff; /**< TX buffer pointer; Must be a uint16_t pointer when transfer
									     size is 16 bits and uint8_t pointer when transfer size is 8 bits (can be NULL only when *txCount is 0) */
  void *rxBuff; /**< RX buffer pointer; Must be uint16_t pointer when transfer size is 16 bits or
									     must be uint8_t pointer when transfer is 8-bits (can be NULL only when *txCount is 0) */
  int32_t
    txCount; /**< Pointer to an int32_t memory (never initialize to NULL) that has the Size of the txBuff in items (not bytes), not modified by driver */
  int32_t
    rxCount; /**< Number of items (not bytes) to send in rxBuff buffer (Never initialize to NULL), not modified by driver */
  uint8_t sselNum;  /**< CS number assigned to this transfer */
  uint8_t u8BitLen; /**< number of bits to transfer 8 , 16 , 32 */
  int32_t
    txDoneCount; /**< Total items (not bytes) transmitted (initialize to 0), modified by driver [In case of underflow txDoneCount will be greater than *txCount] */
  int32_t
    rxDoneCount; /**< Total items (not bytes) received (initialize to 0), modified by driver [In case of over flow rxDoneCount will be greater than *rxCount] */
  uint8_t u8Status;
  sio_Spi_func_ptr_t pfnCb;
} stc_sio_spi_xfer_t;

/*SIO-SPI Internal global structure member*/
typedef struct stc_sio_spi {
  /*SPI*/
  uint8_t u8SpiMosiCh; /**< u8SpiMosiCh is used to hold the user configured SPI channel for internal API usage */
  uint8_t u8SpiMisoCh; /**< u8SpiMisoCh is used to hold the user configured SPI channel for internal API usage */
  uint8_t u8SpiClkCh;  /**< u8SpiClkCh is used to hold the user configured SPI channel for internal API usage */
  uint8_t u8SpiCCh;    /**< u8SpiCCh is used to hold the user configured SPI channel for internal API usage */
  uint8_t u8SpiValid;
  stc_sio_spi_xfer_t *pstscSpiXfer;
} stc_sio_spi_t;

/*SIO-I2S Configuration structure*/
typedef struct stc_sio_i2s_config {
  uint8_t u8SioI2sDataOutChannel;
  uint8_t u8SioI2sDataInChannel;
  uint8_t u8SioI2sWsChannel;
  uint8_t u8SioI2sClkChannel;
  uint32_t u32SampleRate;
  uint8_t u8BitLen;
  uint8_t u8BitOrder;
  uint8_t u8I2sWsWidth;
} stc_sio_i2s_config_t;

/*SIO-UART Configuration structure */
typedef struct stc_sio_uart_config {
  uint8_t u8SioUartTxChannel;
  uint8_t u8SioUartRxChannel;
  uint8_t u8StopBits;
  uint8_t u8Parity;
  uint32_t u32BaudRate;
  uint8_t u8Bitlen;
  sio_Uart_func_ptr_t pfn;
} stc_sio_uart_config_t;

/*SIO-UART internal global structure member */
typedef struct stc_sio_uart {
  /*UART*/
  uint8_t u8UartTxCh; /**< u8UartTxCh is used to hold the user configured UART channel for internal API usage */
  uint8_t u8UartRxCh; /**< u8UartRxCh is used to hold the user configured UART channel for internal API usage */
  uint16_t u16UartTxDone;
  uint16_t u16UartRxDone;
  uint16_t u16UartTxCnt;
  uint16_t u16UartRxCnt;
  void *pvUartTx;
  void *pvUartRx;
  uint8_t u8UartValid;
  uint8_t u8ParityEn;
  uint32_t u32SioUartRxData;
  en_sio_uart_states_t volatile u8Status;
  en_sio_uart_states_t volatile enRxStatus;
  stc_sio_uart_config_t *pstcSioUart;
} stc_sio_uart_t;

/*Global structure to hold the channels for all protocols*/
typedef struct stc_sio_cb {
  /*I2C SIO channel information structure */
  stc_sio_i2c_t i2c_sio;
  /*UART SIO channel information structure */
  stc_sio_uart_t uart_sio;
  /*SPI SIO channel information structure */
  stc_sio_spi_t spi_sio;
  /*I2S SIO channel information structure */
  stc_sio_i2s_t i2s_sio;
} stc_sio_cb_t;

/*Global structure instance */
extern stc_sio_cb_t gstcSioCb;

/**
 * \endcond
 */
/*End of file not truncated*/

/**
 * \ingroup   RSI_SPECIFIC_DRIVERS
 */

/**
 * \defgroup RSI_SIO_DRIVERS RSI:RS1xxxx SIO 
 *  @{
 *
 */
/* SIO_SIO apis function prototype */
static void RSI_SIO_ClockEnable(void);
void RSI_SIO_I2cGenerateStop(volatile SIO_Type *pstcSio);
error_t RSI_SIO_Init(volatile SIO_Type *pstcSio);
error_t RSI_SIO_InitSpi(volatile SIO_Type *pstcSio, stc_sio_spi_cfg_t *pstcSpiConfig);
void RSI_SIO_SpiCsAssert(volatile SIO_Type *pstcSio, uint8_t u8CsNo);
void RSI_SIO_SpiCsDeAssert(volatile SIO_Type *pstcSio, uint8_t u8CsNo);
error_t RSI_SIO_SpiTrasnfer(volatile SIO_Type *pstcSio, stc_sio_spi_xfer_t *pstcSpiXfer);

/* SIO_UART apis function prototype */
uint8_t RSI_SIO_UartInit(SIO_Type *pstcSio, stc_sio_uart_config_t *pstcConfig);
uint8_t RSI_SIO_UartGetParity(uint16_t u16Data);
uint16_t RSI_SIO_UartFramePrepare(uint8_t u8StopBits, uint16_t u16Data, uint8_t u8ParitySet, uint8_t u8DataLen);
uint16_t RSI_SIO_UARTSend(SIO_Type *pstcSio, const void *u16ptr, uint16_t u16Len);
uint16_t RSI_SIO_UartFrameExtract(uint8_t u8StopBits,
                                  uint16_t u16UartDataFrame,
                                  uint8_t u8ParitySet,
                                  uint8_t u8DataLen);
int RSI_SIO_UARTSendBlocking(SIO_Type *pstcSio, const void *data, int numBytes);
int RSI_SIO_UARTRead(volatile SIO_Type *pstcSio, void *data, int numBytes);
int RSI_SIO_UARTReadBlocking(volatile SIO_Type *pstcSio, void *data, int numBytes);

/* SIO_I2C apis function prototype */
static uint32_t RSI_SIO_I2cPrepareWrite(uint8_t u8InDat);
static uint32_t RSI_SIO_I2cPrepareRead(uint32_t u32Indata);
error_t RSI_SIO_I2cWrite(volatile SIO_Type *pstcSio,
                         stc_sio_i2c_config_t *pstcConfig,
                         uint8_t u8SlaveAddr,
                         uint8_t *u8Data,
                         uint16_t u16Len);
error_t RSI_SIO_I2cRead(volatile SIO_Type *pstcSio,
                        stc_sio_i2c_config_t *pstcConfig,
                        uint8_t u8SlaveAddr,
                        uint8_t *u8Data,
                        uint16_t u16Len);
error_t RSI_SIO_I2cTransfer(volatile SIO_Type *pstcSio,
                            stc_sio_i2c_config_t *pstcConfig,
                            uint8_t u8SlaveAddr,
                            uint8_t *u8PtrTxDat,
                            uint16_t u16TxLen,
                            uint8_t *u8PtrRxDat,
                            uint16_t u16RxLen);
void RSI_SIO_I2cGenerateStart(volatile SIO_Type *pstcSio);
void RSI_SIO_I2cGenerateStop(volatile SIO_Type *pstcSio);
#ifdef __cplusplus
}
#endif

#endif // __RSI_SIO_H__

/* @} end of RSI_SIO_DRIVERS */
