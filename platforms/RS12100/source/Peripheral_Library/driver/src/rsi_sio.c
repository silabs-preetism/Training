/*******************************************************************************
* @file  rsi_sio.c
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
#include "rsi_chip.h"
#include "rsi_ccp_common.h"

/*Global structure to hold the SIO channel configuration */
stc_sio_cb_t gstcSioCb;

volatile uint32_t u32Fclk = 0;

/**
 * \ingroup   RSI_SPECIFIC_DRIVERS
 */

/**
 * \defgroup RSI_SIO_DRIVERS RSI:RS1xxxx SIO
 *  @{
 *
 */

/**
 * @fn           void RSI_SIO_ClockEnable(void)
 * @brief        This API is used to enable/set the clock to the SIO module.
 * @return       None
 */
static void RSI_SIO_ClockEnable(void)
{
  RSI_CLK_PeripheralClkEnable3(M4CLK, (SGPIO_PCLK_ENABLE));
  return;
}

/**
 * @fn           Error_t RSI_SIO_Init(volatile SIO_Type *pstcSio)
 * @brief        This API is used to initialize the SIO module
 * @param[in]    pstcSio   : pointer to the register instance of SIO module
 * @return      return \ref RSI_OK on success and on failure return error code.
 */
error_t RSI_SIO_Init(volatile SIO_Type *pstcSio)
{
  /*SIO_0*/
  if (RTE_SIO_0_PIN > 63) {
    RSI_EGPIO_UlpPadReceiverEnable((uint8_t)(RTE_SIO_0_PIN - 64));
    RSI_EGPIO_SetPinMux(EGPIO1, 0, (uint8_t)(RTE_SIO_0_PIN - 64), 6);
  } else {
    RSI_EGPIO_PadReceiverEnable(RTE_SIO_0_PIN);
  }
  RSI_EGPIO_PadSelectionEnable(RTE_SIO_0_PAD);
  RSI_EGPIO_SetPinMux(EGPIO, 0, RTE_SIO_0_PIN, RTE_SIO_0_MUX);

  /*SIO_1*/
  if (RTE_SIO_1_PIN > 63) {
    RSI_EGPIO_UlpPadReceiverEnable((uint8_t)(RTE_SIO_1_PIN - 64));
    RSI_EGPIO_SetPinMux(EGPIO1, 0, (uint8_t)(RTE_SIO_1_PIN - 64), 6);
  } else {
    RSI_EGPIO_PadReceiverEnable(RTE_SIO_1_PIN);
  }
  RSI_EGPIO_PadSelectionEnable(RTE_SIO_1_PAD);
  RSI_EGPIO_SetPinMux(EGPIO, 0, RTE_SIO_1_PIN, RTE_SIO_1_MUX);

  /*SIO_2*/
  if (RTE_SIO_2_PIN > 63) {
    RSI_EGPIO_UlpPadReceiverEnable((uint8_t)(RTE_SIO_2_PIN - 64));
    RSI_EGPIO_SetPinMux(EGPIO1, 0, (uint8_t)(RTE_SIO_2_PIN - 64), 6);
  } else {
    RSI_EGPIO_PadReceiverEnable(RTE_SIO_2_PIN);
  }
  RSI_EGPIO_PadSelectionEnable(RTE_SIO_2_PAD);
  RSI_EGPIO_SetPinMux(EGPIO, 0, RTE_SIO_2_PIN, RTE_SIO_2_MUX);

  /*SIO_3*/
  if (RTE_SIO_3_PIN > 63) {
    RSI_EGPIO_UlpPadReceiverEnable((uint8_t)(RTE_SIO_3_PIN - 64));
    RSI_EGPIO_SetPinMux(EGPIO1, 0, (uint8_t)(RTE_SIO_3_PIN - 64), 6);
  } else {
    RSI_EGPIO_PadReceiverEnable(RTE_SIO_3_PIN);
  }
  RSI_EGPIO_PadSelectionEnable(RTE_SIO_3_PAD);
  RSI_EGPIO_SetPinMux(EGPIO, 0, RTE_SIO_3_PIN, RTE_SIO_3_MUX);

  /*SIO CLock enable */
  RSI_SIO_ClockEnable();
  return RSI_OK;
}

/**
 * @}
 */

void IRQ037_Handler(void) // with interrupt
{
  SIO_Type *pstcSio               = (SIO_Type *)SIO;
  volatile uint32_t u32SwapStsReg = 0;
  volatile uint32_t u32Data = 0, u32EnableReg = 0, u32PauseReg = 0;
  volatile uint16_t u16UartDataFrame = 0;

  /*Read swap interrupt status register */
  u32SwapStsReg = pstcSio->SIO_SWAP_INTR_STATUS_REG;

  /*I2S Handle */
  if (gstcSioCb.i2s_sio.u8I2sValid) {
    /*SIO-I2S DIN Handle */
    if (u32SwapStsReg & (1 << gstcSioCb.i2s_sio.u8I2sDataInCh)) {
      pstcSio->SIO_SWAP_INTR_STATUS_REG = (1 << gstcSioCb.i2s_sio.u8I2sDataInCh);
      if (gstcSioCb.i2s_sio.pstcI2sXfer->rxDoneCount < gstcSioCb.i2s_sio.pstcI2sXfer->rxCount) {
        /*Fill the buffer */
        ((uint32_t *)gstcSioCb.i2s_sio.pstcI2sXfer->rxBuff)[gstcSioCb.i2s_sio.pstcI2sXfer->rxDoneCount] =
          pstcSio->SIO_BUFFER_REG[gstcSioCb.i2s_sio.u8I2sDataInCh];

        /*Check the buffer lengths */

        /*Increment the buffer index for RX */
        gstcSioCb.i2s_sio.pstcI2sXfer->rxDoneCount++;
      }
      if (gstcSioCb.i2s_sio.pstcI2sXfer->rxDoneCount == gstcSioCb.i2s_sio.pstcI2sXfer->rxCount) {
        //FIXME :
        pstcSio->SIO_SWAP_INTR_EN_CLEAR_REG = (1 << gstcSioCb.i2s_sio.u8I2sDataInCh);
        /*invoke the RX done call back to application*/
        gstcSioCb.i2s_sio.pstcI2sXfer->pfnCb(SioI2sRxDone);
      }
    }
    /*SIO-I2S DOUT handle */
    if (u32SwapStsReg & (1 << gstcSioCb.i2s_sio.u8I2sDataOutCh)) {
      pstcSio->SIO_SWAP_INTR_STATUS_REG = (1 << gstcSioCb.i2s_sio.u8I2sDataOutCh);

      if (gstcSioCb.i2s_sio.pstcI2sXfer->txDoneCount < gstcSioCb.i2s_sio.pstcI2sXfer->txCount) {
        pstcSio->SIO_BUFFER_REG[gstcSioCb.i2s_sio.u8I2sDataOutCh] =
          ((uint32_t *)gstcSioCb.i2s_sio.pstcI2sXfer->txBuff)[gstcSioCb.i2s_sio.pstcI2sXfer->txDoneCount];
        if (!gstcSioCb.i2s_sio.u8I2sWsWidth) {
          pstcSio->SIO_BUFFER_REG[gstcSioCb.i2s_sio.u8I2sWsCh] = 0xFFFF0000;
        } else {
          u32Fclk                                              = ~u32Fclk;
          pstcSio->SIO_BUFFER_REG[gstcSioCb.i2s_sio.u8I2sWsCh] = u32Fclk;
        }
        gstcSioCb.i2s_sio.pstcI2sXfer->txDoneCount++;
      }
      if (gstcSioCb.i2s_sio.pstcI2sXfer->txDoneCount == gstcSioCb.i2s_sio.pstcI2sXfer->txCount) {
        /*change SIO i2s state to TX done */
        gstcSioCb.i2s_sio.pstcI2sXfer->u8Status = SioI2sTrasmitDone;
        /*Update the SIO-TX is invalid */
        //FIXME : SIO is not halting in IDLE state due that interrupt is continuously triggering
        // due to that some last send data is sent on SIO-I2S line that has to be stopped
        /*Make DOUT line idle to avoid some junk sound in speaker*/
        pstcSio->SIO_BUFFER_REG[gstcSioCb.i2s_sio.u8I2sDataOutCh] = 0x00000000;
        /*Make SIO-I2S valid to invalid , this is triggered again when I2S Transfer is called*/
        pstcSio->SIO_SWAP_INTR_EN_CLEAR_REG = (1 << gstcSioCb.i2s_sio.u8I2sDataOutCh);
        //gstcSioCb.i2s_sio.u8I2sValid = 0 ;
        /*Trigger TX done call back*/
        gstcSioCb.i2s_sio.pstcI2sXfer->pfnCb(SioI2sTxDone);
      }
    }
  }
  /*SIO-SPI Handle */
  if (gstcSioCb.spi_sio.u8SpiValid) {
    /*SIO-SPI MOSI handling */
    if (u32SwapStsReg & (1 << gstcSioCb.spi_sio.u8SpiMosiCh)) {
      /*Clear the SWAP interrupt */
      pstcSio->SIO_SWAP_INTR_STATUS_REG = (1 << gstcSioCb.spi_sio.u8SpiMosiCh);
      if (gstcSioCb.spi_sio.pstscSpiXfer->txDoneCount < gstcSioCb.spi_sio.pstscSpiXfer->txCount) {
        /*As per bit configuration typecast the pointer */
        if (gstcSioCb.spi_sio.pstscSpiXfer->u8BitLen == 8) {
          u32Data = ((uint8_t *)gstcSioCb.spi_sio.pstscSpiXfer->txBuff)[gstcSioCb.spi_sio.pstscSpiXfer->txDoneCount];
        } else if (gstcSioCb.spi_sio.pstscSpiXfer->u8BitLen == 16) {
          u32Data = ((uint16_t *)gstcSioCb.spi_sio.pstscSpiXfer->txBuff)[gstcSioCb.spi_sio.pstscSpiXfer->txDoneCount];
        } else {
          u32Data = ((uint32_t *)gstcSioCb.spi_sio.pstscSpiXfer->txBuff)[gstcSioCb.spi_sio.pstscSpiXfer->txDoneCount];
        }

        /*check for Bit order*/
        if ((pstcSio->SIO_SHIFT_COUNT_PRELOAD_REG_b[gstcSioCb.spi_sio.u8SpiMisoCh].REVERSE_LOAD == 1)
            && ((pstcSio->SIO_SHIFT_COUNT_PRELOAD_REG_b[gstcSioCb.spi_sio.u8SpiMosiCh].REVERSE_LOAD == 1))) {
          /*MSB First*/
          pstcSio->SIO_BUFFER_REG[gstcSioCb.spi_sio.u8SpiMosiCh] =
            (u32Data << (32 - gstcSioCb.spi_sio.pstscSpiXfer->u8BitLen));
        } else {
          /*LSB First*/
          pstcSio->SIO_BUFFER_REG[gstcSioCb.spi_sio.u8SpiMosiCh] =
            (u32Data >> (32 - gstcSioCb.spi_sio.pstscSpiXfer->u8BitLen));
        }
        /**/
        gstcSioCb.spi_sio.pstscSpiXfer->txDoneCount++;

      } else {
        /*change SIO i2s state to TX done */
        gstcSioCb.spi_sio.pstscSpiXfer->u8Status = SioSpiTrasmitDone;
        /*Trigger TX done call back*/
        pstcSio->SIO_SWAP_INTR_EN_CLEAR_REG =
          ((1 << gstcSioCb.spi_sio.u8SpiMosiCh | (1 << gstcSioCb.spi_sio.u8SpiMisoCh)));
        gstcSioCb.spi_sio.pstscSpiXfer->pfnCb(SioSpiTxDone);
      }

      /*SIO-SPI MISO handling */
      if (u32SwapStsReg & (1 << gstcSioCb.spi_sio.u8SpiMisoCh)) {
        /*Clear the swap interrupt */
        pstcSio->SIO_SWAP_INTR_STATUS_REG = (1 << gstcSioCb.spi_sio.u8SpiMisoCh);
        if (gstcSioCb.spi_sio.pstscSpiXfer->rxDoneCount < gstcSioCb.spi_sio.pstscSpiXfer->rxCount) {
          /*Fill the buffer */
          u32Data = pstcSio->SIO_BUFFER_REG[gstcSioCb.spi_sio.u8SpiMisoCh];

          if (gstcSioCb.spi_sio.pstscSpiXfer->u8BitLen == 8) {
            ((uint8_t *)gstcSioCb.spi_sio.pstscSpiXfer->rxBuff)[gstcSioCb.spi_sio.pstscSpiXfer->rxDoneCount] = u32Data;
          } else if (gstcSioCb.spi_sio.pstscSpiXfer->u8BitLen == 16) {
            ((uint16_t *)gstcSioCb.spi_sio.pstscSpiXfer->rxBuff)[gstcSioCb.spi_sio.pstscSpiXfer->rxDoneCount] = u32Data;
          } else {
            ((uint32_t *)gstcSioCb.spi_sio.pstscSpiXfer->rxBuff)[gstcSioCb.spi_sio.pstscSpiXfer->rxDoneCount] = u32Data;
          }
          /*Check the buffer lengths */

          /*Increment the buffer index for RX */
          gstcSioCb.spi_sio.pstscSpiXfer->rxDoneCount++;
        }
        /*invoke the RX done call back to application*/
        if (gstcSioCb.spi_sio.pstscSpiXfer->rxDoneCount == gstcSioCb.spi_sio.pstscSpiXfer->rxCount) {
          gstcSioCb.spi_sio.pstscSpiXfer->pfnCb(SioSpiRxDone);
        }
      }
      if (gstcSioCb.spi_sio.pstscSpiXfer->u8Status == SioSpiBusy) {
        /*Enable and pause the shift counters */
        pstcSio->SIO_ENABLE_REG_b.SIO_OPERATION_ENABLE |=
          ((1 << gstcSioCb.spi_sio.u8SpiMisoCh) | (1 << gstcSioCb.spi_sio.u8SpiMosiCh)
           | (1 << gstcSioCb.spi_sio.u8SpiClkCh));

        pstcSio->SIO_PAUSE_REG_b.SIO_POSITION_COUNTER_DISABLE |=
          ((1 << gstcSioCb.spi_sio.u8SpiMisoCh) | (1 << gstcSioCb.spi_sio.u8SpiMosiCh)
           | (1 << gstcSioCb.spi_sio.u8SpiClkCh));
      }
    }
  }
  /* UART Handle */
  if (gstcSioCb.uart_sio.u8UartValid) {
    /*SIO RX GPIO Interrupt is triggered */
    if ((pstcSio->SIO_GPIO_INTR_STATUS_REG & (1 << gstcSioCb.uart_sio.u8UartRxCh))) {
      pstcSio->SIO_DATA_POS_COUNT_REG_b[gstcSioCb.uart_sio.u8UartRxCh].RELOAD_VALUE =
        ((gstcSioCb.uart_sio.pstcSioUart->u8Bitlen + gstcSioCb.uart_sio.pstcSioUart->u8StopBits
          + gstcSioCb.uart_sio.u8ParityEn + 1)
         - 1);
      u32EnableReg = pstcSio->SIO_ENABLE_REG_b.SIO_OPERATION_ENABLE;

      /*clear reg */
      u32EnableReg &= ~(1 << gstcSioCb.uart_sio.u8UartRxCh);

      u32EnableReg |= (1 << gstcSioCb.uart_sio.u8UartRxCh);

      pstcSio->SIO_ENABLE_REG_b.SIO_OPERATION_ENABLE = u32EnableReg;
      pstcSio->SIO_GPIO_INTR_EN_CLEAR_REG            = (1 << gstcSioCb.uart_sio.u8UartRxCh);

      /*clear the GPIO status */
      pstcSio->SIO_GPIO_INTR_STATUS_REG = (1 << gstcSioCb.uart_sio.u8UartRxCh);
      /*mask the GPIO reg */
      pstcSio->SIO_GPIO_INTR_MASK_SET_REG = (1 << gstcSioCb.uart_sio.u8UartRxCh);

      u32PauseReg = pstcSio->SIO_PAUSE_REG_b.SIO_POSITION_COUNTER_DISABLE;

      u32PauseReg &= ~(1 << gstcSioCb.uart_sio.u8UartRxCh);

      u32PauseReg |= (1 << gstcSioCb.uart_sio.u8UartRxCh);

      pstcSio->SIO_PAUSE_REG_b.SIO_POSITION_COUNTER_DISABLE = u32PauseReg;
    }
    /*Check RX swap interrupt */
    if ((pstcSio->SIO_SWAP_INTR_STATUS_REG) & (1 << gstcSioCb.uart_sio.u8UartRxCh)) {
      if (gstcSioCb.uart_sio.u16UartRxDone < gstcSioCb.uart_sio.u16UartRxCnt) {
        u32Data = pstcSio->SIO_BUFFER_REG[gstcSioCb.uart_sio.u8UartRxCh];
        /*clear the GPIO mask */
        pstcSio->SIO_GPIO_INTR_MASK_CLEAR_REG = (1 << gstcSioCb.uart_sio.u8UartRxCh);
        pstcSio->SIO_GPIO_INTR_EN_SET_REG     = (1 << gstcSioCb.uart_sio.u8UartRxCh);
        u32Data                               = (u32Data >> (32
                               - (gstcSioCb.uart_sio.pstcSioUart->u8Bitlen + gstcSioCb.uart_sio.pstcSioUart->u8StopBits
                                  + gstcSioCb.uart_sio.u8ParityEn + 1)));

        if (gstcSioCb.uart_sio.pstcSioUart->u8Bitlen == 8) {
          ((uint8_t *)gstcSioCb.uart_sio.pvUartRx)[gstcSioCb.uart_sio.u16UartRxDone] = (u32Data & 0XFF);
        } else if (gstcSioCb.uart_sio.pstcSioUart->u8Bitlen == 9) {
          ((uint16_t *)gstcSioCb.uart_sio.pvUartRx)[gstcSioCb.uart_sio.u16UartRxDone] = (u32Data & 0X1FF);
        }
        /*Check the buffer lengths */

        /*Increment the buffer index for RX */
        gstcSioCb.uart_sio.u16UartRxDone++;

        gstcSioCb.uart_sio.pstcSioUart->pfn(SioUartRecvChar, (u32Data & 0XFF));

        pstcSio->SIO_SWAP_INTR_STATUS_REG = (1 << gstcSioCb.uart_sio.u8UartRxCh);
      }
      if (gstcSioCb.uart_sio.u16UartRxDone == gstcSioCb.uart_sio.u16UartRxCnt) {
        gstcSioCb.uart_sio.enRxStatus = SioUartRxComeplete;
        gstcSioCb.uart_sio.pstcSioUart->pfn(SioUartRxDone, (u32Data & 0XFF));
      }
    }
    /*Check TX swap interrupt */
    if ((u32SwapStsReg) & (1 << gstcSioCb.uart_sio.u8UartTxCh)) {

      if (gstcSioCb.uart_sio.u16UartTxDone < gstcSioCb.uart_sio.u16UartTxCnt) {
        pstcSio->SIO_SWAP_INTR_STATUS_REG = (1 << gstcSioCb.uart_sio.u8UartTxCh);

        /*As per bit configuration typecast the pointer */
        if (gstcSioCb.uart_sio.pstcSioUart->u8Bitlen == 8) {
          u32Data = ((uint8_t *)gstcSioCb.uart_sio.pvUartTx)[gstcSioCb.uart_sio.u16UartTxDone];
        } else if (gstcSioCb.uart_sio.pstcSioUart->u8Bitlen == 9) {
          u32Data = ((uint16_t *)gstcSioCb.uart_sio.pvUartTx)[gstcSioCb.uart_sio.u16UartTxDone];
        }

        /*prepare UART header */
        u16UartDataFrame = RSI_SIO_UartFramePrepare(gstcSioCb.uart_sio.pstcSioUart->u8StopBits,
                                                    u32Data,
                                                    gstcSioCb.uart_sio.pstcSioUart->u8Parity,
                                                    gstcSioCb.uart_sio.pstcSioUart->u8Bitlen);

        gstcSioCb.uart_sio.u16UartTxDone++;

        // actual data
        u32Data = 0xFFFFFE00;
        u32Data |= u16UartDataFrame;
        u32Data = (u32Data << 1);
        u32Data |= 1;

        /*Write TX data to buffer */
        pstcSio->SIO_BUFFER_REG[gstcSioCb.uart_sio.pstcSioUart->u8SioUartTxChannel] = u32Data;
        //FIXME : compilation issue
        pstcSio->SIO_ENABLE_REG |= (1 << gstcSioCb.uart_sio.u8UartTxCh);
        pstcSio->SIO_PAUSE_REG |= (1 << gstcSioCb.uart_sio.u8UartTxCh);
      } else {
        /*Trigger TX done call back*/
        pstcSio->SIO_SWAP_INTR_STATUS_REG   = (1 << gstcSioCb.uart_sio.u8UartTxCh);
        pstcSio->SIO_SWAP_INTR_EN_CLEAR_REG = (1 << gstcSioCb.uart_sio.u8UartTxCh);
        /*Update the status to transmit done*/
        gstcSioCb.uart_sio.u8Status = SioUartTxComeplete;
        /*Transmit done call back */
        gstcSioCb.uart_sio.pstcSioUart->pfn(SioUartTxDone, 0);
      }
    }
  }
  return;
}

/**
 * \ingroup   RSI_SPECIFIC_DRIVERS
 */

/**
 * \defgroup RSI_SIO_DRIVERS RSI:RS1xxxx SIO
 *  @{
 *
 */
/*SIO-SPI */

/**
 * @fn           error_t RSI_SIO_InitSpi(volatile SIO_Type *pstcSio , stc_sio_spi_cfg_t  *pstcSpiConfig)
 * @brief        This API is used to configure the SPI in SIO.
 * @param[in]    pstcSio       : pointer to the register instance of SIO module
 * @param[in]    pstcSpiConfig : pointer to SIO spi configuration structure
 * @return       return \ref RSI_OK on success and on failure return error code.
 */
error_t RSI_SIO_InitSpi(volatile SIO_Type *pstcSio, stc_sio_spi_cfg_t *pstcSpiConfig)
{
  uint32_t u32RelaodVal = 0;

  /* Parameter check for valid parameters */
  if ((NULL == pstcSio) || (NULL == pstcSpiConfig)) {
    return (INVALID_PARAMETERS);
  }

  /*Update the global SIO channel CB*/
  gstcSioCb.spi_sio.u8SpiCCh    = pstcSpiConfig->u8SpiCsCh;
  gstcSioCb.spi_sio.u8SpiMisoCh = pstcSpiConfig->u8SpiMisoCh;
  gstcSioCb.spi_sio.u8SpiMosiCh = pstcSpiConfig->u8SpiMosiCh;
  gstcSioCb.spi_sio.u8SpiClkCh  = pstcSpiConfig->u8SpiClkCh;
  gstcSioCb.spi_sio.u8SpiValid  = 1;

  /*chip select channel mux configuration*/
  pstcSio->SIO_OUT_MUX_REG[pstcSpiConfig->u8SpiCsCh] = 0x00;
  /*chip select channel OUTPUT enable*/
  pstcSio->SIO_GPIO_OEN_REG &= (~(1 << pstcSpiConfig->u8SpiCsCh));
  pstcSio->SIO_GPIO_OUT_REG |= (1 << pstcSpiConfig->u8SpiCsCh);

  /*SPI clock channel configuration*/
  pstcSio->SIO_OUT_MUX_REG[pstcSpiConfig->u8SpiClkCh] = (((0x1 & 0x7) << 3));
  /*Enable clock mode with inverted clock*/
  if (pstcSpiConfig->u8Mode == SioSpiMode0) {
    /*SPI MODE 0*/
    /*Configure SPI clock LOW to HIGH , on idle clock line will be in low state */
    pstcSio->SIO_CONFIG_REG[pstcSpiConfig->u8SpiClkCh] |= BIT(9) | BIT(14);
  } else {
    /*SPI MODE 3*/
    /*Configure SPI clock HIGH to LOW , on idle clock line will be in high state */
    pstcSio->SIO_CONFIG_REG[pstcSpiConfig->u8SpiClkCh] |= BIT(14);
  }
  /*clock channel OUTPUT enable*/
  pstcSio->SIO_GPIO_OEN_REG &= (~(1 << pstcSpiConfig->u8SpiClkCh));
  pstcSio->SIO_GPIO_OUT_REG |= (1 << pstcSpiConfig->u8SpiClkCh);

  /*SPI MOSI MUX configuration*/
  pstcSio->SIO_OUT_MUX_REG[pstcSpiConfig->u8SpiMosiCh] = ((0x4 & 0x7) << 3);
  /*Data sampling on neg edge configuration*/
  pstcSio->SIO_CONFIG_REG[pstcSpiConfig->u8SpiMosiCh] |= BIT(2);

  /*MOSI Output enable */
  pstcSio->SIO_GPIO_OEN_REG &= ~(0x1 << pstcSpiConfig->u8SpiMosiCh);

  /*High the line default*/
  pstcSio->SIO_GPIO_OUT_REG |= (0x1 << pstcSpiConfig->u8SpiMosiCh);

  /*MISO channel configuration as input */
  pstcSio->SIO_INPUT_MUX_REG[pstcSpiConfig->u8SpiMisoCh] = (0x0 << 0);

  /*Enable Swap interrupts for MOSI and MISO channels */
  pstcSio->SIO_SWAP_INTR_EN_SET_REG |= ((1 << pstcSpiConfig->u8SpiMisoCh) | (1 << pstcSpiConfig->u8SpiMosiCh));
  /*unmask interrupts*/
  pstcSio->SIO_SWAP_INTR_MASK_CLEAR_REG |= ((1 << pstcSpiConfig->u8SpiMisoCh) | (1 << pstcSpiConfig->u8SpiMosiCh));

  /*SPI interface clock frequency configuration*/
  /*Derive the reload value*/
  u32RelaodVal = (SystemCoreClock / pstcSpiConfig->u32SpiClockFrq);
  u32RelaodVal = (u32RelaodVal / 2) - 1;

  /*Bit order configuration*/
  if (pstcSpiConfig->u8BitOrder == 1) {
    /*MSB First*/
    pstcSio->SIO_SHIFT_COUNT_PRELOAD_REG_b[pstcSpiConfig->u8SpiMisoCh].REVERSE_LOAD = 1;
    pstcSio->SIO_SHIFT_COUNT_PRELOAD_REG_b[pstcSpiConfig->u8SpiMosiCh].REVERSE_LOAD = 1;
  } else {
    /*LSB First*/
    pstcSio->SIO_SHIFT_COUNT_PRELOAD_REG_b[pstcSpiConfig->u8SpiMisoCh].REVERSE_LOAD = 0;
    pstcSio->SIO_SHIFT_COUNT_PRELOAD_REG_b[pstcSpiConfig->u8SpiMosiCh].REVERSE_LOAD = 0;
  }

  /*Update clock reload for the channels */
  pstcSio->SIO_SHIFT_COUNT_PRELOAD_REG_b[pstcSpiConfig->u8SpiMisoCh].RELOAD_VALUE = (u32RelaodVal);
  pstcSio->SIO_SHIFT_COUNT_PRELOAD_REG_b[pstcSpiConfig->u8SpiMosiCh].RELOAD_VALUE = (u32RelaodVal);
  pstcSio->SIO_SHIFT_COUNT_PRELOAD_REG_b[pstcSpiConfig->u8SpiClkCh].RELOAD_VALUE  = (u32RelaodVal);

  /*Configure data position counters */
  pstcSio->SIO_DATA_POS_COUNT_REG_b[pstcSpiConfig->u8SpiMisoCh].RELOAD_VALUE = (pstcSpiConfig->u8BitLen - 1);
  pstcSio->SIO_DATA_POS_COUNT_REG_b[pstcSpiConfig->u8SpiMosiCh].RELOAD_VALUE = (pstcSpiConfig->u8BitLen - 1);
  pstcSio->SIO_DATA_POS_COUNT_REG_b[pstcSpiConfig->u8SpiClkCh].RELOAD_VALUE  = (pstcSpiConfig->u8BitLen - 1);

  return RSI_OK;
}

/**
 * @fn           void RSI_SIO_SpiCsAssert(volatile SIO_Type *pstcSio , uint8_t u8CsNo)
 * @brief        This API is used to assert the SPI master chip select
 * @param[in]    pstcSio : pointer to the register instance of SIO module
 * @param[in]    u8CsNo  : chip select number(0..7)
 * @return       None
 */
void RSI_SIO_SpiCsAssert(volatile SIO_Type *pstcSio, uint8_t u8CsNo)
{
  /*Assert CS : 0*/
  pstcSio->SIO_GPIO_OUT_REG &= ~(0x1 << u8CsNo);
}

/**
 * @fn           void RSI_SIO_SpiCsDeAssert(volatile SIO_Type *pstcSio , uint8_t u8CsNo)
 * @brief        This API is used to deassert the SPI master chip select
 * @param[in]    pstcSio   : pointer to the register instance of SIO module
 * @param[in]    u8CsNo    : chip select number
 * @return       None
 */
void RSI_SIO_SpiCsDeAssert(volatile SIO_Type *pstcSio, uint8_t u8CsNo)
{
  /*Assert CS : 1*/
  pstcSio->SIO_GPIO_OUT_REG |= (0x1 << u8CsNo);
}

/**
 * @fn           error_t RSI_SIO_SpiTrasnfer(volatile SIO_Type *pstcSio , stc_sio_spi_xfer_t  *pstcSpiXfer)
 * @brief        This API is used to make the SIO-SPI Transfer in non blocking mode.
 * @param[in]    pstcSio     : pointer to the register instance of SIO module
 * @param[in]    pstcSpiXfer : pointer to the SIO-Transfer structure.
 * @return     return \ref RSI_OK on success and on failure return error code.
 */
error_t RSI_SIO_SpiTrasnfer(volatile SIO_Type *pstcSio, stc_sio_spi_xfer_t *pstcSpiXfer)
{
  volatile uint32_t u32Data = 0;
  //volatile uint32_t u32cnt = 0;

  pstcSpiXfer->txDoneCount = 0;
  pstcSpiXfer->rxDoneCount = 0;

  /*Assign SIO-SPI Transfer function pointer to global structure to hold and access from ISR*/
  gstcSioCb.spi_sio.pstscSpiXfer = (stc_sio_spi_xfer_t *)pstcSpiXfer;

  /*Keep SIO-SPI in BUSY State*/
  pstcSpiXfer->u8Status = SioSpiBusy;

  /* Parameter check for valid parameters */
  if ((NULL == pstcSio) || (NULL == pstcSpiXfer)) {
    return (INVALID_PARAMETERS);
  }
  /*As per bit configuration typecast the pointer */
  if (pstcSpiXfer->u8BitLen == 8) {
    u32Data = ((uint8_t *)pstcSpiXfer->txBuff)[pstcSpiXfer->txDoneCount];
  } else if (pstcSpiXfer->u8BitLen == 16) {
    u32Data = ((uint16_t *)pstcSpiXfer->txBuff)[pstcSpiXfer->txDoneCount];
  } else {
    u32Data = ((uint32_t *)pstcSpiXfer->txBuff)[pstcSpiXfer->txDoneCount];
  }

  /*Increment the tx buffer index */
  pstcSpiXfer->txDoneCount++;

  /*Clear the interrupt MASK */
  //pstcSio->SWAP_INTR_MASK_CLEAR_REG |= ((1 << gstcSioCb.spi_sio.u8SpiMosiCh) | (1 << gstcSioCb.spi_sio.u8SpiMisoCh));
  pstcSio->SIO_SWAP_INTR_EN_SET_REG |= ((1 << gstcSioCb.spi_sio.u8SpiMosiCh) | (1 << gstcSioCb.spi_sio.u8SpiMisoCh));
  /*check for Bit order*/
  if ((pstcSio->SIO_SHIFT_COUNT_PRELOAD_REG_b[gstcSioCb.spi_sio.u8SpiMisoCh].REVERSE_LOAD == 1)
      && ((pstcSio->SIO_SHIFT_COUNT_PRELOAD_REG_b[gstcSioCb.spi_sio.u8SpiMosiCh].REVERSE_LOAD == 1))) {
    /*MSB First*/
    pstcSio->SIO_BUFFER_REG[gstcSioCb.spi_sio.u8SpiMosiCh] = (u32Data << (32 - pstcSpiXfer->u8BitLen));
  } else {
    /*LSB First*/
    pstcSio->SIO_BUFFER_REG[gstcSioCb.spi_sio.u8SpiMosiCh] = (u32Data >> (32 - pstcSpiXfer->u8BitLen));
  }

  /*Enable and pause the shift counters */
  pstcSio->SIO_ENABLE_REG_b.SIO_OPERATION_ENABLE =
    ((0x1 << gstcSioCb.spi_sio.u8SpiMisoCh) | (1 << gstcSioCb.spi_sio.u8SpiMosiCh)
     | (1 << gstcSioCb.spi_sio.u8SpiClkCh));

  pstcSio->SIO_PAUSE_REG_b.SIO_POSITION_COUNTER_DISABLE =
    ((1 << gstcSioCb.spi_sio.u8SpiMisoCh) | (0x1 << gstcSioCb.spi_sio.u8SpiMosiCh)
     | (1 << gstcSioCb.spi_sio.u8SpiClkCh));

  return RSI_OK;
}

/*SIO-I2S */

/**
 * @fn           error_t RSI_SIO_InitI2s(volatile SIO_Type *pstcSio , stc_sio_i2s_config_t  *pstcI2sConfig)
 * @brief        This API is used to configure the I2S in SIO.
 * @param[in]    pstcSio       : pointer to the register instance of SIO module
 * @param[in]    pstcI2sConfig : pointer to SIO I2S configuration structure
 * @return      return \ref RSI_OK on success and on failure return error code.
 */
error_t RSI_SIO_InitI2s(volatile SIO_Type *pstcSio, stc_sio_i2s_config_t *pstcI2sConfig)
{
  uint32_t u32RelaodVal = 0;

  /* Parameter check for valid parameters */
  if ((NULL == pstcSio) || (NULL == pstcI2sConfig)) {
    return (INVALID_PARAMETERS);
  }

  /*Update the global SIO channel CB*/
  gstcSioCb.i2s_sio.u8I2sClkCh     = pstcI2sConfig->u8SioI2sClkChannel;
  gstcSioCb.i2s_sio.u8I2sDataInCh  = pstcI2sConfig->u8SioI2sDataInChannel;
  gstcSioCb.i2s_sio.u8I2sDataOutCh = pstcI2sConfig->u8SioI2sDataOutChannel;
  gstcSioCb.i2s_sio.u8I2sWsCh      = pstcI2sConfig->u8SioI2sWsChannel;
  gstcSioCb.i2s_sio.u8I2sValid     = 1;
  gstcSioCb.i2s_sio.u8I2sWsWidth   = pstcI2sConfig->u8I2sWsWidth;

  /*I2S Frame clock channel configuration*/
  pstcSio->SIO_OUT_MUX_REG[pstcI2sConfig->u8SioI2sClkChannel] = (((0x1 & 0x7) << 3));
  /*Enable clock mode with inverted clock*/
  pstcSio->SIO_CONFIG_REG[pstcI2sConfig->u8SioI2sClkChannel] = BIT(9) | BIT(14);
  /*clock channel OUTPUT enable*/
  pstcSio->SIO_GPIO_OEN_REG &= (~(1 << pstcI2sConfig->u8SioI2sClkChannel));
  pstcSio->SIO_GPIO_OUT_REG |= (1 << pstcI2sConfig->u8SioI2sClkChannel);

  /*DOUT channel MUX configuration*/
  pstcSio->SIO_OUT_MUX_REG[pstcI2sConfig->u8SioI2sDataOutChannel] = ((0x4 & 0x7) << 3);
  /*Data sampling on neg edge configuration*/
  pstcSio->SIO_CONFIG_REG[pstcI2sConfig->u8SioI2sDataOutChannel] = BIT(2);
  /*MOSI Output enable */
  pstcSio->SIO_GPIO_OEN_REG &= ~(0x1 << pstcI2sConfig->u8SioI2sDataOutChannel);
  /*High the line default*/
  pstcSio->SIO_GPIO_OUT_REG |= (0x1 << pstcI2sConfig->u8SioI2sDataOutChannel);

  /*WS channel MUX configuration*/
  pstcSio->SIO_OUT_MUX_REG[pstcI2sConfig->u8SioI2sWsChannel] = ((0x4 & 0x7) << 3);
  /*Data sampling on neg edge configuration*/
  pstcSio->SIO_CONFIG_REG[pstcI2sConfig->u8SioI2sWsChannel] = BIT(2);
  /*MOSI Outpit enable */
  pstcSio->SIO_GPIO_OEN_REG &= ~(0x1 << pstcI2sConfig->u8SioI2sWsChannel);
  /*High the line default*/
  pstcSio->SIO_GPIO_OUT_REG |= (0x1 << pstcI2sConfig->u8SioI2sWsChannel);

  /*DIN channel configuration as input */
  pstcSio->SIO_INPUT_MUX_REG[pstcI2sConfig->u8SioI2sDataInChannel] = (0x0 << 0);

  /*Enable Swap interrupts for MOSI and MISO channels */
  pstcSio->SIO_SWAP_INTR_EN_SET_REG |= (1 << pstcI2sConfig->u8SioI2sDataOutChannel)
                                       | (1 << pstcI2sConfig->u8SioI2sDataInChannel);
  /*unmask interrupts*/
  pstcSio->SIO_SWAP_INTR_MASK_CLEAR_REG |= (1 << pstcI2sConfig->u8SioI2sDataOutChannel)
                                           | (1 << pstcI2sConfig->u8SioI2sDataInChannel);
  /*SPI interface clock frequcncy configuration*/
  /*Derive the reload value*/
  u32RelaodVal = (SystemCoreClock / pstcI2sConfig->u32SampleRate);
  u32RelaodVal = (u32RelaodVal / 2) - 1;

  /*Bit order configuration*/
  if (pstcI2sConfig->u8BitOrder == 1) {
    /*MSB First*/
    pstcSio->SIO_SHIFT_COUNT_PRELOAD_REG_b[pstcI2sConfig->u8SioI2sWsChannel].REVERSE_LOAD      = 1;
    pstcSio->SIO_SHIFT_COUNT_PRELOAD_REG_b[pstcI2sConfig->u8SioI2sDataOutChannel].REVERSE_LOAD = 1;
    pstcSio->SIO_SHIFT_COUNT_PRELOAD_REG_b[pstcI2sConfig->u8SioI2sDataInChannel].REVERSE_LOAD  = 1;

  } else {
    /*LSB First*/
    pstcSio->SIO_SHIFT_COUNT_PRELOAD_REG_b[pstcI2sConfig->u8SioI2sWsChannel].REVERSE_LOAD      = 0;
    pstcSio->SIO_SHIFT_COUNT_PRELOAD_REG_b[pstcI2sConfig->u8SioI2sDataOutChannel].REVERSE_LOAD = 0;
    pstcSio->SIO_SHIFT_COUNT_PRELOAD_REG_b[pstcI2sConfig->u8SioI2sDataInChannel].REVERSE_LOAD  = 0;
  }

  /*Update clock reload for the channels */
  pstcSio->SIO_SHIFT_COUNT_PRELOAD_REG_b[pstcI2sConfig->u8SioI2sWsChannel].RELOAD_VALUE      = (u32RelaodVal);
  pstcSio->SIO_SHIFT_COUNT_PRELOAD_REG_b[pstcI2sConfig->u8SioI2sDataOutChannel].RELOAD_VALUE = (u32RelaodVal);
  pstcSio->SIO_SHIFT_COUNT_PRELOAD_REG_b[pstcI2sConfig->u8SioI2sDataInChannel].RELOAD_VALUE  = (u32RelaodVal);
  pstcSio->SIO_SHIFT_COUNT_PRELOAD_REG_b[pstcI2sConfig->u8SioI2sClkChannel].RELOAD_VALUE     = (u32RelaodVal);

  /*Configure data position counters */
  pstcSio->SIO_DATA_POS_COUNT_REG_b[pstcI2sConfig->u8SioI2sWsChannel].RELOAD_VALUE      = (pstcI2sConfig->u8BitLen - 1);
  pstcSio->SIO_DATA_POS_COUNT_REG_b[pstcI2sConfig->u8SioI2sDataOutChannel].RELOAD_VALUE = (pstcI2sConfig->u8BitLen - 1);
  pstcSio->SIO_DATA_POS_COUNT_REG_b[pstcI2sConfig->u8SioI2sDataInChannel].RELOAD_VALUE  = (pstcI2sConfig->u8BitLen - 1);
  pstcSio->SIO_DATA_POS_COUNT_REG_b[pstcI2sConfig->u8SioI2sClkChannel].RELOAD_VALUE     = (pstcI2sConfig->u8BitLen - 1);

  return RSI_OK;
}

/**
 * @fn           error_t RSI_SIO_I2sTrasnfer(volatile SIO_Type *pstcSio , stc_sio_i2s_xfer_t  *pstcI2sXfer)
 * @brief        This API is used to transfer the I2S data , in non blocking mode.
 * @param[in]    pstcSio     : pointer to the register instance of SIO module
 * @param[in]    pstcI2sXfer : pointer to SIO-I2S transfer structure
 * @return      return \ref RSI_OK on success and on failure return error code.
 */
error_t RSI_SIO_I2sTrasnfer(volatile SIO_Type *pstcSio, stc_sio_i2s_xfer_t *pstcI2sXfer)
{
  pstcI2sXfer->txDoneCount = 0;
  pstcI2sXfer->rxDoneCount = 0;

  pstcI2sXfer->u8Status        = SioI2sBusy;
  gstcSioCb.i2s_sio.u8I2sValid = 1;
  /* Parameter check for valid parameters */
  if ((NULL == pstcSio) || (NULL == pstcI2sXfer)) {
    return (INVALID_PARAMETERS);
  }

  /*Update the global buffer with xfer structure */
  gstcSioCb.i2s_sio.pstcI2sXfer = (stc_sio_i2s_xfer_t *)pstcI2sXfer;
  pstcSio->SIO_SWAP_INTR_EN_SET_REG |= (1 << gstcSioCb.i2s_sio.u8I2sDataInCh)
                                       | ((1 << gstcSioCb.i2s_sio.u8I2sDataOutCh));
  /*word select line configuration*/
  if (!gstcSioCb.i2s_sio.u8I2sWsWidth) {
    pstcSio->SIO_BUFFER_REG[gstcSioCb.i2s_sio.u8I2sWsCh] = 0xFFFF0000; /* this is execute when WS is non zero */
  } else {
    u32Fclk                                              = ~u32Fclk;
    pstcSio->SIO_BUFFER_REG[gstcSioCb.i2s_sio.u8I2sWsCh] = u32Fclk;
  }
  //FIXME : shift data as per the data length
  pstcSio->SIO_BUFFER_REG[gstcSioCb.i2s_sio.u8I2sDataOutCh] =
    ((uint32_t *)pstcI2sXfer->txBuff)[pstcI2sXfer->txDoneCount];
  pstcI2sXfer->txDoneCount++;

  /*Enable and pause the shift counters  */ /* enable required channel function */
  pstcSio->SIO_ENABLE_REG_b.SIO_OPERATION_ENABLE |=
    (1 << gstcSioCb.i2s_sio.u8I2sDataOutCh) | (1 << gstcSioCb.i2s_sio.u8I2sWsCh) | (1 << gstcSioCb.i2s_sio.u8I2sClkCh)
    | (1 << gstcSioCb.i2s_sio.u8I2sDataInCh);
  return RSI_OK;
}

/*SIO UART*/
/**
 * @fn           uint8_t RSI_SIO_UartGetParity(uint16_t  u16Data)
 * @brief        This API is used to get the parity from the received data
 * @param[in]    u16Data   : UART received data
 * @return       return the parity value
 */
uint8_t RSI_SIO_UartGetParity(uint16_t u16Data)
{
  uint8_t parity = 0;
  while (u16Data) {
    parity  = !parity;
    u16Data = u16Data & (u16Data - 1);
  }
  return parity;
}

/**
 * @fn           uint16_t RSI_SIO_UartFramePrepare(uint8_t u8StopBits  , uint16_t u16Data , uint8_t u8ParitySet , uint8_t u8DataLen)
 * @brief        This API is used to prepare the UART transmit frame
 * @param[in]    u8StopBits  : stop bits for data transmit
 * @param[in]    u16Data     : data to be transmitted
 * @param[in]    u8ParitySet : parity bit set
 *                                 - \b  1 : Even parity
 *                                 - \b  2 : Odd parity
 * @param[in]    u8DataLen   : length of data to be transmitted
 * @return       returns the frame bits of UART
 */
uint16_t RSI_SIO_UartFramePrepare(uint8_t u8StopBits, uint16_t u16Data, uint8_t u8ParitySet, uint8_t u8DataLen)
{
  uint16_t u16Frame = 0;
  uint8_t u8Parity  = 0;

  if (u8ParitySet == 1) {
    // even parity
    u8Parity = RSI_SIO_UartGetParity(u16Data);

    u16Frame |= (0 << 0);
    u16Frame |= (u16Data << 1);
    u16Frame |= (u8Parity << (u8DataLen + 1));
    if (u8StopBits == 1) {
      u16Frame |= (0x01 << (u8DataLen + 2));
    } else {
      u16Frame |= (0x03 << (u8DataLen + 2));
    }

  }

  else if (u8ParitySet == 2) {
    // Odd parity
    u8Parity = RSI_SIO_UartGetParity(u16Data);
    u8Parity = !u8Parity;

    u16Frame |= (0 << 0);
    u16Frame |= (u16Data << 1);
    u16Frame |= (u8Parity << (u8DataLen + 1));
    if (u8StopBits == 1) {
      u16Frame |= (0x01 << (u8DataLen + 2));
    } else {
      u16Frame |= (0x03 << (u8DataLen + 2));
    }
  } else {
    // Parity none
    u16Frame |= (0 << 0);
    u16Frame |= (u16Data << 1);
    if (u8StopBits == 1) {
      u16Frame |= (0x01 << (u8DataLen + 1));
    } else {
      u16Frame |= (0x03 << (u8DataLen + 1));
    }
  }
  return u16Frame;
}

/**
 * @fn           uint16_t RSI_SIO_UARTSend(SIO_Type *pstcSio , const void *u16ptr , uint16_t u16Len)
 * @brief        This API is used to send the data over UART
 * @param[in]    pstcSio  : pointer to the register instance of SIO module
 * @param[in]    u16ptr   : data pointer to send
 * @param[in]    u16Len   : data length
 * @return       return \ref RSI_OK on success and on failure return error code.
 */
uint16_t RSI_SIO_UARTSend(SIO_Type *pstcSio, const void *u16ptr, uint16_t u16Len)
{
  uint16_t u16UartDataFrame = 0;
  uint32_t u32Data          = 0;
  uint32_t u32EnableReg = 0, u32PauseReg = 0;
  gstcSioCb.uart_sio.u16UartTxDone = 0;

  /*As per bit configuration typecast the pointer */
  if (gstcSioCb.uart_sio.pstcSioUart->u8Bitlen == 8) {
    gstcSioCb.uart_sio.pvUartTx = ((uint8_t *)u16ptr);
    u32Data                     = ((uint8_t *)u16ptr)[gstcSioCb.uart_sio.u16UartTxDone];
  } else if (gstcSioCb.uart_sio.pstcSioUart->u8Bitlen == 9) {
    gstcSioCb.uart_sio.pvUartTx = ((uint16_t *)u16ptr);
    u32Data                     = ((uint16_t *)u16ptr)[gstcSioCb.uart_sio.u16UartTxDone];
  }
  gstcSioCb.uart_sio.u16UartTxCnt = u16Len;

  /*prepare UART header */
  u16UartDataFrame = RSI_SIO_UartFramePrepare(gstcSioCb.uart_sio.pstcSioUart->u8StopBits,
                                              u32Data,
                                              gstcSioCb.uart_sio.pstcSioUart->u8Parity,
                                              gstcSioCb.uart_sio.pstcSioUart->u8Bitlen);
  gstcSioCb.uart_sio.u16UartTxDone++;

  // actual data
  u32Data = 0xFFFFFE00;
  u32Data |= u16UartDataFrame;
  u32Data = (u32Data << 1);
  u32Data |= 1;

  /*Update status to busy*/
  gstcSioCb.uart_sio.u8Status = SioUartsBusy;

  /*Write TX data to buffer */
  pstcSio->SIO_BUFFER_REG[gstcSioCb.uart_sio.pstcSioUart->u8SioUartTxChannel] = u32Data;

  pstcSio->SIO_SWAP_INTR_EN_SET_REG = (1 << gstcSioCb.uart_sio.u8UartTxCh);

  /*enable particular SIO : TX and RX */
  u32EnableReg = pstcSio->SIO_ENABLE_REG_b.SIO_OPERATION_ENABLE;

  /*clear reg */
  u32EnableReg &= ~(1 << gstcSioCb.uart_sio.pstcSioUart->u8SioUartTxChannel);

  u32EnableReg |= (1 << gstcSioCb.uart_sio.pstcSioUart->u8SioUartTxChannel);

  pstcSio->SIO_ENABLE_REG_b.SIO_OPERATION_ENABLE = u32EnableReg;

  u32PauseReg = pstcSio->SIO_PAUSE_REG_b.SIO_POSITION_COUNTER_DISABLE;

  u32PauseReg &= ~(1 << gstcSioCb.uart_sio.pstcSioUart->u8SioUartTxChannel);

  u32PauseReg |= (1 << gstcSioCb.uart_sio.pstcSioUart->u8SioUartTxChannel);

  pstcSio->SIO_PAUSE_REG_b.SIO_POSITION_COUNTER_DISABLE = u32PauseReg;

  return RSI_OK;
}

/**
 * @fn           uint16_t RSI_SIO_UartFrameExtract(uint8_t u8StopBits  , uint16_t u16UartDataFrame , uint8_t u8ParitySet , uint8_t u8DataLen)
 * @brief        This API is used to extract the data frame
 * @param[in]    u8StopBits       : stop bits
 * @param[in]    u16UartDataFrame : data frame
 * @param[in]    u8ParitySet      : data parity
 * @param[in]    u8DataLen        : data length
 * @return       Return the data
 */
uint16_t RSI_SIO_UartFrameExtract(uint8_t u8StopBits, uint16_t u16UartDataFrame, uint8_t u8ParitySet, uint8_t u8DataLen)
{
  uint16_t u16Data = 0;
  uint8_t u8Parity = 0;
  if (u8ParitySet == 1) {
    /*EVEN PARITY*/
    u16Data = (u16UartDataFrame);
    if (u8DataLen == 8) {
      u16Data &= 0xFF;
    } else {
      u16Data &= 0x1FF;
    }
    u8Parity = RSI_SIO_UartGetParity(u16Data);

    if (u8Parity == u16UartDataFrame >> (u8DataLen + 1))
      ;
    {
      return u16Data;
    }
  } else if (u8ParitySet == 2) {
    /*ODD PARITY*/
  } else {
    /*Parity None*/
    u16Data = (u16UartDataFrame >> 1);
    if (u8DataLen == 8) {
      u16Data &= 0xFF;
    } else {
      u16Data &= 0x1FF;
    }
    return u16Data;
  }

  return RSI_OK;
}

/**
 * @fn           int RSI_SIO_UARTSendBlocking(SIO_Type *pstcSio, const void *data, int numBytes)
 * @brief        This API is used to write data in UART mode
 * @param[in]    pstcSio : pointer to the register instance of SIO module
 * @param[in]    data    : data pointer
 * @param[in]    numBytes: number of bytes to send
 * @return       None
 */
int RSI_SIO_UARTSendBlocking(SIO_Type *pstcSio, const void *data, int numBytes)
{
  /*Update the status to Idle */
  gstcSioCb.uart_sio.u8Status = SioUartIdle;

  /*start UART transmit */
  RSI_SIO_UARTSend(pstcSio, data, numBytes);

  /*Wait for transmit done*/
  while (gstcSioCb.uart_sio.u8Status == SioUartsBusy)
    ;

  /*Return the number of bytes sent*/
  return gstcSioCb.uart_sio.u16UartTxDone;
}

/**
 * @fn           int RSI_SIO_UARTRead(volatile SIO_Type *pstcSio, void *data, int numBytes)
 * @brief        This API is used to read data in UART mode
 * @param[in]    pstcSio   : pointer to the register instance of SIO module
 * @param[in]    data      : data pointer
 * @param[in]    numBytes  : number of bytes
 * @return       None
 */
int RSI_SIO_UARTRead(volatile SIO_Type *pstcSio, void *data, int numBytes)
{
  gstcSioCb.uart_sio.enRxStatus    = SioUartIdle;
  gstcSioCb.uart_sio.u16UartRxDone = 0;
  /*As per bit configuration typecast the pointer */
  if (gstcSioCb.uart_sio.pstcSioUart->u8Bitlen == 8) {
    gstcSioCb.uart_sio.pvUartRx = ((uint8_t *)data);
  } else if (gstcSioCb.uart_sio.pstcSioUart->u8Bitlen == 9) {
    gstcSioCb.uart_sio.pvUartRx = ((uint16_t *)data);
  }
  gstcSioCb.uart_sio.u16UartRxCnt = numBytes;
  return RSI_OK;
}

/**
 * @fn           RSI_SIO_UARTReadBlocking(volatile SIO_Type *pstcSio, void *data, int numBytes)
 * @brief        This API is used to read data in UART blocking mode
 * @param[in]    pstcSio   : pointer to the register instance of SIO module
 * @param[in]    data      : data pointer
 * @param[in]    numBytes  : no of bytes
 * @return       None
 */
int RSI_SIO_UARTReadBlocking(volatile SIO_Type *pstcSio, void *data, int numBytes)
{
  gstcSioCb.uart_sio.enRxStatus    = SioUartIdle;
  gstcSioCb.uart_sio.u16UartRxDone = 0;
  /*As per bit configuration typecast the pointer */
  if (gstcSioCb.uart_sio.pstcSioUart->u8Bitlen == 8) {
    gstcSioCb.uart_sio.pvUartRx = ((uint8_t *)data);
  } else if (gstcSioCb.uart_sio.pstcSioUart->u8Bitlen == 9) {
    gstcSioCb.uart_sio.pvUartRx = ((uint16_t *)data);
  }

  gstcSioCb.uart_sio.u16UartRxCnt = numBytes;

  while (gstcSioCb.uart_sio.enRxStatus == SioUartIdle)
    ;

  return RSI_OK;
}

/**
 * @fn           RSI_SIO_UartInit(SIO_Type *pstcSio , stc_sio_uart_config_t *pstcConfig)
 * @brief        This API is used to Initialization of uart in sio
 * @param[in]    pstcSio	  :pointer to the register instance of SIO module
 * @param[in]    pstcConfig	:pointer to the uart config in SIO module
 * @return       None
 */
uint8_t RSI_SIO_UartInit(SIO_Type *pstcSio, stc_sio_uart_config_t *pstcConfig)
{
  uint32_t u32RelaodVal = 0, u32Divider = 0;

  /* Update the global structure */
  gstcSioCb.uart_sio.u8UartTxCh  = pstcConfig->u8SioUartTxChannel;
  gstcSioCb.uart_sio.u8UartRxCh  = pstcConfig->u8SioUartRxChannel;
  gstcSioCb.uart_sio.pstcSioUart = (stc_sio_uart_config_t *)pstcConfig;

  gstcSioCb.uart_sio.u8UartValid = 1;

  /*Parity enable check */
  if (pstcConfig->u8Parity) {
    gstcSioCb.uart_sio.u8ParityEn = 1;
  } else {
    gstcSioCb.uart_sio.u8ParityEn = 0;
  }
  /*Get the desired baud rate*/
  u32Divider = (pstcConfig->u32BaudRate);

  u32RelaodVal = (SystemCoreClock / u32Divider);

  u32RelaodVal = (u32RelaodVal / 2) - 1;

  pstcSio->SIO_BUFFER_REG[pstcConfig->u8SioUartTxChannel] = 0xFFFFFFFF;

  pstcSio->SIO_OUT_MUX_REG[pstcConfig->u8SioUartTxChannel] = ((0x4 & 0x7) << 3);

  //Select is as output pin
  pstcSio->SIO_GPIO_OEN_REG &= (~(1 << pstcConfig->u8SioUartTxChannel));

  //Reset value should be 1
  pstcSio->SIO_GPIO_OUT_REG |= (1 << pstcConfig->u8SioUartTxChannel);

  //UART RX
  pstcSio->SIO_INPUT_MUX_REG[pstcConfig->u8SioUartRxChannel] = (0 << 0);

  /*Falling Edge detection */
  pstcSio->SIO_CONFIG_REG_b[pstcConfig->u8SioUartRxChannel].PIN_DETECTION_MODE = 1;

  /*Enable the GPIO interrupt */
  pstcSio->SIO_GPIO_INTR_EN_SET_REG |= (1 << pstcConfig->u8SioUartRxChannel);

  /*Mask the interrup */
  pstcSio->SIO_GPIO_INTR_MASK_CLEAR_REG |= (1 << pstcConfig->u8SioUartRxChannel);

  /* SWAP Interrupt enable */
  pstcSio->SIO_SWAP_INTR_EN_SET_REG |= BIT(pstcConfig->u8SioUartTxChannel) | BIT(pstcConfig->u8SioUartRxChannel);
  /* Clear the SWAP Interrupt */
  pstcSio->SIO_SWAP_INTR_MASK_CLEAR_REG |= BIT(pstcConfig->u8SioUartTxChannel) | BIT(pstcConfig->u8SioUartRxChannel);

  /*Set TX Baud */
  pstcSio->SIO_SHIFT_COUNT_PRELOAD_REG_b[pstcConfig->u8SioUartTxChannel].RELOAD_VALUE = u32RelaodVal;

  /*Set RX Baud*/
  pstcSio->SIO_SHIFT_COUNT_PRELOAD_REG_b[pstcConfig->u8SioUartRxChannel].RELOAD_VALUE = u32RelaodVal;

  /*Reverse laod in Disable in RX*/
  pstcSio->SIO_SHIFT_COUNT_PRELOAD_REG_b[pstcConfig->u8SioUartRxChannel].REVERSE_LOAD = 0;

  /*set the data length configuration for TX */
  pstcSio->SIO_DATA_POS_COUNT_REG_b[pstcConfig->u8SioUartTxChannel].RELOAD_VALUE =
    ((pstcConfig->u8Bitlen + pstcConfig->u8StopBits + gstcSioCb.uart_sio.u8ParityEn + 10) - 1);
  return RSI_OK;
}

/*I2C*/
/**
 * @fn        static uint32_t RSI_SIO_I2cPrepareWrite(uint8_t   u8InDat)
 * @brief     This API is used to write the data
 * @param[in] u8InDat :8 bits unsigned integer data
 * @return    data
 *
 */

static uint32_t RSI_SIO_I2cPrepareWrite(uint8_t u8InDat)
{
  int16_t i = 0, j = 0;
  uint32_t u32OutDat = 0;

  i = 7;
  while (1) {
    if (u8InDat & (1 << i)) {
      u32OutDat |= (0x3 << j);
      j = j + 2;
    } else {
      u32OutDat |= (0x00 << j);
      j = j + 2;
    }
    if (i == 0) {
      return u32OutDat;
    }
    i--;
  }
}

/**
 * @fn        static uint32_t RSI_SIO_I2cPrepareRead(uint32_t   u32Indata)
 * @brief     This API is used to read the data
 * @param[in] u32InDat :32 bits unsigned integer data
 * @return    data
 *
 */
static uint32_t RSI_SIO_I2cPrepareRead(uint32_t u32Indata)
{
  int16_t i = 0, j = 31;
  uint8_t u8OutDat = 0;

  while (1) {
    if (u32Indata & (1 << j)) {
      u8OutDat |= (1 << i);
      j = j - 2;
    } else {
      u8OutDat |= (0 << i);
      j = j - 2;
    }
    if (i == 7) {
      u8OutDat &= 0xFF;
      return u8OutDat >> 1;
    }
    i++;
  }
}

/**
 * @fn           error_t RSI_SIO_I2cWrite(volatile SIO_Type *pstcSio , stc_sio_i2c_config_t *pstcConfig ,
 *                                          uint8_t u8SlaveAddr , uint8_t *u8Data , uint16_t  u16Len)
 * @brief        This API is used to write data using i2c in sio
 * @param[in]    pstcSio	  : pointer to the register instance of SIO module
 * @param[in]    pstcConfig	: pointer to the I2C config in SIO module
 * @param[in]    u8SlaveAddr: slave address
 * @param[in]    u8Data     : pointer to the data
 * @param[in]    u16Len	    : data length
 * @return       error codes
 */
error_t RSI_SIO_I2cWrite(volatile SIO_Type *pstcSio,
                         stc_sio_i2c_config_t *pstcConfig,
                         uint8_t u8SlaveAddr,
                         uint8_t *u8Data,
                         uint16_t u16Len)
{
  uint32_t u32Enable  = 0;
  uint32_t u32RecvDat = 0, u32I2cClok = 0;
  uint32_t u32Sdat = 0, u32Slav = 0;
  uint16_t cnt = 0;

  pstcSio->SIO_GPIO_OUT_REG          = 0x00c0;
  pstcSio->SIO_GPIO_OEN_REG          = 0xff3f;
  pstcSio->SIO_DATA_POS_COUNT_REG[6] = 0x0008;

  //arm_print("WRITE\n");
  /*Write mode slave address*/
  u32Slav = ((u8SlaveAddr << 1) | 0);

  //arm_print("Sl Add %x \n" , u32Slav);
  /* We need to give address in such a way that dout[1] should be equal to address and dout[0] should also be same*/
  /* Write slave address */
  u32Sdat = RSI_SIO_I2cPrepareWrite(u32Slav);

  u32Sdat |= (0x0003 << 16);
  //arm_print("S D %x \n" , u32Sdat);
  pstcSio->SIO_BUFFER_REG[6] = u32Sdat;
  /*As we need 2 bits to be shifted [er shift clock. We want the value to change at the negedge */
  pstcSio->SIO_CONFIG_REG[6] |= (1 << 15 | 1 << 2 | 1 << 10);

  u32I2cClok = (SystemCoreClock / pstcConfig->u32SampleRate);
  u32I2cClok = (u32I2cClok / 2) - 1;

  pstcSio->SIO_SHIFT_COUNT_PRELOAD_REG[6] = (u32I2cClok << 0);
  pstcSio->SIO_GPIO_OUT_REG               = 0x0040;
  pstcSio->SIO_GPIO_OUT_REG               = 0x0000;
  /* Configuring output mux such that output is equal to slice_dout_6[1] */
  pstcSio->SIO_OUT_MUX_REG[7] = 0x003c;
  /* configuring output mux so that output is equal to its shift clock */
  pstcSio->SIO_OUT_MUX_REG[6] = 0x0008;
  pstcSio->SIO_ENABLE_REG     = 0x0040;
  /* pausing in order to get time so that ack can be checked*/
  pstcSio->SIO_PAUSE_REG = 0x0040;

  /* waiting for enable of slice 6 to go to 0 */
  do {
    /*Read ENABLE Register*/
    u32Enable = pstcSio->SIO_ENABLE_REG;
    /*Check for Slice 6 Enable bit */
    u32Enable = u32Enable & 0x0040;
  } while (u32Enable != 0x0000);

  u32RecvDat = pstcSio->SIO_BUFFER_REG[6];

  /*Check ACK for slave address */
  if ((u32RecvDat) & (1U << 31)) {
    //arm_print("A: NA\n");
    /*giving the stop condition for wrong address */
    pstcSio->SIO_GPIO_OUT_REG   = 0x0040;
    pstcSio->SIO_OUT_MUX_REG[7] = 0x0000;
    pstcSio->SIO_OUT_MUX_REG[6] = 0x0000;
    pstcSio->SIO_PAUSE_REG      = 0x0000;
    pstcSio->SIO_ENABLE_REG     = 0x0040;
    pstcSio->SIO_PAUSE_REG      = 0x0040;
    pstcSio->SIO_GPIO_OUT_REG   = 0x00c0;
    do {
      u32Enable = pstcSio->SIO_ENABLE_REG;
      u32Enable = u32Enable & 0x0040;
    } while (u32Enable != 0x0000);
    /*Return the status as NA (No ACK)*/
    return ERROR_SIO_I2C_NO_ACK;
  }
  for (cnt = 0; cnt < u16Len; cnt++) {
    pstcSio->SIO_PAUSE_REG = 0x0000;
    u32Sdat                = RSI_SIO_I2cPrepareWrite(u8Data[cnt]);
    u32Sdat |= (0x43ff << 16);
    pstcSio->SIO_BUFFER_REG[6] = u32Sdat;
    pstcSio->SIO_ENABLE_REG    = 0x0040;
    pstcSio->SIO_PAUSE_REG     = 0x0040;
    do {
      u32Enable = pstcSio->SIO_ENABLE_REG;
      u32Enable = u32Enable & 0x0040;
    } while (u32Enable != 0x0000);
    u32RecvDat = pstcSio->SIO_BUFFER_REG[6];
    u32RecvDat = (u32RecvDat >> 16);
    u32RecvDat = u32RecvDat & 0x8000;
    if (u32RecvDat != 0) {
      //arm_print("D: NA\n");
      /*giving the stop condition for wrong address */
      pstcSio->SIO_GPIO_OUT_REG   = 0x0040;
      pstcSio->SIO_OUT_MUX_REG[7] = 0x0000;
      pstcSio->SIO_OUT_MUX_REG[6] = 0x0000;
      pstcSio->SIO_PAUSE_REG      = 0x0000;
      pstcSio->SIO_ENABLE_REG     = 0x0040;
      pstcSio->SIO_PAUSE_REG      = 0x0040;
      pstcSio->SIO_GPIO_OUT_REG   = 0x00c0;
      do {
        u32Enable = pstcSio->SIO_ENABLE_REG;
        u32Enable = u32Enable & 0x0040;
      } while (u32Enable != 0x0000);
      /*Return the status as NA (No ACK)*/
      return ERROR_SIO_I2C_NO_ACK;
    }
  }
  /*End of Data transfer -  Generate stop condition */
  pstcSio->SIO_GPIO_OUT_REG   = 0x0040;
  pstcSio->SIO_OUT_MUX_REG[7] = 0x0000;
  pstcSio->SIO_OUT_MUX_REG[6] = 0x0000;
  pstcSio->SIO_PAUSE_REG      = 0x0000;
  pstcSio->SIO_ENABLE_REG     = 0x0040;
  pstcSio->SIO_PAUSE_REG      = 0x0040;
  pstcSio->SIO_GPIO_OUT_REG   = 0x00c0;
  /*Wait for SIO enable go low */
  do {
    u32Enable = pstcSio->SIO_ENABLE_REG;
    u32Enable = u32Enable & 0x0040;
  } while (u32Enable != 0x0000);

  //arm_print("W: D");
  return RSI_OK;
}

/**
 * @fn           error_t RSI_SIO_I2cRead(volatile SIO_Type *pstcSio , stc_sio_i2c_config_t *pstcConfig ,
 *                                          uint8_t u8SlaveAddr , uint8_t *u8Data , uint16_t  u16Len)
 * @brief        This API is used to read data using i2c in sio
 * @param[in]    pstcSio	  : pointer to the register instance of SIO module
 * @param[in]    pstcConfig	: pointer to the I2C config in SIO module
 * @param[in]    u8SlaveAddr: slave address
 * @param[in]    u8Data     : pointer to the data buffer
 * @param[in]    u16Len	    : data length
 * @return       error codes
 */
error_t RSI_SIO_I2cRead(volatile SIO_Type *pstcSio,
                        stc_sio_i2c_config_t *pstcConfig,
                        uint8_t u8SlaveAddr,
                        uint8_t *u8Data,
                        uint16_t u16Len)
{

  uint32_t u32Enable = 0, u32Sdat = 0, u32Slav = 0;
  volatile uint32_t u32RecvDat = 0;
  int cnt;

  //	pstcSio->SIO_GPIO_OUT_REG = 0x00c0;
  //	pstcSio->SIO_GPIO_OEN_REG = 0xff3f;
  pstcSio->SIO_DATA_POS_COUNT_REG[6] = 0x0008;
  //arm_print("READ\n");
  /*Write mode slave address*/
  u32Slav = ((u8SlaveAddr << 1) | 1);
  ////arm_print("Sl Add %x \n" , u32Slav);
  /* We need to give address in such a way that dout[1] should be equal to address and dout[0] should also be same*/
  /* Write slave address */
  u32Sdat = RSI_SIO_I2cPrepareWrite(u32Slav);

  u32Sdat |= (0x0003 << 16);
  ////arm_print("S D %x \n" , u32Sdat);
  pstcSio->SIO_BUFFER_REG[6] = u32Sdat;
  /*As we need 2 bits to be shifted [er shift clock. We want the value to change at the negedge */
  pstcSio->SIO_CONFIG_REG[6] |= (1 << 15 | 1 << 2 | 1 << 10);

  //	u32I2cClok = (SystemCoreClock / pstcConfig->u32SampleRate);
  //	u32I2cClok = (u32I2cClok /2)-1;

  //	pstcSio->SIO_SHIFT_COUNT_PRELOAD_REG[6] =  (u32I2cClok << 0 );

  pstcSio->SIO_GPIO_OUT_REG = 0x0040;

  pstcSio->SIO_GPIO_OUT_REG = 0x0000;
  /* Configuring output mux such that output is equal to slice_dout_6[1] */
  pstcSio->SIO_OUT_MUX_REG[7] = 0x003c;
  /* configuring output mux so that output is equal to its shift clock */
  pstcSio->SIO_OUT_MUX_REG[6] = 0x0008;
  pstcSio->SIO_ENABLE_REG     = 0x0040;
  /* pausing in order to get time so that ack can be checked*/
  pstcSio->SIO_PAUSE_REG = 0x0040;

  /* waiting for enable of slice 6 to go to 0 */
  do {
    /*Read ENABLE Register*/
    u32Enable = pstcSio->SIO_ENABLE_REG;
    /*Check for Slice 6 Enable bit */
    u32Enable = u32Enable & 0x0040;
  } while (u32Enable != 0x0000);

  u32RecvDat = pstcSio->SIO_BUFFER_REG[6];

  /*Check ACK for slave address */
  if ((u32RecvDat) & (1U << 31)) {
    //arm_print("R A: NA\n");
    /*giving the stop condition for wrong address */
    RSI_SIO_I2cGenerateStop(pstcSio);
    /*Return the status as NA (No ACK)*/
    return ERROR_SIO_I2C_NO_ACK;
  }

  for (cnt = 0; cnt < u16Len; cnt++) {
    pstcSio->SIO_BUFFER_REG[6] = 0xfff0ffff;
    pstcSio->SIO_PAUSE_REG     = 0x0000;
    pstcSio->SIO_ENABLE_REG    = 0x0040;
    pstcSio->SIO_PAUSE_REG     = 0x0040;
    do {
      u32Enable = pstcSio->SIO_ENABLE_REG;
      u32Enable = u32Enable & 0x0040;
    } while (u32Enable != 0x0000);

    u32RecvDat = pstcSio->SIO_BUFFER_REG[6];
    if ((u32RecvDat) & (1U << 31)) {
      //arm_print("R D: NA\n");
      /*giving the stop condition for wrong address */
      RSI_SIO_I2cGenerateStop(pstcSio);
      /*Return the status as NA (No ACK)*/
      return ERROR_SIO_I2C_NO_ACK;
    }
    /*Fill the received Data */
    u8Data[cnt] = RSI_SIO_I2cPrepareRead(u32RecvDat);
  }
  //RSI_SIO_I2cGenerateStop(pstcSio);
  //arm_print("R D \n");

  return RSI_OK;
}

/**
 * @fn           error_t RSI_SIO_I2cTransfer(volatile SIO_Type *pstcSio , stc_sio_i2c_config_t *pstcConfig ,
 *                                          uint8_t u8SlaveAddr , uint8_t *u8PtrTxDat , uint16_t  u16TxLen ,
 *                                          uint8_t *u8PtrRxDat , uint16_t  u16RxLen )
 * @brief        This API is used to data transfer using i2c in sio
 * @param[in]    pstcSio	  : pointer to the register instance of SIO module
 * @param[in]    pstcConfig	: pointer to the I2C config in SIO module
 * @param[in]    u8SlaveAddr: slave address
 * @param[in]    u8PtrTxDat : pointer to the data TX buffer
 * @param[in]    u16TxLen	  : TX data length
 * @param[in]    u8PtrRxDat : pointer to the data RX buffer
 * @param[in]    u16RxLen	  : RX data length
 * @return       return \ref RSI_OK on success and on failure return error code.
 */
error_t RSI_SIO_I2cTransfer(volatile SIO_Type *pstcSio,
                            stc_sio_i2c_config_t *pstcConfig,
                            uint8_t u8SlaveAddr,
                            uint8_t *u8PtrTxDat,
                            uint16_t u16TxLen,
                            uint8_t *u8PtrRxDat,
                            uint16_t u16RxLen)
{
  uint32_t u32Enable  = 0;
  uint32_t u32RecvDat = 0, u32I2cClok = 0;
  uint32_t u32Sdat = 0, u32Slav = 0;
  uint16_t cnt = 0;

  //pstcSio->GPIO_OUT_REG = 0x00c0;
  //pstcSio->GPIO_OEN_REG = 0xff3f;
  pstcSio->SIO_DATA_POS_COUNT_REG[6] = 0x0008;

  ////arm_print("WRITE\n");
  /*Write mode slave address*/
  u32Slav = ((u8SlaveAddr << 1) | 0);

  ////arm_print("Sl Add %x \n" , u32Slav);

  /* We need to give address in such a way that dout[1] should be equal to address and dout[0] should also be same*/
  /* Write slave address */
  u32Sdat = RSI_SIO_I2cPrepareWrite(u32Slav);

  u32Sdat |= (0x0003 << 16);
  ////arm_print("S D %x \n" , u32Sdat);
  pstcSio->SIO_BUFFER_REG[6] = u32Sdat;
  /*As we need 2 bits to be shifted [er shift clock. We want the value to change at the negedge */
  pstcSio->SIO_CONFIG_REG[6] |= (1 << 15 | 1 << 2 | 1 << 10);

  u32I2cClok = (SystemCoreClock / pstcConfig->u32SampleRate);
  u32I2cClok = (u32I2cClok / 2) - 1;

  pstcSio->SIO_SHIFT_COUNT_PRELOAD_REG[6] = (u32I2cClok << 0);
  //pstcSio->GPIO_OUT_REG = 0x0040;
  //pstcSio->GPIO_OUT_REG = 0x0000;
  /* Configuring output mux such that output is equal to slice_dout_6[1] */
  pstcSio->SIO_OUT_MUX_REG[7] = 0x003c;
  /* configuring output mux so that output is equal to its shift clock */
  pstcSio->SIO_OUT_MUX_REG[6] = 0x0008;
  pstcSio->SIO_ENABLE_REG     = 0x0040;
  /* pausing in order to get time so that ack can be checked*/
  pstcSio->SIO_PAUSE_REG = 0x0040;

  /* waiting for enable of slice 6 to go to 0 */
  do {
    /*Read ENABLE Register*/
    u32Enable = pstcSio->SIO_ENABLE_REG;
    /*Check for Slice 6 Enable bit */
    u32Enable = u32Enable & 0x0040;
  } while (u32Enable != 0x0000);

  u32RecvDat = pstcSio->SIO_BUFFER_REG[6];

  /*Check ACK for slave address */
  if ((u32RecvDat) & (1U << 31)) {
    //arm_print("A: NA\n");
    /*giving the stop condition for wrong address */
    RSI_SIO_I2cGenerateStop(pstcSio);
    /*Return the status as NA (No ACK)*/
    return ERROR_SIO_I2C_NO_ACK;
  }
  for (cnt = 0; cnt < u16TxLen; cnt++) {
    pstcSio->SIO_PAUSE_REG = 0x0000;
    u32Sdat                = RSI_SIO_I2cPrepareWrite(u8PtrTxDat[cnt]);
    u32Sdat |= (0x43ff << 16);
    pstcSio->SIO_BUFFER_REG[6] = u32Sdat;
    pstcSio->SIO_ENABLE_REG    = 0x0040;
    pstcSio->SIO_PAUSE_REG     = 0x0040;
    do {
      u32Enable = pstcSio->SIO_ENABLE_REG;
      u32Enable = u32Enable & 0x0040;
    } while (u32Enable != 0x0000);
    u32RecvDat = pstcSio->SIO_BUFFER_REG[6];
    u32RecvDat = (u32RecvDat >> 16);
    u32RecvDat = u32RecvDat & 0x8000;
    if (u32RecvDat != 0) {
      //arm_print("D: NA\n");
      RSI_SIO_I2cGenerateStop(pstcSio);
      /*Return the status as NA (No ACK)*/
      return ERROR_SIO_I2C_NO_ACK;
    }
  }
  /*End of Data transfer -  Generate stop condition */
  //RSI_SIO_I2cGenerateStop(pstcConfig);

  //arm_print("W: D \n");
  return RSI_OK;
}

/**
 * @fn           void RSI_SIO_I2cGenerateStart(volatile SIO_Type *pstcSio)
 * @brief        This API is used to I2C generate start in sio
 * @param[in]    pstcSio	  :pointer to the register instance of SIO module
 * @return       None
 */
void RSI_SIO_I2cGenerateStart(volatile SIO_Type *pstcSio)
{
  //arm_print("ST\n");
  pstcSio->SIO_OUT_MUX_REG[7] = 0x0000;
  pstcSio->SIO_OUT_MUX_REG[6] = 0x0000;

  pstcSio->SIO_GPIO_OUT_REG = 0x00c0;
  pstcSio->SIO_GPIO_OEN_REG = 0xff3f;

  pstcSio->SIO_GPIO_OUT_REG = 0x0040;
  pstcSio->SIO_GPIO_OUT_REG = 0x0000;
}

/**
 * @fn           void RSI_SIO_I2cGenerateStop(volatile SIO_Type *pstcSio)
 * @brief        This API is used to I2C generate stop in sio
 * @param[in]    pstcSio	  :pointer to the register instance of SIO module
 * @return       None
 */
void RSI_SIO_I2cGenerateStop(volatile SIO_Type *pstcSio)
{
  uint32_t u32Enable = 0;

  //arm_print("SP\n");

  /*End of Data transfer -  Generate stop condition */
  pstcSio->SIO_GPIO_OUT_REG   = 0x0040;
  pstcSio->SIO_OUT_MUX_REG[7] = 0x0000;
  pstcSio->SIO_OUT_MUX_REG[6] = 0x0000;
  pstcSio->SIO_PAUSE_REG      = 0x0000;
  pstcSio->SIO_ENABLE_REG     = 0x0040;
  pstcSio->SIO_PAUSE_REG      = 0x0040;
  pstcSio->SIO_GPIO_OUT_REG   = 0x00c0;
  /*Wait for SIO enable go low */
  do {
    u32Enable = pstcSio->SIO_ENABLE_REG;
    u32Enable = u32Enable & 0x0040;
  } while (u32Enable != 0x0000);
}

/**
 * @}
 */

/*End of file not truncated */
