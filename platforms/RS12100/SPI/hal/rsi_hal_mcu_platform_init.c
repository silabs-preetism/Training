/*******************************************************************************
* @file  rsi_hal_mcu_platform_init.c
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
* @file       rsi_hal_mcu_platform_init.c
* @version    0.1
* @date       11 DEC 2018
*
*
*
* @brief HAL Board Init: Functions related to platform initialization
*
* @section Description
* This file contains the list of functions for configuring the microcontroller clock.
* Following are list of API's which need to be defined in this file.
*
*/


/**
* Includes
*/

#include "rsi_driver.h"
#include "GSPI.h"
#include "RTE_Device.h" 
#include "rsi_board.h"
#include "rsi_chip.h"

#define	 SPI_BAUD					20000000  //speed at which data transmitted through SPI
#define  SPI_BIT_WIDTH		8				//SPI bit width can be 16/8 for 16/8 bit data transfer 

/* SPI Driver */
extern ARM_DRIVER_SPI Driver_GSPI_MASTER;

volatile uint8_t spi_done = 0;

/*==============================================*/
/**
 * @fn
 * @brief
 * @param[in]   , 
 * @param[out]
 * @return
 *
 *
 * @section description
 * This 
 *
 *
 */

void mySPI_callback(uint32_t event)
{
  switch (event)
  {
    case ARM_SPI_EVENT_TRANSFER_COMPLETE:
      spi_done=1;
      break;
    case ARM_SPI_EVENT_DATA_LOST:
      /*  Occurs in slave mode when data is requested/sent by master
          but send/receive/transfer operation has not been started
          and indicates that data is lost. Occurs also in master mode
          when driver cannot transfer data fast enough. */
      __breakpoint(0);  /* Error: Call debugger or replace with custom error handling */
      break;
    case ARM_SPI_EVENT_MODE_FAULT:
      /*  Occurs in master mode when Slave Select is deactivated and
          indicates Master Mode Fault. */
      __breakpoint(0);  /* Error: Call debugger or replace with custom error handling */
      break;
  }
}
/*==============================================*/
/**
* @fn           void rsi_hal_board_init()
* @brief        This function Initializes the platform
* @param[in]    none 
* @param[out]   none
* @return       none
* @section description
* This function initializes the platform
*
*/

void rsi_hal_board_init(void)
{

  ARM_DRIVER_SPI* SPIdrv = &Driver_GSPI_MASTER;

  SystemCoreClockUpdate();
  RSI_EGPIO_PadDriverDisableState(27,1);
  RSI_EGPIO_PadDriverDisableState(29,2);	
  RSI_EGPIO_PadDriverDisableState(30,2);
  RSI_EGPIO_PadDriverDisableState(25,2);
  RSI_EGPIO_PadDriverDisableState(28,2);

  /*program intf pll to 180MHZ*/
  SPI_MEM_MAP_PLL(INTF_PLL_500_CTRL_REG9) = 0xD900 ;   
  RSI_CLK_SetIntfPllFreq(M4CLK,180000000,40000000);

  //! SDIO connected to TASS
  (*(volatile uint32_t *)(0x41300004)) = (0x1<<5);
  /*Configure m4 soc to 180mhz*/
  RSI_CLK_M4SocClkConfig(M4CLK,M4_INTFPLLCLK,0);

  /*configure socpll to 20mhz*/  
  RSI_CLK_SocPllLockConfig(1,1,0xA4);
  RSI_CLK_SetSocPllFreq(M4CLK,20000000,40000000);

  /* Initialize the SPI driver */
  SPIdrv->Initialize(mySPI_callback);

  /* Power up the SPI peripheral */
  SPIdrv->PowerControl(ARM_POWER_FULL);

  /* Configure the SPI to Master, 16-bit mode @10000 kBits/sec */
  SPIdrv->Control(ARM_SPI_MODE_MASTER | ARM_SPI_CPOL0_CPHA0 | ARM_SPI_SS_MASTER_HW_OUTPUT | ARM_SPI_DATA_BITS(SPI_BIT_WIDTH), SPI_BAUD);	 

  /* Systick config */
	SysTick_Config(180000000 / 1000);
}


/*==============================================*/
/**
 * @fn           void rsi_switch_to_high_clk_freq()
 * @brief        This function intializes SPI to high clock
 * @param[in]    none 
 * @param[out]   none
 * @return       none
 * @section description
 * This function intializes SPI to high clock
 *
 *
 */

void rsi_switch_to_high_clk_freq(void)
{
  ARM_DRIVER_SPI* SPIdrv = &Driver_GSPI_MASTER;

  RSI_CLK_SetSocPllFreq(M4CLK,40000000,40000000);
  //! Configure interrupt
  rsi_hal_intr_config(rsi_interrupt_handler);
  /* Initialize the SPI driver */
  SPIdrv->Initialize(mySPI_callback);

  /* Power up the SPI peripheral */
  SPIdrv->PowerControl(ARM_POWER_FULL);

  /* Configure the SPI to Master, 16-bit mode @10000 kBits/sec */
  SPIdrv->Control(ARM_SPI_MODE_MASTER | ARM_SPI_CPOL0_CPHA0 | ARM_SPI_SS_MASTER_HW_OUTPUT | ARM_SPI_DATA_BITS(SPI_BIT_WIDTH),40000000);	 
}




