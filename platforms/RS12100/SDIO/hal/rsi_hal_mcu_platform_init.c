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
* Includes
*/

#include "rsi_driver.h"
#include "rsi_board.h"
#include "rsi_chip.h"
#include "rsi_events.h"

SMIH_CARD_CONFIG_T      pSmihConfig;
#define SOC_OPER_FREQUENCY_SIM                   200000000 /*220Mhz*/
#define SDMEM_INTFPLLCLK                        (0x46000000 + 0x40)  
#define NWPAON_MEM_HOST_ACCESS_CTRL_CLEAR       (*(volatile uint32_t *)(0x41300004))    
#define NPSS_IO_VOLTAGE_REG                     (*(volatile uint32_t *)(0x41300614))
#define SDIO_CNTD_TO_TASS                       BIT(5)      

#ifndef RSI_MCU_COMPANION_CARD
#define SMIH_CLK                  25
#define SMIH_CMD                  26
#define SMIH_D0                   27
#define SMIH_D1                   28
#define SMIH_D2                   29
#define SMIH_D3                   30
#define SMIH_CD                   6
#define SMIH_WP                   7
#else
#define SMIH_CLK                  46
#define SMIH_CMD                  47
#define SMIH_D0                   48
#define SMIH_D1                   49
#define SMIH_D2                   50
#define SMIH_D3                   51
#define SMIH_CD                   53
#define SMIH_WP                   52
#endif


/*==============================================*/
/**
 * @fn                void smih_callback_handler(uint32_t event)
 * @brief              call back handler for interrupts
 * @param[in]   ,      event
 * @param[out]
 * @return
 *
 * @section description
 * This is sdio call back handler 
 *
 *
 */	

void M4_SDIOH_PinMux(void)
{
#ifdef RSI_MCU_COMPANION_CARD
	/*Pad selection Enable*/
	RSI_EGPIO_PadSelectionEnable(11);
	RSI_EGPIO_PadSelectionEnable(12);
	RSI_EGPIO_PadSelectionEnable(13);	
#else
	RSI_EGPIO_PadSelectionEnable(1);
#endif

  //! SDIO connected to TASS
  NWPAON_MEM_HOST_ACCESS_CTRL_CLEAR = SDIO_CNTD_TO_TASS;
	
  /*Enable Host Pad Gpio modes*/
	RSI_EGPIO_HostPadsGpioModeEnable(SMIH_CLK);
	RSI_EGPIO_HostPadsGpioModeEnable(SMIH_CMD);
	RSI_EGPIO_HostPadsGpioModeEnable(SMIH_D0);
	RSI_EGPIO_HostPadsGpioModeEnable(SMIH_D1);
	RSI_EGPIO_HostPadsGpioModeEnable(SMIH_D2);
	RSI_EGPIO_HostPadsGpioModeEnable(SMIH_D3);

	/*Ren enables for Gpios*/ 
	RSI_EGPIO_PadReceiverEnable(SMIH_CLK);
	RSI_EGPIO_PadReceiverEnable(SMIH_CMD);
	RSI_EGPIO_PadReceiverEnable(SMIH_D0);
	RSI_EGPIO_PadReceiverEnable(SMIH_D1);
	RSI_EGPIO_PadReceiverEnable(SMIH_D2);
	RSI_EGPIO_PadReceiverEnable(SMIH_CD);
	RSI_EGPIO_PadReceiverEnable(SMIH_WP);
	RSI_EGPIO_PadReceiverEnable(SMIH_D3);

	/*Configure software pull ups for cmd ,d0,d1,d2,d3*/
	RSI_EGPIO_PadDriverDisableState(SMIH_CMD,Pullup);
	RSI_EGPIO_PadDriverDisableState(SMIH_D0,Pullup);
	RSI_EGPIO_PadDriverDisableState(SMIH_D1,Pullup);
	RSI_EGPIO_PadDriverDisableState(SMIH_D2,Pullup);
	RSI_EGPIO_PadDriverDisableState(SMIH_D3,Pullup);


	/*Configure Mux*/
	RSI_EGPIO_SetPinMux(EGPIO ,0 , SMIH_CLK ,8);  
	RSI_EGPIO_SetPinMux(EGPIO ,0 , SMIH_CMD ,8);  
	RSI_EGPIO_SetPinMux(EGPIO ,0 , SMIH_D0  ,8);  
	RSI_EGPIO_SetPinMux(EGPIO ,0 , SMIH_D1  ,8);  
	RSI_EGPIO_SetPinMux(EGPIO ,0 , SMIH_D2  ,8);  
	RSI_EGPIO_SetPinMux(EGPIO ,0 , SMIH_D3  ,8);  
	RSI_EGPIO_SetPinMux(EGPIO ,0 , SMIH_CD  ,8); 
	RSI_EGPIO_SetPinMux(EGPIO ,0 , SMIH_WP  ,8);  
	
	 /*Enable clock for EGPIO module*/
  RSI_CLK_PeripheralClkEnable(M4CLK, EGPIO_CLK, ENABLE_STATIC_CLK);
	
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
	/*Configures the system default clock and power configurations*/
	SystemCoreClockUpdate();
#ifdef SDIO_VCC_1P8_V	
	//! Sel-18 NPSS GPIO Bit for 1.8V
  ULP_SPI_MEM_MAP(iPMU_SPARE_REG1_OFFSET) |=  BIT(13);
  //! Enabling SDIO pads operating voltage to 1.8v
  NPSS_IO_VOLTAGE_REG |= BIT(3);  
#endif
	/*Systic Config*/
	SysTick_Config(SOC_OPER_FREQUENCY_SIM / 1000);

	//!Before switching to higher clocks moving clock 
	//to reference clock
	RSI_CLK_M4SocClkConfig(M4CLK,M4_ULPREFCLK,0);
	/*Configure Source clock and  module clock for SDMEM */
	RSI_CLK_SetSocPllFreq(M4CLK,100000000,40000000);
  //! Setting M4SOC clock 100MHz
	RSI_CLK_M4SocClkConfig(M4CLK,M4_SOCPLLCLK,0);
  //! Setting SDMEM clock 50MHz
	RSI_CLK_SdMemClkConfig(M4CLK ,1,SDMEM_SOCPLLCLK,1);
	/*Initialize GPIO pins */
	M4_SDIOH_PinMux();
	
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

void rsi_switch_to_high_clk_freq()
{

}



