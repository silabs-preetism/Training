/*******************************************************************************
* @file  rsi_hal_mcu_interrupt.c
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
 * @file       rsi_hal_mcu_interrupt.c
 * @version    0.1
 * @date       12 dec 2018
 *
 *
 *
 * @brief HAL INTERRUPT: Functions related to HAL Interrupts
 * 
 * @section Description
 * This file contains the list of functions for configuring the microcontroller interrupts. 
 * Following are list of API's which need to be defined in this file.
 *
 */


/**
 * Includes
 */
#include "rsi_driver.h"
#include "rsi_chip.h"
#include "rsi_board.h"

/* Private typedef ------------------------------------------------------------------------------------------------*/

/* Private define -------------------------------------------------------------------------------------------------*/

/* Private macro --------------------------------------------------------------------------------------------------*/
typedef void (* UserIntCallBack_t)(void);

UserIntCallBack_t call_back;

#define PININT_IRQ_HANDLER         IRQ059_Handler              /* GPIO interrupt IRQ function name            */ 
#define PININT_NVIC_NAME           EGPIO_PIN_7_IRQn            /* GPIO interrupt NVIC interrupt name          */ 
#define PIN_INT                    7 
#define M4_GPIO_PIN                25

/*=========================================================*/
/**
 * @fn          void PININT_IRQ_HANDLER(void)
 * @brief       Call back handler for the M4 GPIO pin
 * @param[in]   none 
 * @param[out]  none
 * @return      none
 * @section description
 * This function is the call back handler for M4 GPIO pin 
 *
 */

void PININT_IRQ_HANDLER(void)
{
  uint32_t gintStatus;
  /*get interrupt status*/
  gintStatus=RSI_EGPIO_GetIntStat(EGPIO,PIN_INT);
  RSI_EGPIO_IntMask(EGPIO , PIN_INT);
  rsi_set_event(RSI_RX_EVENT);
  return ;
}


/*===================================================*/
/**
 * @fn           void rsi_hal_intr_config(void (* rsi_interrupt_handler)())
 * @brief        Starts and enables the SPI interrupt
 * @param[in]    rsi_interrupt_handler() ,call back function to handle interrupt
 * @param[out]   none
 * @return       none
 * @description  This HAL API should contain the code to initialize the register/pins
 *               related to interrupts and enable the interrupts.
 */
void rsi_hal_intr_config(void (* rsi_interrupt_handler)(void))
{

  /*Enable clock for EGPIO module*/
  RSI_CLK_PeripheralClkEnable(M4CLK,EGPIO_CLK,ENABLE_STATIC_CLK);

  /*PAD selection*/
  RSI_EGPIO_HostPadsGpioModeEnable(25);
  /*PAD selection*/
  RSI_EGPIO_HostPadsGpioModeEnable(26);

  /*REN enable */
  RSI_EGPIO_PadReceiverEnable(M4_GPIO_PIN);

  /*Configure default GPIO mode(0) */
  RSI_EGPIO_SetPinMux(EGPIO,0 ,M4_GPIO_PIN,EGPIO_PIN_MUX_MODE0);

  /*Selects the pin interrupt for the GPIO*/
  RSI_EGPIO_PinIntSel(EGPIO, PIN_INT , 1, 9);

  /*Configures the edge /level interrupt*/
  RSI_EGPIO_SetIntHighLevelEnable(EGPIO,PIN_INT);
  //RSI_EGPIO_SetIntLowLevelEnable(EGPIO,PIN_INT);

  /*Unmask the  interrupt*/
  RSI_EGPIO_IntUnMask(EGPIO , PIN_INT);

  /*NVIC enable */
  NVIC_EnableIRQ(PININT_NVIC_NAME);

  return;	

}


/*===================================================*/
/** 
 * @fn           void rsi_hal_intr_mask(void)
 * @brief        Disables the SPI Interrupt
 * @param[in]    none
 * @param[out]   none
 * @return       none
 * @description  This HAL API should contain the code to mask/disable interrupts.
 */
void rsi_hal_intr_mask(void)
{
  //! Mask/Disable the interrupt 
  return;

}


/*===================================================*/
/**
 * @fn           void rsi_hal_intr_unmask(void)
 * @brief        Enables the SPI interrupt
 * @param[in]    none  
 * @param[out]   none
 * @return       none
 * @description  This HAL API should contain the code to enable interrupts.
 */
void rsi_hal_intr_unmask(void)
{
  //! Unmask/Enable the interrupt
	RSI_EGPIO_IntUnMask(EGPIO , PIN_INT);
  return;

}



/*===================================================*/
/**
 * @fn           void rsi_hal_intr_clear(void)
 * @brief        Clears the pending interrupt
 * @param[in]    none
 * @param[out]   none
 * @return       none
 * @description  This HAL API should contain the code to clear the handled interrupts.
 */
void rsi_hal_intr_clear(void)
{
   //! Clear the interrupt
   return;

}


/*===================================================*/
/**
 * @fn          uint8_t rsi_hal_intr_pin_status(void)
 * @brief       Checks the SPI interrupt at pin level
 * @param[in]   none  
 * @param[out]  uint8_t, interrupt status 
 * @return      none
 * @description This API is used to check interrupt pin status(pin level whether it is high/low).
 */	
uint8_t rsi_hal_intr_pin_status(void)
{

  volatile uint8_t status = 0;

 //! Gets the status of external interrupt GPIO pin
  status = rsi_hal_get_gpio(RSI_HAL_MODULE_INTERRUPT_PIN);
  
  //! Return interrupt pin  status(high(1) /low (0))
  return status;
}

