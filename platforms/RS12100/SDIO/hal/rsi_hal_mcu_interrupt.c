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
 * Includes
 */
#include "rsi_driver.h"
#include "RS1xxxx.h"
#include "rsi_smih.h"
#include "rsi_board.h"
#include "rsi_chip.h"

volatile uint8_t sdio_init_done;
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
  //! Configure interrupt pin/register in input mode and register the interrupt handler
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
	SMIH->SMIH_NORMAL_INTERRUPT_STATUS_ENABLE_REGISTER |= CARD_INTERRUPT_STATUS_ENABLE;
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
 * @fn          void rsi_hal_intr_pin_status(void)
 * @brief       Checks the SPI interrupt at pin level
 * @param[in]   none  
 * @param[out]  uint8_t, interrupt status 
 * @return      none
 * @description This API is used to check interrupt pin status(pin level whether it is high/low).
 */	
uint8_t rsi_hal_intr_pin_status(void)
{

  volatile uint8_t status = 0;
  
  /* Check if SDIO initialization is done  */
  if (sdio_init_done)
  {
      //! Read interrupt pin  status(high(1) /low (0))
	  status = RSI_EGPIO_GetPin(EGPIO,0,SDIO_D1_PIN);
	}
  
  return status;
	
	
}

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

void smih_callback_handler(uint32_t event)
  {  
    switch(event)
    {
     case COMMAND_COMPLETE   :
       break;
     case TRANSFER_COMPLETE  :
       break;
      case CARD_INSERTION     :
       break;    
      case CARD_REMOVAL       :
        break;
      case CARD_INTERRUPT     :
				  rsi_set_event(RSI_RX_EVENT);
          //! Unmask/Enable the interrupt
					SMIH->SMIH_NORMAL_INTERRUPT_STATUS_ENABLE_REGISTER &= ~CARD_INTERRUPT_STATUS_ENABLE;
        break;
    }  
  }
/*===================================================*/
/**
 * @fn           rsi_reg_flags_t rsi_hal_critical_section_entry(void)
 * @brief        hold interrupt status and disables the SPI interrupt
 * @param[in]    none  
 * @param[out]   none
 * @return       stored interrupt status
 * @description  This HAL API should contain the code to hold interrupt status and disable interrupts.
 */
uint32_t rsi_hal_critical_section_entry(void)
{
	// hold interrupt status before entering critical section
	
	// disable interrupts	

	// return stored interrupt status
	return 0;
}

/*===================================================*/
/**
 * @fn           void rsi_hal_critical_section_exit(void)
 * @brief        Enables the SPI interrupt
 * @param[in]    none  
 * @param[out]   none
 * @return       none
 * @description  This HAL API should contain the code to enable interrupts.
 */
void rsi_hal_critical_section_exit(void)
{
	// restore interrupts while exiting critical section
}
