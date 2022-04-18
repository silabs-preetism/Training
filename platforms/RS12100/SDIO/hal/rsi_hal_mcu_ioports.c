/*******************************************************************************
* @file  rsi_hal_mcu_ioports.c
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
//! Hardware and powersave related include files
#include "rsi_board.h"
#include "rsi_chip.h"
#include "rsi_ps_ram_func.h"
#include "rsi_wisemcu_hardware_setup.h"


/**
 * Global Variales
 */


//#ifdef GPIO_TOGGLE
#define GPIO_LOW                   0U
#define GPIO_HIGH                  1U
//#endif
#define PORT                       0
#define PIN                        6

/*===========================================================*/
/**
 * @fn            void rsi_hal_config_gpio(uint8_t gpio_number,uint8_t mode,uint8_t value)
 * @brief         Configures gpio pin in output mode,with a value
 * @param[in]     uint8_t gpio_number, gpio pin number to be configured
 * @param[in]     uint8_t mode , input/output mode of the gpio pin to configure
 *                0 - input mode
 *                1 - output mode
 * @param[in]     uint8_t value, default value to be driven if gpio is configured in output mode
 *                0 - low
 *                1 - high
 * @param[out]    none
 * @return        none
 * @description This API is used to configure host gpio pin in output mode. 
 */
void rsi_hal_config_gpio(uint8_t gpio_number,uint8_t mode,uint8_t value)
{
	RSI_EGPIO_SetPinMux(EGPIO1 ,PORT , PIN ,EGPIO_PIN_MUX_MODE0);
	//! Set output direction 
	RSI_EGPIO_SetDir(EGPIO1 ,PORT ,PIN, EGPIO_CONFIG_DIR_OUTPUT);

        //! Drive a default value on gpio if gpio is configured in output mode
#ifndef XTAL_CLK_PS_OPT2	
	RSI_NPSSGPIO_InputBufferEn(NPSS_GPIO_3 , 1U);
	RSI_NPSSGPIO_SetPinMux(NPSS_GPIO_3 , 0);
	RSI_NPSSGPIO_SetDir(NPSS_GPIO_3, NPSS_GPIO_DIR_INPUT);
#else	
	RSI_NPSSGPIO_InputBufferEn(NPSS_GPIO_0 , 1U);
	RSI_NPSSGPIO_SetPinMux(NPSS_GPIO_0 , 0);
	RSI_NPSSGPIO_SetDir(NPSS_GPIO_0, NPSS_GPIO_DIR_INPUT);
#endif	
	RSI_NPSSGPIO_InputBufferEn(NPSS_GPIO_2 , 1U);
	RSI_NPSSGPIO_SetPinMux(NPSS_GPIO_2 , 0);
	RSI_NPSSGPIO_SetDir(NPSS_GPIO_2, NPSS_GPIO_DIR_OUTPUT);
	rsi_allow_sleep();  
  
   return;


}



/*===========================================================*/
/**
 * @fn            void rsi_hal_set_gpio(uint8_t gpio_number)
 * @brief         Makes/drives the gpio  value high
 * @param[in]     uint8_t gpio_number, gpio pin number
 * @param[out]    none
 * @return        none 
 * @description   This API is used to drives or makes the host gpio value high. 
 */


void rsi_hal_set_gpio(uint8_t gpio_number)
{
	if(gpio_number ==  RSI_HAL_RESET_PIN)
	{
	  RSI_EGPIO_SetPin(EGPIO1 ,PORT,PIN ,GPIO_HIGH);
	}
	
	if(gpio_number ==  RSI_HAL_SLEEP_CONFIRM_PIN)	
	{
	  RSI_NPSSGPIO_SetPin(NPSS_GPIO_2,GPIO_HIGH);
	}
	
	if(gpio_number ==  RSI_HAL_WAKEUP_INDICATION_PIN)
	{
#ifndef XTAL_CLK_PS_OPT2			
	  RSI_NPSSGPIO_SetPin(NPSS_GPIO_3,GPIO_HIGH);
#else		
		RSI_NPSSGPIO_SetPin(NPSS_GPIO_0,GPIO_HIGH);
#endif
	}
	
	if(gpio_number ==  RSI_HAL_LP_SLEEP_CONFIRM_PIN)
	{
	  RSI_NPSSGPIO_SetPin(NPSS_GPIO_2,GPIO_HIGH);
	}
	
  //! drives a high value on GPIO 
  
  
  return;
}




/*===========================================================*/
/**
 * @fn          uint8_t rsi_hal_get_gpio(void)
 * @brief       get the gpio pin value
 * @param[in]   uint8_t gpio_number, gpio pin number
 * @param[out]  none  
 * @return      gpio pin value 
 * @description This API is used to configure get the gpio pin value. 
 */
uint8_t rsi_hal_get_gpio(uint8_t gpio_number)
{
  volatile uint8_t gpio_value = 0;

//  //! Get the gpio value

	if(gpio_number == RSI_HAL_SLEEP_CONFIRM_PIN)
	{
	 gpio_value =  RSI_NPSSGPIO_GetPin(NPSS_GPIO_2);
	}

	if(gpio_number == RSI_HAL_WAKEUP_INDICATION_PIN)	
	{
#ifndef XTAL_CLK_PS_OPT2			
	 gpio_value =  RSI_NPSSGPIO_GetPin(NPSS_GPIO_3);
#else		
	 gpio_value =  RSI_NPSSGPIO_GetPin(NPSS_GPIO_0);
#endif		
	}

	if(gpio_number ==  RSI_HAL_LP_SLEEP_CONFIRM_PIN)
	{
	  gpio_value =  RSI_NPSSGPIO_GetPin(NPSS_GPIO_2);
	}
	if(gpio_number ==  RSI_HAL_MODULE_INTERRUPT_PIN)
	{
		 gpio_value = RSI_EGPIO_GetPin(EGPIO,0,SDIO_D1_PIN);		
	}
	
  return gpio_value;

}




/*===========================================================*/
/**
 * @fn            void rsi_hal_set_gpio(uint8_t gpio_number)
 * @brief         Makes/drives the gpio value to low
 * @param[in]     uint8_t gpio_number, gpio pin number
 * @param[out]    none
 * @return        none 
 * @description   This API is used to drives or makes the host gpio value low. 
 */
void rsi_hal_clear_gpio(uint8_t gpio_number)
{
		//! drives a low value on GPIO 
	if(gpio_number ==  RSI_HAL_RESET_PIN)
	{
	  	RSI_EGPIO_SetPin(EGPIO1 ,PORT,PIN ,GPIO_LOW);
	}
  //! drives a low value on GPIO 
	if(gpio_number ==  RSI_HAL_SLEEP_CONFIRM_PIN)
	{
	  RSI_NPSSGPIO_SetPin(NPSS_GPIO_2,GPIO_LOW);
	}

	if(gpio_number ==  RSI_HAL_WAKEUP_INDICATION_PIN)
	{
#ifndef XTAL_CLK_PS_OPT2			
	  RSI_NPSSGPIO_SetPin(NPSS_GPIO_3,GPIO_LOW);
#else
		RSI_NPSSGPIO_SetPin(NPSS_GPIO_0,GPIO_LOW);
#endif
	}
	
	if(gpio_number ==  RSI_HAL_LP_SLEEP_CONFIRM_PIN)
	{
	  RSI_NPSSGPIO_SetPin(NPSS_GPIO_2,GPIO_LOW);
	}
	
	
	return;
}


