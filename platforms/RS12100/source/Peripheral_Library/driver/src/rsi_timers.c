/*******************************************************************************
* @file  rsi_timers.c
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
/*************************************************************************
 *
 */

/******************************************************************************/
/*                               INCLUDE FILES                                */
/******************************************************************************/
#include "rsi_ccp_user_config.h"
#ifndef ROMDRIVER_PRESENT
#include "rsi_rom_timer.h"

#ifdef __cplusplus
extern "C" {
#endif

/***********************************************************************/
/*											Global Variables 															 */
/***********************************************************************/

/***********************************************************************/
/*										       API DEFINITIONS			      						   */
/***********************************************************************/

/*====================================================================*/
/**
 * @fn          RSI_DRIVER_VERSION RSI_TIMERS_GetVersion(void)
 * @brief		    This API is used to get version information of the driver implementation 
 * @return 		  structure of type RSI_DRIVER_VERSION and its members are as below
 *								- \ref  RSI_TIMERS_API_VERSION : Version of the CMSIS-Driver specification 
 *																			    used to implement this driver.
 *  							- \ref  RSI_TIMERS_DRV_VERSION : TIMERS peripheral source code version of the 
 *																		    actual driver implementation.
 */
RSI_DRIVER_VERSION RSI_TIMERS_GetVersion(void)
{
  RSI_DRIVER_VERSION vsDriverVersion;

  vsDriverVersion.api = RSI_TIMERS_API_VERSION;
  vsDriverVersion.drv = RSI_TIMERS_DRV_VERSION;
  return vsDriverVersion;
}

/**
 * @fn    	    RSI_TIMERS_CAPABILITIES RSI_TIMERS_GetCapabilities(void)
 * @brief		    The function RSI_TIMERS_GetCapabilities returns information about 
 *							capabilities of this driver implementation
 * @return 		  structure of type RSI_TIMERS_CAPABILITIES and its members are as below
 *                  - \ref timerCount   :Number of Timers
 *                  - \ref microsecmode :supports microsecmode or not
 */
RSI_TIMERS_CAPABILITIES_T RSI_TIMERS_GetCapabilities(void)
{
  RSI_TIMERS_CAPABILITIES_T vsDriverCapabilities;

  vsDriverCapabilities.timerCount   = 4;
  vsDriverCapabilities.microSecMode = 1;

  return vsDriverCapabilities;
}

/* This API is used to configure timer mode as a 1 micro second or 256 micro second mode */
error_t timers_microsec_timer_config(RSI_TIMERS_T *pTIMER,
                                     uint8_t timerNum,
                                     uint16_t integer,
                                     uint8_t fractional,
                                     uint8_t mode)
{
  if (timerNum <= TIMER_3) {
    /* System clock if 1TU micro second timer */
    if (mode == MICRO_SEC_MODE) {
      pTIMER->MCUULP_TMR_US_PERIOD_INT_b.TMR_US_PERIOD_INT   = (uint16_t)integer;
      pTIMER->MCUULP_TMR_US_PERIOD_FRAC_b.TMR_US_PERIOD_FRAC = (uint8_t)fractional;
    }
    /* System clock if 256TU micro second timer */
    else if (mode == _256_MICRO_SEC_MODE) {
      pTIMER->MCUULP_TMR_MS_PERIOD_INT_b.TMR_MS_PERIOD_INT   = (uint16_t)integer;
      pTIMER->MCUULP_TMR_MS_PERIOD_FRAC_b.TMR_MS_PERIOD_FRAC = (uint8_t)fractional;
    } else {
      return ERROR_INVAL_TIMER_MODE;
    }
  } else {
    return ERROR_INVAL_TIMER_MODE;
  }
  return RSI_OK;
}

/* This API is used to get the count of the required timer  */
uint32_t timers_read_timer(RSI_TIMERS_T *pTIMER, uint8_t timerNum, boolean_t countDir)
{
  if (countDir == 0) {
    pTIMER->MATCH_CTRL[timerNum].MCUULP_TMR_CNTRL_b.COUNTER_UP = 0;
    return (pTIMER->MATCH_CTRL[timerNum].MCUULP_TMR_MATCH);
  } else if (countDir == 1) {
    pTIMER->MATCH_CTRL[timerNum].MCUULP_TMR_CNTRL_b.COUNTER_UP = 1;
    return (pTIMER->MATCH_CTRL[timerNum].MCUULP_TMR_MATCH);
  } else
    return ERROR_INVAL_COUNTER_DIR;
}

/*ROM API Structure
const ROM_TIMERS_API_T timer_api = {
		&timers_microsec_timer_config,
		&timers_read_timer
};
*/

#ifdef __cplusplus
}
#endif

#endif //ROMDRIVER_PRESENT
/************* EOF ***********/
