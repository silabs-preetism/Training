/*******************************************************************************
* @file  rsi_processor_sensor.h
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

#ifndef __RSI_PROCESSOR_SENSOR_H__
#define __RSI_PROCESSOR_SENSOR_H__

#ifdef __cplusplus
extern "C" {
#endif

/**
 * \ingroup   RSI_SPECIFIC_DRIVERS
 * \defgroup RSI_PROCESSOR_SENSOR RSI:RS1xxxx PROCESSOR_SENSOR 
 *  @{
 *
 */
#include "rsi_ccp_common.h"

error_t RSI_ProSense_Enable(MCU_ProcessSensor_t *pstcProcessSensor, boolean_t bEN);
uint32_t RSI_ProSense_Read(MCU_ProcessSensor_t *pstcProcessSensor);
uint32_t RSI_ProSense_GetNumCycles(MCU_ProcessSensor_t *pstcProcessSensor);
error_t RSI_ProSense_ClkEnable(MCU_ProcessSensor_t *pstcProcessSensor, boolean_t bEN);
error_t RSI_ProSense_RingClkStart(MCU_ProcessSensor_t *pstcProcessSensor, boolean_t bEN);

#ifdef __cplusplus
}
#endif

/**
 * @}
 */

#endif /*__RSI_PROCESSOR_SENSOR_H__*/
