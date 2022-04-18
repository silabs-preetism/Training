/*******************************************************************************
* @file  aux_reference_volt_config.h
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

#include "rsi_error.h"
#include "base_types.h"

#ifndef __RSI_AUX_REF_VOLT_CONFIG_H__
#define __RSI_AUX_REF_VOLT_CONFIG_H__
error_t RSI_AUX_RefVoltageConfig(float verf_val, float chip_vltg);
#endif
