/*******************************************************************************
* @file  rsi_rng.h
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

#ifndef __RSI_RNG_H__
#define __RSI_RNG_H__

#ifdef __cplusplus
extern "C" {
#endif

#include "rsi_ccp_common.h"
#include "base_types.h"

/******************************************************
 * *                      Macros
 * ******************************************************/
#define RSI_RNG_TRUE_RANDOM   0
#define RSI_RNG_PSEUDO_RANDOM 1
/******************************************************
 * *                    Constants
 * ******************************************************/
/******************************************************
 * *                   Enumerations
 * ******************************************************/
/******************************************************
 * *                 Type Definitions
 * ******************************************************/
/******************************************************
 * *                    Structures
 * ******************************************************/
/******************************************************
 * *                 Global Variables
 * ******************************************************/
/******************************************************
 * *               Function Declarations
 * ******************************************************/
uint32_t rng_start(HWRNG_Type *pRNG, uint8_t rng_mode);
void rng_stop(HWRNG_Type *pRNG);
void rng_get_bytes(HWRNG_Type *pRNG, uint32_t *random_bytes, uint32_t number_of_bytes);

#define HWRNG_CLK_ENABLE 0x400000

#ifdef __cplusplus
}
#endif

#endif // __RSI_RNG_H__
