/*******************************************************************************
* @file  rsi_rng.c
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

/* Includes */

#include "rsi_rom_rng.h"

/*
 This API is used to start the Random Number Generation
*/
uint32_t rng_start(HWRNG_Type *pRNG, uint8_t rngMode)
{
  if (rngMode == 0 || rngMode == 1) {
    if (rngMode == RSI_RNG_TRUE_RANDOM) {
      pRNG->HWRNG_CTRL_REG_b.HWRNG_RNG_ST = 1;
    } else {
      pRNG->HWRNG_CTRL_REG_b.HWRNG_PRBS_ST = 1;
    }
  } else {
    return ERROR_RNG_INVALID_ARG;
  }
  return RSI_OK;
}

/*This API is used to stop the Random Number Generation
*/
void rng_stop(HWRNG_Type *pRNG)
{
  //Disable clock
  pRNG->HWRNG_CTRL_REG = 0;
}

/*This API is used to get the random number bytes
*/
void rng_get_bytes(HWRNG_Type *pRNG, uint32_t *randomBytes, uint32_t numberOfBytes)
{
  uint32_t i;

  for (i = 0; i < numberOfBytes; i++) {
    randomBytes[i] = pRNG->HWRNG_RAND_NUM_REG;
  }
}
const ROM_RNG_API_T rng_api = { &rng_start, &rng_stop, &rng_get_bytes };
