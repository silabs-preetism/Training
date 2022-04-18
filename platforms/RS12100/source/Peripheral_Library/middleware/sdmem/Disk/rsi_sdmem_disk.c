/*******************************************************************************
* @file  rsi_sdmem_disk.c
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

#include <stdio.h>
#include <string.h>
#include "rsi_sdmem.h"
#include "rsi_sdmem_disk.h"

SMIH_CARD_CONFIG_T SdmemConfgstruct;

/**
 * @fn         void sdmem_callback_handler(uint32_t event)
 * @brief      This API used to process SDMEM callback event operations.
 * @param[in]  event  : SDMEM callback event handler.
 * @retval     None
 */
void sdmem_callback_handler(uint32_t event)
{
  switch (event) {
    case COMMAND_COMPLETE:
      break;
    case TRANSFER_COMPLETE:
      break;
    case CARD_INSERTION:
      break;
    case CARD_REMOVAL:
      break;
    case CARD_INTERRUPT:
      break;
    case CMD_TIMEOUT_ERROR:
      printf("CMD_TIMEOUT_ERROR occured \n");
      break;
    case CMD_CRC_ERROR:
      printf("CMD_CRC_ERROR occured \n");
      break;
    case CMD_END_BIT_ERROR:
      printf("CMD_END_BIT_ERROR occured \n");
      break;
    case CMD_INDEX_ERROR:
      printf("CMD_INDEX_ERROR occured \n");
      break;
    case DATA_TIMEOUT_ERROR:
      printf("DATA_TIMEOUT_ERROR occured \n");
      break;
    case DATA_END_BIT_ERROR:
      printf("DATA_END_BIT_ERROR occured \n");
      break;
    case DATA_CRC_ERROR:
      printf("DATA_CRC_ERROR occured \n");
      break;
    case CURRENT_LIMIT_ERROR:
      printf("CURRENT_LIMIT_ERROR occured \n");
      break;
    case AUTO_CMD_ERROR:
      printf("AUTO_CMD_ERROR occured \n");
      break;
    case ADMA_ERROR:
      printf("ADMA_ERROR occured \n");
      break;
  }
}

/**
 * @fn         DRESULT sdmem_disk_read(uint8_t Drive, uint8_t *buffer, uint32_t sector, uint8_t count);
 * @brief      This API is used to write the blocks of data in sd memory.
 * @param[in]  physicalDrive  : drive number.
 * @param[out] buffer         : The data buffer pointer to store write data to the sd memory.
 * @param[in]  sector         : The start sector number to be written.
 * @param[in]  count          : No of sectores to write.
 * @retval     RES_ERROR on Failed.
 * @retval     RES_OK on Success.
 */
DRESULT sdmem_disk_write(uint8_t physicalDrive, const uint8_t *buffer, uint32_t sector, uint8_t count)
{

  /*multi block write*/
  if (RSI_OK != RSI_SDMEM_BlocksWrite(&SdmemConfgstruct, buffer, sector, count)) {
    return RES_ERROR;
  }
  return RES_OK;
}

/**
 * @fn         DRESULT sdmem_disk_read(uint8_t Drive, uint8_t *buffer, uint32_t sector, uint8_t count);
 * @brief      This API is used to read the blocks in sd memory.
 * @param[in]  physicalDrive    : drive number.
 * @param[in]  buffer           : The data buffer pointer to store read data from sd memory.
 * @param[in]  sector           : The start sector number to be read.
 * @param[in]  count            : No of sectors to read
 * @retval     RES_ERROR on Failed.
 * @retval     RES_OK    on Success.
 */
DRESULT sdmem_disk_read(uint8_t physicalDrive, uint8_t *buffer, uint32_t sector, uint8_t count)
{

  /*multi block read*/
  if (RSI_OK != RSI_SDMEM_BlocksRead(&SdmemConfgstruct, buffer, sector, count)) {
    return RES_ERROR;
  }
  return RES_OK;
}

/**
 * @fn     DRESULT sdmem_disk_ioctl(uint8_t Drive, uint8_t command, void *buffer);
 * @brief  This API is used to do Disk IO operation.
 * @param  physicalDrive   : drive number.
 * @param  command         : The command to be set.
 * @param  buffer          : The buffer pointer to store command result.
 * @retval RES_PARERR on Failed.
 * @retval RES_OK on Success.
 */
DRESULT sdmem_disk_ioctl(uint8_t physicalDrive, uint8_t command, void *buffer)
{
  DRESULT result = RES_OK;

  switch (command) {
    case GET_SECTOR_COUNT:
      if (buffer) {
        *(uint32_t *)buffer = 7747584; //SdiohConfg.numberOfBlocks;
      } else {
        result = RES_PARERR;
      }
      break;
    case GET_SECTOR_SIZE:
      if (buffer) {
        *(uint32_t *)buffer = SdmemConfgstruct.byteBlockSize;
      } else {
        result = RES_PARERR;
      }
      break;
    case GET_BLOCK_SIZE:
      if (buffer) {
        *(uint32_t *)buffer = 127;
      } else {
        result = RES_PARERR;
      }
      break;
    case CTRL_SYNC:
      result = RES_OK;
      break;
    default:
      result = RES_PARERR;
      break;
  }

  return result;
}

/**
 * @fn         DSTATUS sdmem_disk_status(uint8_t Drive);
 * @brief      This API is used to check the sdmem disk status
 * @param[in]  physicalDrive  : drive number.
 * @retval     STA_NOINIT on Failed.
 * @retval     RES_OK  on Success.
 */
DSTATUS sdmem_disk_status(uint8_t physicalDrive)
{
  if (physicalDrive != DEV_SD) {
    return STA_NOINIT;
  }
  return 0;
}

/**
 * @fn         DSTATUS sdmem_disk_initialize(uint8_t Drive);
 * @brief      This function is used to initialize the sdmem driver
 * @param[in]  physicalDrive : drive number.
 * @retval     STA_NOINIT on Failed.
 * @retval     RES_OK on Success.
 */
DSTATUS sdmem_disk_initialize(uint8_t physicalDrive)
{
  // Clear structure
  memset(&SdmemConfgstruct, 0, sizeof(SdmemConfgstruct));

  SdmemConfgstruct.highSpeedEnable = HIGH_SPEED_EN;
  SdmemConfgstruct.uhsModeSelect   = UHS_NONE; //uhs mode will work only in 4bit mode
  SdmemConfgstruct.admaMode        = ADMA_ENABLE;

#if (_8BIT_MODE)
  SdmemConfgstruct.busWidthMode = SMIH_BUS_WIDTH8;
#elif (_1BIT_MODE)
  SdmemConfgstruct.busWidthMode = SMIH_BUS_WIDTH1;
#else
  SdmemConfgstruct.busWidthMode = SMIH_BUS_WIDTH4;
#endif

  SdmemConfgstruct.clock   = SD_CLOCK;
  SdmemConfgstruct.voltage = __1P8_VOLTAGE_EN; //set this bit in case of 1.8v

  if (MMC_CARDS) {
    /*initialize card*/
    if (RSI_OK != RSI_SDMMC_Enumeration(&SdmemConfgstruct, sdmem_callback_handler)) {
      return RES_ERROR;
    }
  } else {
    /*initialize sd card*/
    if (RSI_OK != RSI_SDMEM_Enumeration(&SdmemConfgstruct, sdmem_callback_handler)) {
      return RES_ERROR;
    }
  }

  /*change clock after enumeratuion*/
  smih_clock_config(&SdmemConfgstruct, SD_CLOCK);

  return RES_OK;
}
