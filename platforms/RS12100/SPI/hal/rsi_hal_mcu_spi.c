/*******************************************************************************
* @file  rsi_hal_mcu_spi.c
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
 * @file       rsi_hal_mcu_spi.c
 * @version    0.1
 * @date       11 dec 2018
 *
 *
 *
 * @brief: HAL SPI API
 *
 * @Description:
 * This file Contains all the API's related to HAL 
 *
 */


/**
 * Includes
 */
#include "rsi_driver.h"
#include "GSPI.h"
#include "RTE_Device.h" 
#include "rsi_board.h"
extern ARM_DRIVER_SPI Driver_GSPI_MASTER; 

/**
 * Global Variables
 */



/*==================================================================*/
/**
 * @fn         int16 rsi_spi_cs_deassert(void)
 * @param[in]  None
 * @param[out] None
 * @return     None
 * @description 
 * This API is used to deassert the SPI chip select for SPI interface.
 */
void cs_enable(void)
{
	
	
	
}


/*==================================================================*/
/**
 * @fn         int16 rsi_spi_cs_assert(void)
 * @param[in]  None
 * @param[out] None
 * @return     None
 * @description 
 * This API is used to assert the SPI chip select for SPI interface.
 */

void cs_disable(void)
{
	
	

}


/*==================================================================*/
/**
 * @fn         int16_t rsi_spi_transfer(uint8_t *ptrBuf,uint16_t bufLen,uint8_t *valBuf,uint8_t mode)
 * @param[in]  uint8_t *tx_buff, pointer to the buffer with the data to be transfered
 * @param[in]  uint8_t *rx_buff, pointer to the buffer to store the data received
 * @param[in]  uint16_t transfer_length, Number of bytes to send and receive
 * @param[in]  uint8_t mode, To indicate mode 8 BIT/32 BIT mode transfers.
 * @param[out] None
 * @return     0, 0=success
 * @section description  
 * This API is used to tranfer/receive data to the Wi-Fi module through the SPI interface.
 */
uint8_t dummy[1600];
extern volatile uint8_t spi_done;
int16_t rsi_spi_transfer(uint8_t *tx_buff, uint8_t *rx_buff, uint16_t transfer_length,uint8_t mode)
{
  if(tx_buff == NULL)
	{
		tx_buff = &dummy[0];
	}
	else if(rx_buff == NULL)
	{
		rx_buff = &dummy[0];
	}
  ARM_DRIVER_SPI* SPIdrv = &Driver_GSPI_MASTER;
	 /* SS line = ACTIVE = LOW */
  SPIdrv->Control(ARM_SPI_CONTROL_SS, ARM_SPI_SS_ACTIVE); 
	
	SPIdrv->Transfer(tx_buff, rx_buff, transfer_length);
 		
	/* Waits until spi_done=0 */
	while (!spi_done);	
  spi_done = 0;
 
  /* SS line = ACTIVE = LOW */
  SPIdrv->Control(ARM_SPI_CONTROL_SS, ARM_SPI_SS_INACTIVE);
}



