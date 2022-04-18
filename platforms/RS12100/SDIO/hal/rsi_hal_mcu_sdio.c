/*******************************************************************************
* @file  rsi_hal_mcu_sdio.c
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
#include "rsi_pkt_mgmt.h"
#include "rsi_chip.h"

#include <string.h>


#ifndef SPAN_BAORD
#include "rsi_pll.h"
#endif

#include "rsi_driver.h"
#include "base_types.h"

extern SMIH_CARD_CONFIG_T      pSmihConfig ;
extern volatile uint8_t sdio_init_done;

//extern en_result_t Sdio_Block_Byte_Write_reg(stc_sdcard_config_t *pstcCfg,uint8_t *pu8BuffIn,uint16_t Addr);


/*==============================================*/
/**
 * @fn          void rsi_sdio_write_multiple(uint8_t *tx_data, uint32_t Addr, uint16_t no_of_blocks)
 * @brief       This API is used to write the packet on to the SDIO interface in block mode.
 * @param[in]   tx_data is the buffer to be written to sdio.
 * @param[in]   Addr of the mem to which the data has to be written.
 * @param[in]   no_of_blocks is the blocks present to be transfered.
 * @return      0 in case of success ,*              - 1 in case of failure
 * @section description 
 *  This API is used to write the packet on to the SDIO interface
 * 
 *
 */
int16_t rsi_sdio_write_multiple(uint8_t *tx_data, uint32_t Addr, uint16_t no_of_blocks)
{
	pSmihConfig.opCode             = 0;
	pSmihConfig.blockModeEnable    = 1; 
	pSmihConfig.funcNum            = 1;
	pSmihConfig.byteBlockSize      = 256;
	pSmihConfig.numberOfBlocks     = no_of_blocks;	
	
  return RSI_SDIOH_ByteBlockWriteCmd53(&pSmihConfig,(uint8_t *)tx_data, Addr);
}

/*==============================================*/
/**
 * @fn          uint8_t rsi_sdio_read_multiple(uint8_t *read_buff, uint32_t Addr)
 * @brief       API is used to read no of bytes in blocked mode from device.
 * @param[in]   read_buff is the buffer to be stored with the data read from device.
 * @param[in]   Addr of the mem to be read.
 * @return      0 in case of success ,*              - 1 in case of failure
 * @section description This function gets the packet coming from the module and 
 * copies to the buffer pointed 
 * 
 *
 */

int8_t rsi_sdio_read_multiple(uint8_t *read_buff, uint32_t no_of_blocks)
{
	uint32_t Addr = 0;
	pSmihConfig.opCode             = 0;
	pSmihConfig.blockModeEnable    = 1; 
	pSmihConfig.funcNum            = 1;
	pSmihConfig.byteBlockSize      = 256;
  pSmihConfig.numberOfBlocks     = no_of_blocks;	

	Addr = pSmihConfig.byteBlockSize * pSmihConfig.numberOfBlocks;
	
	return RSI_SDIOH_ByteBlockReadCmd53(&pSmihConfig, read_buff, Addr);
}

/*==============================================*/
/**
 * @fn          uint8_t sdio_reg_writeb(uint32_t Addr, uint8_t *dBuf)
 * @brief       API is used to write 1 byte of data to sdio slave register space.
 * @param[in]   Addr of the reg to be written.
 * @param[in]   Buffer of data to be written to sdio slave reg.
 * @return      0 in case of success ,*              - 1 in case of failure
 * @section description This function writes 1 byte of data to the slave device 
 * 
 *
 */
int8_t sdio_reg_writeb(uint32_t Addr, uint8_t *dBuf)
{
    uint32_t cmd_arg;

    cmd_arg = (1 << 31) | ((Addr & 0x1ffff) << 9) | dBuf[0];

   	return RSI_SDIOH_ReadCommandCmd52(&pSmihConfig, cmd_arg);
}

/*==============================================*/
/**
 * @fn          uint8_t sdio_reg_readb(uint32_t Addr, uint8_t *dBuf)
 * @brief       API is used to read 1 byte of data from sdio slave register space.
 * @param[in]   Addr of the reg to be read.
 * @param[in]   Buffer of data to be read from sdio slave reg.
 * @return      0 in case of success ,*              - 1 in case of failure
 * @section description This function gets the 1 byte of data from the slave device 
 * 
 *
 */
int8_t sdio_reg_readb(uint32_t Addr, uint8_t *dBuf)
{
    uint32_t cmd_arg;
    error_t   Stat             = RSI_OK;

    cmd_arg = ((Addr & 0x1ffff) << 9);

   	Stat = RSI_SDIOH_ReadCommandCmd52(&pSmihConfig, cmd_arg);
    *dBuf = pSmihConfig.response[0];

    return Stat;
}

/*==============================================*/
/**
 * @fn          int16_t rsi_sdio_readb(uint32_t addr, uint16_t len, uint8_t *dBuf)
 * @brief       API is used to read n bytes of data from device space in byte mode.
 * @param[in]   Addr of the data to be read.
 * @param[in]   Buffer of data to be read from sdio device.
 * @return      0 in case of success ,*              - 1 in case of failure
 * @section description This function gets the n  bytes of data from the device
 * 
 *
 */
int16_t rsi_sdio_readb(uint32_t addr, uint16_t len, uint8_t *dBuf)
{
   pSmihConfig.blockModeEnable = 0;
	 pSmihConfig.funcNum = 1;
	 pSmihConfig.opCode = 0;
	 pSmihConfig.byteBlockSize = len;
	 pSmihConfig.numberOfBlocks = 1;

	 return RSI_SDIOH_ByteBlockReadCmd53(&pSmihConfig, dBuf, addr);
}

/*==============================================*/
/**
 * @fn          int16_t rsi_sdio_writeb(uint32_t addr, uint16_t len, uint8_t *dBuf)
 * @brief       API is used to write n bytes of data to device space in byte mode.
 * @param[in]   Addr of the data to be written.
 * @param[in]   Buffer of data to be written to sdio device.
 * @return      0 in case of success ,*              - 1 in case of failure
 * @section description This function writes the n bytes of data to the device
 * 
 *
 */
int16_t rsi_sdio_writeb(uint32_t addr, uint16_t len, uint8_t *dBuf)
{
   pSmihConfig.funcNum            = 1;
   pSmihConfig.blockModeEnable    = 0; 
   pSmihConfig.opCode             = 0;
   pSmihConfig.byteBlockSize      = len;
   pSmihConfig.numberOfBlocks     = 1;	//!Invalid in non block mode

	 return RSI_SDIOH_ByteBlockWriteCmd53(&pSmihConfig, dBuf, addr);
}
/*=============================================*/
/**
 * @fn                  int16_t rsi_sdio_init(void)
 * @brief               Start the SDIO interface
 * @param[in]           none
 * @param[out]          none
 * @return              errCode
 * @section description         
 * This API initializes the Wi-Fi modules Slave SDIO interface.
 */
int32_t rsi_mcu_sdio_init(void)
{
  int32_t status;
	SMIH_CARD_CONFIG_T *SmihInfo = NULL;

	/* At this stage the microcontroller clock setting is already configured,
	 * this is done through SystemInit() function which is called from startup
	 * file (startup_rs9116.s) before to branch to application main.
	 * To reconfigure the default setting of SystemInit() function, refer to
	 * system_rs9116.c file
	 */
	error_t              Stat           = RSI_OK;
	uint32_t              i                 = 0;
	int forever = 1;

//	MEMORY_CLEAR(pSmihConfig);
	pSmihConfig.highSpeedEnable    = 1;
	pSmihConfig.uhsModeSelect      = UHS_NONE;      //uhs mode will work only in 4bit mode
	pSmihConfig.admaMode           = 1;
	pSmihConfig.busWidthMode       = SMIH_BUS_WIDTH4;
	pSmihConfig.clock              = 50000000;
#ifdef SDIO_VCC_1P8_V 
	pSmihConfig.voltage            = 1; //set this bit in case of 1.8v
#else
	pSmihConfig.voltage            = 0; //set this bit in case of 1.8v
#endif
	pSmihConfig.funcNum            = 1;
	pSmihConfig.blockModeEnable    = 1; //enable in case of block mode
	pSmihConfig.opCode             = 0;
	pSmihConfig.byteBlockSize      = 256;
	pSmihConfig.numberOfBlocks     = 1;

	/*Smih IO Card Initialization*/
	Stat = RSI_SDIOH_Enumeration(&pSmihConfig,smih_callback_handler);
	if(Stat == RSI_OK)
	{
		Stat = smih_clock_config(&pSmihConfig,50000000);
	}
	else
	{
    return Stat;
	}
     /*Indicate SDIO interface init is done  */
	sdio_init_done =1;
	
	return RSI_OK;

}
