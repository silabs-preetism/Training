/*******************************************************************************
* @file  rsi_usart.h
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
 * @file       rsi_usart.h
 * @version    1.0
 * @date       1 AUG,2016
 *
 *
 *
 * @brief This files contains functions prototypes related to EGPIO peripheral
 * 
 * @section Description
 * This file contains the list of function prototypes for the usart and low level function definitions
 * Following are list of API's which need to be defined in this file.
 *
 */

/**
 * Includes
 */
#ifndef __RSI_USART_H__
#define __RSI_USART_H__

#ifdef __cplusplus
extern "C" {
#endif

#include "USART.h"
#include "rsi_udma_wrapper.h"

//prototypes
void UartIrqHandler(USART_RESOURCES *usart);
void USART_UDMA_Tx_Event(uint32_t event, uint8_t dmaCh, USART_RESOURCES *usart);
void USART_UDMA_Rx_Event(uint32_t event, uint8_t dmaCh, USART_RESOURCES *usart);
int32_t USART_SetBaudrate(uint32_t baudrate, uint32_t baseClk, USART_RESOURCES *usart);
int32_t USART_Initialize(ARM_USART_SignalEvent_t cb_event,
                         USART_RESOURCES *usart,
                         UDMA_RESOURCES *udma,
                         RSI_UDMA_DESC_T *UDMA_Table,
                         RSI_UDMA_HANDLE_T *udmaHandle,
                         uint32_t *mem);
int32_t USART_Uninitialize(USART_RESOURCES *usart, UDMA_RESOURCES *udma);
int32_t USART_PowerControl(ARM_POWER_STATE state,
                           USART_RESOURCES *usart,
                           UDMA_RESOURCES *udma,
                           RSI_UDMA_HANDLE_T udmaHandle);
int32_t USART_Send_Data(const void *data,
                        uint32_t num,
                        USART_RESOURCES *usart,
                        UDMA_RESOURCES *udma,
                        UDMA_Channel_Info *chnl_info,
                        RSI_UDMA_HANDLE_T udmaHandle);
int32_t USART_Receive_Data(const void *data,
                           uint32_t num,
                           USART_RESOURCES *usart,
                           UDMA_RESOURCES *udma,
                           UDMA_Channel_Info *chnl_info,
                           RSI_UDMA_HANDLE_T udmaHandle);
int32_t USART_Transfer(const void *data_out,
                       void *data_in,
                       uint32_t num,
                       USART_RESOURCES *usart,
                       UDMA_RESOURCES *udma,
                       UDMA_Channel_Info *chnl_info,
                       RSI_UDMA_HANDLE_T udmaHandle);
uint32_t USART_GetTxCount(USART_RESOURCES *usart);
uint32_t USART_GetRxCount(USART_RESOURCES *usart);
int32_t USART_Control(uint32_t control,
                      uint32_t arg,
                      uint32_t baseClk,
                      USART_RESOURCES *usart,
                      UDMA_RESOURCES *udma,
                      RSI_UDMA_HANDLE_T udmaHandle);
ARM_USART_STATUS USART_GetStatus(USART_RESOURCES *usart);
int32_t USART_SetModemControl(ARM_USART_MODEM_CONTROL control, USART_RESOURCES *usart);
ARM_USART_MODEM_STATUS USART_GetModemStatus(USART_RESOURCES *usart);

#endif /* __RSI_USART_H__*/
