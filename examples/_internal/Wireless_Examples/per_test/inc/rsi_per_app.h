/*******************************************************************************
* @file  rsi_per_app.h
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
#ifndef RSI_WLAN_PER_H
#define RSI_WLAN_PER_H

#ifdef __cplusplus
extern "C" {
#endif

#include "rsi_bt_common.h"
#include "rsi_bt_common_apis.h"
//! include file to refer data types
#include "rsi_data_types.h"
//! COMMON include file to refer wlan APIs
#include "rsi_common_apis.h"
#include "rsi_bootup_config.h"
//! Error include files
#include "rsi_error.h"
//! OS include file to refer OS specific functionality
#include "rsi_os.h"
#include "rsi_driver.h"
#include "rsi_per_serial.h"

int32_t rsi_send_per_features(void);
int32_t rsi_wlan_per(uint8_t test_state);
int32_t rsi_send_per_features(void);
int32_t rsi_common_command(void);
int32_t rsi_wlan_per_tx_start(void);
int32_t rsi_wlan_per_tx_stop(void);
int32_t rsi_receive_start(void);
int32_t rsi_receive_stop(void);
int32_t rsi_ble_transmit_start(void);
int32_t rsi_ble_transmit_stop(void);
int32_t rsi_ble_receive_start(void);
int32_t rsi_ble_receive_stop(void);
void rsi_ble_stats_receive(void);
int32_t rsi_bt_receive_start(void);
int32_t rsi_bt_receive_stop(void);
int32_t rsi_bt_transmit_start(void);
int32_t rsi_bt_transmit_stop(void);
void rsi_bt_stats_receive(void);

#ifdef __cplusplus
}
#endif

#endif
