/*******************************************************************************
* @file  main.c
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
 * @file    main.c
 * @version 0.1
 * @date    15 Aug 2015
 *
 *
 *
 *  @brief : This file contains example application for TCP server socket
 *
 *  @section Description  This file contains example application for TCP server socket 
 *
 *
 */

#include "rsi_driver.h"

//! Memory length for driver
#define GLOBAL_BUFF_LEN 15000

//! Wlan client mode
#define RSI_WLAN_CLIENT_MODE 0

//! BT Coex mode
#define RSI_BT_MODE 5

//! Parameter to run forever loop
#define RSI_FOREVER 1

//! Memory to initialize driver
uint8_t global_buf[GLOBAL_BUFF_LEN];

//! Function prototypes
extern void rsi_wlan_app_task(void);
extern void rsi_bt_app_init(void);
extern void rsi_bt_app_task(void);

int32_t rsi_wlan_bt_app(void)
{
  int32_t status = RSI_SUCCESS;

  //! Driver initialization
  status = rsi_driver_init(global_buf, GLOBAL_BUFF_LEN);
  if ((status < 0) || (status > GLOBAL_BUFF_LEN)) {
    return status;
  }

  //! Silabs module intialisation
  status = rsi_device_init(LOAD_NWP_FW);
  if (status != RSI_SUCCESS) {
    return status;
  }

  //! WiSeConnect initialization
  status = rsi_wireless_init(RSI_WLAN_CLIENT_MODE, RSI_BT_MODE);
  if (status != RSI_SUCCESS) {
    return status;
  }

  //! BT initialization
  rsi_bt_app_init();

  while (RSI_FOREVER) {
    //! WLAN application tasks
    rsi_wlan_app_task();

    //! BT application tasks
    rsi_bt_app_task();

    //! wireless driver tasks
    rsi_wireless_driver_task();
  }
  return 0;
}

//! Forever in wireless driver task
void main_loop()
{
  while (RSI_FOREVER) {
    rsi_wireless_driver_task();
  }
}

//! main funtion definition
int main(void)
{
  int32_t status;

  //! Call WLAN BT application
  status = rsi_wlan_bt_app();

  //! Application main loop
  main_loop();

  return status;
}
