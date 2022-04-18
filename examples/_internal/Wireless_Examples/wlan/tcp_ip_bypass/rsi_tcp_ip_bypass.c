/*******************************************************************************
* @file  rsi_tcp_ip_bypass.c
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
 * Include files
 * */

//! Include file to refer user configurable details
#include "rsi_wlan_config.h"

//! include file to refer data types
#include "rsi_data_types.h"

//! COMMON include file to refer wlan APIs
#include "rsi_common_apis.h"

//! WLAN include file to refer wlan APIs
#include "rsi_wlan_apis.h"
#include "rsi_wlan_non_rom.h"

//! socket include file to refer socket APIs
#include "rsi_socket.h"

//! Include file to refer the bootup configurations
#include "rsi_bootup_config.h"

//! Error include files
#include "rsi_error.h"

#include <stdlib.h>
//! OS include file to refer OS specific functionality
#include "rsi_os.h"

extern int16_t rsi_update_info(uint8_t type);

//! Access point SSID to connect
#define SSID "SILABS_AP"

//!Scan Channel number , 0 - to scan all channels
#define CHANNEL_NO 0

//! Security type
#define SECURITY_TYPE RSI_OPEN

//! Password
#define PSK NULL

//! Assigning static IP to the module
#define RSI_MODULE_IP_ADDRESS "192.168.100.76" //@ IP Address of the WiFi Module

//! Assigning interface to the module
#define SET_WLAN0_IP system("ifconfig rsi_wlan0 " RSI_MODULE_IP_ADDRESS)

//! Updating join done
#define UPDATE_JOIN_DONE 0x02

//! Memory length for driver
#define GLOBAL_BUFF_LEN 15000

//! Wlan task priority
#define RSI_WLAN_TASK_PRIORITY 1

//! Wireless driver task priority
#define RSI_DRIVER_TASK_PRIORITY 2

//! Wlan task stack size
#define RSI_WLAN_TASK_STACK_SIZE 500

//! Wireless driver task stack size
#define RSI_DRIVER_TASK_STACK_SIZE 500

//! Memory to initialize driver
uint8_t global_buf[GLOBAL_BUFF_LEN];

int32_t rsi_tcp_ip_bypass()
{
  int32_t status = RSI_SUCCESS;

  //! WC initialization
  status = rsi_wireless_init(0, 0);
  if (status != RSI_SUCCESS) {
    return status;
  }

  //! Scan for Access points
  status = rsi_wlan_scan((int8_t *)SSID, (uint8_t)CHANNEL_NO, NULL, 0);
  if (status != RSI_SUCCESS) {
    return status;
  }

  //! Connect to an Access point
  status = rsi_wlan_connect((int8_t *)SSID, SECURITY_TYPE, PSK);
  if (status != RSI_SUCCESS) {
    return status;
  }

  //! Sending join done info to kernel netdevice driver
  status = rsi_update_info(UPDATE_JOIN_DONE);
  if (status != RSI_SUCCESS) {
    return status;
  }
  //! Giving IP address to SiLabs Wireless interface(rsi_wlan0)
  SET_WLAN0_IP;
  return RSI_SUCCESS;
}

void main_loop(void)
{
  while (1) {
    ////////////////////////
    //! Application code ///
    ////////////////////////

    //! event loop
    rsi_wireless_driver_task();
  }
}

int main()
{
  int32_t status;
#ifdef RSI_WITH_OS

  rsi_task_handle_t wlan_task_handle = NULL;

  rsi_task_handle_t driver_task_handle = NULL;
#endif

  //! Driver initialization
  status = rsi_driver_init(global_buf, GLOBAL_BUFF_LEN);
  if ((status < 0) || (status > GLOBAL_BUFF_LEN)) {
    return status;
  }

  //! SiLabs module intialisation
  status = rsi_device_init(LOAD_NWP_FW);
  if (status != RSI_SUCCESS) {
    return status;
  }

#ifdef RSI_WITH_OS
  //! OS case
  //! Task created for WLAN task
  rsi_task_create((rsi_task_function_t)rsi_tcp_ip_bypass,
                  (uint8_t *)"wlan_task",
                  RSI_WLAN_TASK_STACK_SIZE,
                  NULL,
                  RSI_WLAN_TASK_PRIORITY,
                  &wlan_task_handle);

  //! Task created for Driver task
  rsi_task_create((rsi_task_function_t)rsi_wireless_driver_task,
                  (uint8_t *)"driver_task",
                  RSI_DRIVER_TASK_STACK_SIZE,
                  NULL,
                  RSI_DRIVER_TASK_PRIORITY,
                  &driver_task_handle);

  //! OS TAsk Start the scheduler
  rsi_start_os_scheduler();

#else
  //! NON - OS case
  //! Call TCPIP bypass application
  status = rsi_tcp_ip_bypass();

  //! Application main loop
  main_loop();
#endif
  return status;
}
