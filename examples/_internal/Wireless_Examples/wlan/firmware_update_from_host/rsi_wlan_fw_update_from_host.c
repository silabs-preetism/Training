/*******************************************************************************
* @file  rsi_wlan_fw_update_from_host.c
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

//! socket include file to firmware upgrade APIs

#include "stdlib.h"
#include "rsi_driver.h"
#include "ff.h"
#include "diskio.h"
#include "rsi_wlan_config.h"
#include "rsi_firmware_upgradation.h"
//! include file to refer data types
#include "rsi_data_types.h"
#include "fsl_debug_console.h"

uint8_t app_global_buf[GLOBAL_BUFF_LEN];

//! Enumeration for states in application
typedef enum rsi_wlan_app_state_e {
  RSI_WLAN_INITIAL_STATE    = 0,
  RSI_WLAN_RADIO_INIT_STATE = 1,
  RSI_WLAN_FW_UPGRADE       = 2,
  RSI_WLAN_FW_UPGRADE_DONE  = 3
} rsi_wlan_app_state_t;

//! wlan application control block
typedef struct rsi_wlan_app_cb_s {
  //! wlan application state
  rsi_wlan_app_state_t state;

  //! length of buffer to copy
  uint32_t length;

  //! application buffer
  uint8_t buffer[RSI_APP_BUF_SIZE];

  //! to check application buffer availability
  uint8_t buf_in_use;

  //! application events bit map
  uint32_t event_map;

} rsi_wlan_app_cb_t;

//! extern functions

//! application control block
rsi_wlan_app_cb_t rsi_wlan_app_cb;

#define FW_FILE_NAME "/2.rps"
uint8_t header = 0;
uint16_t fwup_chunk_length;
//! SD Card parameters
UINT bytesRead;
FIL g_fileObject_n; /* File object */
uint8_t recv_buffer[1500] = { 0 }, fw[20] = { 0 };

//! Create a new file on SD card
int8_t openFile_sd()
{
  FRESULT error;

  //! Create a file on SD card
  error = f_open(&g_fileObject_n, _T(FW_FILE_NAME), (FA_WRITE | FA_READ | FA_OPEN_EXISTING));
  if (error) {
    PRINTF("Open file failed.\r\n");
    return -1;
  }

  return 0;
}

int32_t rsi_app_task_fw_update_from_host(void)
{
  int32_t status = RSI_SUCCESS;
  FRESULT error;
#ifdef RSI_WITH_OS
  //!  module initialization
  status = rsi_device_init(LOAD_NWP_FW);
  if (status != RSI_SUCCESS) {
    return status;
  }

  //! WiSeConnect initialization
  status = rsi_wireless_init(RSI_WLAN_MODE, RSI_COEX_MODE);
  if (status != RSI_SUCCESS) {
    return status;
  }

  //! Send Feature frame
  status = rsi_send_feature_frame();
  if (status != RSI_SUCCESS) {
    return status;
  }
  while (1)
#endif
  {
    switch (rsi_wlan_app_cb.state) {
      case RSI_WLAN_INITIAL_STATE: {
        PRINTF("DEMO STARTED\n");
        //! update wlan application state
        rsi_wlan_app_cb.state = RSI_WLAN_RADIO_INIT_STATE;
      }
        //no break
      case RSI_WLAN_RADIO_INIT_STATE: {
        status = rsi_wlan_get(RSI_FW_VERSION, fw, sizeof(fw));
        if (status != RSI_SUCCESS) {
          PRINTF("reading fw version failed\n");
          break;
        }
        PRINTF("fw version before update is: %s\n", fw);
        //! WLAN Radio Init
        status = rsi_wlan_radio_init();
        if (status != RSI_SUCCESS) {
          PRINTF("wlan radio init failed\n");
          break;
        }
        PRINTF("WLAN RADIO Init success\n");
        error = openFile_sd();
        if (error) {
          PRINTF("Open file failed.\r\n");
          break;
        }
        rsi_wlan_app_cb.state = RSI_WLAN_FW_UPGRADE;
      }
        //no break
      case RSI_WLAN_FW_UPGRADE: {
        if (!header) {
          error = f_read(&g_fileObject_n, recv_buffer, 64, &bytesRead);
          //! Send RPS header which is received as first chunk
          status = rsi_fwup_start(recv_buffer);
          if (status != RSI_SUCCESS) {
            PRINTF("fw_update header failure\n");
            break;
          }
          header = 1;
        } else {
          error = f_read(&g_fileObject_n, recv_buffer, 1024, &bytesRead);
          //! Send RPS content
          status = rsi_fwup_load(recv_buffer, bytesRead);
          if (status != RSI_SUCCESS) {
            if (status == 3) {
              PRINTF("fw_update success\n");
              rsi_wlan_app_cb.state = RSI_WLAN_FW_UPGRADE_DONE;
#ifndef RSI_WITH_OS
              break;
#else
              continue;
#endif
            } else {
              PRINTF("fw_update content failure\n");
              break;
            }
          }
          PRINTF("fw_update in progress..\n");
        }
      } break;
      case RSI_WLAN_FW_UPGRADE_DONE: {
        status = rsi_wireless_deinit();
        if (status != RSI_SUCCESS) {
          break;
        }

        //! WiSeConnect initialization
        status = rsi_wireless_init(0, 0);
        if (status != RSI_SUCCESS) {
          break;
        }

        memset(fw, 0, sizeof(fw));
        status = rsi_wlan_get(RSI_FW_VERSION, fw, sizeof(fw));
        if (status != RSI_SUCCESS) {
          PRINTF("reading fw version failed\n");
          break;
        }
        PRINTF("fw version after update is: %s\n", fw);
        PRINTF("DEMO DONE");
        while (1)
          ;
      }
        //no break

      default:
        break;
    }
  }
  return status;
}

void main_loop(void)
{
  while (1) {
    ////////////////////////
    //! Application code ///
    ////////////////////////

    //! event loop
    rsi_wireless_driver_task();

    //! mfg loop
    rsi_app_task_fw_update_from_host();
  }
}

/*==============================================*/
/**
 * @fn
 * @brief
 * @param[in]   ,
 * @param[out]
 * @return
 *
 *
 * @section description
 * This
 *
 *
 */

int32_t main(void)
{
  int32_t status = RSI_SUCCESS;
  //! Driver initialization
  status = rsi_driver_init(app_global_buf, GLOBAL_BUFF_LEN);
  if ((status < 0) || (status > GLOBAL_BUFF_LEN)) {
    return status;
  }

  //!  module intialisation
  status = rsi_device_init(LOAD_NWP_FW);
  if (status != RSI_SUCCESS) {
    ASSERTION;
  }

  //! WiSeConnect initialization
  status = rsi_wireless_init(RSI_WLAN_MODE, RSI_COEX_MODE);
  if (status != RSI_SUCCESS) {
    return status;
  }

  status = rsi_send_feature_frame();
  if (status != RSI_SUCCESS) {
    return status;
  }

  status = rsi_app_task_fw_update_from_host();
  if (status != RSI_SUCCESS) {
    ASSERTION;
  }

  main_loop();
  return 0;
}
