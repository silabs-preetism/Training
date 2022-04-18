/*******************************************************************************
* @file  rsi_fw_update_via_bootloader.c
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
#include "rsi_data_types.h"
#include "rsi_firmware_upgradation.h"
#include "rsi_wlan_config.h"
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

#define FW_FILE_NAME        "/1.rps"
#define RSI_CHUNK_SIZE      4096
#define RSI_IN_BETWEEN_FILE 0
#define RSI_START_OF_FILE   1
#define RSI_END_OF_FILE     2
//! SD Card parameters
UINT bytesRead;
FIL g_fileObject_n; /* File object */

//! Firmware up request structure
typedef struct fwupeq_s {
  uint16_t control_flags;
  uint16_t sha_type;
  uint32_t magic_no;
  uint32_t image_size;
  uint32_t fw_version;
  uint32_t flash_loc;
  uint32_t crc;
} fwreq_t;

//! This function gets the size of the firmware
uint32_t get_fw_size(char *buffer)
{
  fwreq_t *fw = (fwreq_t *)buffer;
  return fw->image_size;
}

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

uint32_t chunk_cnt, chunk_check, offset, fw_image_size = 0;
int32_t status                      = RSI_SUCCESS;
uint8_t recv_buffer[RSI_CHUNK_SIZE] = { 0 }, fw_version[20] = { 0 };

FRESULT error;

int32_t rsi_app_task_fw_update_via_bootloader(void)
{
#ifdef RSI_WITH_OS
  status = rsi_device_init(BURN_NWP_FW);
  if (status != RSI_SUCCESS) {
    return status;
  }
  while (1)
#endif
  {
    switch (rsi_wlan_app_cb.state) {
      case RSI_WLAN_INITIAL_STATE: {
        error = openFile_sd();
        if (error) {
          PRINTF("Open file failed.\r\n");
          return -1;
        }
        PRINTF("DEMO STARTED\n");
        //! update wlan application state
        rsi_wlan_app_cb.state = RSI_WLAN_FW_UPGRADE;
      }
        //no break
      case RSI_WLAN_FW_UPGRADE: {

        error = f_read(&g_fileObject_n, recv_buffer, RSI_CHUNK_SIZE, &bytesRead);

        //! Send the first chunk to extract header
        fw_image_size = get_fw_size((char *)recv_buffer);

        //!caluculate the total number of chunks
        chunk_check = (fw_image_size / RSI_CHUNK_SIZE);
        if (fw_image_size % RSI_CHUNK_SIZE)
          chunk_check += 1;

        //!Loop until all the chunks are burnt
        while (offset <= fw_image_size) {
          if (chunk_cnt == chunk_check) {
            PRINTF("fw_update Sucess\n");
            break;
          }
          if (chunk_cnt != 0) {
            error = f_read(&g_fileObject_n, recv_buffer, RSI_CHUNK_SIZE, &bytesRead);
          }
          if (chunk_cnt == 0) {

            status = rsi_bl_upgrade_firmware((uint8_t *)recv_buffer, RSI_CHUNK_SIZE, RSI_START_OF_FILE);
            if (status != RSI_SUCCESS) {

              break;
            }
            PRINTF(".");
          } else if (chunk_cnt == (chunk_check - 1)) {
            status = rsi_bl_upgrade_firmware((uint8_t *)recv_buffer, RSI_CHUNK_SIZE, RSI_END_OF_FILE);
            if (status != RSI_SUCCESS) {

              break;
            }
            PRINTF("fw_update Success\n");
            rsi_wlan_app_cb.state = RSI_WLAN_FW_UPGRADE_DONE;
          } else {
            status = rsi_bl_upgrade_firmware((uint8_t *)recv_buffer, RSI_CHUNK_SIZE, RSI_IN_BETWEEN_FILE);
            if (status != RSI_SUCCESS) {
              break;
            }
            PRINTF(".");
          }
          offset += RSI_CHUNK_SIZE;
          memset(recv_buffer, 0, sizeof(recv_buffer));
          chunk_cnt++;
        }
      } break;
      case RSI_WLAN_FW_UPGRADE_DONE: {
        status = rsi_wireless_deinit();
        if (status != RSI_SUCCESS) {
          return status;
        }

        //! WiSeConnect initialization
        status = rsi_wireless_init(0, 0);
        if (status != RSI_SUCCESS) {
          return status;
        }

        memset(fw_version, 0, sizeof(fw_version));
        status = rsi_wlan_get(RSI_FW_VERSION, fw_version, sizeof(fw_version));
        if (status != RSI_SUCCESS) {
          PRINTF("reading fw version failed\n");
          break;
        }
        PRINTF("fw version after update is: %s\n", fw_version);
        PRINTF("DEMO DONE");
        while (1)
          ;
      }

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
    rsi_app_task_fw_update_via_bootloader();
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
  //!  module initialization
  status = rsi_device_init(BURN_NWP_FW);
  if (status != RSI_SUCCESS) {
    ASSERTION;
  }

  status = rsi_app_task_fw_update_via_bootloader();
  if (status != RSI_SUCCESS) {
    ASSERTION;
  }

  main_loop();
  return 0;
}
