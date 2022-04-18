/*************************************************************************
 * 
 * Copyright (c) 2021 Silicon Lab Incorporated. All Rights Reserved.
 * 
 * NOTICE:  All  information  contained  herein is, and  remains  the  property of 
 * Siliocn Lab Incorporated. The intellectual and technical concepts contained
 * herein  are  proprietary to  Silicon Labs Incorporated and may be covered by 
 * U.S. and Foreign Patents, patents in process, and are protected by trade secret
 * or copyright law. Dissemination  of this  information or reproduction  of  this
 * material is strictly forbidden unless prior written permission is obtained from
 * Siliocn Labs Incorporated.
 */

/**
 * Include files
 * */

//! include file to refer data types
#include "rsi_data_types.h"

//! COMMON include file to refer wlan APIs
#include "rsi_common_apis.h"

//! WLAN include file to refer wlan APIs
#include "rsi_wlan_apis.h"
#include "rsi_wlan_non_rom.h"

//! Error include files
#include "rsi_error.h"

//! OS include file to refer OS specific functionality
#include "rsi_os.h"

//! Include file for crypto Apis
#include "rsi_crypto.h"

#ifdef RSI_ENABLE_DEBUG_PRINT
#include <stdio.h>
#endif

//! Receive data length
#define RECV_BUFFER_SIZE 1027

//! Memory length for driver
#define GLOBAL_BUFF_LEN 15000

//! Wlan task priority
#define RSI_GCM_TASK_PRIORITY 1

//! Wireless driver task priority
#define RSI_DRIVER_TASK_PRIORITY 2

//! Wlan task stack size
#define RSI_GCM_TASK_STACK_SIZE 500

//! Wireless driver task stack size
#define RSI_DRIVER_TASK_STACK_SIZE 500

//! Memory to initialize driver
uint8_t global_buf[GLOBAL_BUFF_LEN];

//!GCM inputs

uint8_t key[] = { 0xca, 0x74, 0xfa, 0xb, 0xb5, 0x9a, 0xb5, 0x28, 0x93, 0x70, 0x35, 0xf2, 0x72, 0xd2, 0x16, 0x32 };

//! IV should be 12 Bytes/96 bits
uint8_t iv[] = { 0x40, 0x92, 0xa5, 0x2c, 0xc9, 0x55, 0xc4, 0xcb, 0xee, 0x97, 0x93, 0x8d };

uint8_t encry_msg[] = {
  0x5d, 0x80, 0x2f, 0xb6, 0x5e, 0x25, 0x20, 0x21, 0xb9, 0xc1, 0xec, 0x63, 0x34, 0x1c, 0x65, 0x41
};

uint8_t header[] = { 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x16, 0x3, 0x3, 0x0, 0x10 };

uint8_t decry_msg[] = {
  0x14, 0x00, 0x00, 0x0c, 0x79, 0xb2, 0x1a, 0xa0, 0xf7, 0xff, 0x7a, 0x77, 0x34, 0x86, 0xaa, 0x32
};
//! Function to use GCM to encrypt and decrypt data
int32_t rsi_crypto_gcm_app()
{
  int32_t status = RSI_SUCCESS;

  //Buffers to store responses
  uint8_t gcm_encry_data[256];
  uint8_t gcm_decry_data[256];

  //! WC initialization
  status = rsi_wireless_init(0, 0);
  if (status != RSI_SUCCESS) {
    return status;
  }

  //!Encrypt using GCM
  memset(gcm_encry_data, 0, 256); //memset response buffers

  status = rsi_gcm(GCM_ENCRYPTION,
                   DMA_ENABLE,
                   encry_msg,
                   sizeof(encry_msg),
                   key,
                   GCM_KEY_SIZE_128,
                   iv,
                   sizeof(iv),
                   header,
                   sizeof(header),
                   gcm_encry_data);
  if (status != RSI_SUCCESS) {
#ifdef RSI_ENABLE_DEBUG_PRINT
    printf("\n GCM Encryption Fail Status 0x%x \n", status);
#endif
    return status;
  }
#ifdef RSI_ENABLE_DEBUG_PRINT
  printf("\n Encryption Done with GCM mode \n");
#endif

  //!Decrypt using GCM mode
  memset(gcm_decry_data, 0, 256); //memset response buffers
  status = rsi_gcm(GCM_DECRYPTION,
                   DMA_ENABLE,
                   decry_msg,
                   sizeof(decry_msg),
                   key,
                   GCM_KEY_SIZE_128,
                   iv,
                   sizeof(iv),
                   header,
                   sizeof(header),
                   gcm_decry_data);
  if (status != RSI_SUCCESS) {
#ifdef RSI_ENABLE_DEBUG_PRINT
    printf("\n GCM Decryption Fail Status 0x%x \n", status);
#endif
    return status;
  }
#ifdef RSI_ENABLE_DEBUG_PRINT
  printf("\n Decryption Done with GCM mode \n ");
#endif

  return 0;
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

  rsi_task_handle_t gcm_task_handle = NULL;

  rsi_task_handle_t driver_task_handle = NULL;
#endif

  //! Driver initialization
  status = rsi_driver_init(global_buf, GLOBAL_BUFF_LEN);
  if ((status < 0) || (status > GLOBAL_BUFF_LEN)) {
    return status;
  }

  //! RS9117 intialisation
  status = rsi_device_init(LOAD_NWP_FW);
  if (status != RSI_SUCCESS) {
    return status;
  }

#ifdef RSI_WITH_OS
  //! OS case
  //! Task created for GCM task
  rsi_task_create(rsi_gcm_encry_decry_app,
                  "gcm_task",
                  RSI_GCM_TASK_STACK_SIZE,
                  NULL,
                  RSI_GCM_TASK_PRIORITY,
                  &gcm_task_handle);

  //! Task created for Driver task
  rsi_task_create(rsi_wireless_driver_task,
                  "driver_task",
                  RSI_DRIVER_TASK_STACK_SIZE,
                  NULL,
                  RSI_DRIVER_TASK_PRIORITY,
                  &driver_task_handle);

  //! OS TAsk Start the scheduler
  rsi_start_os_scheduler();

#else
  //! NON - OS case
  //! Call gcm encryption and decryption application
  status = rsi_crypto_gcm_app();

  //! Application main loop
  main_loop();
#endif
  return status;
}
