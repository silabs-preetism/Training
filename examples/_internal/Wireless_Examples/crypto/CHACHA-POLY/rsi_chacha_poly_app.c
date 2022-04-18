/*************************************************************************
 * 
 * Copyright (c) 2019 SiLabs Signals Incorporated. All Rights Reserved.
 * 
 * NOTICE:  All  information  contained  herein is, and  remains  the  property of 
 * SiLabs Signals Incorporated. The intellectual and technical concepts contained
 * herein  are  proprietary to  SiLabs Signals Incorporated and may be covered by 
 * U.S. and Foreign Patents, patents in process, and are protected by trade secret
 * or copyright law. Dissemination  of this  information or reproduction  of  this
 * material is strictly forbidden unless prior written permission is obtained from
 * SiLabs Signals Incorporated.
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
#define RSI_CHACHAPOLY_TASK_PRIORITY 1

//! Wireless driver task priority
#define RSI_DRIVER_TASK_PRIORITY 2

//! Wlan task stack size
#define RSI_CHACHAPOLY_TASK_STACK_SIZE 500

//! Wireless driver task stack size
#define RSI_DRIVER_TASK_STACK_SIZE 500

//! Memory to initialize driver
uint8_t global_buf[GLOBAL_BUFF_LEN];

/*Chacha20poly1305 */
uint32_t key_chacha[] = {
  0x83828180, 0x87868584, 0x8b8a8988, 0x8f8e8d8c, 0x93929190, 0x97969594, 0x9b9a9998, 0x9f9e9d9c
};

uint32_t nonce[] = { 0x1, 0x00000007, 0x43424140, 0x47464544 }; //! 1st index is IV

uint8_t header_input[] = { 0x50, 0x51, 0x52, 0x53, 0xc0, 0xc1, 0xc2, 0xc3, 0xc4, 0xc5, 0xc6, 0xc7 };

/*Chacha20*/
uint32_t key_chacha_1[] = { 0x03020100, 0x07060504, 0x0b0a0908, 0x0f0e0d0c,
                            0x13121110, 0x17161514, 0x1b1a1918, 0x1f1e1d1c };

uint32_t nonce_1[] = { 0x1, 0x0, 0x4a000000, 0x0 };

uint8_t message[] = { 0x4c, 0x61, 0x64, 0x69, 0x65, 0x73, 0x20, 0x61, 0x6e, 0x64, 0x20, 0x47, 0x65, 0x6e, 0x74,
                      0x6c, 0x65, 0x6d, 0x65, 0x6e, 0x20, 0x6f, 0x66, 0x20, 0x74, 0x68, 0x65, 0x20, 0x63, 0x6c,
                      0x61, 0x73, 0x73, 0x20, 0x6f, 0x66, 0x20, 0x27, 0x39, 0x39, 0x3a, 0x20, 0x49, 0x66, 0x20,
                      0x49, 0x20, 0x63, 0x6f, 0x75, 0x6c, 0x64, 0x20, 0x6f, 0x66, 0x66, 0x65, 0x72, 0x20, 0x79,
                      0x6f, 0x75, 0x20, 0x6f, 0x6e, 0x6c, 0x79, 0x20, 0x6f, 0x6e, 0x65, 0x20, 0x74, 0x69, 0x70,
                      0x20, 0x66, 0x6f, 0x72, 0x20, 0x74, 0x68, 0x65, 0x20, 0x66, 0x75, 0x74, 0x75, 0x72, 0x65,
                      0x2c, 0x20, 0x73, 0x75, 0x6e, 0x73, 0x63, 0x72, 0x65, 0x65, 0x6e, 0x20, 0x77, 0x6f, 0x75,
                      0x6c, 0x64, 0x20, 0x62, 0x65, 0x20, 0x69, 0x74, 0x2e };

//! Keyr and Keys (mode 3)
uint32_t keyr_in[] = { 0xa540921c, 0x8ad355eb, 0x868833f3, 0xf0b5f604 };

uint32_t keys_in[] = { 0xc1173947, 0x09802b40, 0xbc5cca9d, 0xc0757020 };

uint8_t msg_mode3[127] = { 0x27, 0x54, 0x77, 0x61, 0x73, 0x20, 0x62, 0x72, 0x69, 0x6c, 0x6c, 0x69, 0x67, 0x2c, 0x20,
                           0x61, 0x6e, 0x64, 0x20, 0x74, 0x68, 0x65, 0x20, 0x73, 0x6c, 0x69, 0x74, 0x68, 0x79, 0x20,
                           0x74, 0x6f, 0x76, 0x65, 0x73, 0x0a, 0x44, 0x69, 0x64, 0x20, 0x67, 0x79, 0x72, 0x65, 0x20,
                           0x61, 0x6e, 0x64, 0x20, 0x67, 0x69, 0x6d, 0x62, 0x6c, 0x65, 0x20, 0x69, 0x6e, 0x20, 0x74,
                           0x68, 0x65, 0x20, 0x77, 0x61, 0x62, 0x65, 0x3a, 0x0a, 0x41, 0x6c, 0x6c, 0x20, 0x6d, 0x69,
                           0x6d, 0x73, 0x79, 0x20, 0x77, 0x65, 0x72, 0x65, 0x20, 0x74, 0x68, 0x65, 0x20, 0x62, 0x6f,
                           0x72, 0x6f, 0x67, 0x6f, 0x76, 0x65, 0x73, 0x2c, 0x0a, 0x41, 0x6e, 0x64, 0x20, 0x74, 0x68,
                           0x65, 0x20, 0x6d, 0x6f, 0x6d, 0x65, 0x20, 0x72, 0x61, 0x74, 0x68, 0x73, 0x20, 0x6f, 0x75,
                           0x74, 0x67, 0x72, 0x61, 0x62, 0x65, 0x2e };

//! Function to use chachapoly to encrypt and decrypt data
int32_t rsi_crypto_chachapoly_app()
{
  int32_t status = RSI_SUCCESS;

  //Buffers to store responses
  uint8_t chachapoly_encry_data[512];
  uint8_t chachapoly_decry_data[512];

  //! WC initialization
  status = rsi_wireless_init(0, 0);
  if (status != RSI_SUCCESS) {
    return status;
  }

  //!Encrypt using CHAHA20_POLY1305, This mode will generate Encrypted data and Tag
  //!memset response buffers
  memset(chachapoly_encry_data, 0, 512);
  status = rsi_chachapoly(CHACHA20POLY1305,
                          CHACHAPOLY_ENCRYPTION,
                          DMA_ENABLE,
                          message,
                          sizeof(message),
                          (uint8_t *)key_chacha,
                          NULL,
                          NULL,
                          (uint8_t *)nonce,
                          header_input,
                          sizeof(header_input),
                          chachapoly_encry_data);
  if (status != RSI_SUCCESS) {
#ifdef RSI_ENABLE_DEBUG_PRINT
    printf("\n CHACHA20_POLY1305 Operation Failed Status 0x%x \n", status);
#endif
    return status;
  }
#ifdef RSI_ENABLE_DEBUG_PRINT
  printf("\n  CHACHA20_POLY1305 operation is success \n");
#endif

  //!Encrypt using Poly1305 using keyr and keys, This mode will generate Tag
  //!memset buffer response
  memset(chachapoly_encry_data, 0, 512);

  status = rsi_chachapoly(POLY1305_MODE,
                          CHACHAPOLY_ENCRYPTION,
                          DMA_ENABLE,
                          msg_mode3,
                          sizeof(msg_mode3),
                          NULL,
                          (uint8_t *)keyr_in,
                          (uint8_t *)keys_in,
                          NULL,
                          NULL,
                          0,
                          chachapoly_encry_data);

  if (status != RSI_SUCCESS) {
#ifdef RSI_ENABLE_DEBUG_PRINT
    printf("\n POLY_1305 Operation Failed Status 0x%x \n", status);
#endif
    return status;
  }
#ifdef RSI_ENABLE_DEBUG_PRINT
  printf("\n  POLY_1305 operation is success \n");
#endif

  //!Encrypt using CHAHA20, This mode will generate Encrypted data
  //!memset response buffers
  memset(chachapoly_encry_data, 0, 512);
  status = rsi_chachapoly(CHACHA20,
                          CHACHAPOLY_ENCRYPTION,
                          DMA_ENABLE,
                          message,
                          sizeof(message),
                          (uint8_t *)key_chacha_1,
                          NULL,
                          NULL,
                          (uint8_t *)nonce_1,
                          NULL,
                          0,
                          chachapoly_encry_data);
  if (status != RSI_SUCCESS) {
#ifdef RSI_ENABLE_DEBUG_PRINT
    printf("\n CHACHA20 Operation Failed Status 0x%x \n", status);
#endif
    return status;
  }
#ifdef RSI_ENABLE_DEBUG_PRINT
  printf("\n  CHACHA20 operation is success \n");
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

  rsi_task_handle_t chachapoly_task_handle = NULL;

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
  //! Task created for CHACHAPOLY task
  rsi_task_create(rsi_chachapoly_encry_decry_app,
                  "chachapoly_task",
                  RSI_CHACHAPOLY_TASK_STACK_SIZE,
                  NULL,
                  RSI_CHACHAPOLY_TASK_PRIORITY,
                  &chachapoly_task_handle);

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
  //! Call chachapoly encryption and decryption application
  status = rsi_crypto_chachapoly_app();

  //! Application main loop
  main_loop();
#endif
  return status;
}
