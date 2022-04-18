/*
 * Copyright 2010-2015 Amazon.com, Inc. or its affiliates. All Rights Reserved.
 *
 * Licensed under the Apache License, Version 2.0 (the "License").
 * You may not use this file except in compliance with the License.
 * A copy of the License is located at
 *
 *  http://aws.amazon.com/apache2.0
 *
 * or in the "license" file accompanying this file. This file is distributed
 * on an "AS IS" BASIS, WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either
 * express or implied. See the License for the specific language governing
 * permissions and limitations under the License.
 */
/*******************************************************************************
 * @file  aws_iot_multithread.c
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
#include <stdio.h>
#include <stdlib.h>
#include <ctype.h>
#include <limits.h>
#include <string.h>

//! Caws iot includes
#include "aws_iot_config.h"
#include "aws_iot_log.h"
#include "aws_iot_error.h"
#include "aws_iot_version.h"
#include "aws_iot_mqtt_client_interface.h"

#include "rsi_data_types.h"

//! COMMON include file to refer wlan APIs
#include "rsi_common_apis.h"

//! WLAN include file to refer wlan APIs
#include "rsi_wlan_apis.h"
#include "rsi_wlan_non_rom.h"

//! socket include file to refer socket APIs
#include "rsi_socket.h"

#include "rsi_bootup_config.h"
//! Error include files
#include "rsi_error.h"
#include "rsi_driver.h"

//! OS include file to refer OS specific functionality
#include "rsi_os.h"

#include "rsi_wlan.h"
#include "rsi_nwk.h"
#include "rsi_utils.h"
#ifdef RSI_M4_INTERFACE
#include "rsi_chip.h"
#include "rsi_board.h"
#endif

//! certificate includes
#include "aws_client_certificate.pem.crt.h"
#include "aws_client_private_key.pem.key.h"
#include "aws_starfield_ca.pem.h"

//RTOS DEFINES
//! Task stack size defines
#define RSI_MQTT_CLIENT_TASK_STACK_SIZE (512 * 4) //! Common task stack size

#define RSI_MQTT_CLIENT_2_TASK_STACK_SIZE (512 * 4)
#define RSI_DRIVER_TASK_STACK_SIZE        (512 * 2) //! Driver task stack size

#define RSI_MQTT_CLIENT_TASK_PRIORITY   2 //! Common Initializations task priority
#define RSI_MQTT_CLIENT_2_TASK_PRIORITY 2 //! Common Initializations task priority

#define RSI_DRIVER_TASK_PRIORITY 3 //! Wireless driver task priority

#define RSI_MQTT_TOPIC    "$aws/things/Test_IoT/shadow/update"
#define RSI_MQTT_TOPIC_2  "$aws/things/afqp_test/shadow/update"
#define HOST_ADDRESS_SIZE 255

/* PowerSave MACROs */
//! Power Save Profile mode
#define PSP_MODE RSI_SLEEP_MODE_2

//! Power Save Profile type
#define PSP_TYPE RSI_MAX_PSP

#define GPIO_PIN 0

#define DHCP_MODE            1
#define SOCKET_ASYNC_FEATURE 1

//! If DHCP mode is disabled give IP statically
#if !(DHCP_MODE)

//! IP address of the module
//! E.g: 0x650AA8C0 == 192.168.10.101
#define DEVICE_IP 0x65E1A8C0

//! IP address of Gateway
//! E.g: 0x010AA8C0 == 192.168.10.1
#define GATEWAY 0x01E1A8C0

//! IP address of netmask
//! E.g: 0x00FFFFFF == 255.255.255.0
#define NETMASK 0x00FFFFFF
#endif

#define SSID            ""
#define SECURITY_TYPE   RSI_OPEN
#define PSK             ""
#define AWS_DOMAIN_NAME "a25jwtlmds8eip-ats.iot.us-east-2.amazonaws.com"
#define GLOBAL_BUFF_LEN 15000

//! Message to publish
uint8_t publish_message_mq[] = "{\"state\":{\"desired\":{\"toggle\":1}}}";
//uint8_t publish_message_mq_2[] = "{\"state\":{\"desired\":{\"welcome\":aws-iot}}}";
uint8_t publish_message_mq_2[] = "{\"state\":{\"desired\":{\"toggle\":1}}}";

//! user name for login credentials
int8_t username_mq[] = "username";

//! password for login credentials
int8_t password_mq[] = "password";

uint8_t global_buf[GLOBAL_BUFF_LEN];
#ifdef RSI_AWS_MULTITASK
extern volatile uint8_t flag_status;
#endif

unsigned char *buff = NULL;
/**
 * @brief Default MQTT HOST URL is pulled from the aws_iot_config.h
 */
static char HostAddress[HOST_ADDRESS_SIZE] = AWS_IOT_MQTT_HOST;

/**
 * @brief Default MQTT port is pulled from the aws_iot_config.h
 */
static uint32_t port = AWS_IOT_MQTT_PORT;

volatile uint64_t num_bytes = 0;

rsi_task_handle_t common_init_task_handle = NULL;
rsi_task_handle_t wlan_task_handle        = NULL;
rsi_task_handle_t driver_task_handle      = NULL;
rsi_semaphore_handle_t rsi_mqtt_sem;
rsi_semaphore_handle_t rsi_mqtt_sem_2;

void rsi_remote_socket_terminate_handler1(uint16_t status, const uint8_t *buffer, const uint16_t length)
{
  //! Remote socket has been terminated
}
void rsi_wlan_app_callbacks_init(void)
{
  //! Initialze remote terminate call back
  rsi_wlan_register_callbacks(RSI_REMOTE_SOCKET_TERMINATE_CB, rsi_remote_socket_terminate_handler1);
}
//! Enumeration for states in application
typedef enum rsi_wlan_app_state_e {
  RSI_WLAN_INITIAL_STATE                 = 0,
  RSI_WLAN_SCAN_STATE                    = 1,
  RSI_WLAN_UNCONNECTED_STATE             = 2,
  RSI_WLAN_CONNECTED_STATE               = 3,
  RSI_WLAN_IPCONFIG_DONE_STATE           = 4,
  RSI_WLAN_MQTT_INIT_STATE               = 5,
  RSI_WLAN_MQTT_CONNECT_STATE            = 6,
  RSI_WLAN_MQTT_AUTO_RECONNECT_SET_STATE = 7,
  RSI_WLAN_MQTT_SUBSCRIBE_STATE          = 8,
  RSI_WLAN_MQTT_PUBLISH_STATE            = 9,
  RSI_WLAN_MQTT_IDLE_STATE               = 10,
  RSI_WLAN_MQTT_ASYNC_RECV_STATE         = 11
} rsi_wlan_app_state_t;

#define RSI_APP_BUF_SIZE 1600

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

rsi_wlan_app_cb_t rsi_wlan_app_cb; //! application control block

volatile uint8_t publish_flag;
/**
 * @brief This parameter will avoid infinite loop of publish and exit the program after certain number of publishes
 */

static void iot_subscribe_callback_handler(AWS_IoT_Client *pClient,
                                           char *topicName,
                                           uint16_t topicNameLen,
                                           IoT_Publish_Message_Params *params,
                                           void *pData)
{
}

static void disconnectCallbackHandler(AWS_IoT_Client *pClient, void *data)
{

  IoT_Error_t rc = FAILURE;
  LOG_PRINT("MQTT Disconnect");
  if (NULL == pClient) {
    return;
  }
  IOT_UNUSED(data);
  if (aws_iot_is_autoreconnect_enabled(pClient)) {
    LOG_PRINT("Auto Reconnect is enabled, Reconnecting attempt will start now\n");
  } else {
    LOG_PRINT("Auto Reconnect not enabled. Starting manual reconnect...\n");
    rc = aws_iot_mqtt_attempt_reconnect(pClient);
    if (NETWORK_RECONNECTED == rc) {
      LOG_PRINT("Manual Reconnect Successful");
    } else {
      LOG_PRINT("Manual Reconnect \n");
    }
  }
}

AWS_IoT_Client client  = { 0 };
volatile uint8_t value = 0;
//! Call back for Socket Async
void socket_async_recive(uint32_t sock_no, uint8_t *buffer, uint32_t length)
{
  num_bytes += length;
  LOG_PRINT("Socket len:%d,length:%d\n", sock_no, length);
}
IoT_Error_t awsSelect(bool check);

void awsSelectResponse(rsi_fd_set *fd_read, rsi_fd_set *fd_write, rsi_fd_set *fd_except, int32_t status)
{
  if (publish_flag == 0) {
    LOG_PRINT("aws select callback\n");
    rsi_wlan_app_cb.state = RSI_WLAN_MQTT_ASYNC_RECV_STATE;
#ifdef RSI_WITH_OS
    rsi_semaphore_post(&rsi_mqtt_sem);
#endif
  } else {
    LOG_PRINT("aws puback response\n");
  }
}

IoT_Error_t iot_tls_select(Network pNetwork, bool check)
{
  rsi_fd_set read_fds;

  memset(&read_fds, 0, sizeof(rsi_fd_set));
  RSI_FD_SET(pNetwork.socket_id, &read_fds);
  int32_t status;

  status = rsi_select(pNetwork.socket_id + 1, &read_fds, NULL, NULL, NULL, awsSelectResponse);
  /* TODO : Need to add AWS error code */
  return (status);
}

IoT_Error_t awsSelect(bool check)
{
  IoT_Error_t rtn = FAILURE;
  ClientState clientState;

  clientState = aws_iot_mqtt_get_client_state(&client);

  if ((CLIENT_STATE_CONNECTED_YIELD_IN_PROGRESS != clientState)
      && (CLIENT_STATE_CONNECTED_PUBLISH_IN_PROGRESS != clientState)) {
    rtn = iot_tls_select(client.networkStack, check);
  }
  /* TODO : rtn return rsi error code,need to change with AWS error code */
  return rtn;
}

int32_t rsi_mqtt_client_app_2()
{
  IoT_Publish_Message_Params paramsQOS0;
  IoT_Publish_Message_Params paramsQOS1;

  ClientState clientState;
  IoT_Error_t rc                  = FAILURE;
  volatile uint8_t aws_sel_status = 0;

  paramsQOS0.qos        = QOS0;
  paramsQOS0.payload    = (void *)publish_message_mq_2;
  paramsQOS0.isRetained = 0;
  paramsQOS0.payloadLen = strlen((char *)publish_message_mq_2);

  paramsQOS1.qos        = QOS1;
  paramsQOS1.payload    = (void *)publish_message_mq_2;
  paramsQOS1.isRetained = 0;
  paramsQOS1.payloadLen = strlen((char *)publish_message_mq_2);

  while (1) {
#ifdef RSI_WITH_OS
    rsi_semaphore_wait(&rsi_mqtt_sem_2, 0);
#endif
    publish_flag = 1;

    rc = aws_iot_mqtt_publish(&client, RSI_MQTT_TOPIC_2, strlen(RSI_MQTT_TOPIC_2), &paramsQOS1);
    if (rc == MQTT_REQUEST_TIMEOUT_ERROR) {
      LOG_PRINT("QOS1 publish ack not received.\n");
    } else if (rc == 0) {
      LOG_PRINT("T2: Data Published successfully to cloud...\n");
    } else {
      LOG_PRINT("T2: Data Published Fail :%d\n", rc);
    }

    publish_flag = 0;

    rsi_wlan_app_cb.state = RSI_WLAN_MQTT_IDLE_STATE;

#ifdef RSI_WITH_OS
    rsi_semaphore_post(&rsi_mqtt_sem);
#endif
  }
}

int32_t rsi_mqtt_client_app()
{
  int32_t status = RSI_SUCCESS;
#if !(DHCP_MODE)
  uint32_t ip_addr      = DEVICE_IP;
  uint32_t network_mask = NETMASK;
  uint32_t gateway      = GATEWAY;
#else
  uint8_t dhcp_mode = (RSI_DHCP | RSI_DHCP_UNICAST_OFFER);
#endif
  uint32_t server_address = 0;

  uint8_t ip_rsp[18]     = { 0 };
  uint8_t fw_version[10] = { 0 };

  volatile uint8_t aws_sel_status = 0;
  bool infinitePublishFlag        = true;
  IoT_Error_t rc                  = FAILURE;
  rsi_rsp_dns_query_t dns_query_rsp;

  IoT_Client_Init_Params mqttInitParams   = iotClientInitParamsDefault;
  IoT_Client_Connect_Params connectParams = iotClientConnectParamsDefault;

  mqttInitParams.enableAutoReconnect       = false;
  mqttInitParams.pHostURL                  = HostAddress;
  mqttInitParams.port                      = port;
  mqttInitParams.pRootCALocation           = (char *)aws_starfield_ca;
  mqttInitParams.pDeviceCertLocation       = (char *)aws_client_certificate;
  mqttInitParams.pDevicePrivateKeyLocation = (char *)aws_client_private_key;
  mqttInitParams.mqttCommandTimeout_ms     = 20000;
  mqttInitParams.tlsHandshakeTimeout_ms    = 5000;
  mqttInitParams.isSSLHostnameVerify       = true;
  mqttInitParams.disconnectHandler         = disconnectCallbackHandler;
  mqttInitParams.disconnectHandlerData     = NULL;

  connectParams.keepAliveIntervalInSec = 600;
  connectParams.isCleanSession         = true;
  connectParams.MQTTVersion            = MQTT_3_1_1;
  connectParams.pClientID              = AWS_IOT_MQTT_CLIENT_ID;
  connectParams.clientIDLen            = (uint16_t)strlen(AWS_IOT_MQTT_CLIENT_ID);
  connectParams.isWillMsgPresent       = false;

  connectParams.pUsername   = (char *)username_mq;
  connectParams.usernameLen = strlen((char *)username_mq);
  connectParams.pPassword   = (char *)password_mq;
  connectParams.passwordLen = strlen((char *)password_mq);

  IoT_Publish_Message_Params paramsQOS0;
  paramsQOS0.qos        = QOS0;
  paramsQOS0.payload    = (void *)publish_message_mq;
  paramsQOS0.isRetained = 0;
  paramsQOS0.payloadLen = strlen((char *)publish_message_mq);

  /* Harshal Modification */
  IoT_Publish_Message_Params paramsQOS1;
  paramsQOS1.qos        = QOS1;
  paramsQOS1.payload    = (void *)publish_message_mq;
  paramsQOS1.isRetained = 0;
  paramsQOS1.payloadLen = strlen((char *)publish_message_mq);

  //! Silabs module intialisation
  status = rsi_device_init(LOAD_NWP_FW);
  if (status != RSI_SUCCESS) {
    return status;
  }

  //! WC initialization
  status = rsi_wireless_init(0, 0);
  if (status != RSI_SUCCESS) {
    return status;
  }

  status = rsi_send_feature_frame();
  if (status != RSI_SUCCESS) {
    return status;
  }

  status = rsi_wlan_get(RSI_FW_VERSION, fw_version, 10);
  if (status != RSI_SUCCESS) {
    return status;
  }

  status = rsi_wlan_set_certificate(RSI_SSL_CA_CERTIFICATE, aws_starfield_ca, (sizeof(aws_starfield_ca) - 1));
  if (status != RSI_SUCCESS) {
    return status;
  }

  status = rsi_wlan_set_certificate(RSI_SSL_CLIENT, aws_client_certificate, (sizeof(aws_client_certificate) - 1));
  if (status != RSI_SUCCESS) {
    return status;
  }

  status =
    rsi_wlan_set_certificate(RSI_SSL_CLIENT_PRIVATE_KEY, aws_client_private_key, (sizeof(aws_client_private_key) - 1));
  if (status != RSI_SUCCESS) {
    return status;
  }
  rsi_wlan_app_cb.state = RSI_WLAN_INITIAL_STATE;
#ifdef RSI_WITH_OS
  rsi_semaphore_post(&rsi_mqtt_sem);
#endif

  while (1) {
#ifdef RSI_WITH_OS
    rsi_semaphore_wait(&rsi_mqtt_sem, 0);
#endif
    switch (rsi_wlan_app_cb.state) {
      case RSI_WLAN_INITIAL_STATE: {
        rsi_wlan_app_callbacks_init();               //! register callback to initialize WLAN
        rsi_wlan_app_cb.state = RSI_WLAN_SCAN_STATE; //! update WLAN application state to unconnected state
#ifdef RSI_WITH_OS
        rsi_semaphore_post(&rsi_mqtt_sem);
#endif
      } break;
      case RSI_WLAN_SCAN_STATE: {
        //! Scan for Access points
        status = rsi_wlan_scan((int8_t *)SSID, 0, NULL, 0);
        if (status != RSI_SUCCESS) {
          LOG_PRINT("Scan Failed...\n");
          rsi_wlan_app_cb.state = RSI_WLAN_SCAN_STATE;
        } else {
          rsi_wlan_app_cb.state = RSI_WLAN_UNCONNECTED_STATE; //! update WLAN application state to unconnected state
        }
#ifdef RSI_WITH_OS
        rsi_semaphore_post(&rsi_mqtt_sem);
#endif
      } break;
      case RSI_WLAN_UNCONNECTED_STATE: {
        //! Connect to an Access point
        LOG_PRINT("Wifi Connecting...\n");
        status = rsi_wlan_connect((int8_t *)SSID, SECURITY_TYPE, PSK);
        if (status != RSI_SUCCESS) {
          LOG_PRINT("Wifi Connection failed\n");
          rsi_wlan_app_cb.state = RSI_WLAN_UNCONNECTED_STATE;
        } else {
          rsi_wlan_app_cb.state = RSI_WLAN_CONNECTED_STATE; //! update WLAN application state to unconnected state
        }
#ifdef RSI_WITH_OS
        rsi_semaphore_post(&rsi_mqtt_sem);
#endif
      } break;
      case RSI_WLAN_CONNECTED_STATE: {
        //! Configure IP
#if DHCP_MODE
        status = rsi_config_ipaddress(RSI_IP_VERSION_4, dhcp_mode, 0, 0, 0, ip_rsp, 18, 0);
#else
        status = rsi_config_ipaddress(RSI_IP_VERSION_4,
                                      RSI_STATIC,
                                      (uint8_t *)&ip_addr,
                                      (uint8_t *)&network_mask,
                                      (uint8_t *)&gateway,
                                      NULL,
                                      0,
                                      0);
#endif
        if (status != RSI_SUCCESS) {
          LOG_PRINT("Ip config fail...\n");
          rsi_wlan_app_cb.state = RSI_WLAN_CONNECTED_STATE;
        } else {
          rsi_wlan_app_cb.state = RSI_WLAN_IPCONFIG_DONE_STATE;
        }
#ifdef RSI_WITH_OS
        rsi_semaphore_post(&rsi_mqtt_sem);
#endif
      } break;
      case RSI_WLAN_IPCONFIG_DONE_STATE: {
        rsi_wlan_app_cb.state = RSI_WLAN_MQTT_INIT_STATE;
#ifdef RSI_WITH_OS
        rsi_semaphore_post(&rsi_mqtt_sem);
#endif
      } break;
      case RSI_WLAN_MQTT_INIT_STATE: {
        rc = aws_iot_mqtt_init(&client, &mqttInitParams);
        if (SUCCESS != rc) {
          rsi_wlan_app_cb.state = RSI_WLAN_MQTT_INIT_STATE;
          LOG_PRINT("aws_iot_mqtt_init returned error ");
        } else {
          rsi_wlan_app_cb.state = RSI_WLAN_MQTT_CONNECT_STATE;
        }
#ifdef RSI_WITH_OS
        rsi_semaphore_post(&rsi_mqtt_sem);
#endif
      } break;
      case RSI_WLAN_MQTT_CONNECT_STATE: {
        LOG_PRINT("AWS IOT MQTT Connecting...\n");
        rc = aws_iot_mqtt_connect(&client, &connectParams);
        if (SUCCESS != rc) {
          LOG_PRINT("Error in connection\n");
          rsi_wlan_app_cb.state = RSI_WLAN_MQTT_CONNECT_STATE;
        } else {
          rsi_wlan_app_cb.state = RSI_WLAN_MQTT_AUTO_RECONNECT_SET_STATE;
        }
#ifdef RSI_WITH_OS
        rsi_semaphore_post(&rsi_mqtt_sem);
#endif
      } break;
      case RSI_WLAN_MQTT_AUTO_RECONNECT_SET_STATE: {
        rc = aws_iot_mqtt_autoreconnect_set_status(&client, false);
        if (SUCCESS != rc) {
          rsi_wlan_app_cb.state = RSI_WLAN_MQTT_AUTO_RECONNECT_SET_STATE;
          LOG_PRINT("Unable to set Auto Reconnect to true\n ");
        } else {
          rsi_wlan_app_cb.state = RSI_WLAN_MQTT_SUBSCRIBE_STATE;
        }
#ifdef RSI_WITH_OS
        rsi_semaphore_post(&rsi_mqtt_sem);
#endif
      } break;
      case RSI_WLAN_MQTT_SUBSCRIBE_STATE: {
        LOG_PRINT("AWS IOT MQTT Subscribe...\n");
        rc = aws_iot_mqtt_subscribe(&client,
                                    RSI_MQTT_TOPIC,
                                    strlen(RSI_MQTT_TOPIC) /*11*/,
                                    QOS0,
                                    iot_subscribe_callback_handler,
                                    /*paramsQOS0.payload */ client.clientData.readBuf);
        if (SUCCESS != rc) {
          LOG_PRINT("Error subscribing\n ");
          rsi_wlan_app_cb.state = RSI_WLAN_MQTT_SUBSCRIBE_STATE;
        } else {
          rsi_wlan_app_cb.state = RSI_WLAN_MQTT_IDLE_STATE;
        }
#ifdef RSI_WITH_OS
        rsi_semaphore_post(&rsi_mqtt_sem);
#endif
      } break;
      case RSI_WLAN_MQTT_IDLE_STATE: {
#ifdef RSI_WITH_OS
        rsi_semaphore_post(&rsi_mqtt_sem_2);
#endif
        aws_sel_status = awsSelect(0);
        LOG_PRINT("AWS select API status: %d\n", aws_sel_status);
        if (aws_sel_status != 0) {
          LOG_PRINT("Async select failed: %d\n", aws_sel_status);
        }
      } break;
      case RSI_WLAN_MQTT_ASYNC_RECV_STATE: {
        LOG_PRINT("Thread Number1\n");

#ifdef MQTT_YIELD_EN

        //! Waiting for data from cloud
        LOG_PRINT("Waiting for data from cloud...\n");

        memset(client.clientData.readBuf, 0, sizeof(client.clientData.readBuf));

        rc = aws_iot_mqtt_yield(&client, 100);
        if (NETWORK_ATTEMPTING_RECONNECT == rc) {
          // If the client is attempting to reconnect we will skip the rest of the loop.
          rsi_wlan_app_cb.state = RSI_WLAN_MQTT_AUTO_RECONNECT_SET_STATE;
          continue;
        } else if (rc == 0) {
          LOG_PRINT("T1: Data Received Successfully From Cloud...\n");

          LOG_PRINT("T1: Async Data :%s\n", client.clientData.readBuf);
        } else {
          LOG_PRINT("T1: Data Received Faild...\n");
        }
#endif
#ifdef RSI_AWS_MULTITASK
        flag_status = 1;
#endif

        rsi_wlan_app_cb.state = RSI_WLAN_MQTT_IDLE_STATE;

#ifdef RSI_WITH_OS
        rsi_semaphore_post(&rsi_mqtt_sem);
#endif
        break;
      }
      default:
        break;
    }
  }
}

void rsi_create_app_tasks(void)
{
#ifdef RSI_WITH_OS
  //! Create Semaphores
  rsi_semaphore_create(&rsi_mqtt_sem, 0);

  rsi_semaphore_create(&rsi_mqtt_sem_2, 0);

  //! Common Init task
  rsi_task_create((void *)rsi_mqtt_client_app,
                  (uint8_t *)"mqtt_client_task",
                  RSI_MQTT_CLIENT_TASK_STACK_SIZE,
                  NULL,
                  RSI_MQTT_CLIENT_TASK_PRIORITY,
                  &common_init_task_handle);

  rsi_task_create((void *)rsi_mqtt_client_app_2,
                  (uint8_t *)"mqtt_client_task2",
                  RSI_MQTT_CLIENT_2_TASK_STACK_SIZE,
                  NULL,
                  RSI_MQTT_CLIENT_2_TASK_PRIORITY,
                  &common_init_task_handle);

  //! Driver task
  rsi_task_create((void *)rsi_wireless_driver_task,
                  (uint8_t *)"driver_task",
                  RSI_DRIVER_TASK_STACK_SIZE,
                  NULL,
                  RSI_DRIVER_TASK_PRIORITY,
                  &driver_task_handle);
#endif
}

int main()
{
  int32_t status = RSI_SUCCESS;

  //! Board Initialization
  rsi_hal_board_init();

  status = rsi_driver_init(global_buf, GLOBAL_BUFF_LEN);
  if ((status < 0) || (status > GLOBAL_BUFF_LEN)) {
    return status;
  }

#ifdef RSI_WITH_OS
  //! Create RTOS tasks
  rsi_create_app_tasks();
  //! OS Task Start the scheduler
  rsi_start_os_scheduler();
#else
  rsi_mqtt_client_app();
#endif

  while (1)
    ;
}
