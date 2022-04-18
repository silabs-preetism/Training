/*******************************************************************************
* @file  rsi_wlan_app.c
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
/*************************************************************************
 *
 */

/*================================================================================
 * @brief : This file contains example application for Wlan Station BLE
 * Provisioning
 * @section Description :
 * This application explains how to get the WLAN connection functionality using
 * BLE provisioning.
 * SiLabs Module starts advertising and with BLE Provisioning the Access Point
 * details are fetched.
 * SiLabs device is configured as a WiFi station and connects to an Access Point.
 =================================================================================*/

/**
 * Include files
 * */
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

//! include file to refer data types
#include "rsi_data_types.h"

//! COMMON include file to refer wlan APIs
#include "rsi_common_apis.h"

//! WLAN include file to refer wlan APIs
#include "rsi_wlan_apis.h"

//! WLAN include file for configuration
#include <rsi_wlan_config.h>

//! socket include file to refer socket APIs
#include "rsi_socket.h"

#include "rsi_bootup_config.h"
//! Error include files
#include "rsi_error.h"

#include "rsi_wlan_non_rom.h"
#include "rsi_hal.h"

//! OS include file to refer OS specific functionality
#include "rsi_os.h"
#ifdef RSI_M4_INTERFACE
#include "rsi_board.h"
#include "rsi_chip.h"
#endif

//! certificate includes
#include "aws_client_certificate.pem.crt.h"
#include "aws_client_private_key.pem.key.h"
#include "aws_starfield_ca.pem.h"

#define RSI_APP_BUF_SIZE 1600

rsi_semaphore_handle_t rsi_mqtt_sem;
#define RSI_MQTT_TOPIC    "$aws/things/Test_IoT/shadow/update"
#define HOST_ADDRESS_SIZE 255

unsigned char *buff = NULL;

/**
	 * @brief Default MQTT HOST URL is pulled from the aws_iot_config.h
	 */
static char HostAddress[HOST_ADDRESS_SIZE] = AWS_IOT_MQTT_HOST;

/**
	 * @brief Default MQTT port is pulled from the aws_iot_config.h
	 */
static uint32_t port = AWS_IOT_MQTT_PORT;

//! Message to publish
uint8_t publish_message_mq[] = "{\"state\":{\"desired\":{\"toggle\":1}}}";

//! user name for login credentials
int8_t username_mq[] = "username";

//! password for login credentials
int8_t password_mq[] = "password";

#define GPIO_PIN 0

#define DHCP_MODE       1
#define SSID            "SILABS"
#define SECURITY_TYPE   RSI_WPA2
#define PSK             "12345678"
#define AWS_DOMAIN_NAME "a25jwtlmds8eip-ats.iot.us-east-2.amazonaws.com"

//! Memory length for driver
#define GLOBAL_BUFF_LEN 15000

//! Memory length for send buffer
#define BUF_SIZE 1400

#define PKT_SEND_INTERVAL 55000
#define DNS_FAILED_COUNT  5
//! Wlan task priority
#define RSI_WLAN_TASK_PRIORITY 1

//! Wireless driver task priority
#define RSI_DRIVER_TASK_PRIORITY 1

//! Wlan task stack size
#define RSI_WLAN_TASK_STACK_SIZE 500

//! Wireless driver task stack size
#define RSI_DRIVER_TASK_STACK_SIZE 500

/*
 *********************************************************************************************************
 *                                         LOCAL GLOBAL VARIABLES
 *********************************************************************************************************
 */

rsi_rsp_scan_t scan_result;
uint16_t scanbuf_size;
uint8_t connected = 0, timeout = 0;
uint8_t disconnected = 0, disassosiated = 0, wlan_connected;
uint8_t a = 0;
uint8_t rsp_buf[18];

uint32_t wifi_connections, wifi_disconnections, rejoin_failures, dns_success, dns_query_fail;
uint32_t raising_time, falling_time, initial_time;

/*
 *********************************************************************************************************
 *                                               DATA TYPES
 *********************************************************************************************************
 */

typedef enum rsi_app_cmd_e {
  RSI_DATA                = 0,
  RSI_WLAN_SCAN_RESP      = 1,
  RSI_WLAN_CONN_STATUS    = 2,
  RSI_WLAN_DISCONN_STATUS = 3,
  RSI_WLAN_DISCONN_NOTIFY = 4,
  RSI_WLAN_TIMEOUT_NOTIFY = 5

} rsi_app_cmd_t;

extern uint8_t coex_ssid[50], dis_ssid[50], pwd[34], sec_type;
extern uint8_t cid[100];
uint8_t retry = 1;

uint8_t conn_status;

extern uint8_t magic_word;

extern rsi_wlan_app_cb_t rsi_wlan_app_cb;
int32_t rsi_wlan_power_save_profile(uint8_t psp_mode, uint8_t psp_type);
extern int32_t rsi_wlan_filter_broadcast(uint16_t beacon_drop_threshold,
                                         uint8_t filter_bcast_in_tim,
                                         uint8_t filter_bcast_tim_till_next_cmd);
void rsi_wlan_async_module_state(uint16_t status, uint8_t *payload, const uint32_t payload_length);
void rsi_wlan_async_module_state(uint16_t status, uint8_t *payload, const uint32_t payload_length)
{
  int i = 0, j = 0;
  char *unknown       = "unknown";
  char *higher_nibble = unknown;
  char *lower_nibble  = unknown;
  char *reason_code   = unknown;
  uint8_t bssid_string[18];
  if (payload_length != sizeof(rsi_state_notification_t))
    return;
  rsi_state_notification_t *state = (rsi_state_notification_t *)payload;
  for (i = 0; i < WLAN_MODULE_STATES; i++) {
    //higher nibble information
    if (STATE[i].bit == (state->StateCode & 0xF0)) {
      higher_nibble = STATE[i].string;
    }
    //lower nibble information
    if (STATE[i].bit == (state->StateCode & 0x0F)) {
      lower_nibble = STATE[i].string;
    }
  }
  for (j = 0; j < WLAN_REASON_CODES; j++) {
    //!reason code info
    if (REASONCODE[j].bit == (state->reason_code & 0xFF)) {
      reason_code = REASONCODE[j].string;
    }
  }
  if (higher_nibble == unknown && lower_nibble == unknown && reason_code == unknown)
    return;
  rsi_6byte_dev_address_to_ascii((uint8_t *)bssid_string, state->rsi_bssid);

  if (*higher_nibble == 'A') {
    wifi_connections++;
    LOG_PRINT("Wi-Fi connections %d\n", wifi_connections);
  }
  if (*higher_nibble == 'U') {
    wifi_disconnections++;
    LOG_PRINT("Wi-Fi disconnections %d\n", wifi_disconnections);
  }
  if (state->rsi_channel == 0 && state->rsi_rssi == 100) {

  } else if (state->rsi_channel == 0 && state->rsi_rssi != 100) {

  } else if (state->rsi_channel != 0 && state->rsi_rssi == 100) {

  } else {
  }
}

//extern volatile uint32_t _dwTickCount;

uint32_t received_length = 0;

void rsi_wlan_mqtt_task(void);

//! Call back for Socket Async
void socket_async_recive(uint32_t sock_no, uint8_t *buffer, uint32_t length)
{
  received_length += length;
}

//! rejoin failure call back handler in station mode
void rsi_join_fail_handler(uint32_t status, const uint8_t *buffer, const uint32_t length)
{
  //! update wlan application state
  disconnected          = 1;
  connected             = 0;
  rsi_wlan_app_cb.state = RSI_WLAN_DISCONNECTED_STATE;
}
void rsi_remote_socket_terminate_handler1(uint16_t status, uint8_t *buffer, const uint32_t length)
{
  //! Remote socket has been terminated
  rsi_wlan_app_cb.state = RSI_WLAN_IPCONFIG_DONE_STATE;
}
void rsi_wlan_app_call_backs_init(void)
{
  rsi_wlan_register_callbacks(RSI_WLAN_ASYNC_STATS, rsi_wlan_async_module_state);
  //! Initialize join fail call back
  rsi_wlan_register_callbacks(RSI_JOIN_FAIL_CB, rsi_join_fail_handler);
  rsi_wlan_register_callbacks(RSI_REMOTE_SOCKET_TERMINATE_CB, rsi_remote_socket_terminate_handler1);
}

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
/*============================*/
void rsi_give_wakeup_indication()
{
  if (rsi_hal_get_gpio(RSI_HAL_WAKEUP_INDICATION_PIN)) {
    raising_time = rsi_hal_gettickcount();
    if (raising_time && falling_time) {
      LOG_PRINT("S %d\n", (raising_time - falling_time));
    }
  }
  if (!rsi_hal_get_gpio(RSI_HAL_WAKEUP_INDICATION_PIN)) {
    falling_time = rsi_hal_gettickcount();
    if (raising_time && falling_time) {
      LOG_PRINT("W %d\n", (falling_time - raising_time));
    }
  }
}

int32_t rsi_wlan_mqtt_certs_init(void)
{
  int32_t status = RSI_SUCCESS;

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
}

int32_t rsi_wlan_app_task()
{
  int32_t status = RSI_SUCCESS;
  uint8_t ipconfig[20];

#ifdef RSI_WITH_OS
  while (1) {
#endif
    switch (rsi_wlan_app_cb.state) {
      case RSI_WLAN_INITIAL_STATE: {
        //! update wlan application state
#if (WYZBEE_CONFIGURATOR || BLE_SCANNER)
        rsi_wlan_app_call_backs_init();

        if (magic_word) {
          rsi_wlan_app_cb.state = RSI_WLAN_FLASH_STATE;
        } else {
          rsi_wlan_app_cb.state = RSI_WLAN_UNCONNECTED_STATE;
        }

#elif (STATIC_AP_OPEN || STATIC_AP_WPA2_SECURE || STATIC_AP_WPA_SECURE || STATIC_AP_MIXED)
      rsi_wlan_app_cb.state = RSI_WLAN_SCAN_STATE;
#endif
      } break;

      case RSI_WLAN_UNCONNECTED_STATE: {
        //Any additional code if required
      } break;

      case RSI_WLAN_SCAN_STATE: {
        scanbuf_size = sizeof(scan_result);
        memset(&scan_result, 0, sizeof(scan_result));

        status = rsi_wlan_scan(0, 0, &scan_result, scanbuf_size);
        if (status != RSI_SUCCESS) {
          LOG_PRINT("\r\nWLAN Scan Failed, Error Code : 0x%X\r\n", status);
          break;
        } else {
          //! update wlan application state

#if WYZBEE_CONFIGURATOR
          rsi_wlan_app_cb.state = RSI_WLAN_UNCONNECTED_STATE;
          rsi_wlan_app_send_to_ble(RSI_WLAN_SCAN_RESP, (uint8_t *)&scan_result, scanbuf_size);

#elif (STATIC_AP_OPEN || STATIC_AP_WPA2_SECURE || STATIC_AP_WPA_SECURE || STATIC_AP_MIXED)
        rsi_wlan_app_cb.state = RSI_WLAN_JOIN_STATE;
#endif
        }
      } break;

      case RSI_WLAN_JOIN_STATE: {
#if WYZBEE_CONFIGURATOR
        if (sec_type == 0 || sec_type == '0') {
          status = rsi_wlan_connect((int8_t *)coex_ssid, RSI_OPEN, NULL);
        } else if (sec_type == 1 || sec_type == '1') {
          status = rsi_wlan_connect((int8_t *)coex_ssid, RSI_WPA, pwd);
        } else if (sec_type == 2 || sec_type == '2') {
          status = rsi_wlan_connect((int8_t *)coex_ssid, RSI_WPA2, pwd);
        } else if (sec_type == 6 || sec_type == '6') {
          status = rsi_wlan_connect((int8_t *)coex_ssid, RSI_WPA_WPA2_MIXED, pwd);
        }
#endif

#if STATIC_AP_OPEN
        status = rsi_wlan_connect((int8_t *)static_ap_ssid, RSI_OPEN, NULL);
#endif

#if STATIC_AP_WPA2_SECURE
        status = rsi_wlan_connect((int8_t *)static_ap_ssid, RSI_WPA2, static_ap_pwd);
#endif

#if STATIC_AP_WPA_SECURE
        status = rsi_wlan_connect((int8_t *)static_ap_ssid, RSI_WPA, static_ap_pwd);
#endif

#if STATIC_AP_MIXED
        status = rsi_wlan_connect((int8_t *)static_ap_ssid, RSI_WPA_WPA2_MIXED, static_ap_pwd);
#endif

        if (status != RSI_SUCCESS) {
#if WYZBEE_CONFIGURATOR
          a++;
          if (a == 5) {
            a       = 0;
            timeout = 1;
            rsi_wlan_app_send_to_ble(RSI_WLAN_TIMEOUT_NOTIFY, (uint8_t *)&timeout, 1);
            rsi_wlan_app_cb.state = RSI_WLAN_ERROR_STATE;
          }
#endif
          LOG_PRINT("\r\nWLAN Connect Failed, Error Code : 0x%X\r\n", status);
          break;
        } else {
          a = 0;
          //! update wlan application state
          rsi_wlan_app_cb.state = RSI_WLAN_CONNECTED_STATE;
        }
      } break;

      case RSI_WLAN_FLASH_STATE: {
        if (retry) {
#if WYZBEE_CONFIGURATOR
          if (sec_type == 0 || sec_type == '0') {
            status = rsi_wlan_connect((int8_t *)coex_ssid, RSI_OPEN, NULL);
          } else if (sec_type == 1 || sec_type == '1') {
            status = rsi_wlan_connect((int8_t *)coex_ssid, RSI_WPA, pwd);
          } else if (sec_type == 2 || sec_type == '2') {
            status = rsi_wlan_connect((int8_t *)coex_ssid, RSI_WPA2, pwd);
          } else if (sec_type == 6 || sec_type == '6') {
            status = rsi_wlan_connect((int8_t *)coex_ssid, RSI_WPA_WPA2_MIXED, pwd);
          }
#endif
          if (status != RSI_SUCCESS) {
            LOG_PRINT("\r\nWLAN Connect Failed, Error Code : 0x%X\r\n", status);
            break;
          } else {
            rsi_wlan_app_cb.state = RSI_WLAN_CONNECTED_STATE;
          }
        }
      } break;

      case RSI_WLAN_CONNECTED_STATE: {
        //! Configure IP
        status = rsi_config_ipaddress(RSI_IP_VERSION_4, RSI_DHCP, 0, 0, 0, rsp_buf, sizeof(rsp_buf), 0);
        if (status != RSI_SUCCESS) {
#if WYZBEE_CONFIGURATOR
          a++;
          if (a == 3) {
            a       = 0;
            timeout = 1;
            status  = rsi_wlan_disconnect();
            if (status == RSI_SUCCESS) {
              connected     = 0;
              disassosiated = 1;
              rsi_wlan_app_send_to_ble(RSI_WLAN_TIMEOUT_NOTIFY, (uint8_t *)&timeout, 1);
              rsi_wlan_app_cb.state = RSI_WLAN_ERROR_STATE;
            }
          }
#endif
          LOG_PRINT("\r\nIP Config Failed, Error Code : 0x%X\r\n", status);
          break;
        } else {
          a             = 0;
          connected     = 1;
          conn_status   = 1;
          disconnected  = 0;
          disassosiated = 0;
          //! update wlan application state
          rsi_wlan_app_cb.state = RSI_WLAN_IPCONFIG_DONE_STATE;

#if WYZBEE_CONFIGURATOR
          rsi_wlan_app_send_to_ble(RSI_WLAN_CONN_STATUS, (uint8_t *)&connected, 1);
#endif
        }

        //! Enable Broadcast data filter
        status = rsi_wlan_filter_broadcast(5000, 1, 1);
        if (status != RSI_SUCCESS) {
          LOG_PRINT("\r\nBroadcast Data Filtering Failed with Error Code : 0x%X\r\n", status);
          return status;
        }
        status = rsi_wlan_power_save_profile(PSP_MODE, PSP_TYPE);
        if (status != RSI_SUCCESS) {
          LOG_PRINT("\r\nWLAN Power Save Failed, Error Code : 0x%X\r\n", status);
          return status;
        } else {
          LOG_PRINT("\r\nWLAN Power Save Success\r\n");
        }
        status = rsi_bt_power_save_profile(PSP_MODE, PSP_TYPE);
        if (status != RSI_SUCCESS) {
          LOG_PRINT("\r\nBLE Power Save Failed, Error Code : 0x%X\r\n", status);
          return status;
        } else {
          LOG_PRINT("\r\nBLE Power Save Success\r\n");
        }
      } break;

      case RSI_WLAN_IPCONFIG_DONE_STATE: {
        wlan_connected        = 1;
        rsi_wlan_app_cb.state = RSI_WLAN_MQTT_INIT_STATE;
#ifdef RSI_WITH_OS
        rsi_wlan_mqtt_task();
#endif
      } break;

      case RSI_WLAN_ERROR_STATE: {

      } break;

      case RSI_WLAN_DISCONNECTED_STATE: {
        retry = 1;
        rsi_wlan_app_send_to_ble(RSI_WLAN_DISCONN_STATUS, (uint8_t *)&disconnected, 1);
        rsi_wlan_app_cb.state = RSI_WLAN_FLASH_STATE;
      } break;

      case RSI_WLAN_DISCONN_NOTIFY_STATE: {
        status = rsi_wlan_disconnect();
        if (status == RSI_SUCCESS) {
#if RSI_WISE_MCU_ENABLE
          rsi_flash_erase((uint32_t)FLASH_ADDR_TO_STORE_AP_DETAILS);
#endif
          LOG_PRINT("\r\nWLAN Disconnect success");
          disassosiated  = 1;
          connected      = 0;
          wlan_connected = 0;
          rsi_wlan_app_send_to_ble(RSI_WLAN_DISCONN_NOTIFY, (uint8_t *)&disassosiated, 1);
          rsi_wlan_app_cb.state = RSI_WLAN_UNCONNECTED_STATE;
        }
      } break;
      default:

        break;
#ifdef RSI_WITH_OS
    }
#endif
  }
}
IoT_Client_Init_Params mqttInitParams   = IoT_Client_Init_Params_initializer;
IoT_Client_Connect_Params connectParams = IoT_Client_Connect_Params_initializer;
void rsi_wlan_mqtt_task()
{
  uint8_t i;
  int32_t status           = 0;
  bool infinitePublishFlag = true;
  IoT_Error_t rc           = FAILURE;

  static AWS_IoT_Client client = { 0 };

  IoT_Publish_Message_Params paramsQOS0;
  IoT_Publish_Message_Params paramsQOS1;

  switch (rsi_wlan_app_cb.state) {
    case RSI_WLAN_MQTT_INIT_STATE: {
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
      rc                                       = aws_iot_mqtt_init(&client, &mqttInitParams);
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
      LOG_PRINT("AWS IOT MQTT Connecting...\n");
      rc = aws_iot_mqtt_connect(&client, &connectParams);
      if (SUCCESS != rc) {
        status = rsi_wlan_power_save_disable_and_enable(PSP_MODE, PSP_TYPE);
        if (status != RSI_SUCCESS) {
          LOG_PRINT("\r\nWLAN Power Save Failed, Error Code : 0x%X\r\n", status);
          //return status;
        } else {
          LOG_PRINT("\r\nWLAN Power Save Success\r\n");
        }

        LOG_PRINT("AWS connection Error :0x%x\n", rc);
        rsi_wlan_app_cb.state = RSI_WLAN_MQTT_INIT_STATE;
      } else {
        // LOG_PRINT("\r\nDNS Query success %d\r\n", ++dns_success);
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
        rsi_wlan_app_cb.state = RSI_WLAN_MQTT_PUBLISH_STATE;
      }
#ifdef RSI_WITH_OS
      rsi_semaphore_post(&rsi_mqtt_sem);
#endif
    } break;
    case RSI_WLAN_MQTT_PUBLISH_STATE: {
      LOG_PRINT("AWS IOT MQTT Publish...\n");
      paramsQOS0.qos        = QOS0;
      paramsQOS0.payload    = (void *)publish_message_mq;
      paramsQOS0.isRetained = 0;

      paramsQOS1.qos        = QOS1;
      paramsQOS1.payload    = (void *)publish_message_mq;
      paramsQOS1.isRetained = 0;
#ifdef MQTT_YIELD_EN
      LOG_PRINT("AWS IOT MQTT Yield ...\n");
      //! Configure LED
      RSI_EGPIO_SetPinMux(EGPIO1, EGPIO_PORT0, GPIO_PIN, EGPIO_PIN_MUX_MODE0);
      RSI_EGPIO_SetDir(EGPIO1, EGPIO_PORT0, GPIO_PIN, EGPIO_CONFIG_DIR_OUTPUT);

      //! Waiting for data from cloud
      LOG_PRINT("Waiting for data from cloud...\n");
      rc = aws_iot_mqtt_yield(&client, 1000);
      if (NETWORK_ATTEMPTING_RECONNECT == rc) {
        // If the client is attempting to reconnect we will skip the rest of the loop.
        continue;
      }
      //! toggle LED based on msg received from cloud
      for (i = 0; i < AWS_IOT_MQTT_RX_BUF_LEN; i++) {
        if (client.clientData.readBuf[i] == 't' && client.clientData.readBuf[i + 1] == 'o'
            && client.clientData.readBuf[i + 2] == 'g') {
          LOG_PRINT("Toggling LED\n");
          RSI_EGPIO_TogglePort(EGPIO1, EGPIO_PORT0, (0x1 << GPIO_PIN));
          break;
        }
      }
#endif
      paramsQOS0.payloadLen = strlen((char *)publish_message_mq);
      rc                    = aws_iot_mqtt_publish(&client, RSI_MQTT_TOPIC, strlen(RSI_MQTT_TOPIC), &paramsQOS0);

      paramsQOS1.payloadLen = strlen((char *)publish_message_mq);
      rc                    = aws_iot_mqtt_publish(&client, RSI_MQTT_TOPIC, strlen(RSI_MQTT_TOPIC), &paramsQOS1);
      if (rc == MQTT_REQUEST_TIMEOUT_ERROR) {
        LOG_PRINT("QOS1 publish ack not received.\n");
        rsi_wlan_app_cb.state = RSI_WLAN_MQTT_PUBLISH_STATE;
      } else if (rc != SUCCESS) {
        LOG_PRINT("\nAWS PUBLISH Error:0x%x\n", rc);
      }
#ifdef RSI_WITH_OS
      rsi_semaphore_post(&rsi_mqtt_sem);
#endif

    } break;
  }
}
