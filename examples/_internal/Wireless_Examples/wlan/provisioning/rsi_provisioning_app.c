/*******************************************************************************
* @file  rsi_provisioning_app.c
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

/*================================================================================
 * @brief : This file contains example application for Provisioning
 * @section Description :
 * The provisioning application demonstrates how to provide provisioning to
 * connect desired Access Point using the http server, MDNS functionality of
 * SiLabs device.
 =================================================================================*/

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

//! socket include file to refer socket APIs
#include "rsi_socket.h"

#include "rsi_bootup_config.h"
//! Error include files
#include "rsi_error.h"

#ifdef RSI_M4_INTERFACE
#include "rsi_board.h"
#endif

//! OS include file to refer OS specific functionality
#include "rsi_os.h"
#include "rsi_nwk.h"

//! include the provisioning sample webpage
#include "provisioning.txt"

#include "rsi_json_handlers.h"
#include "rsi_mdnsd.h"
#include "rsi_utils.h"

//! configurattion Parameters

//! Access point SSID to connect
#define SSID "SILABS_AP"

//!Channel number
#define CHANNEL_NO 2

//! Security type
#define SECURITY_TYPE RSI_OPEN

//! Encryption type
#define ENCRYPTION_TYPE RSI_NONE

//! Password
#define PSK NULL

//! Beacon interval
#define BEACON_INTERVAL 100

//! DTIM interval
#define DTIM_INTERVAL 4

//! IP version
#define MDNSD_IP_VERSION 4

//! Time to live for MDNSD host name
#define MDNSD_INIT_TTL 300

//! Time to live for added service
#define MDNSD_SERVICE_TTL 300

//! MDNSD service port
#define MDNSD_SERVICE_PORT 80

//! MDNSD more services bit: set to 0 if it is last service
#define MDNSD_SERVICE_MORE 0

//! MDNSD host name
#define MDNSD_HOST_NAME "wiseconnect.local."

//! MDNSD service pointer name
#define MDNSD_POINTER_NAME "_http._tcp.local."

//! MDNSD service name
#define MDNSD_SERVICE_NAME "wiseconnect._http._tcp.local"

//! MDNSD service text
#define MDNSD_SERVICE_TEXT "text_field"

//! File name of the webpage and json object
#define FILE_NAME "provisioning.html"

//! WEB_PAGE associated to JSON 8- ENABLE 0- Disable
#define WEB_PAGE_ASSOCIATED_TO_JSON 8

//! Flags
#define FLAGS WEB_PAGE_ASSOCIATED_TO_JSON

//! IP address of the module
//! E.g: 0x650AA8C0 == 192.168.10.101
#define DEVICE_IP 0x010AA8C0

//! IP address of Gateway
//! E.g: 0x010AA8C0 == 192.168.10.1
#define GATEWAY 0x010AA8C0

//! IP address of netmask
//! E.g: 0x00FFFFFF == 255.255.255.0
#define NETMASK 0x00FFFFFF

#define HTTP_SERVER_USERNAME "admin"

#define HTTP_SERVER_PASSWORD "silabs"

//! Json object data buffer
uint8_t json_object_buffer[512];
uint8_t scan_result[513];

//! Json object data structure
rsi_json_object_t json_object_data;

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

//! standard defines
uint8_t rsp_received;

//! Memory to initialize driver
uint8_t global_buf[GLOBAL_BUFF_LEN];

//! Function prototypes
void main_loop(void);

//! Scan submit loop
int32_t scan_loop(void);

//! json object update handler
void rsi_json_object_update_handler(uint8_t *filename, uint8_t *json_object_str, uint32_t length, uint32_t status);

//! json object event handler
void rsi_json_object_event_handler(uint32_t status, uint8_t *json_object_str, uint32_t length);

//! webpage request handler
void rsi_webpage_request_handler(uint8_t type,
                                 uint8_t *url_name,
                                 uint8_t *post_content_buffer,
                                 uint32_t post_content_length,
                                 uint32_t status);

//! wireless firmware upgrade handler
void rsi_wireless_fw_upgrade_handler(uint8_t type, uint32_t status);

int32_t rsi_wc_provisioning(void);
//! To register HTTP server related callbacks
void rsi_http_server_cbs_init(void)
{
  //! Register json object update callback
  rsi_wlan_nwk_register_json_update_cb(RSI_WLAN_NWK_JSON_UPDATE_CB, rsi_json_object_update_handler);

  //! Register json event callback
  rsi_wlan_nwk_register_json_event_cb(RSI_WLAN_NWK_JSON_EVENT_CB, rsi_json_object_event_handler);

  //! Register webpage request callback
  rsi_wlan_nwk_register_webpage_req_cb(RSI_WLAN_NWK_URL_REQ_CB, rsi_webpage_request_handler);

  //! Register wirless firmware upgrade callback
  rsi_wlan_nwk_register_wireless_fw_upgrade_cb(RSI_WLAN_NWK_FW_UPGRADE_CB, rsi_wireless_fw_upgrade_handler);
}

//! Provisioning Application
int32_t rsi_provisioning_app()
{
  int32_t status = RSI_SUCCESS;
#if !(DHCP_MODE)
  uint32_t ip_addr      = DEVICE_IP;
  uint32_t network_mask = NETMASK;
  uint32_t gateway      = GATEWAY;
#endif
  uint8_t *json_object_str;

  //! WC initialization
  status = rsi_wireless_init(6, 0);
  if (status != RSI_SUCCESS) {
    LOG_PRINT("\r\nWireless Initialization Failed, Error Code : 0x%X\r\n", status);
    return status;
  } else {
    LOG_PRINT("\r\nWireless Initialization Success\r\n");
  }

  status = rsi_http_credentials((int8_t *)HTTP_SERVER_USERNAME, (int8_t *)HTTP_SERVER_PASSWORD);
  if (status != RSI_SUCCESS) {
    LOG_PRINT("\r\nHTTP Credentials Failed, Error Code : 0x%X\r\n", status);
    return status;
  } else {
    LOG_PRINT("\r\nHTTP Credentials Success\r\n");
  }

  //! register HTTP server call backs
  rsi_http_server_cbs_init();

  status = rsi_webpage_erase(FILE_NAME);
  //! Status will return ERROR for the first time becasue there is no preloaded webpgae to erase

  //! Load sample webpage
  status = rsi_webpage_load(FLAGS, FILE_NAME, provisioning, (sizeof(provisioning) - 1));
  if (status != RSI_SUCCESS) {
    LOG_PRINT("\r\nLoad Webpage Failed, Error Code : 0x%X\r\n", status);
    return status;
  } else {
    LOG_PRINT("\r\nLoad Webpage Success\r\n");
  }

  //! Initialize the json object data structure with default values
  rsi_json_object_init(&json_object_data);

  //! Prepare json object string from json object data structure
  json_object_str = rsi_json_object_stringify(json_object_buffer, &json_object_data);

  //! Create json object data to the assoicated webpage
  status = rsi_json_object_create(0, FILE_NAME, json_object_str, strlen(json_object_str));
  if (status != RSI_SUCCESS) {
    LOG_PRINT("\r\nCreate JSON Object Failed, Error Code : 0x%X\r\n", status);
    return status;
  } else {
    LOG_PRINT("\r\nCreate JSON Object Success\r\n");
  }

  //! Configure IP
  status = rsi_config_ipaddress(RSI_IP_VERSION_4,
                                RSI_STATIC,
                                (uint8_t *)&ip_addr,
                                (uint8_t *)&network_mask,
                                (uint8_t *)&gateway,
                                NULL,
                                0,
                                0);
  if (status != RSI_SUCCESS) {
    LOG_PRINT("\r\nIP Config Failed, Error Code : 0x%X\r\n", status);
    return status;
  } else {
    LOG_PRINT("\r\nIP Config Success\r\n");
  }

  //! Start Access point
  status =
    rsi_wlan_ap_start((int8_t *)SSID, CHANNEL_NO, SECURITY_TYPE, ENCRYPTION_TYPE, PSK, BEACON_INTERVAL, DTIM_INTERVAL);
  if (status != RSI_SUCCESS) {
    LOG_PRINT("\r\nAP Start Failed, Error Code : 0x%X\r\n", status);
    return status;
  } else {
    LOG_PRINT("\r\nAP Start Success\r\n");
  }

  //! Initialize MDNSD service
  status = rsi_mdnsd_init((uint8_t)MDNSD_IP_VERSION, (uint16_t)MDNSD_INIT_TTL, (uint8_t *)MDNSD_HOST_NAME);

  if (status != RSI_SUCCESS) {
    LOG_PRINT("\r\nInitialize MDNSD service Failed, Error Code : 0x%X\r\n", status);
    return status;
  } else {
    LOG_PRINT("\r\nInitialize MDNSD service Success\r\n");
  }

  //! Add required services
  status = rsi_mdnsd_register_service(MDNSD_SERVICE_PORT,
                                      MDNSD_SERVICE_TTL,
                                      MDNSD_SERVICE_MORE,
                                      MDNSD_POINTER_NAME,
                                      MDNSD_SERVICE_NAME,
                                      MDNSD_SERVICE_TEXT);

  if (status != RSI_SUCCESS) {
    LOG_PRINT("\r\nAdd MDNSD register service Failed, Error Code : 0x%X\r\n", status);
    return status;
  } else {
    LOG_PRINT("\r\nAdd MDNSD register service Success\r\n");
  }

  //! wait for the success response
  status = scan_loop();
  if (status != RSI_SUCCESS) {
    return status;
  }

  //!De-initialize MSDNSD service
  status = rsi_mdnsd_deinit();
  if (status != RSI_SUCCESS) {
    LOG_PRINT("\r\nDe-initialize MSDNSD service Failed, Error Code : 0x%X\r\n", status);
    return status;
  } else {
    LOG_PRINT("\r\nDe-initialize MSDNSD service Success\r\n");
  }

  //! Connect the AP based on Updated JSON object data vaues
  rsi_wc_provisioning();
  return 0;
}

int32_t rsi_wc_provisioning(void)
{

  int32_t status = RSI_SUCCESS;
  uint8_t *json_object_str;

  //! Get Configuraton parameters from JSON object data structure
  uint8_t *ssid      = json_object_data.ssid;
  uint8_t channel    = json_object_data.channel;
  uint8_t sec_enable = json_object_data.sec_enable;
  uint8_t sec_type   = json_object_data.sec_type;
  uint8_t *psk       = json_object_data.psk;
  uint8_t dhcp_mode  = (RSI_DHCP | RSI_DHCP_UNICAST_OFFER);

  //! Check for security enable
  if (!sec_enable) {
    //! Set to OPEN security mode
    sec_type = RSI_OPEN;

    //! Set to emty PSK
    psk = NULL;
  }

  //! Disconnect to the connect AP
  status = rsi_wireless_deinit();
  if (status != RSI_SUCCESS) {
    LOG_PRINT("\r\nWireless De-Initialization Failed, Error Code : 0x%X\r\n", status);
    return status;
  } else {
    LOG_PRINT("\r\nWireless De-Initialization Success\r\n");
  }

  //! WC initialization
  status = rsi_wireless_init(0, 0);
  if (status != RSI_SUCCESS) {
    LOG_PRINT("\r\nWireless Initialization Failed, Error Code : 0x%X\r\n", status);
    return status;
  } else {
    LOG_PRINT("\r\nWireless Initialization Success\r\n");
  }

  //! Set client connected state
  json_object_data.conec = 3;

  //! Prepare json object string from json object data structure
  json_object_str = rsi_json_object_stringify(json_object_buffer, &json_object_data);

  //! Create json object data to the assoicated webpage
  status = rsi_json_object_create(0, FILE_NAME, json_object_str, strlen((const char *)json_object_str));
  if (status != RSI_SUCCESS) {
    LOG_PRINT("\r\nCreate JSON Object Failed, Error Code : 0x%X\r\n", status);
    return status;
  } else {
    LOG_PRINT("\r\nCreate JSON Object Success\r\n");
  }

  //! Scan for Access points
  status = rsi_wlan_scan((int8_t *)ssid, channel, NULL, 0);
  if (status != RSI_SUCCESS) {
    LOG_PRINT("\r\nWLAN AP Scan Failed, Error Code : 0x%X\r\n", status);
    return status;
  } else {
    LOG_PRINT("\r\nWLAN AP Scan Success\r\n");
  }

  //! Connect to an Access point
  status = rsi_wlan_connect((int8_t *)ssid, (rsi_security_mode_t)sec_type, psk);
  if (status != RSI_SUCCESS) {
    LOG_PRINT("\r\nWLAN AP Connect Failed, Error Code : 0x%X\r\n", status);
    return status;
  } else {
    LOG_PRINT("\r\nWLAN AP Connect Success\r\n");
  }

  //! Configure IP through DHCP
  status = rsi_config_ipaddress(RSI_IP_VERSION_4, dhcp_mode, 0, 0, 0, NULL, 0, 0);
  if (status != RSI_SUCCESS) {
    LOG_PRINT("\r\nIP Config Failed, Error Code : 0x%X\r\n", status);
    return status;
  } else {
    LOG_PRINT("\r\nIP Config Success\r\n");
  }

#ifndef RSI_WITH_OS
  //! wait for the success response
  main_loop();
#endif

  //! Connect the AP based on Updated JSON object data vaues
  rsi_wc_provisioning();

  return status;
}

int32_t scan_loop(void)
{
  int32_t status = RSI_SUCCESS;
  uint8_t *json_object_str;

  while (1) {
#ifndef RSI_WITH_OS
    //! event loop
    rsi_wireless_driver_task();
#endif

    if (rsp_received == 3) {

      //!Delete json object data before updating scan results
      status = rsi_json_object_delete(FILE_NAME);
      if (status != RSI_SUCCESS) {
        LOG_PRINT("\r\nDelete JSON Object Failed, Error Code : 0x%X\r\n", status);
        return status;
      } else {
        LOG_PRINT("\r\nDelete JSON Object Success\r\n");
      }

      //! Scan for Access points
      status = rsi_wlan_scan(NULL, 0, (rsi_rsp_scan_t *)scan_result, sizeof(rsi_rsp_scan_t));
      if (status != RSI_SUCCESS) {
        LOG_PRINT("\r\nWLAN AP Scan Failed, Error Code : 0x%X\r\n", status);
        return status;
      } else {
        LOG_PRINT("\r\nWLAN AP Scan Success\r\n");
      }

      rsi_json_object_scan_list_update(&json_object_data, (rsi_rsp_scan_t *)scan_result);

      json_object_data.conec = 2;

      //! Prepare json object string from json object data structure
      json_object_str = rsi_json_object_stringify(json_object_buffer, &json_object_data);

      //! Create json object data to the assoicated webpage
      status = rsi_json_object_create(0, FILE_NAME, json_object_str, strlen((const char *)json_object_str));
      if (status != RSI_SUCCESS) {
        LOG_PRINT("\r\nCreate JSON Object Failed, Error Code : 0x%X\r\n", status);
        return status;
      } else {
        LOG_PRINT("\r\nCreate JSON Object Success\r\n");
      }

      rsp_received = 0;
    }

    if (rsp_received) {
      rsp_received = 0;
      return status;
    }
  }
}

void main_loop(void)
{
  while (1) {
    ////////////////////////
    //! Application code ///
    ////////////////////////

    //! event loop
    rsi_wireless_driver_task();

    if (rsp_received) {
      rsp_received = 0;
      break;
    }
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
    LOG_PRINT("\r\nDevice Initialization Failed, Error Code : 0x%X\r\n", status);
    return status;
  } else {
    LOG_PRINT("\r\nDevice Initialization Success\r\n");
  }

#ifdef RSI_WITH_OS
  //! OS case
  //! Task created for WLAN task
  rsi_task_create(rsi_provisioning_app,
                  "wlan_task",
                  RSI_WLAN_TASK_STACK_SIZE,
                  NULL,
                  RSI_WLAN_TASK_PRIORITY,
                  &wlan_task_handle);

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
  //! Call  Provisioning application
  status = rsi_provisioning_app();

  //! Application main loop
  main_loop();
#endif
  return status;
}

//! json object update handler
void rsi_json_object_update_handler(uint8_t *filename, uint8_t *json_object_str, uint32_t length, uint32_t status)
{
  if (status == RSI_SUCCESS) {
    //! Update the JSON object data structure with received JSON object data
    rsi_json_object_update(&json_object_data, json_object_str, filename);
    rsp_received = 1;
  }
}

void rsi_json_object_event_handler(uint32_t status, uint8_t *json_object_str, uint32_t length)
{
  if (status == RSI_SUCCESS) {
    //! Update the JSON object data structure with received JSON object data
    rsp_received = 3;
  }
}

//! webpage request handler
void rsi_webpage_request_handler(uint8_t type,
                                 uint8_t *url_name,
                                 uint8_t *post_content_buffer,
                                 uint32_t post_content_length,
                                 uint32_t status)
{
  if (status == RSI_SUCCESS) {
    rsp_received = 1;
  }
}

//! wireless firmware upgrade handler
void rsi_wireless_fw_upgrade_handler(uint8_t type, uint32_t status)
{
  if (status == RSI_SUCCESS) {
    rsp_received = 1;
  }
}
