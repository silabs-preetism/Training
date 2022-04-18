/*******************************************************************************
* @file  rsi_wlan_http_s.c
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
 * @file    rsi_wlan_http_s.c
 * @version 0.1
 * @date    01 May 2021
 *
 *
 *  @brief : This file contains example application for TCP+SSL client socket
 *
 *  @section Description  This file contains example application for TCP+SSL client socket
 *
 *
 */
/*=======================================================================*/
//  ! INCLUDES
/*=======================================================================*/
#ifdef FRDM_K28F
#include "rsi_common_app.h"
#endif
#include "stdlib.h"
#include "rsi_driver.h"
#include "rsi_utils.h"
#include <rsi_wlan_non_rom.h>
#include <rsi_bt_common_apis.h>
#include "rsi_common_config.h"
#include "rsi_socket.h"
#include "rsi_wlan_config.h"
#if RSI_ENABLE_WLAN_TEST

#if SSL
#include "servercert.pem" //! Include SSL CA certificate
#endif

/*=======================================================================*/
//   ! MACROS
/*=======================================================================*/

/*=======================================================================*/
//   ! GLOBAL VARIABLES
/*=======================================================================*/
struct rsi_sockaddr_in server_addr, client_addr; //! server and client IP addresses

rsi_wlan_app_cb_t rsi_wlan_app_cb; //! application control block
int32_t client_socket;             //! client socket id
//! Throughput parameters
uint32_t pkts       = 0;
uint64_t num_bits   = 0;
uint64_t total_bits = 0;
uint32_t xfer_time;
uint32_t total_time = 0;
uint64_t xfer_time_usec;
uint32_t t_end;
float throughput;
float throughput_mbps;
volatile uint8_t data_recvd = 0;
volatile uint64_t num_bytes = 0;
uint8_t ip[18]              = { 0 };
//! HTTP/HTTPS headers
#if HTTPS_DOWNLOAD
const char httpreq[] = "GET /" DOWNLOAD_FILENAME " HTTPS/1.1\r\n"
                       "Host: " SERVER_IP_ADDRESS "\r\n"
                       "User-Agent: silabs/1.0.4a\r\n"
                       "Accept: */*\r\n";
#else
const char httpreq[] = "GET " DOWNLOAD_FILENAME " HTTP/1.1\r\n"
                       "Host: " SERVER_IP_ADDRESS "\r\n"
                       "User-Agent: silabs/1.0.4a\r\n"
                       "Accept: */*\r\n";
const char http_req_str_end[] = "\r\n";
#endif
const char http_req_str_connection_close[] = "Connection: close\r\n";
int8_t recv_buffer1[RECV_BUFFER_SIZE];
int8_t recv_buffer2[RECV_BUFFER_SIZE];
int32_t recv_size2 = RECV_BUFFER_SIZE;

/*=======================================================================*/
//   ! EXTERN VARIABLES
/*=======================================================================*/
extern rsi_semaphore_handle_t ble_main_task_sem, ble_slave_conn_sem, bt_app_sem, wlan_app_sem, bt_inquiry_sem,
  ble_scan_sem;
#if WLAN_SYNC_REQ
extern rsi_semaphore_handle_t sync_coex_ble_sem, sync_coex_bt_sem;
#endif
extern bool rsi_ble_running, rsi_bt_running, rsi_wlan_running, wlan_radio_initialized, powersave_cmd_given;
extern rsi_mutex_handle_t power_cmd_mutex;
extern rsi_semaphore_handle_t wlan_app_sem;
#if BLE_PROVISIONING_ENABLED
extern uint8_t coex_ssid[50], dis_ssid[50], pwd[34], sec_type;
uint8_t retry;
rsi_rsp_scan_t scan_result;
uint16_t scanbuf_size;
uint8_t connected = 0, timeout = 0;
uint8_t disconnected = 0, disassosiated = 0;
uint8_t conn_retries = 0;
extern uint8_t ble_provision_conn_id;
extern void rsi_wlan_app_send_to_ble(uint8_t conn_id, uint16_t msg_type, uint8_t *data, uint16_t data_len);
#endif
/*=======================================================================*/
//   ! EXTERN FUNCTIONS
/*=======================================================================*/

/*=======================================================================*/
//   ! PROCEDURES
/*=======================================================================*/
void rsi_remote_socket_terminate_handler(uint16_t status, uint8_t *buffer, const uint32_t length);
void rsi_join_fail_handler(uint16_t status, uint8_t *buffer, const uint32_t length);
void rsi_ip_renewal_fail_handler(uint16_t status, uint8_t *buffer, const uint32_t length);
void rsi_remote_socket_terminate_handler(uint16_t status, uint8_t *buffer, const uint32_t length);
void rsi_ip_change_notify_handler(uint16_t status, uint8_t *buffer, const uint32_t length);
void rsi_stations_connect_notify_handler(uint16_t status, uint8_t *buffer, const uint32_t length);
void rsi_stations_disconnect_notify_handler(uint16_t status, uint8_t *buffer, const uint32_t length);

/*************************************************************************/
//!  CALLBACK FUNCTIONS
/*************************************************************************/
/*====================================================*/
/**
 * @fn          void rsi_wlan_app_callbacks_init(void)
 * @brief       To initialize WLAN application callback
 * @param[in]   void
 * @return      void
 * @section description
 * This callback is used to initialize WLAN
 * ==================================================*/
void rsi_wlan_app_callbacks_init(void)
{
  rsi_wlan_register_callbacks(RSI_JOIN_FAIL_CB, rsi_join_fail_handler);     //! Initialize join fail call back
  rsi_wlan_register_callbacks(RSI_IP_FAIL_CB, rsi_ip_renewal_fail_handler); //! Initialize IP renewal fail call back
  rsi_wlan_register_callbacks(RSI_REMOTE_SOCKET_TERMINATE_CB,
                              rsi_remote_socket_terminate_handler); //! Initialize remote terminate call back
  rsi_wlan_register_callbacks(RSI_IP_CHANGE_NOTIFY_CB,
                              rsi_ip_change_notify_handler); //! Initialize IP change notify call back
  rsi_wlan_register_callbacks(RSI_STATIONS_CONNECT_NOTIFY_CB,
                              rsi_stations_connect_notify_handler); //! Initialize IP change notify call back
  rsi_wlan_register_callbacks(RSI_STATIONS_DISCONNECT_NOTIFY_CB,
                              rsi_stations_disconnect_notify_handler); //! Initialize IP change notify call back
}

/*====================================================*/
/**
 * @fn         void socket_async_recive(uint32_t sock_no, uint8_t *buffer, uint32_t length)
 * @brief      Function to create Async socket
 * @param[in]  uint32_t sock_no, uint8_t *buffer, uint32_t length
 * @return     void
 * @section description
 * Callback for Socket Async Receive
 * ====================================================*/
void socket_async_recive(uint32_t sock_no, uint8_t *buffer, uint32_t length)
{
  UNUSED_PARAMETER(length);  //This statement is added only to resolve compilation warning, value is unchanged
  UNUSED_PARAMETER(buffer);  //This statement is added only to resolve compilation warning, value is unchanged
  UNUSED_PARAMETER(sock_no); //This statement is added only to resolve compilation warning, value is unchanged
  num_bytes += length;
}

#if SSL
/*====================================================*/
/**
 * @fn         int32_t rsi_app_load_ssl_cert()
 * @brief      Function to load SSL certificate
 * @param[in]  void
 * @return     void
 *====================================================*/
int32_t rsi_app_load_ssl_cert()
{
  int32_t status = RSI_SUCCESS;
  status         = rsi_wlan_set_certificate(RSI_SSL_CA_CERTIFICATE, NULL, 0); //! erase existing certificate
  if (status != RSI_SUCCESS) {
    LOG_PRINT("CA cert erase failed \r\n");
    return status;
  }

  status =
    rsi_wlan_set_certificate(RSI_SSL_CA_CERTIFICATE, servercert, (sizeof(servercert) - 1)); //! Load SSL CA certificate
  if (status != RSI_SUCCESS) {
    LOG_PRINT("CA cert load failed \r\n");
    return status;
  }
  return status;
}
#endif

int32_t rsi_app_wlan_socket_create()
{
  int32_t status = RSI_SUCCESS;

#if SOCKET_ASYNC_FEATURE
#if SSL
  client_socket = rsi_socket_async(AF_INET, SOCK_STREAM, 1, socket_async_recive);
#else
  client_socket = rsi_socket_async(AF_INET, SOCK_STREAM, 0, socket_async_recive);
#endif
#else
  client_socket = rsi_socket(AF_INET, SOCK_STREAM, 0);
#endif
  if (client_socket < 0) {
    LOG_PRINT("socket open failed\n");
    status = rsi_wlan_get_status();
    return status;
  }

  LOG_PRINT("socket create\n");

#if HTTPS_DOWNLOAD
  status = rsi_setsockopt(client_socket, SOL_SOCKET, SO_SSL_ENABLE, NULL, 0);
  if (status != RSI_SUCCESS) {
    return status;
  }
#endif
  //! Reset server structure
  memset(&server_addr, 0, sizeof(server_addr));

  //! Set server address family
  server_addr.sin_family = AF_INET;

  //! Set server port number, using htons function to use proper byte order
  server_addr.sin_port = htons(SERVER_PORT);

  //! Set IP address to localhost
  server_addr.sin_addr.s_addr = ip_to_reverse_hex((char *)SERVER_IP_ADDRESS);

  //LOG_PRINT("socket connect\r\n");

  //! Connect to server socket
  status = rsi_connect(client_socket, (struct rsi_sockaddr *)&server_addr, sizeof(server_addr));
  if (status != RSI_SUCCESS) {
    status = rsi_wlan_get_status();
    rsi_shutdown(client_socket, 0);
    LOG_PRINT("socket connect failed\n");
    return status;
  }

  return status;
}

/*====================================================*/
/**
 * @fn         int32_t  rsi_wlan_app_task(void)
 * @brief      Function to work with application task
 * @param[in]  void
 * @return     void
 *=====================================================*/
int32_t rsi_wlan_app_task(void)
{
  int32_t status        = RSI_SUCCESS;
  uint8_t stop_download = 0;
  uint8_t dhcp_mode     = (RSI_DHCP | RSI_DHCP_UNICAST_OFFER);
  uint32_t i;
  volatile uint8_t download_complete = 0;
  uint8_t no_of_iterations           = 0;
  uint32_t bytes_cnt                 = 0;

  while (1) {
    switch (rsi_wlan_app_cb.state) {
      case RSI_POWER_SAVE_STATE: {

      } break;
      case RSI_WLAN_INITIAL_STATE: {
        rsi_wlan_app_callbacks_init(); //! register callback to initialize WLAN
#if BLE_PROVISIONING_ENABLED
        LOG_PRINT("run ble provisioning app \r\n");
        rsi_wlan_app_cb.state = RSI_WLAN_UNCONNECTED_STATE;
#else
        rsi_wlan_app_cb.state = RSI_WLAN_SCAN_STATE;
#endif

#if ENABLE_POWER_SAVE
        rsi_mutex_lock(&power_cmd_mutex);
        if (!powersave_cmd_given) {
          status = rsi_initiate_power_save();
          if (status != RSI_SUCCESS) {
            LOG_PRINT("failed to keep module in power save \r\n");
            return status;
          }
          powersave_cmd_given = true;
        }
        rsi_mutex_unlock(&power_cmd_mutex);
        LOG_PRINT("Module is in deepsleep \r\n");
#endif
      } break;
      case RSI_WLAN_UNCONNECTED_STATE: {
        //! do nothing
      } break;
      case RSI_WLAN_SCAN_STATE: {
        LOG_PRINT("WLAN scan started \r\n");
#if BLE_PROVISIONING_ENABLED
        scanbuf_size = sizeof(scan_result);
        memset(&scan_result, 0, sizeof(scan_result));
        status = rsi_wlan_scan(0, 0, &scan_result, scanbuf_size);
        if (status != RSI_SUCCESS) {
          LOG_PRINT("\r\nWLAN Scan Failed, Error Code : 0x%lX\r\n", status);
          break;
        } else {
          //! update wlan application state
          rsi_wlan_app_cb.state = RSI_WLAN_UNCONNECTED_STATE;
          if (ble_provision_conn_id != 0xff) {
            rsi_wlan_app_send_to_ble(ble_provision_conn_id, RSI_WLAN_SCAN_RESP, (uint8_t *)&scan_result, scanbuf_size);
          } else {
            LOG_PRINT("ble");
          }
        }
#else
        status                = rsi_wlan_scan((int8_t *)SSID, (uint8_t)CHANNEL_NO, NULL, 0);
        if (status != RSI_SUCCESS) {
          LOG_PRINT("scan failed \r\n");
          break;
        } else {
          rsi_wlan_app_cb.state = RSI_WLAN_JOIN_STATE; //! update WLAN application state to connected state
#if ENABLE_POWER_SAVE
          LOG_PRINT("Module is in standby \r\n");
#endif
          LOG_PRINT("wlan scan done \r\n");
        }
#endif
      } break;
      case RSI_WLAN_JOIN_STATE: {
#if BLE_PROVISIONING_ENABLED
        if (sec_type == 0 || sec_type == '0') {
          status = rsi_wlan_connect((int8_t *)coex_ssid, RSI_OPEN, NULL);
        } else if (sec_type == 1 || sec_type == '1') {
          status = rsi_wlan_connect((int8_t *)coex_ssid, RSI_WPA, pwd);
        } else if (sec_type == 2 || sec_type == '2') {
          status = rsi_wlan_connect((int8_t *)coex_ssid, RSI_WPA2, pwd);
        } else if (sec_type == 6 || sec_type == '6') {
          status = rsi_wlan_connect((int8_t *)coex_ssid, RSI_WPA_WPA2_MIXED, pwd);
        }
#else
        status = rsi_wlan_connect((int8_t *)SSID, SECURITY_TYPE, PSK);
#endif
        if (status != RSI_SUCCESS) {
#if BLE_PROVISIONING_ENABLED
          conn_retries++;
          if (conn_retries == 5) {
            conn_retries = 0;
            timeout      = 1;
            rsi_wlan_app_send_to_ble(ble_provision_conn_id, RSI_WLAN_TIMEOUT_NOTIFY, (uint8_t *)&timeout, 1);
            rsi_wlan_app_cb.state = RSI_WLAN_ERROR_STATE;
          }
#endif
          LOG_PRINT("WLAN Connect Failed, Error Code : 0x%lX\r\n", status);
          break;
        } else {
#if BLE_PROVISIONING_ENABLED
          conn_retries = 0;
#endif
          LOG_PRINT("WLAN connected state \r\n");
          //! update wlan application state
          rsi_wlan_app_cb.state = RSI_WLAN_CONNECTED_STATE;
        }
      } break;
      case RSI_WLAN_CONNECTED_STATE: {
        //! Configure IP
        status = rsi_config_ipaddress(RSI_IP_VERSION_4, dhcp_mode, 0, 0, 0, ip, sizeof(ip), 0);
        if (status != RSI_SUCCESS) {
#if BLE_PROVISIONING_ENABLED
          conn_retries++;
          if (conn_retries == 3) {
            conn_retries = 0;
            timeout      = 1;
            status       = rsi_wlan_disconnect();
            if (status == RSI_SUCCESS) {
              connected     = 0;
              disassosiated = 1;
              rsi_wlan_app_send_to_ble(ble_provision_conn_id, RSI_WLAN_TIMEOUT_NOTIFY, (uint8_t *)&timeout, 1);
              rsi_wlan_app_cb.state = RSI_WLAN_ERROR_STATE;
            }
          }
#endif
          LOG_PRINT("IP Config failed \r\n");
          break;
        } else {
          rsi_wlan_app_cb.state = RSI_WLAN_IPCONFIG_DONE_STATE;
          LOG_PRINT("WLAN ipconfig done state \r\n");
          LOG_PRINT("RSI_STA IP ADDR: %d.%d.%d.%d \r\n", ip[6], ip[7], ip[8], ip[9]);
#if BLE_PROVISIONING_ENABLED
          conn_retries  = 0;
          connected     = 1;
          disconnected  = 0;
          disassosiated = 0;
          rsi_wlan_app_send_to_ble(ble_provision_conn_id, RSI_WLAN_CONN_STATUS, (uint8_t *)&connected, 1);
#endif
        }
#if (SSL && LOAD_CERTIFICATE)
        status = rsi_app_load_ssl_cert();
        if (status != RSI_SUCCESS) {
          break;
        }
#endif
      }
      //no break
      case RSI_WLAN_IPCONFIG_DONE_STATE: {
        if (stop_download)
          break;

        if (data_recvd) {
          //! Clear data receive flag
          data_recvd = 0;
#if HTTPS_DOWNLOAD
          LOG_PRINT("\r\n HTTPS download completed \r\n");
#elif !HTTPS_DOWNLOAD
          LOG_PRINT("\r\n HTTP download completed \r\n");
#endif
          status = rsi_shutdown(client_socket, 0);
          if (status != RSI_SUCCESS) {
            LOG_PRINT("WLAN shutdown failed\n");
            return status;
          }
          LOG_PRINT("\r\n closing the socket\r\n");
          rsi_os_task_delay(50);
#if !CONTINUOUS_HTTP_DOWNLOAD
          stop_download = 1;
          break;
#endif
        }
        num_bytes = 0;

#if HIGH_PERFORMANCE_ENABLE
        status = rsi_socket_config();
        if (status < 0) {
          LOG_PRINT("high-performance socket config failed \r\n");
          status = rsi_wlan_get_status();
          break;
        }
        //LOG_PRINT("high-performance socket config success \r\n");
#endif

        //! Create socket and connect to server
        status = rsi_app_wlan_socket_create();
        if (status != RSI_SUCCESS) {
          break;
        } else {
          //! update wlan application state
          rsi_wlan_app_cb.state = RSI_WLAN_SOCKET_CONNECTED_STATE;
          LOG_PRINT("\r\n Module connected to the server \r\n");
        }
      }
      case RSI_WLAN_SOCKET_CONNECTED_STATE: {
#if WLAN_SYNC_REQ
        //! unblock other protocol activities
        if (rsi_bt_running) {
          rsi_semaphore_post(&sync_coex_bt_sem);
        }
        if (rsi_ble_running) {
          rsi_semaphore_post(&sync_coex_ble_sem);
        }
#endif
        /* Send first set of HTTP/HTTPS headers to server */
        bytes_cnt = 0;
        while (bytes_cnt != strlen(httpreq)) {
          status = rsi_send(client_socket, (const int8_t *)(httpreq + bytes_cnt), (strlen(httpreq) - bytes_cnt), 0);
          if (status < 0) {
            status = rsi_wlan_get_status();
            rsi_shutdown(client_socket, 0);
            LOG_PRINT("send failed\n");
            rsi_wlan_app_cb.state = RSI_WLAN_IPCONFIG_DONE_STATE;
            break;
          }
          bytes_cnt += status;
        }

        /* Send connection close headers to server */
#if USE_CONNECTION_CLOSE
        bytes_cnt = 0;
        while (bytes_cnt != strlen(http_req_str_connection_close)) {
          status = rsi_send(client_socket,
                            (const int8_t *)(http_req_str_connection_close + bytes_cnt),
                            (strlen(http_req_str_connection_close) - bytes_cnt),
                            0);
          if (status < 0) {
            status = rsi_wlan_get_status();
            rsi_shutdown(client_socket, 0);
            LOG_PRINT("\r\n send failed\r\n");
            rsi_wlan_app_cb.state = RSI_WLAN_IPCONFIG_DONE_STATE;
            break;
          }
          bytes_cnt += status;
        }
#endif
        /* Send last set of HTTP headers to server */
#if !HTTPS_DOWNLOAD
        bytes_cnt = 0;
        while (bytes_cnt != strlen(http_req_str_end)) {
          status = rsi_send(client_socket,
                            (const int8_t *)(http_req_str_end + bytes_cnt),
                            (strlen(http_req_str_end) - bytes_cnt),
                            0);
          if (status < 0) {
            status = rsi_wlan_get_status();
            rsi_shutdown(client_socket, 0);
            LOG_PRINT("send failed\n");
            rsi_wlan_app_cb.state = RSI_WLAN_IPCONFIG_DONE_STATE;
            break;
          }
          bytes_cnt += status;
        }
#endif
        rsi_wlan_app_cb.state = RSI_WLAN_DATA_RECEIVE_STATE;
#if HTTPS_DOWNLOAD
        LOG_PRINT("\r\n HTTPS download started \r\n");
#elif !HTTPS_DOWNLOAD
        LOG_PRINT("\r\n HTTP download started \r\n");
#endif
        break;
      }
      case RSI_WLAN_DATA_RECEIVE_STATE: {
#if !SOCKET_ASYNC_FEATURE
        status = rsi_recv(client_socket, recv_buffer2, recv_size2, 0);
        if (status < 0) {
          status = rsi_wlan_get_status();
          if (status == RSI_RX_BUFFER_CHECK) {
            continue;
          } else {
            LOG_PRINT("\r\n failed to receive packets, status =%d\r\n", status);
          }
        }
#endif
      } break;
      case RSI_WLAN_DISCONNECTED_STATE: {
#if BLE_PROVISIONING_ENABLED
        retry = 1;
        rsi_wlan_app_send_to_ble(ble_provision_conn_id, RSI_WLAN_DISCONN_STATUS, (uint8_t *)&disconnected, 1);
#endif
        rsi_wlan_app_cb.state = RSI_WLAN_JOIN_STATE;
      } break;
      case RSI_WLAN_DISCONN_NOTIFY_STATE: {
        status = rsi_wlan_disconnect();
        if (status == RSI_SUCCESS) {
          LOG_PRINT("\r\nWLAN Disconnect Failed, Error Code : 0x%lX\r\n", status);
#if BLE_PROVISIONING_ENABLED
          disassosiated = 1;
          connected     = 0;
          rsi_wlan_app_send_to_ble(ble_provision_conn_id, RSI_WLAN_DISCONN_NOTIFY, (uint8_t *)&disassosiated, 1);
#endif
          rsi_wlan_app_cb.state = RSI_WLAN_UNCONNECTED_STATE;
        }
      }
      case RSI_WLAN_ERROR_STATE: {

      } break;
      default:
        break;
    }
  }
}

uint32_t rsi_convert_4R_to_BIG_Endian_uint32(uint32_t *pw)
{
  uint32_t val;
  uint8_t *pw1 = (uint8_t *)pw;
  val          = pw1[0];
  val <<= 8;
  val |= pw1[1] & 0x000000ff;
  val <<= 8;
  val |= pw1[2] & 0x000000ff;
  val <<= 8;
  val |= pw1[3] & 0x000000ff;
  return val;
}

/*====================================================*/
/**
 * @fn         void rsi_join_fail_handler(uint16_t status, uint8_t *buffer, const uint32_t length)
 * @brief      Callback handler in station mode at rejoin failure
 * @param[in]  uint16_t status, uint8_t *buffer, const uint32_t length
 * @return     void
 *=====================================================*/
void rsi_join_fail_handler(uint16_t status, uint8_t *buffer, const uint32_t length)
{
  UNUSED_PARAMETER(buffer);       //This statement is added only to resolve compilation warning, value is unchanged
  UNUSED_PARAMETER(status);       //This statement is added only to resolve compilation warning, value is unchanged
  UNUSED_CONST_PARAMETER(length); //This statement is added only to resolve compilation warning, value is unchanged
#if BLE_PROVISIONING_ENABLED
  //! update wlan application state
  disconnected          = 1;
  connected             = 0;
  rsi_wlan_app_cb.state = RSI_WLAN_DISCONNECTED_STATE;
#else
  rsi_wlan_app_cb.state = RSI_WLAN_JOIN_STATE;
#endif
}

/*====================================================*/
/**
 * @fn         void rsi_ip_renewal_fail_handler(uint16_t status, uint8_t *buffer, const uint32_t length)
 * @brief      IP renewal failure call back handler in station mode
 * @param[in]  uint16_t status, uint8_t *buffer, const uint32_t length
 * @return     void
 *=====================================================*/
void rsi_ip_renewal_fail_handler(uint16_t status, uint8_t *buffer, const uint32_t length)
{
  UNUSED_CONST_PARAMETER(length); //This statement is added only to resolve compilation warning, value is unchanged
  UNUSED_PARAMETER(buffer);       //This statement is added only to resolve compilation warning, value is unchanged
  UNUSED_PARAMETER(status);       //This statement is added only to resolve compilation warning, value is unchanged
  //! update wlan application state
  rsi_wlan_app_cb.state = RSI_WLAN_CONNECTED_STATE;
}

/*====================================================*/
/**
 * @fn         void rsi_remote_socket_terminate_handler(uint16_t status, uint8_t *buffer, const uint32_t length)
 * @brief      Callback handler to terminate stations remote socket
 * @param[in]  uint16_t status, uint8_t *buffer, const uint32_t length
 * @return     void
 *=====================================================*/
void rsi_remote_socket_terminate_handler(uint16_t status, uint8_t *buffer, const uint32_t length)
{
  UNUSED_PARAMETER(buffer);       //This statement is added only to resolve compilation warning, value is unchanged
  UNUSED_CONST_PARAMETER(length); //This statement is added only to resolve compilation warning, value is unchanged
  UNUSED_PARAMETER(status);       //This statement is added only to resolve compilation warning, value is unchanged
  data_recvd            = 1;      //Set data receive flag
  rsi_wlan_app_cb.state = RSI_WLAN_IPCONFIG_DONE_STATE; //! update wlan application state
}

/*====================================================*/
/**
 * @fn         void rsi_ip_change_notify_handler(uint16_t status, uint8_t *buffer, const uint32_t length)
 * @brief      Callback handler to notify IP change in Station mode
 * @param[in]  uint16_t status, uint8_t *buffer, const uint32_t length
 * @return     void
 *=====================================================*/
void rsi_ip_change_notify_handler(uint16_t status, uint8_t *buffer, const uint32_t length)
{
  UNUSED_PARAMETER(buffer);       //This statement is added only to resolve compilation warning, value is unchanged
  UNUSED_CONST_PARAMETER(length); //This statement is added only to resolve compilation warning, value is unchanged
  UNUSED_PARAMETER(status);       //This statement is added only to resolve compilation warning, value is unchanged
  //! update wlan application state
  rsi_wlan_app_cb.state = RSI_WLAN_IPCONFIG_DONE_STATE;
}

/*====================================================*/
/**
 * @fn         void rsi_stations_connect_notify_handler(uint16_t status, uint8_t *buffer, const uint32_t length)
 * @brief      Callback handler to notify stations connect in AP mode
 * @param[in]  uint16_t status, uint8_t *buffer, const uint32_t length
 * @return     void
 *=====================================================*/
void rsi_stations_connect_notify_handler(uint16_t status, uint8_t *buffer, const uint32_t length)
{
  UNUSED_PARAMETER(buffer);       //This statement is added only to resolve compilation warning, value is unchanged
  UNUSED_CONST_PARAMETER(length); //This statement is added only to resolve compilation warning, value is unchanged
  UNUSED_PARAMETER(status);       //This statement is added only to resolve compilation warning, value is unchanged
}

/*====================================================*/
/**
 * @fn         void rsi_stations_disconnect_notify_handler(uint16_t status, uint8_t *buffer, const uint32_t length)
 * @brief      Callback handler to notify stations disconnect in AP mode
 * @param[in]  uint16_t status, uint8_t *buffer, const uint32_t length
 * @return     void
 *=====================================================*/
void rsi_stations_disconnect_notify_handler(uint16_t status, uint8_t *buffer, const uint32_t length)
{
  UNUSED_PARAMETER(buffer);       //This statement is added only to resolve compilation warning, value is unchanged
  UNUSED_CONST_PARAMETER(length); //This statement is added only to resolve compilation warning, value is unchanged
  UNUSED_PARAMETER(status);       //This statement is added only to resolve compilation warning, value is unchanged
}
#endif
