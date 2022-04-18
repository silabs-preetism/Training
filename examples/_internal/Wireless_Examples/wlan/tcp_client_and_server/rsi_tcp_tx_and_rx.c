/*******************************************************************************
* @file  rsi_tcp_tx_and_rx.c
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
 * @brief : This file contains example application for Transmitting and Receiving 
 * on 2 TCP sockets and 1 SSL socket.
 * @section Description :
 * This application will demonstrate How to Transmit and Receive by connecting
 * to 3 different sockets
 =================================================================================*/

/*=======================================================================*/
//  ! INCLUDES
/*=======================================================================*/
#include "rsi_driver.h"

// include file to refer data types
#include "rsi_data_types.h"

// COMMON include file to refer wlan APIs
#include "rsi_common_apis.h"

// WLAN include file to refer wlan APIs
#include "rsi_wlan_apis.h"
#include "rsi_wlan_non_rom.h"

// WLAN include file for configuration
#include "rsi_wlan_config.h"

// socket include file to refer socket APIs
#include "rsi_socket.h"

#include "rsi_bootup_config.h"
// Error include files
#include "rsi_error.h"

// OS include file to refer OS specific functionality
#include "rsi_os.h"
#ifndef LINUX_PLATFORM
#include "rsi_hal.h"
#endif

// Access point SSID to connect
#define SSID "Silabs"

//Scan Channel number , 0 - to scan all channels
#define CHANNEL_NO 0

// Security type
#define SECURITY_TYPE RSI_WPA2

// Password
#define PSK "123456789"

// DHCP mode 1- Enable 0- Disable
#define DHCP_MODE 1

//#define RSI_HIGH_PERFORMANCE_SOCKET         1
// If DHCP mode is disabled give IP statically
#if !(DHCP_MODE)

// IP address of the module
// E.g: 0x650AA8C0 == 192.168.10.101
#define DEVICE_IP 0x6500A8C0

// IP address of Gateway
// E.g: 0x010AA8C0 == 192.168.10.1
#define GATEWAY 0x010AA8C0

// IP address of netmask
// E.g: 0x00FFFFFF == 255.255.255.0
#define NETMASK 0x00FFFFFF

#endif

//Module port number
#define DEVICE_PORT1 5001

// Server port number
#define SERVER_PORT1 5001
//! Device port 2 number
#define DEVICE_PORT2 5002

//! Server port 2 number
#define SERVER_PORT2 5002

//! Device port 3 number
#define DEVICE_PORT3 5003

//! Server port 3 number
#define SERVER_PORT3 5003
// Server IP address.
#define SERVER_IP_ADDRESS "192.168.0.6"

// Memory length for driver
#define GLOBAL_BUFF_LEN 15000

// Wlan task priority
#define RSI_APPLICATION_TASK_PRIORITY 1

// Wireless driver task priority
#define RSI_DRIVER_TASK_PRIORITY 2

// Wlan task stack size
#define RSI_APPLICATION_TASK_STACK_SIZE (512 * 2)

// Wireless driver task stack size
#define RSI_DRIVER_TASK_STACK_SIZE (512 * 2)

#define SOCKET_ASYNC_FEATURE 0

//! Receive data length
#define RECV_BUFFER_SIZE   1000
#define RSI_SSL_BIT_ENABLE 0
#define SSL                1
#define LOAD_CERTIFICATE   1

#define TX_ENABLE 1
#define RX_ENABLE 0

volatile uint8_t data_recvd = 0;
volatile uint64_t num_bytes = 0;
uint32_t t_end;
// Memory to initialize driv
uint8_t global_buf[GLOBAL_BUFF_LEN];
uint8_t ip_buff[20];

#if SSL
// Include SSL CA certificate
#include "cacert.pem"
#endif
extern void rsi_remote_socket_terminate_handler(uint16_t status, uint8_t *buffer, const uint32_t length);
extern uint64_t ip_to_reverse_hex(char *ip);

// Call back for Socket Async
void socket_async_recive(uint32_t sock_no, uint8_t *buffer, uint32_t length)
{
  UNUSED_PARAMETER(sock_no);
  UNUSED_PARAMETER(buffer);

  num_bytes += length;
}
/*====================================================*/
/**
 * @fn         int32_t application(void)
 * @brief      This function explains how to scan, connect and tx/rx wlan packets from remote device
 * @param[in]  void
 * @return     int32_t
 * @section description
 *====================================================*/
int32_t application()
{
  int32_t server_socket, server_socket2, server_socket3, new_socket, new_socket2, new_socket3;
  int32_t client_socket, client_socket2, client_socket3;
  struct rsi_sockaddr_in server_addr, client_addr;
  int32_t status       = RSI_SUCCESS;
  int32_t packet_count = 0, recv_size = 0, recv_size2 = 0, recv_size3 = 0;
  // buffer to receive data over TCP/UDP client socket
  int8_t recv_buffer[RECV_BUFFER_SIZE], recv_buffer2[RECV_BUFFER_SIZE], recv_buffer3[RECV_BUFFER_SIZE];
  uint32_t total_bytes_rx = 0;
  int32_t addr_size;
  uint8_t high_performance_socket = RSI_HIGH_PERFORMANCE_SOCKET;
  uint8_t ssl_bit_map             = RSI_SSL_BIT_ENABLE;
#if !(DHCP_MODE)
  uint32_t ip_addr      = DEVICE_IP;
  uint32_t network_mask = NETMASK;
  uint32_t gateway      = GATEWAY;
#else
  uint8_t dhcp_mode = (RSI_DHCP | RSI_DHCP_UNICAST_OFFER);
#endif
  uint16_t i              = 0;
  uint32_t total_bytes_tx = 0, tt_start = 0, tt_end = 0, pkt_cnt = 0;
#ifdef RSI_WITH_OS
  rsi_task_handle_t driver_task_handle = NULL;
#endif

  // Driver initialization
  status = rsi_driver_init(global_buf, GLOBAL_BUFF_LEN);
  if ((status < 0) || (status > GLOBAL_BUFF_LEN)) {
    return status;
  }

  // Silicon labs module intialisation
  status = rsi_device_init(LOAD_NWP_FW);
  if (status != RSI_SUCCESS) {
    LOG_PRINT("\r\nDevice Initialization Failed, Error Code : 0x%lX\r\n", status);
    return status;
  }
  LOG_PRINT("\r\nDevice Initialization Success\r\n");

#ifdef RSI_WITH_OS
  // Task created for Driver task
  rsi_task_create((rsi_task_function_t)rsi_wireless_driver_task,
                  (uint8_t *)"driver_task",
                  RSI_DRIVER_TASK_STACK_SIZE,
                  NULL,
                  RSI_DRIVER_TASK_PRIORITY,
                  &driver_task_handle);
#endif

  // WC initialization
  status = rsi_wireless_init(0, 0);
  if (status != RSI_SUCCESS) {
    LOG_PRINT("\r\nWireless Initialization Failed, Error Code : 0x%lX\r\n", status);
    return status;
  }
  LOG_PRINT("\r\nWireless Initialization Success\r\n");

  // Send feature frame
  status = rsi_send_feature_frame();
  if (status != RSI_SUCCESS) {
    LOG_PRINT("\r\nSend Feature Frame Failed, Error Code : 0x%lX\r\n", status);
    return status;
  }
  LOG_PRINT("\r\nSend Feature Frame Success\r\n");

  rsi_wlan_register_callbacks(RSI_REMOTE_SOCKET_TERMINATE_CB,
                              rsi_remote_socket_terminate_handler); // Initialize remote terminate call back

#if LOAD_CERTIFICATE
#if SSL
  // Load SSL CA certificate
  status = rsi_wlan_set_certificate(RSI_SSL_CA_CERTIFICATE, cacert, (sizeof(cacert) - 1));
  if (status != RSI_SUCCESS) {
    LOG_PRINT("\r\nLoad SSL CA Certificate Failed, Error Code : 0x%lX\r\n", status);
    return status;
  }
  LOG_PRINT("\r\nLoad SSL CA Certificate Success\r\n");
#endif
#endif

#if TX_RX_RATIO_ENABLE
  status = rsi_wlan_buffer_config();
  if (status != RSI_SUCCESS) {
    return status;
  }
#endif
  // Scan for Access points
  status = rsi_wlan_scan((int8_t *)SSID, (uint8_t)CHANNEL_NO, NULL, 0);
  if (status != RSI_SUCCESS) {
    LOG_PRINT("\r\nWLAN AP Scan Failed, Error Code : 0x%lX\r\n", status);
    return status;
  }
  LOG_PRINT("\r\nWLAN AP Scan Success\r\n");

  // Connect to an Access point
  status = rsi_wlan_connect((int8_t *)SSID, SECURITY_TYPE, PSK);
  if (status != RSI_SUCCESS) {
    LOG_PRINT("\r\nWLAN AP Connect Failed, Error Code : 0x%lX\r\n", status);
    return status;
  }
  LOG_PRINT("\r\nWLAN AP Connect Success\r\n");

  // Configure IP
#if DHCP_MODE
  status = rsi_config_ipaddress(RSI_IP_VERSION_4, dhcp_mode, 0, 0, 0, ip_buff, sizeof(ip_buff), 0);
#else
  status            = rsi_config_ipaddress(RSI_IP_VERSION_4,
                                RSI_STATIC,
                                (uint8_t *)&ip_addr,
                                (uint8_t *)&network_mask,
                                (uint8_t *)&gateway,
                                NULL,
                                0,
                                0);
#endif
  if (status != RSI_SUCCESS) {
    LOG_PRINT("\r\nIP Config Failed, Error Code : 0x%lX\r\n", status);
    return status;
  }
  LOG_PRINT("\r\nIP Config Success\r\n");
  LOG_PRINT("RSI_STA IP ADDR: %d.%d.%d.%d \r\n", ip_buff[6], ip_buff[7], ip_buff[8], ip_buff[9]);

#if TX_ENABLE
  //! Create socket
  client_socket = rsi_socket(AF_INET, SOCK_STREAM, 0);
  if (client_socket < 0) {
    status = rsi_wlan_get_status();
    LOG_PRINT("\r\nSocket 1 Create Failed, Error Code : 0x%lX\r\n", status);
    return status;
  } else {
    LOG_PRINT("\r\nSocket 1 Create Success\r\n");
  }

  //! Memset client structure
  memset(&client_addr, 0, sizeof(client_addr));

  //! Set family type
  client_addr.sin_family = AF_INET;

  //! Set local port number
  client_addr.sin_port = htons(DEVICE_PORT1);

  //! Bind socket
  status = rsi_bind(client_socket, (struct rsi_sockaddr *)&client_addr, sizeof(client_addr));
  if (status != RSI_SUCCESS) {
    status = rsi_wlan_get_status();
    rsi_shutdown(client_socket, 0);
    LOG_PRINT("\r\nBind socket 1 Failed, Error code : 0x%lX\r\n", status);
    return status;
  } else {
    LOG_PRINT("\r\nBind socket 1 Success\r\n");
  }

  //! Set server structure
  memset(&server_addr, 0, sizeof(server_addr));

  //! Set server address family
  server_addr.sin_family = AF_INET;

  //! Set server port number, using htons function to use proper byte order
  server_addr.sin_port = htons(SERVER_PORT1);

  //! Set IP address to localhost
  server_addr.sin_addr.s_addr = ip_to_reverse_hex((char *)SERVER_IP_ADDRESS);
  //! Connect to server socket
  status = rsi_connect(client_socket, (struct rsi_sockaddr *)&server_addr, sizeof(server_addr));
  if (status != RSI_SUCCESS) {
    status = rsi_wlan_get_status();
    rsi_shutdown(client_socket, 0);
    LOG_PRINT("\r\nConnect to Server Socket 1 Failed, Error Code : 0x%lX\r\n", status);
    return status;
  } else {
    LOG_PRINT("\r\nConnect to Server Socket 1 Success\r\n");
  }

  //! Create socket
  client_socket2 = rsi_socket(AF_INET, SOCK_STREAM, 0);
  if (client_socket2 < 0) {
    status = rsi_wlan_get_status();
    LOG_PRINT("\r\nSocket 2 Create Failed, Error Code : 0x%lX\r\n", status);
    return status;
  } else {
    LOG_PRINT("\r\nSocket 2 Create Success\r\n");
  }

  //! Memset client structure
  memset(&client_addr, 0, sizeof(client_addr));

  //! Set family type
  client_addr.sin_family = AF_INET;

  //! Set local port number
  client_addr.sin_port = htons(DEVICE_PORT2);

  //! Bind socket
  status = rsi_bind(client_socket2, (struct rsi_sockaddr *)&client_addr, sizeof(client_addr));
  if (status != RSI_SUCCESS) {
    status = rsi_wlan_get_status();
    rsi_shutdown(client_socket2, 0);
    LOG_PRINT("\r\nBind socket 2 Failed, Error code : 0x%lX\r\n", status);
    return status;
  } else {
    LOG_PRINT("\r\nBind socket 2 Success\r\n");
  }

  //! Set server structure
  memset(&server_addr, 0, sizeof(server_addr));

  //! Set server address family
  server_addr.sin_family = AF_INET;

  //! Set server port number, using htons function to use proper byte order
  server_addr.sin_port = htons(SERVER_PORT2);

  //! Set IP address to localhost
  server_addr.sin_addr.s_addr = ip_to_reverse_hex((char *)SERVER_IP_ADDRESS);

  //! Connect to server socket
  status = rsi_connect(client_socket2, (struct rsi_sockaddr *)&server_addr, sizeof(server_addr));
  if (status != RSI_SUCCESS) {
    status = rsi_wlan_get_status();
    rsi_shutdown(client_socket2, 0);
    LOG_PRINT("\r\nConnect to Server Socket 2 Failed, Error Code : 0x%lX\r\n", status);
    return status;
  } else {
    LOG_PRINT("\r\nConnect to Server Socket 2 Success\r\n");
  }

  //! Create socket
  client_socket3 = rsi_socket(AF_INET, SOCK_STREAM, 1);
  if (client_socket3 < 0) {
    status = rsi_wlan_get_status();
    LOG_PRINT("\r\nSocket 3 Create Failed, Error Code : 0x%lX\r\n", status);
    return status;
  } else {
    LOG_PRINT("\r\nSocket 3 Create Success\r\n");
  }

  status = rsi_setsockopt(client_socket3, SOL_SOCKET, SO_SSL_ENABLE, &ssl_bit_map, sizeof(ssl_bit_map));
  if (status != RSI_SUCCESS) {
    status = rsi_wlan_get_status();
    LOG_PRINT("\r\nSet Socket 3 Options Failed, Error Code : 0x%lX\r\n", status);
    return status;
  } else {
    LOG_PRINT("\r\nSet Socket 3 Options Success\r\n");
  }

  //! Memset client structure
  memset(&client_addr, 0, sizeof(client_addr));

  //! Set family type
  client_addr.sin_family = AF_INET;

  //! Set local port number
  client_addr.sin_port = htons(DEVICE_PORT3);

  //! Bind socket
  status = rsi_bind(client_socket3, (struct rsi_sockaddr *)&client_addr, sizeof(client_addr));
  if (status != RSI_SUCCESS) {
    status = rsi_wlan_get_status();
    rsi_shutdown(client_socket3, 0);
    LOG_PRINT("\r\nBind socket 3 Failed, Error code : 0x%lX\r\n", status);
    return status;
  } else {
    LOG_PRINT("\r\nBind socket 3 Success\r\n");
  }

  //! Set server structure
  memset(&server_addr, 0, sizeof(server_addr));

  //! Set server address family
  server_addr.sin_family = AF_INET;

  //! Set server port number, using htons function to use proper byte order
  server_addr.sin_port = htons(SERVER_PORT3);

  //! Set IP address to localhost
  server_addr.sin_addr.s_addr = ip_to_reverse_hex((char *)SERVER_IP_ADDRESS);

  //! Connect to server socket
  status = rsi_connect(client_socket3, (struct rsi_sockaddr *)&server_addr, sizeof(server_addr));
  if (status != RSI_SUCCESS) {
    status = rsi_wlan_get_status();
    rsi_shutdown(client_socket, 0);
    LOG_PRINT("\r\nConnect to Server Socket 3 Failed, Error Code : 0x%lX\r\n", status);
    return status;
  } else {
    LOG_PRINT("\r\nConnect to Server Socket 3 Success\r\n");
  }
  while (1) //packet_count < NUMBER_OF_PACKETS)
  {
    //! Send data on socket
    status = rsi_send(client_socket, (int8_t *)"Hello from TCP client!!!", (sizeof("Hello from TCP client!!!") - 1), 0);
    if (status < 0) {
      status = rsi_wlan_get_status();
      rsi_shutdown(client_socket, 0);
      LOG_PRINT("\r\nFailed to Send data to TCP Server socket 1, Error Code : 0x%lX\r\n", status);
      return status;
    }

    //! Send data on socket2
    status =
      rsi_send(client_socket2, (int8_t *)"Hello from TCP client!!!", (sizeof("Hello from TCP client!!!") - 1), 0);
    if (status < 0) {
      status = rsi_wlan_get_status();
      rsi_shutdown(client_socket2, 0);
      LOG_PRINT("\r\nFailed to Send data to TCP Server socket 2, Error Code : 0x%lX\r\n", status);
      return status;
    }

    //! Send data on socket
    status = rsi_send(client_socket3,
                      (int8_t *)"Hello from SSL TCP client!!!",
                      (sizeof("Hello from SSL TCP client!!!") - 1),
                      0);
    if (status < 0) {
      status = rsi_wlan_get_status();
      rsi_shutdown(client_socket3, 0);
      LOG_PRINT("\r\nFailed to Send data on socket 3, Error Code : 0x%lX\r\n", status);
      return status;
    }

    packet_count++;
  }
#endif
#if RX_ENABLE
  // Create socket
#if SOCKET_ASYNC_FEATURE
  server_socket = rsi_socket_async(AF_INET, SOCK_STREAM, 0, socket_async_recive);
#else
  server_socket  = rsi_socket(AF_INET, SOCK_STREAM, 0);
#endif
  if (server_socket < 0) {
    status = rsi_wlan_get_status();
    LOG_PRINT("\r\nSocket 1 Create Failed, Error Code : 0x%lX\r\n", status);
    return status;
  }
  LOG_PRINT("\r\nSocket 1 Create Success\r\n");

  status = rsi_setsockopt(server_socket,
                          SOL_SOCKET,
                          SO_HIGH_PERFORMANCE_SOCKET,
                          &high_performance_socket,
                          sizeof(high_performance_socket));
  if (status != RSI_SUCCESS) {
    LOG_PRINT("\r\nSet Socket 1 Option Failed, Error Code : 0x%lX\r\n", status);
    return status;
  }
  LOG_PRINT("\r\nSet Socket 1 Option Success\r\n");

  // Set server structure
  memset(&server_addr, 0, sizeof(server_addr));

  // Set family type
  server_addr.sin_family = AF_INET;

  // Set local port number
  server_addr.sin_port = htons(SERVER_PORT1);

  // Bind socket
  status = rsi_bind(server_socket, (struct rsi_sockaddr *)&server_addr, sizeof(server_addr));
  if (status != RSI_SUCCESS) {
    status = rsi_wlan_get_status();
    rsi_shutdown(server_socket, 0);
    LOG_PRINT("\r\nBind socket 1 Failed, Error code : 0x%lX\r\n", status);
    return status;
  }
  LOG_PRINT("\r\nBind socket 1 Success\r\n");

  // Socket listen
  status = rsi_listen(server_socket, 1);
  if (status != RSI_SUCCESS) {
    status = rsi_wlan_get_status();
    rsi_shutdown(server_socket, 0);
    LOG_PRINT("\r\nListen socket 1 Failed, Error code : 0x%lX\r\n", status);
    return status;
  }
  LOG_PRINT("\r\nListen socket 1 Success\r\n");

  addr_size = sizeof(server_socket);

  // Socket accept
  new_socket = rsi_accept(server_socket, (struct rsi_sockaddr *)&client_addr, &addr_size);
  if (new_socket < 0) {
    status = rsi_wlan_get_status();
    rsi_shutdown(server_socket, 0);
    LOG_PRINT("\r\nSocket 1 Accept Failed, Error code : 0x%lX\r\n", status);
    return status;
  }
  LOG_PRINT("\r\nSocket 1 Accept Success\r\n");

  //! Create socket 2
#if SOCKET_ASYNC_FEATURE
  server_socket2 = rsi_socket_async(AF_INET, SOCK_STREAM, 0, socket_async_recive);
#else
  server_socket2 = rsi_socket(AF_INET, SOCK_STREAM, 0);
#endif
  if (server_socket2 < 0) {
    status = rsi_wlan_get_status();
    LOG_PRINT("\r\nSocket 2 Create Failed, Error Code : 0x%lX\r\n", status);
    return status;
  }
  LOG_PRINT("\r\nSocket 2 Create Success\r\n");

  status = rsi_setsockopt(server_socket2,
                          SOL_SOCKET,
                          SO_HIGH_PERFORMANCE_SOCKET,
                          &high_performance_socket,
                          sizeof(high_performance_socket));
  if (status != RSI_SUCCESS) {
    LOG_PRINT("\r\nSet Socket 2 Option Failed, Error Code : 0x%lX\r\n", status);
    return status;
  }
  LOG_PRINT("\r\nSet Socket 2 Option Success\r\n");

  // Set server structure
  memset(&server_addr, 0, sizeof(server_addr));

  // Set family type
  server_addr.sin_family = AF_INET;

  // Set local port number
  server_addr.sin_port = htons(SERVER_PORT2);

  // Bind socket
  status = rsi_bind(server_socket2, (struct rsi_sockaddr *)&server_addr, sizeof(server_addr));
  if (status != RSI_SUCCESS) {
    status = rsi_wlan_get_status();
    rsi_shutdown(server_socket2, 0);
    LOG_PRINT("\r\nBind socket 2 Failed, Error code : 0x%lX\r\n", status);
    return status;
  }
  LOG_PRINT("\r\nBind socket 2 Success\r\n");

  // Socket listen
  status = rsi_listen(server_socket2, 1);
  if (status != RSI_SUCCESS) {
    status = rsi_wlan_get_status();
    rsi_shutdown(server_socket2, 0);
    LOG_PRINT("\r\nListen socket 2 Failed, Error code : 0x%lX\r\n", status);
    return status;
  }
  LOG_PRINT("\r\nListen socket 2 Success\r\n");

  addr_size = sizeof(server_socket2);

  // Socket accept
  new_socket2 = rsi_accept(server_socket2, (struct rsi_sockaddr *)&client_addr, &addr_size);
  if (new_socket2 < 0) {
    status = rsi_wlan_get_status();
    rsi_shutdown(server_socket2, 0);
    LOG_PRINT("\r\nSocket 2 Accept Failed, Error code : 0x%lX\r\n", status);
    return status;
  }
  LOG_PRINT("\r\nSocket 2 Accept Success\r\n");

  LOG_PRINT("\r\n SSL RX Test\r\n");

#if HIGH_PERFORMANCE_ENABLE
  status = rsi_socket_config();
  if (status < 0) {
    status = rsi_wlan_get_status();
    return status;
  }
#endif
  // Create socket 3
#if SOCKET_ASYNC_FEATURE
  client_socket3 = rsi_socket_async(AF_INET, SOCK_STREAM, 1, socket_async_recive);
#else
  client_socket3 = rsi_socket(AF_INET, SOCK_STREAM, 1);
#endif
  if (client_socket3 < 0) {
    status = rsi_wlan_get_status();
    LOG_PRINT("\r\nSocket 3 Create Failed, Error Code : 0x%lX\r\n", status);
    return status;
  }
  LOG_PRINT("\r\nSocket 3 Create Success\r\n");

  status = rsi_setsockopt(client_socket3, SOL_SOCKET, SO_SSL_ENABLE, &ssl_bit_map, sizeof(ssl_bit_map));
  if (status != RSI_SUCCESS) {
    LOG_PRINT("\r\nSet Socket 3 Option Failed, Error Code : 0x%lX\r\n", status);
    return status;
  }
  LOG_PRINT("\r\nSet Socket 3 Option Success\r\n");

  status = rsi_setsockopt(client_socket3,
                          SOL_SOCKET,
                          SO_HIGH_PERFORMANCE_SOCKET,
                          &high_performance_socket,
                          sizeof(high_performance_socket));
  if (status != RSI_SUCCESS) {
    LOG_PRINT("\r\nSet Socket 3 Option Failed, Error Code : 0x%lX\r\n", status);
    return status;
  }
  LOG_PRINT("\r\nSet Socket 3 Option Success\r\n");

  // Memset client structrue
  memset(&client_addr, 0, sizeof(client_addr));

  // Set family type
  client_addr.sin_family = AF_INET;

  // Set local port number
  client_addr.sin_port = htons(DEVICE_PORT3);

  // Bind socket
  status = rsi_bind(client_socket3, (struct rsi_sockaddr *)&client_addr, sizeof(client_addr));
  if (status != RSI_SUCCESS) {
    status = rsi_wlan_get_status();
    rsi_shutdown(client_socket3, 0);
    LOG_PRINT("\r\nBind socket 3 Failed, Error code : 0x%lX\r\n", status);
    return status;
  }
  LOG_PRINT("\r\nBind socket 3 Success\r\n");

  // Set server structure
  memset(&server_addr, 0, sizeof(server_addr));

  // Set server address family
  server_addr.sin_family = AF_INET;

  // Set server port number, using htons function to use proper byte order
  server_addr.sin_port = htons(SERVER_PORT3);

  // Set IP address to localhost
  server_addr.sin_addr.s_addr = ip_to_reverse_hex((char *)SERVER_IP_ADDRESS);
  addr_size                   = sizeof(client_addr);
  // Connect to server socket
  status = rsi_connect(client_socket3, (struct rsi_sockaddr *)&server_addr, sizeof(server_addr));
  if (status != RSI_SUCCESS) {
    status = rsi_wlan_get_status();
    rsi_shutdown(client_socket3, 0);
    LOG_PRINT("\r\nFailed to Connect to Server socket 3, Error code : 0x%lX\r\n", status);
    return status;
  }
  LOG_PRINT("\r\nConnect to Server socket 3 Success\r\n");
#if !SOCKET_ASYNC_FEATURE
  while (1) {
    recv_size = RECV_BUFFER_SIZE;

    do {
      //! Receive data on socket
      status = rsi_recvfrom(new_socket, recv_buffer, recv_size, 0, (struct rsi_sockaddr *)&client_addr, &addr_size);
      if (status < 0) {
        status = rsi_wlan_socket_get_status(new_socket);
        if (status == RSI_RX_BUFFER_CHECK) {
          continue;
        }
        rsi_shutdown(server_socket, 0);
        LOG_PRINT("\r\nFailed to Rceive data to socket 1, Error Code : 0x%lX\r\n", status);
        return status;
      }

      //! subtract received bytes
      recv_size -= status;

    } while (recv_size > 0);

    recv_size2 = RECV_BUFFER_SIZE;

    do {
      //! Receive data on socket
      status = rsi_recvfrom(new_socket2, recv_buffer2, recv_size2, 0, (struct rsi_sockaddr *)&client_addr, &addr_size);
      if (status < 0) {
        status = rsi_wlan_socket_get_status(new_socket2);
        if (status == RSI_RX_BUFFER_CHECK) {
          continue;
        }
        rsi_shutdown(server_socket2, 0);
        LOG_PRINT("\r\nFailed to Rceive data to socket 2, Error Code : 0x%lX\r\n", status);
        return status;
      }

      //! subtract received bytes
      recv_size2 -= status;

    } while (recv_size2 > 0);

    recv_size3 = RECV_BUFFER_SIZE;

    do {
      //! Receive data on socket
      status =
        rsi_recvfrom(client_socket3, recv_buffer3, recv_size3, 0, (struct rsi_sockaddr *)&client_addr, &addr_size);
      if (status < 0) {
        status = rsi_wlan_socket_get_status(client_socket3);
        if (status == RSI_RX_BUFFER_CHECK) {
          continue;
        }
        rsi_shutdown(client_socket3, 0);
        LOG_PRINT("\r\nFailed to Rceive data to socket 3, Error Code : 0x%lX\r\n", status);
        return status;
      }
      //! subtract received bytes
      recv_size3 -= status;

    } while (recv_size3 > 0);
  }
#endif
#endif

  return 0;
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
  UNUSED_PARAMETER(status);
  UNUSED_PARAMETER(buffer);
  UNUSED_CONST_PARAMETER(length);
  data_recvd = 1; // Set data receive flag

  t_end = rsi_hal_gettickcount(); // capture time-stamp after data transfer is completed
}

// main function definition
int main(void)
{
#ifdef RSI_WITH_OS
  rsi_task_handle_t application_handle = NULL;

  // Create application task
  rsi_task_create((rsi_task_function_t)application,
                  (uint8_t *)"application_task",
                  RSI_APPLICATION_TASK_STACK_SIZE,
                  NULL,
                  RSI_APPLICATION_TASK_PRIORITY,
                  &application_handle);

  // Start the scheduler
  rsi_start_os_scheduler();
#else
  application();
#endif
}
