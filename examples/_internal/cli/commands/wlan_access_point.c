#include "command_processor.h"
#include "rsi_error.h"
#include "rsi_common_apis.h"
#include "rsi_wlan_non_rom.h"

//! IP address of the module
#define DEVICE_IP 0x010AA8C0 // 192.168.10.1

//! IP address of Gateway
#define GATEWAY 0x010AA8C0 // 192.168.10.1

//! IP address of netmask
#define NETMASK 0x00FFFFFF // 255.255.255.0

static void stations_connect_notify_handler(uint16_t status, uint8_t *buffer, const uint32_t length);
static void stations_disconnect_notify_handler(uint16_t status, uint8_t *buffer, const uint32_t length);

int wlan_access_point_command(command_processor_args_t *arguments)
{
  char *ssid         = (char *)arguments->arg[0];
  uint8_t channel    = (uint8_t)arguments->arg[1];
  uint8_t security   = (uint8_t)arguments->arg[2];
  uint8_t encryption = (uint8_t)arguments->arg[3];
  char *secret       = (char *)arguments->arg[4];
  uint16_t beacon    = IS_COMMAND_ARG_VALID(5) ? (uint16_t)arguments->arg[5] : 100;
  uint8_t dtim       = IS_COMMAND_ARG_VALID(6) ? (uint8_t)arguments->arg[6] : 4;

  uint32_t ip_addr      = DEVICE_IP;
  uint32_t network_mask = NETMASK;
  uint32_t gateway      = GATEWAY;

  rsi_wlan_register_callbacks(RSI_STATIONS_CONNECT_NOTIFY_CB, stations_connect_notify_handler);
  rsi_wlan_register_callbacks(RSI_STATIONS_DISCONNECT_NOTIFY_CB, stations_disconnect_notify_handler);

  //! Configure IP
  int status = rsi_config_ipaddress(RSI_IP_VERSION_4,
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
  status = rsi_wlan_ap_start((int8_t *)ssid, channel, security, encryption, (uint8_t *)secret, beacon, dtim);
  if (status != RSI_SUCCESS) {
    LOG_PRINT("\r\nAP Start Failed, Error Code : 0x%X\r\n", status);
    return status;
  } else {
    LOG_PRINT("\r\nAP Start Success\r\n");
  }

  return status;
}

//! This callback function is called when Station is connected.
//! Buffer has the MAC address of the station connected
void stations_connect_notify_handler(uint16_t status, uint8_t *buffer, const uint32_t length)
{
  printf("Client connected: %02x:%02x:%02x:%02x:%02x:%02x\r\n",
         buffer[0],
         buffer[1],
         buffer[2],
         buffer[3],
         buffer[4],
         buffer[5]);
}

//! This callback function is called when Station is disconnected.
//! Buffer has the MAC address of the station disconnected
void stations_disconnect_notify_handler(uint16_t status, uint8_t *buffer, const uint32_t length)
{
  printf("Client disconnected: %02x:%02x:%02x:%02x:%02x:%02x\r\n",
         buffer[0],
         buffer[1],
         buffer[2],
         buffer[3],
         buffer[4],
         buffer[5]);
}
