#include "rsi_wlan_apis.h"
#include "rsi_error.h"
#include "rsi_wlan.h"
#include "command_processor.h"
#include "utilities.h"

#define SCAN_RESULT_COUNT (10)
#define SCAN_RESULT_SIZE  (sizeof(rsi_rsp_scan_t) * SCAN_RESULT_COUNT)

static void scan_response_handler(uint16_t status, const uint8_t *buffer, const uint16_t length);
static void join_response_handler(uint16_t status, const uint8_t *buffer, const uint16_t length);
static void wlan_ping_response_handler(uint16_t status, const uint8_t *buffer, const uint16_t length);

int rsi_wlan_scan_command(command_processor_args_t *arguments)
{
  rsi_rsp_scan_t *scan_results = malloc(SCAN_RESULT_SIZE);
  if (scan_results == NULL) {
    return -1004;
  }

  char *ssid      = (char *)arguments->arg[0];
  uint8_t channel = (uint8_t)arguments->arg[1];
  print_status(rsi_wlan_scan(ssid, channel, scan_results, SCAN_RESULT_SIZE));

  free(scan_results);

  return RSI_ERROR_NONE;
}

int rsi_wlan_scan_async_command(command_processor_args_t *arguments)
{
  char *ssid      = (char *)arguments->arg[0];
  uint8_t channel = (uint8_t)arguments->arg[1];
  print_status(rsi_wlan_scan_async(ssid, channel, scan_response_handler));

  return RSI_ERROR_NONE;
}

int rsi_wlan_connect_command(command_processor_args_t *arguments)
{
  char *ssid       = (char *)arguments->arg[0];
  uint8_t security = (uint8_t)arguments->arg[1];
  char *secret     = (char *)arguments->arg[2];
  print_status(rsi_wlan_connect(ssid, security, secret));

  return RSI_ERROR_NONE;
}

int rsi_wlan_connect_async_command(command_processor_args_t *arguments)
{
  char *ssid       = (char *)arguments->arg[0];
  uint8_t security = (uint8_t)arguments->arg[1];
  char *secret     = (char *)arguments->arg[2];
  print_status(rsi_wlan_connect_async(ssid, security, secret, join_response_handler));

  return RSI_ERROR_NONE;
}

int rsi_wlan_ap_start_command(command_processor_args_t *arguments)
{
  char *ssid         = (char *)arguments->arg[0];
  uint8_t channel    = (uint8_t)arguments->arg[1];
  uint8_t security   = (uint8_t)arguments->arg[2];
  uint8_t encryption = (uint8_t)arguments->arg[3];
  char *secret       = (char *)arguments->arg[4];
  uint16_t beacon    = (uint16_t)arguments->arg[5];
  uint8_t dtim       = (uint8_t)arguments->arg[6];

  print_status(rsi_wlan_ap_start(ssid, channel, security, encryption, secret, beacon, dtim));

  return RSI_ERROR_NONE;
}

int rsi_wlan_execute_post_connect_cmds_command(command_processor_args_t *arguments)
{
  (void)arguments;
  print_status(rsi_wlan_execute_post_connect_cmds());

  return RSI_ERROR_NONE;
}

int rsi_wlan_disconnect_command(command_processor_args_t *arguments)
{
  (void)arguments;
  switch (rsi_wlan_disconnect()) {
    case RSI_ERROR_PKT_ALLOCATION_FAILURE:
      printf("Disconnect fail\n");
      break;
    case RSI_ERROR_WLAN_CMD_IN_PROGRESS:
      printf("Disconnect fail\n");
      break;
    default:
      break;
  }

  return RSI_ERROR_NONE;
}

int rsi_wlan_disconnect_stations_command(command_processor_args_t *arguments)
{
  char *mac   = (char *)arguments->arg[0];
  size_t size = strlen(mac);

  if (size == 6) {
    print_status(rsi_wlan_disconnect_stations(mac));
  } else {
    printf("Bad MAC address\n");
  }

  return RSI_ERROR_NONE;
}

int rsi_wlan_dhcp_command(command_processor_args_t *arguments)
{
  (void)arguments;
  int status = rsi_config_ipaddress(RSI_IP_VERSION_4, 1, 0, 0, 0, NULL, 0, 0);
  return status;
}

int rsi_wlan_get_status_command(command_processor_args_t *arguments)
{
  (void)arguments;
  print_status(rsi_wlan_get_status());
  return RSI_ERROR_NONE;
}

int rsi_wlan_wps_generate_pin_command(command_processor_args_t *arguments)
{
  (void)arguments;
  uint8_t pin[RSI_WPS_PIN_LEN];
  int32_t status = rsi_wlan_wps_generate_pin(pin, sizeof(pin));
  if (status == RSI_ERROR_NONE) {
    for (int a = 0; a < sizeof(pin); ++a) {
      printf("%c", ('0' + pin[a]));
    }
    printf("\n");
  } else {
    print_status(status);
  }
  return RSI_ERROR_NONE;
}

int rsi_wlan_wps_enter_pin_command(command_processor_args_t *arguments)
{
  char *pin         = (char *)arguments->arg[0];
  size_t pin_length = strlen(pin);
  for (int a = 0; a < pin_length; ++a) {
    pin[a] = pin[a] - '0';
  }
  print_status(rsi_wlan_wps_enter_pin(pin));
  return RSI_ERROR_NONE;
}

int rsi_wlan_wps_push_button_event_command(command_processor_args_t *arguments)
{
  char *ssid = (char *)arguments->arg[0];
  print_status(rsi_wlan_wps_push_button_event(ssid));
  return RSI_ERROR_NONE;
}

int rsi_transmit_test_start_command(command_processor_args_t *arguments)
{
  uint16_t power   = (uint16_t)arguments->arg[0];
  uint32_t rate    = (uint32_t)arguments->arg[1];
  uint16_t length  = (uint16_t)arguments->arg[2];
  uint16_t mode    = (uint16_t)arguments->arg[3];
  uint16_t channel = (uint16_t)arguments->arg[4];
  print_status(rsi_transmit_test_start(power, rate, length, mode, channel));
  return RSI_ERROR_NONE;
}

int rsi_transmit_test_stop_command(command_processor_args_t *arguments)
{
  (void)arguments;
  print_status(rsi_transmit_test_stop());
  return RSI_ERROR_NONE;
}

int rsi_wlan_receive_stats_start_command(command_processor_args_t *arguments)
{
  uint16_t channel = (uint16_t)arguments->arg[0];
  print_status(rsi_wlan_receive_stats_start(channel));
  return RSI_ERROR_NONE;
}

int rsi_wlan_receive_stats_stop_command(command_processor_args_t *arguments)
{
  (void)arguments;
  print_status(rsi_wlan_receive_stats_stop());
  return RSI_ERROR_NONE;
}

int rsi_wlan_get_command(command_processor_args_t *arguments)
{
  uint16_t command = (uint16_t)arguments->arg[0];
  uint8_t response[100];
  print_status(rsi_wlan_get(command, response, sizeof(response)));
  return RSI_ERROR_NONE;
}

int rsi_wlan_set_command(command_processor_args_t *arguments)
{
  uint16_t command = (uint16_t)arguments->arg[0];
  print_status(rsi_wlan_set(command, NULL, 0));
  return RSI_ERROR_NONE;
}

int rsi_wlan_enable_auto_config_command(command_processor_args_t *arguments)
{
  uint8_t enable = (uint8_t)arguments->arg[0];
  uint32_t type  = (uint32_t)arguments->arg[1];
  print_status(rsi_wlan_enable_auto_config(enable, type));
  return RSI_ERROR_NONE;
}

int rsi_wlan_bgscan_profile_command(command_processor_args_t *arguments)
{
  void *scan_results = malloc(SCAN_RESULT_SIZE);
  if (scan_results == NULL) {
    return -1004;
  }

  uint8_t cmd = (uint8_t)arguments->arg[0];
  print_status(rsi_wlan_bgscan_profile(cmd, scan_results, SCAN_RESULT_SIZE));

  free(scan_results);
  return RSI_ERROR_NONE;
}

int rsi_wlan_scan_with_bitmap_options_command(command_processor_args_t *arguments)
{
  rsi_rsp_scan_t *scan_results = malloc(SCAN_RESULT_SIZE);
  if (scan_results == NULL) {
    return -1004;
  }

  char *ssid           = (char *)arguments->arg[0];
  uint8_t channel      = (uint8_t)arguments->arg[1];
  uint32_t scan_bitmap = (uint32_t)arguments->arg[2];
  print_status(rsi_wlan_scan_with_bitmap_options(ssid, channel, scan_results, SCAN_RESULT_SIZE, scan_bitmap));

  free(scan_results);
  return RSI_ERROR_NONE;
}

int rsi_wlan_scan_async_with_bitmap_options_command(command_processor_args_t *arguments)
{
  char *ssid           = (char *)arguments->arg[0];
  uint8_t channel      = (uint8_t)arguments->arg[1];
  uint32_t scan_bitmap = (uint32_t)arguments->arg[2];
  print_status(rsi_wlan_scan_async_with_bitmap_options(ssid, channel, scan_bitmap, scan_response_handler));
  return RSI_ERROR_NONE;
}

int rsi_wlan_ping_async_command(command_processor_args_t *arguments)
{
  uint8_t flags         = (uint8_t)arguments->arg[0];
  char *ip_string       = (char *)arguments->arg[1];
  uint16_t size         = (uint16_t)arguments->arg[2];
  uint8_t ip_address[4] = { 10, 0, 0, 1 };
  print_status(rsi_wlan_ping_async(flags, ip_address, size, wlan_ping_response_handler));
  return RSI_ERROR_NONE;
}

// ---------------- Static helper functions

static void scan_response_handler(uint16_t status, const uint8_t *buffer, const uint16_t length)
{
  rsi_rsp_scan_t *scan_results = (rsi_rsp_scan_t *)buffer;
  printf("Scan status: 0x%X\n", status);
  printf("%d scan results:\n", scan_results->scan_count[0]);
  for (int a = 0; a < scan_results->scan_count[0]; ++a) {
    printf("%s    %d   -%u\n",
           scan_results->scan_info[a].ssid,
           scan_results->scan_info[a].rf_channel,
           scan_results->scan_info[a].rssi_val);
  }
}

static void join_response_handler(uint16_t status, const uint8_t *buffer, const uint16_t length)
{
  printf("got join result: 0x%X\n", status);
}

static void wlan_ping_response_handler(uint16_t status, const uint8_t *buffer, const uint16_t length)
{
  printf("got ping response result: 0x%X\n", status);
}
