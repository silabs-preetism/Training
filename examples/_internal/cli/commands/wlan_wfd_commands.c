
static void wlan_wfd_discovery_notify_handler(uint16_t status, uint8_t *buffer, const uint32_t length);
static void wlan_wfd_connection_request_notify_handler(uint16_t status, uint8_t *buffer, const uint32_t length);

int rsi_wlan_wfd_start_discovery_command(command_processor_args_t *arguments)
{
  uint16_t go_intent  = (uint16_t)arguments->arg[0];
  char *device_name   = (char *)arguments->arg[1];
  uint16_t channel    = (uint16_t)arguments->arg[2];
  char *ssid_post_fix = (char *)arguments->arg[3];
  char *psk           = (char *)arguments->arg[4];

  print_status(rsi_wlan_wfd_start_discovery(go_intent,
                                            device_name,
                                            channel,
                                            ssid_post_fix,
                                            psk,
                                            wlan_wfd_discovery_notify_handler,
                                            wlan_wfd_connection_request_notify_handler));
  return RSI_ERROR_NONE;
}

int rsi_wlan_wfd_connect_command(command_processor_args_t *arguments)
{
  char *device_name = (char *)arguments->arg[0];
  print_status(rsi_wlan_wfd_connect(device_name, join_response_handler));
  return RSI_ERROR_NONE;
}

static void wlan_wfd_discovery_notify_handler(uint16_t status, uint8_t *buffer, const uint32_t length)
{
  printf("got wfd discovery status: 0x%X\n", status);
}
static void wlan_wfd_connection_request_notify_handler(uint16_t status, uint8_t *buffer, const uint32_t length)
{
  printf("got wfd connection status: 0x%X\n", status);
}
