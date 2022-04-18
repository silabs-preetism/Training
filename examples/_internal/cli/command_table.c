#include "command_processor.h"
#include <stdbool.h>

extern int rsi_device_init_command(command_processor_args_t *arguments);
extern int rsi_wlan_scan_command(command_processor_args_t *arguments);
extern int rsi_wlan_scan_async_command(command_processor_args_t *arguments);
extern int rsi_wlan_connect_command(command_processor_args_t *arguments);
extern int rsi_wlan_connect_async_command(command_processor_args_t *arguments);
extern int rsi_wlan_execute_post_connect_cmds_command(command_processor_args_t *arguments);
extern int rsi_wlan_disconnect_command(command_processor_args_t *arguments);
extern int rsi_wlan_disconnect_stations_command(command_processor_args_t *arguments);
extern int rsi_wlan_get_status_command(command_processor_args_t *arguments);
extern int rsi_wlan_wps_generate_pin_command(command_processor_args_t *arguments);
extern int rsi_wlan_wps_enter_pin_command(command_processor_args_t *arguments);
extern int rsi_wlan_wps_push_button_event_command(command_processor_args_t *arguments);
extern int rsi_transmit_test_start_command(command_processor_args_t *arguments);
extern int rsi_transmit_test_stop_command(command_processor_args_t *arguments);
extern int rsi_wlan_receive_stats_start_command(command_processor_args_t *arguments);
extern int rsi_wlan_receive_stats_stop_command(command_processor_args_t *arguments);
extern int rsi_wlan_enable_auto_config_command(command_processor_args_t *arguments);
extern int rsi_wlan_bgscan_profile_command(command_processor_args_t *arguments);
extern int rsi_wlan_scan_with_bitmap_options_command(command_processor_args_t *arguments);
extern int rsi_wlan_scan_async_with_bitmap_options_command(command_processor_args_t *arguments);
extern int rsi_wlan_ping_async_command(command_processor_args_t *arguments);
extern int help_command_handler(command_processor_args_t *arguments);
extern int connected_sleep_demo_command(command_processor_args_t *arguments);
extern int rsi_wlan_dhcp_command(command_processor_args_t *arguments);
extern int wlan_udp_client_command(command_processor_args_t *arguments);
extern int wlan_access_point_command(command_processor_args_t *arguments);

static const command_processor_descriptive_command_t cli_cmd__init               = { rsi_device_init_command,
                                                                       "Initialize the RS911x device",
                                                                       "operation mode"
                                                                       "|"
                                                                       "coex mode",
                                                                       {
                                                                         COMMAND_OPTIONAL_ARG('o', COMMAND_ARG_UINT16),
                                                                         COMMAND_OPTIONAL_ARG('c', COMMAND_ARG_UINT16),
                                                                         COMMAND_ARG_END,
                                                                       } };
static const command_processor_descriptive_command_t cli_cmd__wlan_scan          = { rsi_wlan_scan_command,
                                                                            "Scan for a Wi-Fi AP",
                                                                            "SSID"
                                                                            "|"
                                                                            "Channel",
                                                                            {
                                                                              COMMAND_ARG_STRING,
                                                                              COMMAND_ARG_UINT32,
                                                                              COMMAND_ARG_END,
                                                                            } };
static const command_processor_descriptive_command_t cli_cmd__wlan_scan_async    = { rsi_wlan_scan_async_command,
                                                                                  "Scan for a Wi-Fi AP asynchronously",
                                                                                  "SSID"
                                                                                  "|"
                                                                                  "Channel",
                                                                                  {
                                                                                    COMMAND_ARG_STRING,
                                                                                    COMMAND_ARG_UINT32,
                                                                                    COMMAND_ARG_END,
                                                                                  } };
static const command_processor_descriptive_command_t cli_cmd__wlan_connect       = { rsi_wlan_connect_command,
                                                                               "Connect to AP",
                                                                               "SSID"
                                                                               "|"
                                                                               "Security"
                                                                               "|"
                                                                               "Passphrase",
                                                                               {
                                                                                 COMMAND_ARG_STRING,
                                                                                 COMMAND_ARG_UINT8,
                                                                                 COMMAND_ARG_STRING,
                                                                                 COMMAND_ARG_END,
                                                                               } };
static const command_processor_descriptive_command_t cli_cmd__wlan_connect_async = { rsi_wlan_connect_async_command,
                                                                                     "Connect to AP asynchronously",
                                                                                     "SSID"
                                                                                     "|"
                                                                                     "Security"
                                                                                     "|"
                                                                                     "Passphrase",
                                                                                     {
                                                                                       COMMAND_ARG_STRING,
                                                                                       COMMAND_ARG_UINT8,
                                                                                       COMMAND_ARG_STRING,
                                                                                       COMMAND_ARG_END,
                                                                                     } };
static const command_processor_descriptive_command_t cli_cmd__wlan_execute_post_connect_cmds = {
  rsi_wlan_execute_post_connect_cmds_command,
  "To be called after rsi_wlan_connect_async",
  "",
  {
    COMMAND_ARG_END,
  }
};
static const command_processor_descriptive_command_t cli_cmd__wlan_disconnect          = { rsi_wlan_disconnect_command,
                                                                                  "Disconnect from AP",
                                                                                  "",
                                                                                  {
                                                                                    COMMAND_ARG_END,
                                                                                  } };
static const command_processor_descriptive_command_t cli_cmd__wlan_disconnect_stations = {
  rsi_wlan_disconnect_stations_command,
  "Disconnect a station from soft AP",
  "Client MAC",
  {
    COMMAND_ARG_STRING,
    COMMAND_ARG_END,
  }
};
static const command_processor_descriptive_command_t cli_cmd__wlan_get_status       = { rsi_wlan_get_status_command,
                                                                                  "Retrieve Wi-Fi status",
                                                                                  "",
                                                                                  {
                                                                                    COMMAND_ARG_END,
                                                                                  } };
static const command_processor_descriptive_command_t cli_cmd__wlan_wps_generate_pin = {
  rsi_wlan_wps_generate_pin_command,
  "Generate a WPS PIN",
  "",
  {
    COMMAND_ARG_END,
  }
};
static const command_processor_descriptive_command_t cli_cmd__wlan_wps_enter_pin = { rsi_wlan_wps_enter_pin_command,
                                                                                     "Set the WPS PIN",
                                                                                     "PIN",
                                                                                     {
                                                                                       COMMAND_ARG_STRING,
                                                                                       COMMAND_ARG_END,
                                                                                     } };
static const command_processor_descriptive_command_t cli_cmd__wlan_wps_push_button_event = {
  rsi_wlan_wps_push_button_event_command,
  "Start WPS push button mode",
  "SSID",
  {
    COMMAND_ARG_STRING,
    COMMAND_ARG_END,
  }
};
static const command_processor_descriptive_command_t cli_cmd__transmit_test_start = { rsi_transmit_test_start_command,
                                                                                      "Start TX test",
                                                                                      "power"
                                                                                      "|"
                                                                                      "rate"
                                                                                      "|"
                                                                                      "length"
                                                                                      "|"
                                                                                      "mode"
                                                                                      "|"
                                                                                      "channel",
                                                                                      {
                                                                                        COMMAND_ARG_UINT16,
                                                                                        COMMAND_ARG_UINT32,
                                                                                        COMMAND_ARG_UINT16,
                                                                                        COMMAND_ARG_UINT16,
                                                                                        COMMAND_ARG_UINT16,
                                                                                        COMMAND_ARG_END,
                                                                                      } };
static const command_processor_descriptive_command_t cli_cmd__transmit_test_stop  = { rsi_transmit_test_stop_command,
                                                                                     "Stop TX test",
                                                                                     "",
                                                                                     {
                                                                                       COMMAND_ARG_END,
                                                                                     } };
static const command_processor_descriptive_command_t cli_cmd__wlan_receive_stats_start = {
  rsi_wlan_receive_stats_start_command,
  "Start RX stats",
  "channel",
  {
    COMMAND_ARG_UINT16,
    COMMAND_ARG_END,
  }
};
static const command_processor_descriptive_command_t cli_cmd__wlan_receive_stats_stop = {
  rsi_wlan_receive_stats_stop_command,
  "Stop RX stats",
  "",
  {
    COMMAND_ARG_END,
  }
};
static const command_processor_descriptive_command_t cli_cmd__wlan_enable_auto_config = {
  rsi_wlan_enable_auto_config_command,
  "Enable auto-config",
  "enable"
  "|"
  "type",
  {
    COMMAND_ARG_UINT8,
    COMMAND_ARG_UINT32,
    COMMAND_ARG_END,
  }
};
static const command_processor_descriptive_command_t cli_cmd__wlan_bgscan_profile = { rsi_wlan_bgscan_profile_command,
                                                                                      "bgscan",
                                                                                      "command",
                                                                                      {
                                                                                        COMMAND_ARG_UINT8,
                                                                                        COMMAND_ARG_END,
                                                                                      } };
static const command_processor_descriptive_command_t cli_cmd__wlan_scan_with_bitmap_options = {
  rsi_wlan_scan_with_bitmap_options_command,
  "Scan with bitmap options",
  "SSID"
  "|"
  "channel"
  "|"
  "bitmap",
  {
    COMMAND_ARG_STRING,
    COMMAND_ARG_UINT16,
    COMMAND_ARG_UINT32,
    COMMAND_ARG_END,
  }
};
static const command_processor_descriptive_command_t cli_cmd__wlan_scan_async_with_bitmap_options = {
  rsi_wlan_scan_async_with_bitmap_options_command,
  "Async scan with bitmap options",
  "SSID"
  "|"
  "channel"
  "|"
  "bitmap",
  {
    COMMAND_ARG_STRING,
    COMMAND_ARG_UINT16,
    COMMAND_ARG_UINT32,
    COMMAND_ARG_END,
  }
};
static const command_processor_descriptive_command_t cli_cmd__wlan_ping_async = { rsi_wlan_ping_async_command,
                                                                                  "Ping asynchronously",
                                                                                  "flags"
                                                                                  "|"
                                                                                  "IP address"
                                                                                  "|"
                                                                                  "packet size",
                                                                                  {
                                                                                    COMMAND_ARG_UINT8,
                                                                                    COMMAND_ARG_STRING,
                                                                                    COMMAND_ARG_UINT16,
                                                                                    COMMAND_ARG_END,
                                                                                  } };
static const command_processor_descriptive_command_t cli_cmd__wlan_dhcp       = { rsi_wlan_dhcp_command,
                                                                            "Perform DHCP",
                                                                            "",
                                                                            {
                                                                              COMMAND_ARG_END,
                                                                            } };
static const command_processor_descriptive_command_t cli_cmd__wlan_udp_client = {
  wlan_udp_client_command,
  "UDP Client demo",
  "server IP"
  "|"
  "server port"
  "|"
  "packet count",
  {
    COMMAND_ARG_IP_ADDRESS,
    COMMAND_ARG_UINT16,
    COMMAND_OPTIONAL_ARG('p', COMMAND_ARG_UINT16),
    COMMAND_ARG_END,
  }
};
static const command_processor_descriptive_command_t cli_cmd__connected_sleep_demo_command = {
  connected_sleep_demo_command,
  "Demonstrate connected sleep",
  "beacon drop threshold",
  {
    COMMAND_ARG_INT32,
    COMMAND_ARG_END,
  }
};
static const command_processor_descriptive_command_t cli_cmd__wlan_access_point = {
  wlan_access_point_command,
  "Start soft AP",
  "SSID"
  "|"
  "channel"
  "|"
  "security"
  "|"
  "encryption"
  "|"
  "passphrase"
  "|"
  "beacon"
  "|"
  "dtim",
  {
    COMMAND_ARG_STRING,
    COMMAND_ARG_UINT8,
    COMMAND_ARG_UINT8,
    COMMAND_ARG_UINT8,
    COMMAND_ARG_STRING,
    COMMAND_OPTIONAL_ARG('b', COMMAND_ARG_UINT16),
    COMMAND_OPTIONAL_ARG('d', COMMAND_ARG_UINT8),
    COMMAND_ARG_END,
  }
};

static const command_processor_descriptive_command_t help_command = { help_command_handler,
                                                                      "",
                                                                      "",
                                                                      { COMMAND_ARG_END } };

const command_processor_database_t command_table = {
  .argument_types = command_argument_types,
  COMMAND_DATABASE_ENTRIES({ "?", &help_command },
                           { "help", &help_command },
                           { "init", &cli_cmd__init },
                           { "wlan_scan", &cli_cmd__wlan_scan },
                           { "wlan_scan_async", &cli_cmd__wlan_scan_async },
                           { "wlan_connect", &cli_cmd__wlan_connect },
                           { "wlan_connect_async", &cli_cmd__wlan_connect_async },
                           { "wlan_execute_post_connect_cmds", &cli_cmd__wlan_execute_post_connect_cmds },
                           { "wlan_disconnect", &cli_cmd__wlan_disconnect },
                           { "wlan_disconnect_stations", &cli_cmd__wlan_disconnect_stations },
                           { "wlan_get_status", &cli_cmd__wlan_get_status },
                           { "wlan_wps_generate_pin", &cli_cmd__wlan_wps_generate_pin },
                           { "wlan_wps_enter_pin", &cli_cmd__wlan_wps_enter_pin },
                           { "wlan_wps_push_button_event", &cli_cmd__wlan_wps_push_button_event },
                           { "transmit_test_start", &cli_cmd__transmit_test_start },
                           { "transmit_test_stop", &cli_cmd__transmit_test_stop },
                           { "wlan_receive_stats_start", &cli_cmd__wlan_receive_stats_start },
                           { "wlan_receive_stats_stop", &cli_cmd__wlan_receive_stats_stop },
                           { "wlan_enable_auto_config", &cli_cmd__wlan_enable_auto_config },
                           { "wlan_bgscan_profile", &cli_cmd__wlan_bgscan_profile },
                           { "wlan_scan_with_bitmap_options", &cli_cmd__wlan_scan_with_bitmap_options },
                           { "wlan_scan_async_with_bitmap_options", &cli_cmd__wlan_scan_async_with_bitmap_options },
                           { "wlan_ping_async", &cli_cmd__wlan_ping_async },
                           { "wlan_dhcp", &cli_cmd__wlan_dhcp },
                           { "wlan_udp_client", &cli_cmd__wlan_udp_client },
                           { "demo_sleep", &cli_cmd__connected_sleep_demo_command },
                           { "wlan_access_point", &cli_cmd__wlan_access_point }, )
};
