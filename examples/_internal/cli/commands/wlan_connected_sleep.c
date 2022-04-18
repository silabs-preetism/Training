#include "command_processor.h"
#include "rsi_common_apis.h"
#include "rsi_error.h"
#include <stdint.h>

int connected_sleep_demo_command(command_processor_args_t *arguments)
{
  // Validate correct starting condition

  // Parse arguments
  int beacon_drop_threshold = (int)arguments->arg[0];

  // Enable Broadcast data filter
  int32_t status = rsi_wlan_filter_broadcast(beacon_drop_threshold, 1, 1);
  if (status != RSI_SUCCESS) {
    LOG_PRINT("\r\nBroadcast Data Filtering Failed with Error Code : 0x%X\r\n", status);
  }
  LOG_PRINT("\r\nBroadcast Data Filtering Enabled\r\n");

  // Apply power save profile with connected sleep
  status = rsi_wlan_power_save_profile(RSI_SLEEP_MODE_2, RSI_MAX_PSP);
  if (status != RSI_SUCCESS) {
    LOG_PRINT("\r\nPowersave Config Failed, Error Code : 0x%X\r\n", status);
  }

  return RSI_ERROR_NONE;
}
