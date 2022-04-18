#include "rsi_common_apis.h"
#include "rsi_wlan_apis.h"
#include "rsi_wlan.h"
#include "command_processor.h"
#include <stdint.h>

#define RSI_FWUP_RSP_OK     1
#define RSI_FWUP_RSP_DONE   2
#define RSI_FWUP_RSP_FAILED 3

static void rsi_wireless_fw_upgrade_handler(uint8_t type, uint32_t status);

// standard defines
uint8_t rsp_received;
// Buffer to read Fw version
uint8_t fw_version[20];

int wireless_firmware_upgrade_demo_command(command_processor_args_t *arguments)
{
  uint8_t fw_version[20];
  int32_t status;
  uint8_t rsp_received;

  // Register wirless firmware upgrade callback
  rsi_wlan_nwk_register_wireless_fw_upgrade_cb(RSI_WLAN_NWK_FW_UPGRADE_CB, rsi_wireless_fw_upgrade_handler);

  // Get current firmware version
  status = rsi_wlan_get(RSI_FW_VERSION, fw_version, sizeof(fw_version));
  if (status != RSI_SUCCESS) {
    LOG_PRINT("\r\nFirmware Version Request Failed, Error Code : 0x%X\r\n", status);
    return status;
  }
  LOG_PRINT("\nFirmware Version is :%s\r\n", fw_version);

  // Check for firmware upgrade request ok
  if (rsp_received == RSI_FWUP_RSP_OK) {
    // Send wireless firmware upgrade request
    status = rsi_req_wireless_fwup();
    if (status != RSI_SUCCESS) {
      LOG_PRINT("\r\nWireless Firmware Upgrade Request Failed, Error Code : 0x%X\r\n", status);
      return status;
    }

    LOG_PRINT("\nFirmware upgrade process started\r\n");
    LOG_PRINT("\nUpgradation going on.....\r\n");

    rsp_received = 0;
  }

  // Check for firmware upgrade success
  if (rsp_received == RSI_FWUP_RSP_DONE) {
    rsp_received = 0;

    LOG_PRINT("\r\nFirmware Upgrade Success\r\n");
    return RSI_ERROR_NONE;
  }

  // Check for error response
  if (rsp_received == RSI_FWUP_RSP_FAILED) {
    rsp_received = 0;

    // Get error status
    status = rsi_wlan_get_status();

    LOG_PRINT("\r\nFirmware Upgrade Failed, Error Code : 0x%X\r\n", status);
    return status;
  }

  if (status != RSI_SUCCESS) {
    return status;
  }

  rsi_bl_module_power_cycle();

  LOG_PRINT("\nModule is Rebooted\r\n");
  LOG_PRINT("\nwait for 1.5minutes to reflect updated firmware\r\n");
  rsi_delay_ms(90000);

  // Do driver init

  //  status = rsi_driver_deinit();
  //  if (status != RSI_SUCCESS) {
  //    LOG_PRINT("\nDriver deinit Failed,Error Code is:0x%04x\r\n", status);
  //    return status;
  //  }
  //  LOG_PRINT("\nDriver deinit is successful\r\n");
  //  status = rsi_driver_init(global_buf, GLOBAL_BUFF_LEN);
  //  if ((status < 0) || (status > GLOBAL_BUFF_LEN)) {
  //    LOG_PRINT("\nDriver init Failed,Error Code is:0x%04x\r\n", status);
  //    return status;
  //  }
  //  LOG_PRINT("\nDriver init is successful\r\n");
  //  status = rsi_device_init(LOAD_NWP_FW);
  //  if (status != RSI_SUCCESS) {
  //    LOG_PRINT("\nDevice init Failed,Error Code is:0x%04x\r\n", status);
  //    return status;
  //  }
  memset(fw_version, 0, sizeof(fw_version));
  status = rsi_wlan_get(RSI_FW_VERSION, fw_version, sizeof(fw_version));
  if (status != RSI_SUCCESS) {
    return status;
  }
  LOG_PRINT("\nUpdated Firmware Version is :%s\r\n", fw_version);

  return RSI_ERROR_NONE;
}

// wireless firmware upgrade handler
static void rsi_wireless_fw_upgrade_handler(uint8_t type, uint32_t status)
{
  if (status != RSI_SUCCESS) {
    rsp_received = RSI_FWUP_RSP_FAILED;
    rsi_wlan_set_status(status);
    LOG_PRINT("\r\n__________________ FWUP FAILED _______________\r\n");
    return;
  }

  switch (type) {
    case RSI_WLAN_RSP_WIRELESS_FWUP_OK:
      rsp_received = RSI_FWUP_RSP_OK;
      LOG_PRINT("\r\n__________________ FWUPOK _______________\r\n");
      break;

    case RSI_WLAN_RSP_WIRELESS_FWUP_DONE:
      rsp_received = RSI_FWUP_RSP_DONE;
      LOG_PRINT("\r\n__________________ FWUP SUCCESS _____________\r\n");
      break;
  }
}
