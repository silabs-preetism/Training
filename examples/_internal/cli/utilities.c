#include "utilities.h"
#include "rsi_error.h"

#define IPV4_FORMAT "%u.%u.%u.%u"

uint32_t str_to_ipv4(const char *string)
{
  uint32_t temp[4];
  int nchars                        = -1;
  static const char str_to_ip_fmt[] = IPV4_FORMAT "%n";

  if (sscanf(string,
             str_to_ip_fmt,
             (unsigned int *)&temp[0],
             (unsigned int *)&temp[1],
             (unsigned int *)&temp[2],
             (unsigned int *)&temp[3],
             &nchars)
      != 4) {
    return 0;
  }
  // Check for extra characters not read in
  else if (*(string + nchars) != '\0') {
    return 0;
  } else if (temp[0] > 255 || temp[1] > 255 || temp[2] > 255 || temp[3] > 255) {
    return 0;
  }

  return (uint32_t)temp[3] << 24 | temp[2] << 16 | temp[1] << 8 | temp[0];
}

void print_status(int32_t status)
{
  switch (status) {
    case RSI_ERROR_NONE: /*printf("success\r\n");*/
      break;
    case RSI_ERROR_PKT_ALLOCATION_FAILURE:
      printf("Error: packet allocation fail\r\n");
      break;
    case RSI_ERROR_WLAN_CMD_IN_PROGRESS:
      printf("Error: WLAN command in progress\r\n");
      break;
    default:
      printf("status: 0x%X\n", status);
      break;
  }
}
