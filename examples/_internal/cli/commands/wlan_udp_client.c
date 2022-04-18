#include "command_processor.h"
#include "rsi_error.h"
#include "rsi_socket.h"

int wlan_udp_client_command(command_processor_args_t *arguments)
{
  uint32_t address      = (uint32_t)arguments->arg[0];
  uint16_t port         = (uint16_t)arguments->arg[1];
  uint16_t packet_count = IS_COMMAND_ARG_VALID(2) ? (uint16_t)arguments->arg[2] : 1000;
  uint16_t i            = 0;
  int status            = 0;
  int32_t client_socket;
  struct rsi_sockaddr_in server_addr;

  // Create socket
  client_socket = rsi_socket(AF_INET, SOCK_DGRAM, 0);
  if (client_socket < 0) {
    status = rsi_wlan_get_status();
    printf("\r\nSocket Create Failed, Error Code : 0x%X\r\n", status);
    return status;
  } else {
    printf("\r\nSocket Create Success\r\n");
  }

  // Set server structure
  memset(&server_addr, 0, sizeof(server_addr));

  // Set server address
  server_addr.sin_family      = AF_INET;
  server_addr.sin_port        = htons(port);
  server_addr.sin_addr.s_addr = address;

  printf("\r\nUDP TX start\r\n");
  for (uint16_t i = 0; i < packet_count; ++i) {
    // Send data on socket
    status = rsi_sendto(client_socket,
                        (int8_t *)"Hello from UDP client!!!",
                        (sizeof("Hello from UDP client!!!") - 1),
                        0,
                        (struct rsi_sockaddr *)&server_addr,
                        sizeof(server_addr));
    if (status < 0) {
      status = rsi_wlan_get_status();
      rsi_shutdown(client_socket, 0);
      printf("\r\nFailed to send data to UDP Server, Error Code : 0x%X\r\n", status);
    }
  }
  return status;
}
