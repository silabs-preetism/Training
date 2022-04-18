#include "rsi_socket.h"
#include "rsi_wlan_apis.h"
#include "command_processor.h"
#include "utilities.h"
#include <stdint.h>
#include <stdbool.h>

extern int read_variable(uint32_t nvm_key, uint8_t *buffer, size_t *length);

void rsi_socket_command(command_processor_args_t *arguments);
void rsi_bind_command(command_processor_args_t *arguments);
void rsi_connect_command(command_processor_args_t *arguments);
void rsi_listen_command(command_processor_args_t *arguments);
void rsi_accept_command(command_processor_args_t *arguments);
void rsi_recvfrom_command(command_processor_args_t *arguments);
void rsi_recv_command(command_processor_args_t *arguments);
void rsi_sendto_command(command_processor_args_t *arguments);
void rsi_send_command(command_processor_args_t *arguments);
void rsi_send_large_data_async_command(command_processor_args_t *arguments);
void rsi_select_command(command_processor_args_t *arguments);
void rsi_shutdown_command(command_processor_args_t *arguments);
void rsi_socket_async_command(command_processor_args_t *arguments);
void rsi_config_ipaddress_command(command_processor_args_t *arguments);
void rsi_check_state_command(command_processor_args_t *arguments);
//void rsi_socket_create_command(command_processor_args_t *arguments);
void rsi_get_application_socket_descriptor_command(command_processor_args_t *arguments);
void rsi_clear_sockets_command(command_processor_args_t *arguments);
void calculate_buffers_required_command(command_processor_args_t *arguments);
void calculate_length_to_send_command(command_processor_args_t *arguments);
void rsi_send_async_command(command_processor_args_t *arguments);
void rsi_sendto_async_command(command_processor_args_t *arguments);
void rsi_setsockopt_command(command_processor_args_t *arguments);
void rsi_get_app_socket_descriptor_command(command_processor_args_t *arguments);
void rsi_get_primary_socket_id_command(command_processor_args_t *arguments);
void rsi_fd_isset_command(command_processor_args_t *arguments);
void rsi_set_fd_command(command_processor_args_t *arguments);
void rsi_fd_clr_command(command_processor_args_t *arguments);

static const command_processor_descriptive_command_t rsi_socket_command_info                = { rsi_socket_command,
                                                                                 "Create a socket",
                                                                                 "family"
                                                                                 "|"
                                                                                 "type"
                                                                                 "|"
                                                                                 "protocol",
                                                                                 {
                                                                                   COMMAND_ARG_INT32,
                                                                                   COMMAND_ARG_INT32,
                                                                                   COMMAND_ARG_INT32,
                                                                                   COMMAND_ARG_END,
                                                                                 } };
static const command_processor_descriptive_command_t rsi_bind_command_info                  = { rsi_bind_command,
                                                                               "Bind socket to port",
                                                                               "socket"
                                                                               "|"
                                                                               "port"
                                                                               "|"
                                                                               "ip",
                                                                               {
                                                                                 COMMAND_ARG_INT32,
                                                                                 COMMAND_ARG_UINT16,
                                                                                 COMMAND_ARG_STRING,
                                                                                 COMMAND_ARG_END,
                                                                               } };
static const command_processor_descriptive_command_t rsi_connect_command_info               = { rsi_connect_command,
                                                                                  "Connect to a remote socket",
                                                                                  "socket"
                                                                                  "|"
                                                                                  "ip"
                                                                                  "|"
                                                                                  "port",
                                                                                  {
                                                                                    COMMAND_ARG_INT32,
                                                                                    COMMAND_ARG_STRING,
                                                                                    COMMAND_ARG_UINT16,
                                                                                    COMMAND_ARG_END,
                                                                                  } };
static const command_processor_descriptive_command_t rsi_listen_command_info                = { rsi_listen_command,
                                                                                 "Listen",
                                                                                 "socket"
                                                                                 "backlog",
                                                                                 {
                                                                                   COMMAND_ARG_INT32,
                                                                                   COMMAND_ARG_INT32,
                                                                                   COMMAND_ARG_END,
                                                                                 } };
static const command_processor_descriptive_command_t rsi_accept_command_info                = { rsi_accept_command,
                                                                                 "Accept",
                                                                                 "socket",
                                                                                 {
                                                                                   COMMAND_ARG_INT32,
                                                                                   COMMAND_ARG_END,
                                                                                 } };
static const command_processor_descriptive_command_t rsi_recvfrom_command_info              = { rsi_recvfrom_command,
                                                                                   "Receive from",
                                                                                   "socket",
                                                                                   {
                                                                                     COMMAND_ARG_INT32,
                                                                                     COMMAND_ARG_END,
                                                                                   } };
static const command_processor_descriptive_command_t rsi_recv_command_info                  = { rsi_recv_command,
                                                                               "",
                                                                               "socket",
                                                                               {
                                                                                 COMMAND_ARG_INT32,
                                                                                 COMMAND_ARG_END,
                                                                               } };
static const command_processor_descriptive_command_t rsi_sendto_command_info                = { rsi_sendto_command,
                                                                                 "",
                                                                                 "",
                                                                                 {
                                                                                   COMMAND_ARG_END,
                                                                                 } };
static const command_processor_descriptive_command_t rsi_send_command_info                  = { rsi_send_command,
                                                                               "",
                                                                               "",
                                                                               {
                                                                                 COMMAND_ARG_END,
                                                                               } };
static const command_processor_descriptive_command_t rsi_send_large_data_async_command_info = {
  rsi_send_large_data_async_command,
  "",
  "",
  {
    COMMAND_ARG_END,
  }
};
static const command_processor_descriptive_command_t rsi_select_command_info           = { rsi_select_command,
                                                                                 "",
                                                                                 "",
                                                                                 {
                                                                                   COMMAND_ARG_END,
                                                                                 } };
static const command_processor_descriptive_command_t rsi_shutdown_command_info         = { rsi_shutdown_command,
                                                                                   "",
                                                                                   "",
                                                                                   {
                                                                                     COMMAND_ARG_END,
                                                                                   } };
static const command_processor_descriptive_command_t rsi_socket_async_command_info     = { rsi_socket_async_command,
                                                                                       "",
                                                                                       "",
                                                                                       {
                                                                                         COMMAND_ARG_END,
                                                                                       } };
static const command_processor_descriptive_command_t rsi_config_ipaddress_command_info = { rsi_config_ipaddress_command,
                                                                                           "Configure the client IP",
                                                                                           "[dhcp, static]",
                                                                                           {
                                                                                             COMMAND_ARG_STRING,
                                                                                             COMMAND_ARG_END,
                                                                                           } };
static const command_processor_descriptive_command_t rsi_check_state_command_info      = { rsi_check_state_command,
                                                                                      "",
                                                                                      "",
                                                                                      {
                                                                                        COMMAND_ARG_END,
                                                                                      } };
static const command_processor_descriptive_command_t rsi_get_application_socket_descriptor_command_info = {
  rsi_get_application_socket_descriptor_command,
  "",
  "",
  {
    COMMAND_ARG_END,
  }
};
static const command_processor_descriptive_command_t rsi_clear_sockets_command_info = { rsi_clear_sockets_command,
                                                                                        "",
                                                                                        "",
                                                                                        {
                                                                                          COMMAND_ARG_END,
                                                                                        } };
static const command_processor_descriptive_command_t calculate_buffers_required_command_info = {
  calculate_buffers_required_command,
  "",
  "",
  {
    COMMAND_ARG_END,
  }
};
static const command_processor_descriptive_command_t calculate_length_to_send_command_info = {
  calculate_length_to_send_command,
  "",
  "",
  {
    COMMAND_ARG_END,
  }
};
static const command_processor_descriptive_command_t rsi_send_async_command_info   = { rsi_send_async_command,
                                                                                     "",
                                                                                     "",
                                                                                     {
                                                                                       COMMAND_ARG_END,
                                                                                     } };
static const command_processor_descriptive_command_t rsi_sendto_async_command_info = { rsi_sendto_async_command,
                                                                                       "",
                                                                                       "",
                                                                                       {
                                                                                         COMMAND_ARG_END,
                                                                                       } };
static const command_processor_descriptive_command_t rsi_setsockopt_command_info   = { rsi_setsockopt_command,
                                                                                     "",
                                                                                     "",
                                                                                     {
                                                                                       COMMAND_ARG_END,
                                                                                     } };
static const command_processor_descriptive_command_t rsi_get_app_socket_descriptor_command_info = {
  rsi_get_app_socket_descriptor_command,
  "",
  "",
  {
    COMMAND_ARG_END,
  }
};
static const command_processor_descriptive_command_t rsi_get_primary_socket_id_command_info = {
  rsi_get_primary_socket_id_command,
  "",
  "",
  {
    COMMAND_ARG_END,
  }
};
static const command_processor_descriptive_command_t rsi_fd_isset_command_info = { rsi_fd_isset_command,
                                                                                   "",
                                                                                   "",
                                                                                   {
                                                                                     COMMAND_ARG_END,
                                                                                   } };
static const command_processor_descriptive_command_t rsi_set_fd_command_info   = { rsi_set_fd_command,
                                                                                 "",
                                                                                 "",
                                                                                 {
                                                                                   COMMAND_ARG_END,
                                                                                 } };
static const command_processor_descriptive_command_t rsi_fd_clr_command_info   = { rsi_fd_clr_command,
                                                                                 "",
                                                                                 "",
                                                                                 {
                                                                                   COMMAND_ARG_END,
                                                                                 } };
//static const sl_cli_command_info_t rsi_socket_create_command_info                     = SL_CLI_COMMAND( rsi_socket_create_command, "Create a socket?", "socket"  "type"  "backlog", { COMMAND_ARG_INT32, COMMAND_ARG_INT32, COMMAND_ARG_INT32, COMMAND_ARG_END, });

sl_cli_command_entry_t wlan_socket_command_table[] = {
  { "rsi_socket", &rsi_socket_command_info, false },
  { "rsi_bind", &rsi_bind_command_info, false },
  { "rsi_connect", &rsi_connect_command_info, false },
  { "rsi_listen", &rsi_listen_command_info, false },
  { "rsi_accept", &rsi_accept_command_info, false },
  { "rsi_recvfrom", &rsi_recvfrom_command_info, false },
  { "rsi_recv", &rsi_recv_command_info, false },
  { "rsi_sendto", &rsi_sendto_command_info, false },
  { "rsi_send", &rsi_send_command_info, false },
  { "rsi_send_large_data_async", &rsi_send_large_data_async_command_info, false },
  { "rsi_select", &rsi_select_command_info, false },
  { "rsi_shutdown", &rsi_shutdown_command_info, false },
  { "rsi_socket_async", &rsi_socket_async_command_info, false },
  { "rsi_config_ipaddress", &rsi_config_ipaddress_command_info, false },
  { "rsi_check_state", &rsi_check_state_command_info, false },
  { "rsi_get_application_socket_descriptor", &rsi_get_application_socket_descriptor_command_info, false },
  { "rsi_clear_sockets", &rsi_clear_sockets_command_info, false },
  { "calculate_buffers_required", &calculate_buffers_required_command_info, false },
  { "calculate_length_to_send", &calculate_length_to_send_command_info, false },
  { "rsi_send_async", &rsi_send_async_command_info, false },
  { "rsi_sendto_async", &rsi_sendto_async_command_info, false },
  { "rsi_setsockopt", &rsi_setsockopt_command_info, false },
  { "rsi_get_app_socket_descriptor", &rsi_get_app_socket_descriptor_command_info, false },
  { "rsi_get_primary_socket_id", &rsi_get_primary_socket_id_command_info, false },
  { "rsi_fd_isset", &rsi_fd_isset_command_info, false },
  { "rsi_set_fd", &rsi_set_fd_command_info, false },
  { "rsi_fd_clr", &rsi_fd_clr_command_info, false },
  //   { "rsi_socket_create", &rsi_socket_create_command_info, false },
};

rsi_rsp_ipv4_parmas_t current_ip;

void rsi_socket_command(command_processor_args_t *arguments)
{
  int32_t protocol_family = sl_cli_get_argument_int32(arguments, 0);
  int32_t type            = sl_cli_get_argument_int32(arguments, 1);
  int32_t protocol        = sl_cli_get_argument_int32(arguments, 2);

  int result = rsi_socket(protocol_family, type, protocol);
  if (result >= 0) {
    printf("socket #: %d", result);
  } else {
    print_status(result);
  }
}

void rsi_bind_command(command_processor_args_t *arguments)
{
  int32_t sock_id       = sl_cli_get_argument_int32(arguments, 0);
  uint16_t local_port   = sl_cli_get_argument_int16(arguments, 1);
  char *local_ip_string = sl_cli_get_argument_string(arguments, 2);

  uint32_t local_ip = str_to_ipv4(local_ip_string);
  if (local_ip == 0) {
    print_status(RSI_ERROR_INVALID_PARAM);
    return;
  }

  struct rsi_sockaddr_in address = {
    .sin_family = AF_INET,
    .sin_port   = local_port,
    .sin_addr   = local_ip,
  };
  print_status(rsi_bind(sock_id, &address, sizeof(address)));
}

void rsi_connect_command(command_processor_args_t *arguments)
{
  int32_t sock_id        = sl_cli_get_argument_int32(arguments, 0);
  char *remote_ip_string = sl_cli_get_argument_string(arguments, 1);
  uint16_t remote_port   = sl_cli_get_argument_uint16(arguments, 2);

  uint32_t remote_ip = str_to_ipv4(remote_ip_string);
  if (remote_ip == 0) {
    print_status(RSI_ERROR_INVALID_PARAM);
    return;
  }

  struct rsi_sockaddr_in remote_address = {
    .sin_family = AF_INET,
    .sin_port   = remote_port,
    .sin_addr   = remote_ip,
  };
  print_status(rsi_connect(sock_id, &remote_address, sizeof(remote_address)));
}

void rsi_listen_command(command_processor_args_t *arguments)
{
  int32_t sockID  = sl_cli_get_argument_int32(arguments, 0);
  int32_t backlog = sl_cli_get_argument_int32(arguments, 1);
  print_status(rsi_listen(sockID, backlog));
}

void rsi_accept_command(command_processor_args_t *arguments)
{
  int32_t sockID = sl_cli_get_argument_int32(arguments, 0);
  struct rsi_sockaddr_in client_address;
  int32_t address_length;
  print_status(rsi_accept(sockID, (struct rsi_sockaddr *)&client_address, &address_length));
  printf("Accepted connection from 0x%x\n", client_address.sin_addr.s_addr);
}

void rsi_recvfrom_command(command_processor_args_t *arguments)
{
  int32_t sockID      = sl_cli_get_argument_int32(arguments, 0);
  int32_t buffer_size = 1024;
  uint8_t *buffer     = malloc(buffer_size);
  int32_t flags;
  struct rsi_sockaddr_in client_address;
  int32_t client_address_length = sizeof(client_address);

  if (buffer == NULL)
    return;

  print_status(rsi_recvfrom(sockID, buffer, buffer_size, flags, &client_address, &client_address_length));

  free(buffer);
}

void rsi_recv_command(command_processor_args_t *arguments)
{
  int32_t sockID      = sl_cli_get_argument_int32(arguments, 0);
  int32_t buffer_size = 1024;
  uint8_t *buffer     = malloc(buffer_size);
  int32_t flags;

  if (buffer == NULL)
    return;

  print_status(rsi_recv(sockID, buffer, buffer_size, flags));

  free(buffer);
}

void rsi_sendto_command(command_processor_args_t *arguments)
{
}

void rsi_send_command(command_processor_args_t *arguments)
{
}

void rsi_send_large_data_async_command(command_processor_args_t *arguments)
{
}

void rsi_select_command(command_processor_args_t *arguments)
{
}

void rsi_shutdown_command(command_processor_args_t *arguments)
{
}

void rsi_socket_async_command(command_processor_args_t *arguments)
{
}

typedef struct {
  uint8_t ip_address[4];
  uint8_t netmask[4];
  uint8_t gateway[4];
} ipv4_address_setting_t;
void rsi_config_ipaddress_command(command_processor_args_t *arguments)
{
  ipv4_address_setting_t ip;
  uint8_t mode;
  char *mode_string = sl_cli_get_argument_string(arguments, 0);

  if (strncmp(mode_string, "static", 6) == 0) {
    mode          = RSI_STATIC;
    size_t length = sizeof(ip);
    //    read_variable(WLAN_IP_NVM_ID, &ip, &length);
  } else if (strncmp(mode_string, "dhcp", 4) == 0) {
    mode = RSI_DHCP;
    memset(&ip, 0, sizeof(ip));
  } else {
    print_status(RSI_ERROR_INVALID_PARAM);
    return;
  }

  int32_t status = rsi_config_ipaddress(RSI_IP_VERSION_4,
                                        mode,
                                        ip.ip_address,
                                        ip.netmask,
                                        ip.gateway,
                                        &current_ip,
                                        sizeof(current_ip),
                                        0);
  print_status(status);

  if (status == RSI_ERROR_NONE) {
    printf("IP:      %d.%d.%d.%d\n",
           current_ip.ipaddr[0],
           current_ip.ipaddr[1],
           current_ip.ipaddr[2],
           current_ip.ipaddr[3]);
    printf("netmask: %d.%d.%d.%d\n",
           current_ip.netmask[0],
           current_ip.netmask[1],
           current_ip.netmask[2],
           current_ip.netmask[3]);
    printf("gateway: %d.%d.%d.%d\n",
           current_ip.gateway[0],
           current_ip.gateway[1],
           current_ip.gateway[2],
           current_ip.gateway[3]);
  }
}

void rsi_check_state_command(command_processor_args_t *arguments)
{
  int32_t type = sl_cli_get_argument_int32(arguments, 0);
  print_status(rsi_check_state(type));
}

//void rsi_socket_create_command(command_processor_args_t *arguments)
//{
//   int32_t socket  = sl_cli_get_argument_int32(arguments, 0);
//   int32_t type    = sl_cli_get_argument_int32(arguments, 1);
//   int32_t backlog = sl_cli_get_argument_int32(arguments, 2);
//
//   print_status( rsi_socket_create(socket, type, backlog) );
//}

void rsi_get_application_socket_descriptor_command(command_processor_args_t *arguments)
{
}

void rsi_clear_sockets_command(command_processor_args_t *arguments)
{
  int32_t sock_id = sl_cli_get_argument_int32(arguments, 0);
  rsi_clear_sockets(sock_id);
  print_status(RSI_ERROR_NONE);
}

void calculate_buffers_required_command(command_processor_args_t *arguments)
{
}

void calculate_length_to_send_command(command_processor_args_t *arguments)
{
}

void rsi_send_async_command(command_processor_args_t *arguments)
{
}

void rsi_sendto_async_command(command_processor_args_t *arguments)
{
}

void rsi_setsockopt_command(command_processor_args_t *arguments)
{
}

void rsi_get_app_socket_descriptor_command(command_processor_args_t *arguments)
{
}

void rsi_get_primary_socket_id_command(command_processor_args_t *arguments)
{
}

void rsi_fd_isset_command(command_processor_args_t *arguments)
{
}

void rsi_set_fd_command(command_processor_args_t *arguments)
{
}

void rsi_fd_clr_command(command_processor_args_t *arguments)
{
}
