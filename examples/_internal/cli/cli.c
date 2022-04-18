/*******************************************************************************
* @file  rsi_http_client_app.c
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

#include "rsi_driver.h"
#include "rsi_os.h"
#include "command_processor/command_processor.h"
#include "command_processor/default_argument_types.h"

// Wireless driver task stack size and priority
#define RSI_DRIVER_TASK_STACK_SIZE 500
#define RSI_DRIVER_TASK_PRIORITY   2

// Application task stack size and priority
#define APPLICATION_TASK_STACK_SIZE 2048
#define APPLICATION_TASK_PRIORITY   1

// Memory length for driver
#define GLOBAL_BUFFER_LENGTH 10000

static void application_task(void *arg);
static void print_command_args(const command_processor_descriptive_command_t *command);

extern const command_processor_database_t command_table;

// Memory to initialize driver
uint8_t global_buffer[GLOBAL_BUFFER_LENGTH];

rsi_semaphore_handle_t cli_line_ready;

int main()
{
  rsi_task_handle_t application_task_handle = NULL;

  rsi_semaphore_create(&cli_line_ready, 0);

  rsi_hal_board_init();

  // Task created for the application task
  rsi_task_create(application_task,
                  "application_task",
                  APPLICATION_TASK_STACK_SIZE,
                  NULL,
                  APPLICATION_TASK_PRIORITY,
                  &application_task_handle);

  // OS TAsk Start the scheduler
  rsi_start_os_scheduler();

  return RSI_ERROR_NONE;
}

static void application_task(void *arg)
{
  (void)arg;
  command_processor_args_t args;
  const command_processor_descriptive_command_t *command;

  while (1) {
    rsi_com_port_send("\r\n> ", 4);

    rsi_semaphore_wait(&cli_line_ready, 0);

    int result = process_buffer_line(&command_table, &args, &command);

    if (result == RSI_ERROR_NONE) {
      if (command->handler) {
        result = command->handler(&args);
        if (result == RSI_ERROR_NONE) {
          rsi_com_port_send("\r\nSuccess\r\n", 11);
        } else if (result == -1004) {
          rsi_com_port_send("\r\nNo Mem\r\n", 10);
        } else if (result == -1005) {
          rsi_com_port_send("\r\nTimed out\r\n", 13);
        } else {
          rsi_com_port_send("\r\nFailed\r\n", 10);
        }
      }
    } else if (result == -1002) {
      rsi_com_port_send("\r\nArgs: ", 8);
      print_command_args(command);
    } else {
      rsi_com_port_send("\r\nNot supported\r\n", 17);
    }
  }
}

rsi_task_handle_t driver_task_handle = NULL;
int rsi_device_init_command(command_processor_args_t *arguments)
{
  int32_t status;
  uint16_t oper_mode = IS_COMMAND_ARG_VALID(0) ? (uint16_t)arguments->arg[0] : 0;
  uint16_t coex_mode = IS_COMMAND_ARG_VALID(1) ? (uint16_t)arguments->arg[1] : 0;

  // Driver initialization
  status = rsi_driver_init(global_buffer, GLOBAL_BUFFER_LENGTH);
  if ((status < 0) || (status > GLOBAL_BUFFER_LENGTH)) {
    return status;
  }

  // Task created for Driver task
  rsi_task_create(rsi_wireless_driver_task,
                  "driver_task",
                  RSI_DRIVER_TASK_STACK_SIZE,
                  NULL,
                  RSI_DRIVER_TASK_PRIORITY,
                  &driver_task_handle);

  // Silabs module intialisation
  status = rsi_device_init(LOAD_NWP_FW);
  if (status != RSI_SUCCESS) {
    LOG_PRINT("Device Initialization Failed, Error Code : 0x%X\r\n", status);
    return status;
  } else {
    LOG_PRINT("Device Initialization Success\r\n");
  }

  // WC initialization
  status = rsi_wireless_init(oper_mode, coex_mode);
  if (status != RSI_SUCCESS) {
    LOG_PRINT("Wireless Initialization Failed, Error Code : 0x%X\r\n", status);
    return status;
  } else {
    LOG_PRINT("Wireless Initialization Success\r\n");
  }

  return RSI_ERROR_NONE;
}

int help_command_handler(command_processor_args_t *arguments)
{
  UNUSED_PARAMETER(arguments);
  for (uint8_t a = 0; a < command_table.length; ++a) {
    const command_processor_descriptive_command_t *temp =
      (command_processor_descriptive_command_t *)command_table.entries[a].value;
    rsi_com_port_send("\r\n", 2);
    rsi_com_port_send(command_table.entries[a].key, strlen(command_table.entries[a].key));
    rsi_com_port_send(" ", 1);
    print_command_args(temp);
    rsi_com_port_send("- ", 2);
    rsi_com_port_send(temp->description, strlen(temp->description));
  }
  rsi_com_port_send("\r\n", 2);
  return RSI_ERROR_NONE;
}

static void print_command_args(const command_processor_descriptive_command_t *command)
{
  const char *arg_name = command->argument_help;
  for (int a = 0; command->argument_list[a] != COMMAND_ARG_END; ++a) {
    if (command->argument_list[a] & COMMAND_ARG_OPTIONAL) {
      char option_char[2] = { (char)command->argument_list[a] & COMMAND_ARG_OPTIONAL_CHARACTER_MASK, 0 };
      rsi_com_port_send("-", 1);
      rsi_com_port_send(option_char, 1);
      rsi_com_port_send(" ", 1);
    } else if (command->argument_list[a] & COMMAND_ARG_ENUM) {
      rsi_com_port_send("[", 1);
      uint8_t enum_index = command->argument_list[a] & COMMAND_ARG_ENUM_INDEX_MASK;
      for (int b = 0; command_table.argument_types[enum_index][b] != NULL; /* Increment occurs in internal logic */) {
        rsi_com_port_send(command_table.argument_types[enum_index][b],
                          strlen(command_table.argument_types[enum_index][b]));
        if (command_table.argument_types[enum_index][++b]) {
          rsi_com_port_send("|", 1);
        }
      }
      rsi_com_port_send("] ", 2);
    } else {
      rsi_com_port_send("<", 1);
      const char *arg_end = arg_name;
      while (*arg_end != '|' && *arg_end != '\0')
        ++arg_end;
      rsi_com_port_send(arg_name, arg_end - arg_name);
      arg_name = arg_end + 1;
      rsi_com_port_send("> ", 2);
    }
  }
}
