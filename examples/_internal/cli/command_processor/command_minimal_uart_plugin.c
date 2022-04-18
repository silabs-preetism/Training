/*
 * EVALUATION AND USE OF THIS SOFTWARE IS SUBJECT TO THE TERMS AND
 * CONDITIONS OF THE CONTROLLING LICENSE AGREEMENT FOUND AT LICENSE.md
 * IN THIS SDK. IF YOU DO NOT AGREE TO THE LICENSE TERMS AND CONDITIONS,
 * PLEASE RETURN ALL SOURCE FILES TO SILICON LABORATORIES.
 * (c) Copyright 2018, Silicon Laboratories Inc.  All rights reserved.
 */

/** @file
 *
 * File Description
 *
 */
#include "command_processor.h"
#include "rsi_os.h"
#include <stdint.h>
#include <string.h>
#include <stdbool.h>

/******************************************************
 *                      Macros
 ******************************************************/

/******************************************************
 *                    Constants
 ******************************************************/

#define USER_RX_BUFFER_SIZE  (512)
#define USER_RX_BUFFER_COUNT (4)
#define INVALID_INDEX        (0xFF)

// Special characters
#define BACKSPACE_CHARACTER           '\b'
#define DELETE_CHARACTER              (127)
#define LINEFEED_CHARACTER            (10)
#define CARRIAGE_RETURN_CHARACTER     (13)
#define ESCAPE_CHARACTER              (27)
#define SPACE_CHARACTER               (32)
#define OPEN_SQUARE_BRACKET_CHARACTER (91)

/******************************************************
 *                   Enumerations
 ******************************************************/

/******************************************************
 *                 Type Definitions
 ******************************************************/

/******************************************************
 *                    Structures
 ******************************************************/

/******************************************************
 *               Static Function Declarations
 ******************************************************/

/******************************************************
 *               Variable Definitions
 ******************************************************/

static uint8_t user_rx_buffer[USER_RX_BUFFER_COUNT][USER_RX_BUFFER_SIZE];
static uint16_t user_rx_buffer_write_pointer = 0;
static uint8_t current_buffer_index          = 0;
static uint8_t buffer_ready_index            = INVALID_INDEX;

extern rsi_semaphore_handle_t cli_line_ready;

/******************************************************
 *               Function Definitions
 ******************************************************/

int process_buffer_line(const command_processor_database_t *command_database,
                        command_processor_args_t *args,
                        const command_processor_descriptive_command_t **command)
{
  int result = -1001;

  if (buffer_ready_index != INVALID_INDEX) {
    command_processor_add_to_history((char *)&user_rx_buffer[buffer_ready_index],
                                     strlen((char *)&user_rx_buffer[buffer_ready_index]));
    result =
      command_processor_parse_command((char *)user_rx_buffer[buffer_ready_index], command_database, args, command);
    buffer_ready_index = INVALID_INDEX;
  }

  return result;
}

void uart_rx_handler(char character)
{
  static uint8_t escape_sequence = 0;

  // Support for CRLF
  static uint8_t last_char_cr = false;

  if (escape_sequence == 0) {
    switch (character) {
      case DELETE_CHARACTER:
      case BACKSPACE_CHARACTER:
        last_char_cr = false;
        if (user_rx_buffer_write_pointer > 0) {
          user_rx_buffer_write_pointer--;
          //          rsi_uart_send(&character, 1);
        }
        break;

      case LINEFEED_CHARACTER:
        if (last_char_cr != false) {
          last_char_cr = false;
          break;
        }
        __attribute__((fallthrough));

      case CARRIAGE_RETURN_CHARACTER:
        if (character == '\r') {
          last_char_cr = true;
        }
        user_rx_buffer[current_buffer_index][user_rx_buffer_write_pointer] = 0;
        buffer_ready_index                                                 = current_buffer_index;
        user_rx_buffer_write_pointer                                       = 0;
        current_buffer_index++;

        if (current_buffer_index >= USER_RX_BUFFER_COUNT) {
          current_buffer_index = 0;
        }
        rsi_semaphore_post(&cli_line_ready);
        break;

      case ESCAPE_CHARACTER:
        last_char_cr    = false;
        escape_sequence = 1;
        break;

      default:
        last_char_cr                                                       = false;
        user_rx_buffer[current_buffer_index][user_rx_buffer_write_pointer] = character;
        user_rx_buffer_write_pointer++;

        //        rsi_uart_send(&character, 1);
        break;
    }
  } else {
    if (escape_sequence == 1) {
      if (character == OPEN_SQUARE_BRACKET_CHARACTER) {
        escape_sequence = 2;
      } else {
        escape_sequence = 0;
      }
    } else {
      switch (character) {
        case 'A':
          // Up arrow pressed
          current_buffer_index--;

          if (current_buffer_index >= USER_RX_BUFFER_COUNT) {
            current_buffer_index = USER_RX_BUFFER_COUNT - 1;
          }

          for (user_rx_buffer_write_pointer = 0; user_rx_buffer_write_pointer < USER_RX_BUFFER_SIZE;
               user_rx_buffer_write_pointer++) {
            if (user_rx_buffer[current_buffer_index][user_rx_buffer_write_pointer] == 0) {
              break;
            } else {
              //              rsi_uart_send(&user_rx_buffer[current_buffer_index][user_rx_buffer_write_pointer], 1);
            }
          }
          break;
      }

      escape_sequence = 0;
    }
  }
}
