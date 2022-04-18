/*
 * ZentriOS SDK LICENSE AGREEMENT | Zentri.com, 2016.
 *
 * Use of source code and/or libraries contained in the ZentriOS SDK is
 * subject to the Zentri Operating System SDK license agreement and
 * applicable open source license agreements.
 *
 */

/** @file
 *
 * File Description
 *
 */

#include "command_processor.h"
#include "rsi_error.h"
#include <stdint.h>
#include <stddef.h>
#include <string.h>
#include <stdlib.h>

/******************************************************
 *                      Macros
 ******************************************************/

#define MIN(a, b) ((a) < (b) ? (a) : (b))

/******************************************************
 *                    Constants
 ******************************************************/

#ifndef COMMAND_HISTORY_BUFFER_SIZE
#define COMMAND_HISTORY_BUFFER_SIZE 128
#endif

/******************************************************
 *                   Enumerations
 ******************************************************/

/******************************************************
 *                 Type Definitions
 ******************************************************/

/******************************************************
 *                    Structures
 ******************************************************/

typedef struct {
  uint8_t length;
  uint8_t data[];
} command_history_entry_t;

/******************************************************
 *               Static Function Declarations
 ******************************************************/

static inline int parse_arg(const command_processor_database_t *db,
                            command_argument_type_t type,
                            char *line,
                            uint32_t *arg_result);
static inline uint8_t parse_enum_arg(const char *line, const char *const *options);

/******************************************************
 *               Variable Definitions
 ******************************************************/

const char *const command_argument_type_strings[] = {
  [COMMAND_ARG_NONE] = "",           [COMMAND_ARG_UINT8] = "uint8_t", [COMMAND_ARG_UINT16] = "uint16_t",
  [COMMAND_ARG_UINT32] = "uint32_t", [COMMAND_ARG_INT8] = "int8_t",   [COMMAND_ARG_INT16] = "int16_t",
  [COMMAND_ARG_INT32] = "int32_t",   [COMMAND_ARG_STRING] = "char*",  [COMMAND_ARG_REMAINING_COMMAND_LINE] = "...",
};

char command_history_buffer[COMMAND_HISTORY_BUFFER_SIZE];

uint32_t command_history_first = 0xFFFFFFFF;
uint32_t command_history_last  = 0xFFFFFFFF;
uint32_t command_history_end   = 0;

/******************************************************
 *               Function Definitions
 ******************************************************/

int command_processor_parse_command(char *command_line,
                                    const command_processor_database_t *db,
                                    command_processor_args_t *args,
                                    const command_processor_descriptive_command_t **output_command)
{
  int result                            = 0;
  const command_database_entry_t *entry = NULL;
  char *lasts                           = NULL;
  char *token;
  uint32_t token_length;
  uint32_t index;

start_processing_command_from_new_database:

  args->bitmap = 0;

  token        = strtok_r(command_line, " ", &lasts);
  token_length = strlen(token);

  // Try find matching command
  index  = 0;
  result = command_processor_find_command(token, token_length, db, &entry, &index);
  if (result != 0)
    return result;

  *output_command = entry->value;

  // Try parse command args
  if ((*output_command)->argument_list != NULL) {
    int a         = 0;
    int arg_count = 0;
    command_argument_type_t type;

    token = strtok_r(NULL, " ", &lasts);
    while ((type = (*output_command)->argument_list[a]) != COMMAND_ARG_END) {
      if (token == NULL) {
        if (type & COMMAND_ARG_OPTIONAL) {
          a += 2;
          ++arg_count;
          continue;
        }
        return -1002;
      }

      if (type & COMMAND_ARG_OPTIONAL) {
        // Verify format
        if (token[0] != '-') {
          return -1002;
        }
        // Verify character
        if (token[1] != (type & COMMAND_ARG_OPTIONAL_CHARACTER_MASK)) {
          // It's an option but not the one we expect. Skip to the next arg option
          a += 2;
          ++arg_count;
          continue;
        }
        // Skip to next token
        token = strtok_r(NULL, " ", &lasts);
        ++a;
        type = (*output_command)->argument_list[a];
      } else if (type == COMMAND_ARG_SUB_COMMAND) {
        command_line = token; // Not sure if this is right
        db           = (const command_processor_database_t *)((uint32_t)(*output_command)->argument_list[a + 1]
                                                    + ((*output_command)->argument_list[a + 2] << 8)
                                                    + ((*output_command)->argument_list[a + 3] << 16)
                                                    + ((*output_command)->argument_list[a + 4] << 24));
        goto start_processing_command_from_new_database;
      }

      result = parse_arg(db, type, token, &args->arg[arg_count]);
      if (result == 0) {
        token = strtok_r(NULL, " ", &lasts);
        args->bitmap |= (1 << arg_count);
      }
      ++a;
      ++arg_count;
    }
  }

  return result;
}

void command_processor_add_to_history(const char *line, uint8_t line_length)
{
  const uint32_t entry_length = (line_length + sizeof(command_history_entry_t) + 1);
  //    uint32_t end = (command_history_last == 0xFFFFFFFF) ? 0 : (command_history_last + command_history_buffer[command_history_last] + 1) % sizeof( command_history_buffer );

  // Ensure there is enough space between the end and the start
  uint32_t available_space =
    (command_history_first == 0xFFFFFFFF)
      ? sizeof(command_history_buffer)
      : (sizeof(command_history_buffer) + command_history_first - command_history_end) % sizeof(command_history_buffer);

  // Drop items from the start until there is enough available space
  while (available_space <= entry_length) {
    available_space += command_history_buffer[command_history_first] + sizeof(command_history_entry_t) + 1;
    command_history_first =
      (command_history_first + command_history_buffer[command_history_first] + sizeof(command_history_entry_t) + 1)
      % sizeof(command_history_buffer);
  }

  // Write the entry header. There end should always point to the first available byte
  command_history_last = command_history_end;
  if (command_history_first == 0xFFFFFFFF) {
    command_history_first = command_history_last;
  }
  command_history_buffer[command_history_end] = line_length;

  command_history_end = (command_history_end + 1) % sizeof(command_history_buffer);

  // First contiguous copy
  uint8_t copy_length = MIN((sizeof(command_history_buffer) - command_history_end), line_length);
  memcpy(&command_history_buffer[command_history_end], line, copy_length);
  command_history_end = (command_history_end + line_length) % sizeof(command_history_buffer);

  // Check if we need to copy more because we need to wrap around the end of the buffer
  if (copy_length != line_length) {
    memcpy(&command_history_buffer[0], line + copy_length, line_length - copy_length);
  }

  command_history_buffer[command_history_end] = line_length;
  command_history_end                         = (command_history_end + 1) % sizeof(command_history_buffer);
}

// Simple linear database search. This function could be replaced with a more complicated version
int command_processor_find_command(const char *command_string,
                                   uint32_t command_string_length,
                                   const command_processor_database_t *db,
                                   const command_database_entry_t **entry,
                                   uint32_t *starting_index)
{
  // Validate inputs
  if ((command_string == NULL) || (command_string_length == 0) || (starting_index == NULL))
    return -1001;

  for (uint32_t i = *starting_index; i < db->length; i++) {
    if (strncmp(command_string, db->entries[i].key, command_string_length) == 0) {
      *entry          = &(db->entries[i]);
      *starting_index = i;

      if (strlen(db->entries[i].key) == command_string_length) {
        return RSI_ERROR_NONE;
      } else {
        return -1003;
      }
    }
  }

  return -1001;
}

static inline int parse_arg(const command_processor_database_t *db,
                            command_argument_type_t type,
                            char *line,
                            uint32_t *arg_result)
{
  if (type == COMMAND_ARG_NONE) {
    *arg_result = 0;
    return RSI_ERROR_NONE;
  }

  if (type & COMMAND_ARG_ENUM) {
    uint8_t enum_index = type & COMMAND_ARG_ENUM_INDEX_MASK;
    *arg_result        = parse_enum_arg(line, db->argument_types[enum_index]);
    if (*arg_result < 0) {
      return -1002;
    }
    return RSI_ERROR_NONE;
  }

  // Otherwise parse standard arg types
  switch (type & COMMAND_ARG_ENUM_INDEX_MASK) {
    case COMMAND_ARG_UINT8:
    case COMMAND_ARG_UINT16:
    case COMMAND_ARG_UINT32:
      *arg_result = strtoul(line, 0, 10);
      break;

    case COMMAND_ARG_INT8:
    case COMMAND_ARG_INT16:
    case COMMAND_ARG_INT32:
      *arg_result = strtol(line, 0, 10);
      break;

    case COMMAND_ARG_STRING:
      *arg_result = (uint32_t)line;
      break;

    case COMMAND_ARG_IP_ADDRESS:
      *arg_result   = 0;
      uint8_t *temp = (uint8_t *)arg_result;
      char *lasts   = NULL;
      char *token   = strtok_r(line, ".", &lasts);
      for (uint8_t i = 4; i != 0; --i, token = strtok_r(NULL, ".", &lasts)) {
        if (token == NULL) {
          return -1002;
        }
        *temp++ = strtoul(token, 0, 10);
      }
      break;
  }
  return RSI_ERROR_NONE;
}

static inline uint8_t parse_enum_arg(const char *line, const char *const *options)
{
  uint8_t a;
  for (a = 0; options[a] != NULL; ++a) {
    if (strcmp(line, options[a]) == 0) {
      return a;
    }
  }
  return -1;
}
