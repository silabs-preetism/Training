/*
 * ZentriOS SDK LICENSE AGREEMENT | Zentri.com, 2016.
 *
 * Use of source code and/or libraries contained in the ZentriOS SDK is
 * subject to the Zentri Operating System SDK license agreement and
 * applicable open source license agreements.
 *
 */

/**
 * @file
 *
 * File description
 */

#pragma once

#include "default_argument_types.h"
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/******************************************************
 *                     Macros
 ******************************************************/

#define COMMAND_ENUM_ARG(type)                (COMMAND_ARG_ENUM | COMMAND_PROCESSOR_##type##_TYPE)
#define COMMAND_OPTIONAL_ARG(character, type) (COMMAND_ARG_OPTIONAL | ((character)&0x7F)), type
#define COMMAND_SUB_COMMAND_ARG(command_table)                                           \
  COMMAND_ARG_SUB_COMMAND, ((command_table >> 0) & 0xFF), ((command_table >> 8) & 0xFF), \
    ((command_table >> 16) & 0xFF), ((command_table >> 24) & 0xFF)

#define COMMAND_DATABASE_ENTRIES(...)                                                                \
  .length  = sizeof((command_database_entry_t[]){ __VA_ARGS__ }) / sizeof(command_database_entry_t), \
  .entries = { __VA_ARGS__ }

#define COMMAND_VARIABLE(name_string, var, ...)                   \
  {                                                               \
    .name = name_string, .variable = var, .type = { __VA_ARGS__ } \
  }
#define COMMAND_VARIABLE_ENUM(name_string, var, type)                                                     \
  {                                                                                                       \
    .name = name_string, .variable = var, .type = {(COMMAND_ARG_ENUM | COMMAND_PROCESSOR_##type##_TYPE) } \
  }

#define IS_COMMAND_ARG_VALID(arg_number) ((arguments->bitmap & (1 << arg_number)) != 0)

#define COMMAND_AUTO_TABLE_ENTRY(name) { STRINGIFY(name), &name##_command },
#define COMMAND_AUTO_FUNCTION_DEFINITION(name)                   \
  extern const command_processor_basic_command_t name##_command; \
  int name##_command_handler(command_processor_args_t *arguments)

/******************************************************
 *                    Constants
 ******************************************************/

#ifndef COMMAND_PROCESSOR_MAXIMUM_ARG_COUNT
#define COMMAND_PROCESSOR_MAXIMUM_ARG_COUNT (10)
#endif

#define COMMAND_ARG_OPTIONAL_CHARACTER_MASK 0x7F
#define COMMAND_ARG_ENUM_INDEX_MASK         0x3F

/******************************************************
 *                   Enumerations
 ******************************************************/

/*
 * Optional arguments use the lower 7 bits to record the character and are then followed by a non-optional argument type
 * e.g.
 */
typedef enum {
  COMMAND_ARG_NONE, // Used for optional arguments that are just flags and don't need any further data
  COMMAND_ARG_UINT8,
  COMMAND_ARG_UINT16,
  COMMAND_ARG_UINT32,
  COMMAND_ARG_INT8,
  COMMAND_ARG_INT16,
  COMMAND_ARG_INT32,
  COMMAND_ARG_STRING,
  COMMAND_ARG_IP_ADDRESS,
  COMMAND_ARG_MAC_ADDRESS,
  COMMAND_ARG_SUB_COMMAND,
  COMMAND_ARG_REMAINING_COMMAND_LINE,
  COMMAND_NUMBER_OF_ARGS_WITH_STRINGS, // This is the number of argument types that match to a specific string. Other arg types should follow

  COMMAND_ARG_ENUM     = (1 << 6),
  COMMAND_ARG_OPTIONAL = (1 << 7),
  COMMAND_ARG_END      = 0xFF
} command_argument_type_t;

typedef enum {
  COMMAND_VARIABLE_UINT8,
  COMMAND_VARIABLE_UINT16,
  COMMAND_VARIABLE_UINT32,
  COMMAND_VARIABLE_INT8,
  COMMAND_VARIABLE_INT16,
  COMMAND_VARIABLE_INT32,
  COMMAND_VARIABLE_STRING,
  COMMAND_VARIABLE_MAC,
  COMMAND_VARIABLE_IPV4_ADDRESS,
  COMMAND_VARIABLE_IPV6_ADDRESS,
  COMMAND_VARIABLE_ARRAY,
  COMMAND_VARIABLE_STRUCTURE, // Assumed to be packed
  COMMAND_VARIABLE_UNPACKED_STRUCTURE,

  COMMAND_VARIABLE_GROUP_NODE,
  COMMAND_VARIABLE_ENUM = (1 << 7),
} command_variable_type_t;

/******************************************************
 *                 Type Definitions
 ******************************************************/

typedef const char *const_string_t;
typedef const const_string_t *arg_list_t;

#ifdef USE_COMMAND_VARIABLES
typedef void (*command_variable_modification_handler_t)(command_processor_variable_t *variable);
#endif

/******************************************************
 *                    Structures
 ******************************************************/

typedef struct {
  uint32_t bitmap;
  uint32_t arg[COMMAND_PROCESSOR_MAXIMUM_ARG_COUNT];
} command_processor_args_t;

typedef int (*command_processor_handler_t)(command_processor_args_t *arguments);

typedef struct {
  command_processor_handler_t handler;
  const char *description;
  const char *argument_help;
  command_argument_type_t argument_list[];
} command_processor_descriptive_command_t;

typedef struct {
  uint8_t type;
} command_argument_t;

typedef struct {
  const char *key;
  const void *value;
} command_database_entry_t;

typedef struct {
  const arg_list_t *argument_types;
  uint32_t length;
  command_database_entry_t entries[];
} command_processor_database_t;

//#ifdef USE_COMMAND_VARIABLES
typedef struct {
  const char *name;
  void *variable;
  command_variable_type_t type[];
} command_processor_variable_t;

/******************************************************
 *                 Global Variables
 ******************************************************/

extern const char *const command_argument_type_strings[COMMAND_NUMBER_OF_ARGS_WITH_STRINGS];

/******************************************************
 *               Function Declarations
 ******************************************************/

int command_processor_parse_command(char *command_line,
                                    const command_processor_database_t *db,
                                    command_processor_args_t *args,
                                    const command_processor_descriptive_command_t **output_command);
int command_processor_find_command(const char *command_string,
                                   uint32_t command_string_length,
                                   const command_processor_database_t *db,
                                   const command_database_entry_t **entry,
                                   uint32_t *starting_index);
void command_processor_add_to_history(const char *line, uint8_t line_length);

#ifdef __cplusplus
} /*extern "C" */
#endif
