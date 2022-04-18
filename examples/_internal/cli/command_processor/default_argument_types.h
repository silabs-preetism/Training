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

#include <stddef.h>

#ifdef __cplusplus
extern "C" {
#endif

/******************************************************
 *                     Macros
 ******************************************************/

#define COMMAND_PROCESSOR_TYPE(name) COMMAND_PROCESSOR_##name##_TYPE
//#define DEFINE_CUSTOM_COMMAND_ARGUMENT_TYPE(name, ...)      [COMMAND_PROCESSOR_##name##_TYPE] = (const arg_list_t)(const const_string_t[]){ __VA_ARGS__, NULL },

/******************************************************
 *                    Constants
 ******************************************************/

/******************************************************
 *                   Enumerations
 ******************************************************/

/* This enumeration can be dynamically generated from a BOB component LIST */
typedef enum {
  COMMAND_PROCESSOR_TYPE(BUS),
  COMMAND_PROCESSOR_TYPE(BUS_MODE),
  COMMAND_PROCESSOR_TYPE(PERIPHERAL),
  COMMAND_PROCESSOR_TYPE(ENDPOINT),
  COMMAND_PROCESSOR_TYPE(INTERFACE),

  COMMAND_PROCESSOR_TYPE(WF200_TEST_MODE),
  COMMAND_PROCESSOR_TYPE(WF200_CW_MODE),
  COMMAND_PROCESSOR_TYPE(WF200_HT_FORMAT),
  COMMAND_PROCESSOR_TYPE(WF200_TX_RATE),
  COMMAND_PROCESSOR_TYPE(WF200_COEX),
  COMMAND_PROCESSOR_TYPE(WF200_SECURITY),
  COMMAND_PROCESSOR_TYPE(WF200_POWER_MODE),
  COMMAND_PROCESSOR_TYPE(WF200_ANTENNA),
  COMMAND_PROCESSOR_TYPE(WF200_OPERATIONAL_POWER_MODE),
  COMMAND_PROCESSOR_TYPE(GPIO_PORT),
  COMMAND_PROCESSOR_TYPE(GPIO_MODE),

} command_processor_type_t;

/******************************************************
 *                 Type Definitions
 ******************************************************/

typedef const char *const_string_t;
typedef const const_string_t *arg_list_t;
typedef const arg_list_t *all_arg_list_t;

/******************************************************
 *                    Structures
 ******************************************************/

/******************************************************
 *                 Global Variables
 ******************************************************/

// clang-format off
#ifndef FUZZING
/* This array can be dynamically generated from a BOB component LIST */
static const arg_list_t command_argument_types[] = (const arg_list_t[])
{
    [COMMAND_PROCESSOR_BUS_TYPE]        = (const arg_list_t)(const const_string_t[]){ "uart", "spi", "i2c", NULL },
    [COMMAND_PROCESSOR_BUS_MODE_TYPE]   = (const arg_list_t)(const const_string_t[]){ "stream", "packet", "command", NULL },
    [COMMAND_PROCESSOR_PERIPHERAL_TYPE] = (const arg_list_t)(const const_string_t[]){ "adc", "dac", "i2c", "gpio", "spi", "uart", "rtc", "watchdog", NULL },
    [COMMAND_PROCESSOR_ENDPOINT_TYPE]   = (const arg_list_t)(const const_string_t[]){ "uart", "usb", "spi", "tcp","udp","mqtt","homekit","bluemix", "logfile", NULL },
    [COMMAND_PROCESSOR_INTERFACE_TYPE]  = (const arg_list_t)(const const_string_t[]){ "wlan", "softap", "ethernet", "thread", "bluetooth", "zwave", NULL },

    // WF200 argument types
    [COMMAND_PROCESSOR_TYPE( WF200_TEST_MODE )] = (const arg_list_t)(const const_string_t[]){ "tx_cw", "tx_packet", "rx", NULL  },
    [COMMAND_PROCESSOR_TYPE( WF200_CW_MODE   )] = (const arg_list_t)(const const_string_t[]){ "single", "dual", NULL  },
    [COMMAND_PROCESSOR_TYPE( WF200_HT_FORMAT )] = (const arg_list_t)(const const_string_t[]){ "mm", "gf", NULL  },
    [COMMAND_PROCESSOR_TYPE( WF200_TX_RATE   )] = (const arg_list_t)(const const_string_t[]){ "B_1M", "B_2M", "B_5.5M", "B_11M", "G_6M", "G_9M", "G_12M", "G_18M", "G_24M", "G_36M", "G_48M", "G_54M", "N_MCS0", "N_MCS1", "N_MCS2", "N_MCS3", "N_MCS4", "N_MCS5", "N_MCS6", "N_MCS7", NULL  },
    [COMMAND_PROCESSOR_TYPE( WF200_COEX      )] = (const arg_list_t)(const const_string_t[]){ "wlan", "efr",  "auto", NULL  },
    [COMMAND_PROCESSOR_TYPE( WF200_SECURITY  )] = (const arg_list_t)(const const_string_t[]){ "open", "wep", "wpa-mixed", "wpa-mixed-eap", "wpa2-psk", NULL  }, // Notes that the order of these names MUST match the order in wfm_security_mode
    [COMMAND_PROCESSOR_TYPE( WF200_POWER_MODE )] = (const arg_list_t)(const const_string_t[]){ "active", "ps", "auto", NULL  },
    [COMMAND_PROCESSOR_TYPE( WF200_ANTENNA )] = (const arg_list_t)(const const_string_t[]){ "TX1_RX1", "TX2_RX2", "TX2_RX1", "TX1_RX2", "DIVERSITY", NULL  }, 
    [COMMAND_PROCESSOR_TYPE( WF200_OPERATIONAL_POWER_MODE )] = (const arg_list_t)(const const_string_t[]){ "active", "doze", "quiescent", NULL  },
    [COMMAND_PROCESSOR_TYPE( GPIO_PORT )] = (const arg_list_t)(const const_string_t[]){ "a", "b", "c", "d", "e", "f", "g", "h", NULL  },
    [COMMAND_PROCESSOR_TYPE( GPIO_MODE )] = (const arg_list_t)(const const_string_t[]){ "disabled", "in", "in_pull", "in_pull_filter", "out", NULL  },
};
// clang-format on
#else
static const const_string_t bus_types[4]          = { "uart", "spi", "i2c", NULL };
static const arg_list_t command_argument_types[1] = { bus_types };
#endif

/******************************************************
 *               Function Declarations
 ******************************************************/

#ifdef __cplusplus
} /*extern "C" */
#endif
