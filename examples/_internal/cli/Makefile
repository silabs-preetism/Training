# Make File
PROGNAME=cli
RSI_SDK_PATH = ../../../

# Includes
CFLAGS += -I./command_processor 
# Defines
CFLAGS += 
# Libraries
LIBS += 
# Sources
APPLICATION_SOURCES = cli.c command_table.c wlan_api_commands.c utilities.c command_processor/command_processor.c command_processor/command_minimal_uart_plugin.c commands/wlan_connected_sleep.c commands/wlan_udp_client.c commands/wlan_access_point.c 

# SDK features
SDK_FEATURES = wlan sockets freertos firmware_update http_server 

include $(RSI_SDK_PATH)/sapi/sapi.mk
