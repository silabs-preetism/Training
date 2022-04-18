# Make File
PROGNAME=azure_iot
RSI_SDK_PATH = ../../../../../

# Includes
CFLAGS += 
# Defines
CFLAGS += -DNO_LOGGING -DAZURE_ENABLE -DSTRINGS_C_SPRINTF_BUFFER_SIZE=512 
# Libraries
LIBS += -lm 
# Sources
APPLICATION_SOURCES = rsi_azure_telemetry_c2d.c 

# SDK features
SDK_FEATURES = wlan sockets certificates azure 

include $(RSI_SDK_PATH)/sapi/sapi.mk
