rm=/bin/rm -f
CC=cc
AR=ar

SDK_FEATURES += linux

#VALID_BUSES := uart usb

VALID_BUSES := usb
#VALID_BUSES := uart

LINKER_FLAGS= -pthread -lrt

CFLAGS+= -D LINUX_PLATFORM -D RSI_UART_INTERFACE

INCLUDES += -I $(RSI_SDK_PATH)/platforms/linux/ \
            -I $(RSI_SDK_PATH)/platforms/linux/common/application

linux_SOURCES =  $(RSI_SDK_PATH)/platforms/linux/uart/rsi_uart.c \
                 $(RSI_SDK_PATH)/platforms/linux/common/application/rsi_nl_app.c \
                 $(RSI_SDK_PATH)/platforms/linux/common/application/rsi_linux_apis.c \
                 $(RSI_SDK_PATH)/platforms/linux/common/application/rsi_bootup_config.c \
                 $(RSI_SDK_PATH)/platforms/linux/common/application/rsi_linux_app_init.c \
                 $(RSI_SDK_PATH)/platforms/linux/hal/rsi_hal_mcu_interrupt.c \
                 $(RSI_SDK_PATH)/platforms/linux/hal/rsi_hal_mcu_ioports.c \
                 $(RSI_SDK_PATH)/platforms/linux/hal/rsi_hal_mcu_timer.c \
                 $(RSI_SDK_PATH)/platforms/linux/hal/rsi_hal_mcu_platform_init.c
                 
linux: all