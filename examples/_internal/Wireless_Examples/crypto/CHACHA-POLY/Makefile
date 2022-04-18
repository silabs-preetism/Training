# Makefile
PROGNAME= rsi_wc_app
RSI_SDK_PATH = ../../../
include $(RSI_SDK_PATH)/platforms/linux/linux_build.mk

LINKS= -pthread  -lrps
PATH_CONFIG=$(shell pwd)
RSI_DRIVER_PATH = $(RSI_SDK_PATH)/sapi/include
RSI_APP_PATH = ../../../crypto/

CFLAGS=  -I$(RSI_DRIVER_PATH)/
CFLAGS+= -I$(RSI_APP_PATH)/
CFLAGS+=  -I$(INCLUDE_PATH)/
CFLAGS+= -D RSI_SAMPLE_HAL
CFLAGS+= -D RSI_ENABLE_DEBUG_PRINT #Enable to get prints in LINUX
CHACHAPOLY_APP = rsi_chacha_poly_app.o  

all: $(CHACHAPOLY_APP) 

	make crypto CC=$(CC) -C ../../../build/ path_config=$(PATH_CONFIG) 
	$(CC) $(CFLAGS) *.o $(LINKS) -o $(PROGNAME) 
	rm -f  *.o  

clean:
	$(rm) $(CHACHAPOLY_APP) core *.o *~ rsi_wc_app 