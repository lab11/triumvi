DEFINES+=PROJECT_CONF_H=\"project-conf.h\"
CONTIKI_PROJECT = triumviV9

TARGETDIRS += ../../platform

all: $(CONTIKI_PROJECT)

UIP_CONF_IPV6=0

CONTIKI = ../../../../contiki
include $(CONTIKI)/Makefile.include

#ifneq ($(MAKECMDGOALS),clean)
#ifndef calFile
#$(warning No calibration file, try make calFile=calibrate/V8node12SineTable.txt)
#$(warning For me details: https://github.com/lab11/G2/issues/3)
#$(error Must define a calibration file)
#endif
#endif

CFLAGS += -DCC2538_RF_CONF_CHANNEL=11
CFLAGS += -DTHREEPHASE_ID=$(threephaseID)
#CFLAGS += -DCALIBRATE_FILE_NAME=\"$(calFile)\"
# Syntax is following
# make calFile=calibrate/V8node12SineTable.txt threephaseID=1
