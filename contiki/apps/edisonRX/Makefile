DEFINES+=PROJECT_CONF_H=\"project-conf.h\"
CONTIKI_PROJECT = triumviMain

TARGETDIRS += ../../platform

all: $(CONTIKI_PROJECT)

UIP_CONF_IPV6=0

CONTIKI = ../../../../contiki
include $(CONTIKI)/Makefile.include

CFLAGS += -DCC2538_RF_CONF_CHANNEL=11
