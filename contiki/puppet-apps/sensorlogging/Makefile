CONTIKI=../../

WITH_UIP=1
UIP_CONF_IPV6=1
WITH_COAP=1

PLATFORM_HAS_SHT11=1
CFLAGS += -DPROJECT_CONF_H=\"project-conf.h\"
CFLAGS += -DWITH_COAP

APPS += rest-coap
APPS += pcommon

include $(CONTIKI)/Makefile.include
