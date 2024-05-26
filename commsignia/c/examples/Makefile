# Makefile for Commsignia V2X Remote C SDK examples

ifndef CMS_V2X_SDK_DIR
	CMS_V2X_SDK_DIR=..
endif

EXAMPLES=$(wildcard $(CMS_V2X_SDK_DIR)/examples/*.c)
ASN_EXAMPLES=$(wildcard $(CMS_V2X_SDK_DIR)/examples/asn/*.c)

TARGETS=$(patsubst %.c,%,$(EXAMPLES)) $(patsubst %.c,%,$(ASN_EXAMPLES))

all: $(TARGETS)

CFLAGS  += -I$(CMS_V2X_SDK_DIR)/include
CFLAGS  += -I$(CMS_V2X_SDK_DIR)/include/asn1
LDFLAGS += -L$(CMS_V2X_SDK_DIR)/lib
LDFLAGS += -Wl,-rpath=$(CMS_V2X_SDK_DIR)/lib
LDLIBS  += -lits-rem -lits-asnsdk -lm -lrt

clean:
	rm -f $(TARGETS)
.PHONY:clean
