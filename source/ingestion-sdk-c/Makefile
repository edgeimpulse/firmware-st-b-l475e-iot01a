NAME = ingestion-sdk-example

CC ?= gcc
CFLAGS ?= -Wall

MACROS += -DEI_SENSOR_AQ_STREAM=FILE
CFLAGS += -I. -Imbedtls/include -Imbedtls/crypto/include -IQCBOR/inc -IQCBOR/src -Iinc -Iinc/signing
LIB_CFILES += QCBOR/src/*.c mbedtls/library/*.c

all: build

.PHONY: build clean

build:
	$(CC) $(MACROS) $(CFLAGS) test/main.c $(LIB_CFILES) -o $(NAME)

clean:
	rm $(NAME)
