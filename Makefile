CC = gcc
CFLAGS = -O2 -Wall -Wextra -std=c11 -Iinclude
LDFLAGS = -lm
TARGET = bin/telemetry_parser

all: $(TARGET)

$(TARGET): src/telemetry_parser.c src/main_telemetry_demo.c include/telemetry_parser.h
	@mkdir -p bin
	$(CC) $(CFLAGS) src/telemetry_parser.c src/main_telemetry_demo.c -o $(TARGET) $(LDFLAGS)

clean:
	rm -rf bin/*