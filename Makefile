PROJECT = gnss_m8l_i2c
 
CC      = gcc
OPTIONS = -Wall

#SRCS := $(wildcard *.c)
SRCS := gnss_m8l_i2c.c broadcast_socket.c

#ifndef DEBUG
#DEBUG=0
#endif 

.PHONY: clean

all: $(PROJECT)
#ifneq ($(DEBUG), 0)
#	@echo "DEBUG enabled"
#else
#	@echo "DEBUG disabled"
#endif

$(PROJECT): $(SRCS)
	@echo "build..."
	$(CC) $(SRCS) $(OPTIONS) -o  $(PROJECT)

broadcast:
	@echo "build..."
	$(CC) $(SRCS) $(OPTIONS) -D SEND_TO_BROADCAST -o  $(PROJECT)_broadcast

debug:
	@echo "build debug..."
	$(CC) $(SRCS) $(OPTIONS) -D DEBUG -o $(PROJECT)_debug
	
clean:
	rm -f $(PROJECT)
	rm -f $(PROJECT_DEBUG)
