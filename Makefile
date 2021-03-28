PROJECT = gnss_m8l_i2c
PROJECT_DEBUG =gnss_m8l_i2c_debug
 
CC      = gcc
OPTIONS = -Wall

#SRCS := $(wildcard *.c)
SRCS := gnss_m8l_i2c.c

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

debug:
	@echo "build debug..."
	$(CC) $(SRCS) $(OPTIONS) -D DEBUG -o $(PROJECT_DEBUG)
	
clean:
	rm -f $(PROJECT)
	rm -f $(PROJECT_DEBUG)
