.PHONY: build flash monitor all

build:
	idf.py build

flash:
	idf.py -p /dev/ttyUSB0 flash
	#idf.py -p /dev/ttyACM0 flash

monitor:
	idf.py monitor

all: build flash monitor

# Default target when just running 'make'
.DEFAULT_GOAL := all
