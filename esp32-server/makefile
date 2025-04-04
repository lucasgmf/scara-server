IDF = idf.py
PORT = /dev/ttyACM0

.PHONY: build flash buildflash

build:
	$(IDF) build

flash:
	$(IDF) -p $(PORT) flash

buildflash: build flash

