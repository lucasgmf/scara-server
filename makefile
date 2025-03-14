BUILD_DIR = rp2040src/build

all: build

build:
	mkdir -p $(BUILD_DIR)
	cd $(BUILD_DIR) && cmake ..
	cd $(BUILD_DIR) && make -j$(nproc)

clean:
	rm -rf $(BUILD_DIR)

