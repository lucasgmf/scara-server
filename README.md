# 4-DOF Educational SCARA Manipulator

An affordable, open-source robotic manipulator designed for educational environments, featuring four degrees of freedom and an integrated gripper system. Master's thesis in Electrical and Computer Engineering at the [Faculty of Engineering, University of Porto](https://www.up.pt/feup/en/).

## Dependencies

This project requires the following dependencies:

- [Python](https://www.python.org/)
- [ESP-IDF (Espressif IoT Development Framework)](https://docs.espressif.com/projects/esp-idf/en/stable/esp32/get-started/index.html)

If you are using the provided [Nix flake](./flake.nix), most of the dependencies are already handled.  
Make sure to install the ESP-IDF toolchain and source it before building:

```bash
. ~/esp-idf/export.sh
```

## Build & Flash (Makefile)

This project includes a `Makefile` that simplifies the workflow with the ESP-IDF toolchain.  
Instead of remembering multiple commands, you can just run `make` and it will take care of everything.

### Targets

- **`build`**  
  - Converts the web page `components/network/page.html` into a C header file (`page_html.h`) using `xxd`.  
    This allows embedding HTML directly into the firmware.  
  - Runs `idf.py build` to compile the project with ESP-IDF.

- **`flash`**  
  - Uploads the compiled firmware to the ESP32 via the specified serial port:  
    ```bash
    idf.py -p /dev/ttyUSB0 flash
    ```
  - If your board shows up differently, you can change the port (e.g. `/dev/ttyACM0`).

- **`monitor`**  
  - Starts the ESP-IDF serial monitor:  
    ```bash
    idf.py monitor
    ```
  - Useful for reading logs and debugging output from the ESP32.

- **`all`**  
  - Runs `build → flash → monitor` in sequence.  
  - This is also the **default target**, meaning just running:
    ```bash
    make
    ```
    will compile, flash, and open the serial monitor in one step.

### Example workflow

```bash
# Build the firmware only
make build

# Flash the firmware to ESP32
make flash

# Open the ESP-IDF monitor
make monitor

# Do everything: build, flash, and monitor
make
```

## Internet Connectivity

To enable Wi-Fi connectivity on the ESP32, you need to provide your own network credentials.

1. Open the template file located at:
```bash
components/network/credentials_TEMPLATE.h
```

2. Edit the file and replace the placeholders with your Wi-Fi details:
```c
   #ifndef CREDENTIALS_H
   #define CREDENTIALS_H

   #define WIFI_USERNAME "upxxxxxxxxx@up.pt"
   #define WIFI_PASSWORD "password"

   #endif
```
Where:
   - WIFI_USERNAME → your Wi-Fi SSID (e.g., the network name or enterprise username)
   - WIFI_PASSWORD → your Wi-Fi password

3. Save the file and rename it from:
   credentials_TEMPLATE.h
   to:
   credentials.h

Do not commit your personal credentials to GitHub ;).
