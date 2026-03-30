# ESP32-P4 WiFi RTL-SDR SIMD Project

## Build Environment

Before running any `idf.py` or `cmake` build commands, always source the ESP-IDF environment first:

```bash
source /home/rax/esp/v5.5.1/esp-idf/export.sh
```

This sets `IDF_PATH`, toolchain paths, and Python environment needed for ESP-IDF builds.

## Build Commands

```bash
idf.py build          # build the project
idf.py flash monitor  # flash and open serial monitor
idf.py menuconfig     # configure project options
```

## Target

This project targets ESP32-P4 (`CONFIG_IDF_TARGET="esp32p4"`).
