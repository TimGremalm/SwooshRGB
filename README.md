# SwooshRGB
PWM RGB led strip over sACN (E1.31, DMX512) esp-idf

# Setup
* Configure esp-idf environment (ESP-IDF v5.0-dev-4001-g495d35949d 2nd stage bootloader)
* https://docs.espressif.com/projects/esp-idf/en/latest/esp32/get-started/index.html

## Rehash build environment
```bash
get_idf
```

# Build and flash

## Configure
```bash
idf.py menuconfig
```

## Build
```bash
idf.py build
```

## Flash
```bash
idf.py -p /dev/ttyUSB0 flash
```

# Serial Monitor
## Monitor
```bash
idf.py monitor
```

## Close monitor
Press <kbd>CTRL</kbd>+<kbd>]</kbd> to exit.

## Build and flash within monitor
Press <kbd>CTRL</kbd>+<kbd>T</kbd> then <kbd>A</kbd> to build and flash.

