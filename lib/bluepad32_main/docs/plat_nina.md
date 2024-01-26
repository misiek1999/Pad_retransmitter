# Bluepad32 firmware for NINA

## What is NINA

NINA is a family of [ESP32 modules][nina-esp32].
These modules are present on some Arduino boards like:

- [Arduino Nano RP2040 Connect][nano_rp2040]
- [Arduino Nano 33 IoT][nano_33_iot]
- [Arduino MKR WiFi 1010][mkr_wifi]
- [Arduino UNO WiFi Rev.2][uni_wifi]
- [Arduino Arduino MKR Vidor 4000][mkr_vidor_4000]

NINA modules are co-processors, usually used only to bring WiFi or BLE to the main processor.

In order to have gamepad support, the original NINA firmware must be replaced
with Bluepad32 firmware. This is a simple step that needs to be done just once,
and can be "undone" at any time.

![how-does-it-work](images/bluepad32-nina-how-does-it-work.png)

This is how it works:

- Gamepad (A) talks to NINA module (B)
- NINA module (B) talks to main processor (C)

Bluepad32 firmware is "compatible-enough" with the original firmware:

- Uses SPI, and the same GPIOs to talk to the main processor
- Uses the same protocol that runs on top of SPI
- But not all messages are implemented. Only the ones that are needed
  to have gamepad support working.

[nina-esp32]: https://www.u-blox.com/en/product/nina-w10-series-open-cpu
[nina-fw]: https://github.com/arduino/nina-fw
[nano_rp2040]: https://store-usa.arduino.cc/products/arduino-nano-rp2040-connect-with-headers
[nano_33_iot]: https://store-usa.arduino.cc/products/arduino-nano-33-iot
[mkr_wifi]: https://store-usa.arduino.cc/products/arduino-mkr-wifi-1010
[uni_wifi]: https://store-usa.arduino.cc/products/arduino-uno-wifi-rev2
[mkr_vidor_4000]: https://store.arduino.cc/products/arduino-mkr-vidor-4000

## Flashing pre-compiled Bluepad32 firmware

To flash Bluepad32 firmware, you have to:

### Download arduino-fwuploader

Download latest binary from here: https://github.com/arduino/arduino-fwuploader/releases

### Select correct board name

* `arduino:samd:mkrwifi1010` for Arduino MKR WiFi 1010
* `arduino:samd:nano_33_iot` for Arduino NANO 33 IoT
* `arduino:samd:mkrvidor4000` for Arduino MKR Vidor 4000
* `arduino:megaavr:uno2018` for Arduino Uno WiFi Rev2
* `arduino:mbed_nano:nanorp2040connect` for Arduino Nano RP2040 Connect

You can see all boards names by doing:
```shell
$ arduino-fwuploader firmware list
```

### Flash it

```shell
# Replace name and address with the correct ones
export BOARD=arduino:samd:nano_33_iot
export ADDRESS=/dev/ttyACM0
$ arduino-fwuploader firmware flash -b $BOARD -a $ADDRESS -i bluepad32-nina-full.bin
```

### Verify

To verify that the flash was successful, do:

```shell
$ arduino-fwuploader firmware get-version -b $BOARD -a $ADDRESS
```

And you should see:

```
...

Firmware version installed: Bluepad32 for NINA v3.6.0-rc0
```

## Flashing self-compiled Bluepad32 firmware

To flash a self-compiled firmware, you should do:

1. Put the Arduino board in "pass-through" mode
2. Compile it yourself and flash it.

### 1. Put Arduino board in "passthrough" mode

Before flash Bluepad32 firmware, you have to put the Arduino board in "pass-through" mode:

1. Open Arduino IDE
2. Install the WiFiNINA library (just do it once)
3. And finally open the `SerialNINAPassthrough` sketch:

- File -> Examples -> WiFiNINA -> Tools -> SerialNINAPassthrough

Compile it and flash it to the Arduino board.

### 2. Compile it yourself and flash it

Install the requirements described here: [README.md][readme].

Chose `nina` as the target platform:

```sh
cd ${BLUEPAD32}/src

# Select Nina platform:
# Components config -> Bluepad32 -> Target Platform -> Nina
idf.py menuconfig

# And then compile it!
idf.py build
```

On Nano 32 IoT / MKR WIFI 1010, doing `idf.py flash` will just work.

```sh
# Only valid for:
#   * Nano 33 IoT
#   * MKR WIFI 1010

# Port might be different
export ESPPORT=/dev/ttyACM0

idf.py flash
```

But on NANO RP2040 Connect and UNO WiFi Rev.2, you have to flash it using the `--before no_reset` option,
and **NOT** `--before default_reset`. E.g:

```sh
# Only valid for:
#   * Nano RP2040 Connect
#   * UNO WiFi Rev.2

# Port might be different
export ESPPORT=/dev/ttyACM0

esptool.py --port ${ESPPORT} --baud 115200 --before no_reset write_flash 0x1000 ./build/bootloader/bootloader.bin 0x10000 ./build/bluepad32-airlift.bin 0x8000 ./build/partitions_singleapp.bin
```

[readme]: https://gitlab.com/ricardoquesada/bluepad32/-/blob/master/README.md

## Example

The Bluepad32 library for Arduino with examples is available here:

- http://gitlab.com/ricardoquesada/bluepad32-arduino
