# Sesame

Sesame is an after-market firmware for the Genie 1155 garage door opener.

## Features

* MQTT support
* HTTP server for configuration and control
* Over-the-air updates

## Installation

### Pre-requisites

Software:

* OpenOCD 0.12 or later
* protoc, the Protobuf compiler

Hardware:

* [TC2050-IDC-050] 10-pin cable to connect to the Cortex debug interface
* JTAG board with a 10-pin Cortex debug header, such as the [Tigard]

Optional but useful:

* [TC2030-PKT] 6-pin cable for serial console
* USB-to-TTL serial port adapter (you can also use the Tigard for this)

### First-time install

To install Sesame, you will use the Cortex debugger and OpenOCD
to write the firmware image to the flash memory of the MW300 module.

1. Disconnect the header cable and remove the iDCM board from the unit.
1. Connect the push-pin connector of the 10-pin cable to header J7 on the iDCM board,
   and the other end to the Cortex debug interface on the Tigard.
1. (optional) Connect the 6-pin UART cable to header J3 on the iDCM board, and the other end
   to a USB-to-TTL serial converter (3.3V only). NOTE: The serial converter must supply 3.3V only.
1. Configure the Tigard for JTAG mode and 3.3V.
1. Connect Tigard to your computer.
1. (optional) If you connected the UART interface, you can now look at logs from the device
   in your terminal of choice, eg `minicom -D /dev/ttyUSB0 -b 115200`.
   At this point you should see the console output from the factory firmware.

Future firmware upgrades can be done over the air, so this procedure is only required the first time.

## Setup

echo 'syslog_config: { enabled: true, syslog_host: "nuc.example.org", syslog_port: 514 }' | protoc --encode=LoggingConfig proto/app_config.proto | curl --data-binary @- -H content-type:application/protobuf -v 'http://sesame2/cfg/logging'


echo 'enabled: true, broker_host: "mqtt.example.org", broker_port: 1883, client_id: "sesame-dev", username: "sesame", password: "***REMOVED***", prefix: "sesame-dev"' | protoc --encode=MqttConfig proto/app_config.proto | curl --data-binary @- -H content-type:application/protobuf -v 'http://sesame2/cfg/mqtt

echo 'hostname: "sesame", ssid: "MY_WIFI", security: 2, password: "seekrit"' | protoc --encode=NetworkConfig proto/app_config.proto | curl --data-binary @- -H content-type:application/protobuf -v 'http://192.168.4.1/cfg/network'

echo 'url: "http://example.org/sesame.bin"'  | protoc --encode=FirmwareUpgradeFetchRequest proto/api.proto | curl --data-binary @- -H content-type:application/protobuf -v 'http://sesame2/fwupgrade'


## Development

The firmware can be built on Linux.

### Prerequisites

* CMake
* Ninja build system
* A cross-compiler for ARM.

On Debian:
```sh
apt install cmake ninja gcc-arm-none-eabi libnewlib-arm-none-eabi binutils-arm-none-eabi
```

The cross-compiler is specified in the `toolchain.cmake` file.

### Building

Two builds are required: A native build to compile the axf2firmware tool,
and a cross-compile of the firmware.

```sh
mkdir build-native
cd build-native
cmake -G Ninja ..
cd ..
mkdir build
cd build
cmake --no-warn-unused-cli -DCMAKE_BUILD_TYPE:STRING=Debug -DUSE_BACKTRACE=ON \
  -DCMAKE_EXPORT_COMPILE_COMMANDS:BOOL=TRUE -DCMAKE_TOOLCHAIN_FILE=../toolchain.cmake \
  -G Ninja -Daxf2firmware_DIR=~/src/sesame/build-native ..
```

[TC2030-PKT]: https://www.tag-connect.com/product/tc2030-pkt-6-pin-cable-with-legs-for-use-with-microchip-pickit-3
[TC2050-IDC-050]: https://www.tag-connect.com/product/tc2050-idc-050
[Tigard]: https://github.com/tigard-tools/tigard
