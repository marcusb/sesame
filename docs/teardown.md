# Hardware teardown

The opener has two circuit boards:

* The main board, part [40976R.S]
* The Integrated DCM Circuit Board (iDCM), part [39732R.S]

The main board contains the power supply, motor control, radio interface
for the wireless remotes, and wall console connector. It has a PIC16
microcontroller. Sesame does not touch this board at all.

The iDCM board is responsible for the WiFi interface. It also sports a PIC16
MCU, as well as a Marvell MW300 WiFi module with a single-core ARM Cortex-M4F
CPU. This is where the Sesame software runs.

## iDCM board components

| Component | Description |
| --------- | ----------- |
| J3  | Cortex UART (for TagConnect [TC2030-PKT] cable)|
| J7  | Cortex Debug JTAG/SWD (for TagConnect [TC2050-IDC-050] cable)|
| J8  | ICSP interface or the PIC16 |
| U6  | Marvell MW 300 WiFi module |
| U10 | PIC16LF15355 microcontroller |

There are also two LEDs, two push buttons and a buzzer.

The MW300 module handles all the WiFi networking, whereas the PIC16
controls the LEDs, buzzer and door control (via a connection to the main
board's MCU). The PIC16 and the MW300 are connected via serial port
(UART1 on the MW300) with a full-duplex message protocol.
This protocol also has a facility to update the firmware of the PIC16.

## The MW300 module

This module is likely identical to the [NXP 88MW320]. 
It sports 512 kB SRAM, 4 MB flash (XIP capable), and 802.11b/g/n WiFi interface.
A datasheet can be obtained from NXP, as well as a [dev board].

The board is supported by an outdated
[AWS IoT Starter Kit](https://docs.aws.amazon.com/freertos/latest/userguide/getting_started_mw32x.html),
as well as a newer [SDK from NXP](https://github.com/nxptest/mw320_sdk/).
Sesame utilizes this newer SDK.

It is possible to program the flash of the MW300 via the JTAG/SWD
interface.

[40976R.S]: https://store.geniecompany.com/products/circuit-board-40976r-s
[39732R.S]: https://store.geniecompany.com/products/integrated-dcm-circuit-board-39732r-s
[TC2030-PKT]: https://www.tag-connect.com/product/tc2030-pkt-6-pin-cable-with-legs-for-use-with-microchip-pickit-3
[TC2050-IDC-050]: https://www.tag-connect.com/product/tc2050-idc-050
[NXP 88MW320]: https://www.nxp.com/products/88MW30X
[dev board]: https://www.nxp.com/part/RDMW320-R0
