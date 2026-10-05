# Hardware teardown

The opener has two circuit boards:

- The main board, part [40976R.S]
- The Integrated DCM Circuit Board (iDCM), part [39732R.S]

The main board contains the power supply, motor control, radio interface
for the wireless remotes, and wall console connector. It has a PIC16
microcontroller. Sesame does not touch this board at all.

The iDCM board is responsible for the WiFi interface. It also sports a PIC16
MCU, as well as a Marvell MW300 WiFi module with a single-core ARM Cortex-M4F
CPU. This is where the Sesame software runs.

## iDCM board components

| Component | Description                      |
| --------- | -------------------------------- |
| J2        | Factory test / diagnostic header |
| J3        | Cortex UART                      |
| J7        | Cortex Debug JTAG/SWD            |
| J8        | ICSP interface or the PIC16      |
| U6        | Marvell MW 300 WiFi module       |
| U10       | PIC16LF15355 microcontroller     |

J2, J3, and J8 are compatible with the TagConnect [TC2030-PKT] cable.
J7 is compatible with the TagConnect [TC2050-IDC-050] cable.

There are also two LEDs, two push buttons and a buzzer.

The MW300 module handles all the WiFi networking, whereas the PIC16
controls the LEDs, buzzer and door control (via a connection to the main
board's MCU). The PIC16 and the MW300 are connected via serial port
(UART1 on the MW300 to UART1 on the PIC16 @ RC6/RC7) with a full-duplex message protocol.
This protocol also has a facility to update the firmware of the PIC16.

### U10 PIC16LF15355 Pinout & Peripheral Assignment

| Pin | Name     | Dir | Function / Description |
| --- | -------- | --- | ---------------------- |
| RA0 | AN0      | IN  | Analog sensor / voltage reference input |
| RA1 | LATA1    | OUT | Door trigger pulse: ~10 ms active-high pulse simulating wall console button |
| RA2 | LATA2    | OUT | Board status / debug indicator |
| RA3 | LATA3    | OUT | Board control output |
| RA4 | RA4      | IN  | Sense input line |
| RA5 | LATA5    | OUT | **Alert light pulse line to Main Board**: toggled at 1.0 Hz (500 ms ON / 500 ms OFF) |
| RA6 | RA6PPS   | OUT | PWM6 output (PPS 0x0D) |
| RA7 | LATA7    | OUT | Piezo buzzer output (1 = sound ON, 0 = OFF) |
| RB0 | CLCIN2   | IN  | Motor optical encoder Phase A (PPS 0x08, CLC2 and IOC interrupt) |
| RB1 | CLCIN3   | IN  | Motor optical encoder Phase B (PPS 0x09, CLC1 and IOC interrupt) |
| RB2 | LATB2    | OUT | Status / control output |
| RB3 | RB3PPS   | OUT | **UART2 TX** (PPS 0x11) to Main Board PIC (115200 baud, 8-N-1) |
| RB4 | RX2DTPPS | IN  | **UART2 RX** (PPS 0x0C) from Main Board PIC (115200 baud, 8-N-1) |
| RB6 | ICSPCLK  | IN  | ICSP clock (Header J8 pin 5) |
| RB7 | ICSPDAT  | I/O | ICSP data (Header J8 pin 4) |
| RC2 | RC2      | OUT | Test point (Header J8 pin 6) |
| RC5 | RC5      | OUT | Test header line (Header J2 pin 6) |
| RC6 | RC6PPS   | OUT | **UART1 TX** (PPS 0x0F) to MW300 UART1 RX (115200 baud, 8-N-1) |
| RC7 | RX1DTPPS | IN  | **UART1 RX** (PPS 0x17) from MW300 UART1 TX (115200 baud, 8-N-1) |

### PIC-to-PIC Inter-board Signaling & Alert Light Protocol

Communication between the iDCM PIC and the main board PIC uses both dedicated hardware lines and an asynchronous serial protocol:

1. **Alert Light Flashing (1 Hz Square Wave)**:
   - When the MW300 sends `DCM_MSG_ALERT_CMD` (`0x11`) with duration `N` seconds, the iDCM PIC configures the alert state machine in RAM.
   - The PIC's main superloop runs on a 50 ms tick (driven by a 1 ms Timer 1 ISR).
   - In each tick, the timer counts down: it pulls **RA5 (`LATA5`)** HIGH for 10 ticks (500 ms), then pulls **RA5** LOW for 10 ticks (500 ms).
   - This creates a **1.0 Hz square wave** (50% duty cycle) on RA5, which connects directly to the main board to pulse the overhead opener light.
   - Simultaneous audio alert tone is driven on **RA7** (buzzer output).
   - Once the duration expires, RA5 is cleared and `OPS_LIGHT_ALERT_DONE` (`0x04`) is sent back to the MW300.

2. **Door Actuation**:
   - When `DCM_MSG_DOOR_CMD` (`0x10`) arrives, the iDCM PIC asserts **RA1 (`LATA1`)** HIGH and loads a 10 ms countdown timer. When the timer expires, RA1 is pulled LOW. This 10 ms pulse simulates pressing the physical wall console button.

3. **UART2 Binary Protocol (115200 baud, 8-N-1)**:
   - Pins RB3 (TX) and RB4 (RX) run a point-to-point binary protocol with the main board PIC.
   - Fixed 9-byte packet structure:
     `[ 0x01 (SOH), 0x06 (LEN), <cmd>, <param>, <d0>, <d1>, <d2>, <d3>, 0x04 (EOT) ]`
   - Commands:
     - `0x08`: Movement / status report
     - `0x0C`: Periodic heartbeat query
     - `0x2E`: Position calibration sync
     - `0x64`: Up limit parameter sync
     - `0x65`: Down limit parameter sync
     - `0xC8`: Motor current threshold calibration
     - `0xD1`: Motor operational status report
     - `0x03`: Motor stop / abort command

### Headers

**J2 — Factory test / diagnostic header:**

| Pin | Signal                      | Description |
| --- | --------------------------- | ----------- |
| 1   | VSS                         | Ground |
| 2   | RB3                         | UART2 TX (iDCM PIC to Main Board PIC) |
| 3   | VDD (10 kΩ pull-up to MCLR) | Target Power / Reset |
| 4   | RB4                         | UART2 RX (Main Board PIC to iDCM PIC) |
| 5   | VSS                         | Ground |
| 6   | RC5                         | GPIO test line |

J2 exposes the MCLR net and the inter-PIC UART2 bus (RB3/RB4), allowing automated test fixtures to monitor communications between the two PICs or inject commands.

**J8 — PIC ICSP programming header** (standard PICkit pinout):

| Pin | Signal              |
| --- | ------------------- |
| 1   | VPP/MCLR            |
| 2   | VDD                 |
| 3   | VSS                 |
| 4   | RB7 / ICSPDAT (PGD) |
| 5   | RB6 / ICSPCLK (PGC) |
| 6   | RC2                 |

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
