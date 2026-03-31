# Robot Hand

Dual-ESP32 PlatformIO project for an 8-motor tendon-driven robot hand.

The system is split across two ESP32 boards:
- Primary ESP32: USB serial command interface, controls motors 1-4, coordinates the full hand
- Secondary ESP32: receives UART commands from the primary, controls motors 5-8, reports encoder feedback

Each board drives 4 N20 gearmotors with quadrature encoders through 2 DRV8833 drivers.

## Current Status

The project now supports:
- 8 motors total
- encoder-based position moves
- direct jog commands
- encoder direction calibration
- stall safety in the shared motor layer
- a staged calibration routine `c2` that currently runs motors in this order:
  - 2, 1
  - 6, 5
  - 4, 3
  - 8, 7

## Project Layout

```text
Robot Hand/
├── platformio.ini
├── src/
│   ├── primary/
│   │   └── main.cpp
│   └── secondary/
│       └── main.cpp
├── lib/
│   ├── Communication/
│   ├── DRV8833/
│   ├── Encoder/
│   └── Motor/
├── include/
└── test/
```

## Hardware Summary

System mapping:
- Motors 1-4 live on the primary ESP32
- Motors 5-8 live on the secondary ESP32

Per board:
- 4 N20 motors with quadrature encoders
- 2 DRV8833 dual H-bridge drivers

## Pinout

The primary and secondary use the same local motor pin layout. On the secondary, those local motors map to system motors 5-8.

### Local Motor 1
- PWM A: GPIO 22
- PWM B: GPIO 21
- Encoder A: GPIO 36
- Encoder B: GPIO 39

### Local Motor 2
- PWM A: GPIO 19
- PWM B: GPIO 18
- Encoder A: GPIO 16
- Encoder B: GPIO 17

### Local Motor 3
- PWM A: GPIO 25
- PWM B: GPIO 26
- Encoder A: GPIO 34
- Encoder B: GPIO 35

### Local Motor 4
- PWM A: GPIO 27
- PWM B: GPIO 14
- Encoder A: GPIO 32
- Encoder B: GPIO 33

### UART

Primary ESP32:
- Serial2 RX: GPIO 4
- Serial2 TX: GPIO 23

Secondary ESP32:
- Serial2 RX: GPIO 23
- Serial2 TX: GPIO 4

Baud:
- `115200`

## Build

Build primary:

```bash
platformio run -e primary
```

Build secondary:

```bash
platformio run -e secondary
```

Upload primary:

```bash
platformio run -e primary -t upload
```

Upload secondary:

```bash
platformio run -e secondary -t upload
```

Monitor serial:

```bash
platformio device monitor -e primary
platformio device monitor -e secondary
```

## Primary Serial Commands

Commands are entered on the primary USB serial port at `115200`.

- `h`
  - Show help
- `cf`
  - Calibrate encoder direction on motors 1-8
- `c2`
  - Run the current staged calibration routine for pairs `2,1`, `6,5`, `4,3`, `8,7`
- `j<id>f`
  - Jog motor forward
- `j<id>b`
  - Jog motor backward
- `j<id><f|b>,<counts>,<tol>`
  - Relative encoder move
- `p<id>,<pos>,<tol>`
  - Move one motor to encoder position
- `p<id>,<pos>,<tol>;<id>,<pos>,<tol>`
  - Move multiple motors at once
- `s<id>`
  - Stop one motor
- `sa`
  - Stop all motors
- `e`
  - Print all encoder counts
- `ra`
  - Reset all encoder counts to zero
- `mz`
  - Move all motors to encoder position 0 with tolerance 5

Examples:

```text
j2f
j2b,1000,5
p1,500,5;p2,500,5;p3,500,5;p4,500,5
sa
ra
c2
```

## Communication Layer

The two boards communicate through `UART_Comm`.

Current packet families include:
- short command packets for speed, stop, reset, calibration, stall-run, ack
- position command packets
- encoder feedback packets

The secondary periodically sends encoder feedback for its four motors, and the primary keeps a mirrored remote state table for motors 5-8.

## Main Classes

### `DRV8833`

Low-level dual motor driver wrapper.
- configures PWM channels
- sets signed motor speed
- handles stop behavior

### `Encoder`

Quadrature encoder reader.
- interrupt-driven count updates
- exposes pulse count
- computes speed over a time window

### `Motor`

Combines one driver channel and one encoder.
- signed speed control
- logical encoder inversion
- position access
- shared stall safety logic

### `UART_Comm`

Serial protocol wrapper used between primary and secondary.
- sends and receives command packets
- sends and receives encoder feedback
- performs checksum validation

## Notes

- The top-level `src/main.cpp` is only a placeholder and is not built.
- The active firmware lives in:
  - `src/primary/main.cpp`
  - `src/secondary/main.cpp`
- Secondary motors are direction-compensated in software because of their physical mounting.
- The project currently uses a local PlatformIO core directory during development, which is intentionally excluded from Git.

## Next Documentation Improvements

- Document the full tendon routing and motor-to-finger mapping
- Add a board wiring diagram
- Add a command reference table with expected responses
- Document the current calibration workflow in more detail
