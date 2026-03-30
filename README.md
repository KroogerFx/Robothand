# Robot Hand - Dual Motor Control System

OOP-style project for controlling 2 N20 motors with DRV8833 drivers on 2 separate ESP32 microcontrollers.

## Project Overview

This system uses a **Primary-Secondary architecture**:
- **Primary ESP32**: Receives USB serial commands, controls its motors, communicates with Secondary
- **Secondary ESP32**: Receives commands from Primary via UART, controls its motors, sends encoder feedback

## Hardware Setup

### Per ESP32 Board:
- **2× N20 Motors** with quadrature encoders
- **1× DRV8833 Dual Motor Driver**
- **Encoder connections** (quadrature feedback)

### Pin Configuration

#### Motor 1 (Driver)
- PWM A: GPIO 12
- PWM B: GPIO 13
- Encoder A: GPIO 34
- Encoder B: GPIO 35

#### Motor 2 (Driver)
- PWM A: GPIO 14
- PWM B: GPIO 15
- Encoder A: GPIO 36
- Encoder B: GPIO 39

#### UART Communication (Primary ↔ Secondary)
- RX: GPIO 16 (Serial1 RX)
- TX: GPIO 17 (Serial1 TX)
- Baud: 115200

## Project Structure

```
Robot Hand/
├── platformio.ini          # Configuration with primary/secondary environments
├── README.md              # This file
├── src/
│   ├── primary/
│   │   └── main.cpp      # Primary ESP32 firmware
│   └── secondary/
│       └── main.cpp      # Secondary ESP32 firmware
└── lib/
    ├── DRV8833/
    │   ├── DRV8833.h
    │   └── DRV8833.cpp
    ├── Encoder/
    │   ├── Encoder.h
    │   └── Encoder.cpp
    ├── Motor/
    │   ├── Motor.h
    │   └── Motor.cpp
    └── Communication/
        ├── UART_Comm.h
        └── UART_Comm.cpp
```

## Building & Uploading

### Build Primary Firmware
```bash
platformio run -e primary
```

### Upload to Primary ESP32
```bash
platformio run -e primary -t upload
```

### Build Secondary Firmware
```bash
platformio run -e secondary
```

### Upload to Secondary ESP32
```bash
platformio run -e secondary -t upload
```

### Monitor Serial Output
```bash
platformio device monitor -e primary   # Primary
platformio device monitor -e secondary # Secondary
```

## Primary ESP32 - USB Serial Commands

Send commands via USB Serial (115200 baud):

| Command | Description | Example |
|---------|-------------|---------|
| `m1<speed>` | Set motor 1 speed (-255 to 255) | `m1100` |
| `m2<speed>` | Set motor 2 speed (-255 to 255) | `m2-150` |
| `s1` | Stop motor 1 | `s1` |
| `s2` | Stop motor 2 | `s2` |
| `sa` | Stop all motors | `sa` |
| `d` | Request encoder data | `d` |

### Serial Output Format
```
M1 Pos: 1234 Spd: 567 | M2 Pos: 5678 Spd: 890
```
- **Pos**: Encoder pulse count (position)
- **Spd**: Encoder speed in pulses per second

## Communication Protocol

### Command Packet (Motor Control)
```
[Type: 1 byte] [Motor ID: 1 byte] [Speed MSB: 1 byte] [Speed LSB: 1 byte] [Checksum: 1 byte]
```

### Data Packet (Encoder Feedback)
```
[Type: 1 byte] [Motor ID: 1 byte] [Pos 24-31: 1 byte] [Pos 16-23: 1 byte] 
[Pos 8-15: 1 byte] [Pos 0-7: 1 byte] [Speed MSB: 1 byte] [Speed LSB: 1 byte] [Checksum: 1 byte]
```

Checksum is XOR of all data bytes.

## Class Reference

### DRV8833
Dual motor driver controller using PWM.
```cpp
DRV8833 driver(motor1_a, motor1_b, motor2_a, motor2_b);
driver.init();
driver.setMotor1Speed(200);  // 0-255 forward, 0 stop, -255 reverse
driver.stopAll();
```

### Encoder
Quadrature encoder reader with interrupt support.
```cpp
Encoder enc(pin_a, pin_b);
enc.init();
int32_t pos = enc.getPulseCount();
int32_t spd = enc.getSpeed();  // pulses/second
```

### Motor
Combined motor + encoder controller.
```cpp
Motor motor(&driver, &encoder, motor_id);
motor.init();
motor.setSpeed(150);
int32_t position = motor.getPosition();
int32_t speed = motor.getEncoderSpeed();
```

### UART_Comm
Serial communication protocol between ESP32s.
```cpp
UART_Comm uart(rx_pin, tx_pin, baud_rate);
uart.init();

UART_Comm::MotorCommand cmd;
cmd.type = UART_Comm::CMD_SET_MOTOR_SPEED;
cmd.motor_id = 1;
cmd.speed = 200;
uart.sendCommand(cmd);
```

## Wiring Checklist

- [ ] Primary ESP32 connected to PC via USB
- [ ] Secondary ESP32 powered independently
- [ ] UART lines connected (TX→RX, RX→TX, GND common)
- [ ] DRV8833 PWM pins connected to GPIO
- [ ] Motor encoder pins connected to GPIO (with pull-ups if needed)
- [ ] Motors connected to DRV8833 outputs
- [ ] 5V power supply for motors (if needed)

## Troubleshooting

**Motors not responding:**
- Check GPIO pin assignments match your connections
- Verify DRV8833 is getting power
- Test PWM output with multimeter or oscilloscope

**Encoder not reading:**
- Verify encoder pins and pull-up resistors
- Check encoder wiring (A, B, GND)
- Count encoder pulses per motor revolution

**UART communication failing:**
- Verify TX/RX are crossed between devices
- Check baud rate matches (115200)
- Ensure GND is common between devices
- Monitor with logic analyzer if available

## Future Enhancements

- [ ] PID control loops for motor speed regulation
- [ ] Position control with target setpoint
- [ ] Acceleration/deceleration profiles
- [ ] Motor current monitoring
- [ ] Web interface for control
- [ ] Telemetry logging

## Notes

- PWM frequency is 5kHz (configurable in DRV8833.h)
- Encoder readings use GPIO interrupts
- UART feedback runs at ~100ms intervals
- Speed calculation uses 100ms windows

