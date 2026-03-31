// Secondary ESP32 - Motor Controller
// Receives commands from Primary ESP32 via UART
// Controls motors and sends back encoder feedback

#include <Arduino.h>
#include "../lib/DRV8833/DRV8833.h"
#include "../lib/Encoder/Encoder.h"
#include "../lib/Motor/Motor.h"
#include "../lib/Communication/UART_Comm.h"

// ===== PIN CONFIGURATION =====
// Driver 1
#define MOTOR1_PWM_A 22
#define MOTOR1_PWM_B 21
#define MOTOR2_PWM_A 19
#define MOTOR2_PWM_B 18

// Driver 2
#define MOTOR3_PWM_A 25
#define MOTOR3_PWM_B 26
#define MOTOR4_PWM_A 27
#define MOTOR4_PWM_B 14

// Encoder pins
#define MOTOR1_ENC_A 36
#define MOTOR1_ENC_B 39
#define MOTOR2_ENC_A 16
#define MOTOR2_ENC_B 17
#define MOTOR3_ENC_A 34
#define MOTOR3_ENC_B 35
#define MOTOR4_ENC_A 32
#define MOTOR4_ENC_B 33

// UART Communication to Primary ESP32
#define UART_RX_PIN 23
#define UART_TX_PIN 4
#define UART_BAUD 115200
#define STATUS_LED_PIN 2
#define UART_LED_BLINK_MS 100
#define JOG_SPEED 180
#define STALL_TEST_SPEED 128
#define JOG_DURATION_MS 37
#define POSITION_MOVE_SPEED 170
#define POSITION_SETTLE_TIME_MS 200
#define CALIBRATION_SETTLE_MS 75
#define STALL_CALIBRATION_MAX_RUNTIME_MS 5000
#define MOTOR_ODD_MAX_POSITION 3000
#define MOTOR_EVEN_MAX_POSITION 3500

// ===== HARDWARE OBJECTS =====
DRV8833 driver1(MOTOR1_PWM_A, MOTOR1_PWM_B, MOTOR2_PWM_A, MOTOR2_PWM_B);
DRV8833 driver2(MOTOR3_PWM_A, MOTOR3_PWM_B, MOTOR4_PWM_A, MOTOR4_PWM_B);

Encoder encoder1(MOTOR1_ENC_A, MOTOR1_ENC_B);
Encoder encoder2(MOTOR2_ENC_A, MOTOR2_ENC_B);
Encoder encoder3(MOTOR3_ENC_A, MOTOR3_ENC_B);
Encoder encoder4(MOTOR4_ENC_A, MOTOR4_ENC_B);

Motor motor1(&driver1, &encoder1, 1);
Motor motor2(&driver1, &encoder2, 2);
Motor motor3(&driver2, &encoder3, 1);
Motor motor4(&driver2, &encoder4, 2);

UART_Comm uart_comm(UART_RX_PIN, UART_TX_PIN, UART_BAUD, &Serial2);

unsigned long last_feedback_time = 0;
unsigned long led_on_until = 0;

struct PositionMoveState {
    bool active = false;
    bool hold_enabled = true;
    int32_t target = 0;
    int32_t tolerance = 0;
    unsigned long in_tolerance_since = 0;
    int32_t last_abs_error = 0;
    unsigned long last_progress_time = 0;
};

PositionMoveState position_moves[4];
bool travel_limits_enabled = false;

void clearPositionMove(uint8_t motor_id);

int32_t getMotorMinLimit(uint8_t motor_id) {
    (void)motor_id;
    return 0;
}

int32_t getMotorMaxLimit(uint8_t motor_id) {
    return (motor_id % 2 == 0) ? MOTOR_EVEN_MAX_POSITION
                               : MOTOR_ODD_MAX_POSITION;
}

int32_t clampMotorTargetToLimits(uint8_t motor_id, int32_t target) {
    if (!travel_limits_enabled) {
        return target;
    }

    return constrain(target, getMotorMinLimit(motor_id), getMotorMaxLimit(motor_id));
}

bool isCommandTowardMotorLimit(uint8_t motor_id, int16_t speed, int32_t position) {
    if (!travel_limits_enabled) {
        return false;
    }

    if (speed > 0 && position >= getMotorMaxLimit(motor_id)) {
        return true;
    }

    if (speed < 0 && position <= getMotorMinLimit(motor_id)) {
        return true;
    }

    return false;
}

bool isMotorCommandInverted(uint8_t motor_id) {
    // System motors 5-8 map to secondary motors 1-4 and are mounted reversed.
    return motor_id >= 1 && motor_id <= 4;
}

int16_t applyMotorCommandDirection(uint8_t motor_id, int16_t speed) {
    return isMotorCommandInverted(motor_id) ? -speed : speed;
}

void setMotorCommandSpeed(uint8_t motor_id, Motor* motor, int16_t speed) {
    if (motor != nullptr) {
        int32_t current_position = motor->getPosition();
        if (isCommandTowardMotorLimit(motor_id, speed, current_position)) {
            motor->stop();
            clearPositionMove(motor_id);
            Serial.print("Safety limit stop: motor ");
            Serial.print(motor_id);
            Serial.println(" reached travel limit");
            return;
        }

        motor->setSpeed(applyMotorCommandDirection(motor_id, speed));
    }
}

Motor* getLocalMotor(uint8_t motor_id) {
    switch (motor_id) {
        case 1: return &motor1;
        case 2: return &motor2;
        case 3: return &motor3;
        case 4: return &motor4;
        default: return nullptr;
    }
}

void sendMotorFeedback(uint8_t motor_id, Motor& motor) {
    UART_Comm::EncoderData data;
    data.motor_id = motor_id;
    data.position = motor.getPosition();
    data.speed = motor.getEncoderSpeed();
    uart_comm.sendEncoderData(data);
}

void blinkStatusLed() {
    digitalWrite(STATUS_LED_PIN, HIGH);
    led_on_until = millis() + UART_LED_BLINK_MS;
}

void clearPositionMove(uint8_t motor_id) {
    if (motor_id >= 1 && motor_id <= 4) {
        position_moves[motor_id - 1].active = false;
        position_moves[motor_id - 1].hold_enabled = true;
        position_moves[motor_id - 1].target = 0;
        position_moves[motor_id - 1].tolerance = 0;
        position_moves[motor_id - 1].in_tolerance_since = 0;
        position_moves[motor_id - 1].last_abs_error = 0;
        position_moves[motor_id - 1].last_progress_time = 0;
    }
}

void startPositionMove(uint8_t motor_id, int32_t target, int32_t tolerance) {
    if (motor_id < 1 || motor_id > 4 || tolerance < 0) {
        return;
    }

    target = clampMotorTargetToLimits(motor_id, target);

    position_moves[motor_id - 1].active = true;
    position_moves[motor_id - 1].hold_enabled = true;
    position_moves[motor_id - 1].target = target;
    position_moves[motor_id - 1].tolerance = tolerance;
    position_moves[motor_id - 1].in_tolerance_since = 0;
    position_moves[motor_id - 1].last_abs_error = 0x7fffffff;
    position_moves[motor_id - 1].last_progress_time = millis();
}

int16_t calibrateMotorDirection(uint8_t motor_id, int16_t jog_speed) {
    Motor* motor = getLocalMotor(motor_id);
    if (motor == nullptr) {
        return -1;
    }

    motor->setEncoderInverted(false);
    delay(CALIBRATION_SETTLE_MS);

    int32_t start_position = motor->getRawPosition();
    setMotorCommandSpeed(motor_id, motor, jog_speed);
    delay(JOG_DURATION_MS);
    motor->stop();
    delay(CALIBRATION_SETTLE_MS);

    int32_t delta = motor->getRawPosition() - start_position;
    bool inverted = delta < 0;

    motor->setEncoderInverted(inverted);

    if (delta == 0) {
        return -1;
    }
    return inverted ? 1 : 0;
}

bool runMotorForwardUntilStall(uint8_t motor_id, int16_t jog_speed) {
    Motor* motor = getLocalMotor(motor_id);
    if (motor == nullptr) {
        return false;
    }

    motor->stop();
    clearPositionMove(motor_id);
    delay(CALIBRATION_SETTLE_MS);

    setMotorCommandSpeed(motor_id, motor, abs(jog_speed));

    unsigned long start_time = millis();
    while (millis() - start_time < STALL_CALIBRATION_MAX_RUNTIME_MS) {
        if (motor->updateSafety()) {
            clearPositionMove(motor_id);
            return true;
        }

        delay(5);
    }

    motor->stop();
    clearPositionMove(motor_id);
    return false;
}

void updatePositionMoves() {
    unsigned long now = millis();
    const unsigned long POSITION_PROGRESS_TIMEOUT_MS = 150;
    const int32_t POSITION_PROGRESS_MARGIN_COUNTS = 10;

    for (uint8_t motor_id = 1; motor_id <= 4; motor_id++) {
        if (!position_moves[motor_id - 1].active) {
            continue;
        }

        Motor* motor = getLocalMotor(motor_id);
        PositionMoveState& move = position_moves[motor_id - 1];
        int32_t current_position = motor->getPosition();
        int32_t error = move.target - current_position;
        int32_t abs_error = abs(error);

        if (abs_error <= move.tolerance) {
            motor->stop();
            move.in_tolerance_since = now;
            move.last_abs_error = abs_error;
            move.last_progress_time = now;

            continue;
        }

        if (isCommandTowardMotorLimit(
                motor_id,
                error > 0 ? POSITION_MOVE_SPEED : -POSITION_MOVE_SPEED,
                current_position)) {
            motor->stop();
            clearPositionMove(motor_id);
            Serial.print("Safety limit stop: motor ");
            Serial.print(motor_id);
            Serial.println(" reached travel limit");
            continue;
        }

        if (move.last_abs_error == 0x7fffffff ||
            abs_error + POSITION_PROGRESS_MARGIN_COUNTS < move.last_abs_error) {
            move.last_abs_error = abs_error;
            move.last_progress_time = now;
        } else if (now - move.last_progress_time >= POSITION_PROGRESS_TIMEOUT_MS) {
            motor->stop();
            clearPositionMove(motor_id);
            Serial.print("Position hold aborted: motor ");
            Serial.print(motor_id);
            Serial.println(" not making progress");
            continue;
        }

        move.in_tolerance_since = 0;
        setMotorCommandSpeed(motor_id, motor, error > 0 ? POSITION_MOVE_SPEED : -POSITION_MOVE_SPEED);
    }
}

void updateMotorSafety() {
    for (uint8_t motor_id = 1; motor_id <= 4; motor_id++) {
        Motor* motor = getLocalMotor(motor_id);
        if (motor != nullptr && motor->updateSafety()) {
            clearPositionMove(motor_id);
            Serial.print("Safety stop: motor ");
            Serial.print(motor_id);
            Serial.println(" encoder stalled");
        }
    }
}

void setup() {
    Serial.begin(115200);
    pinMode(STATUS_LED_PIN, OUTPUT);
    digitalWrite(STATUS_LED_PIN, LOW);

    driver1.init();
    driver2.init();
    motor1.init();
    motor2.init();
    motor3.init();
    motor4.init();

    uart_comm.init();
    travel_limits_enabled = false;
    Serial.println("====================================");
    Serial.println("SECONDARY ESP32 - Motor Controller");
    Serial.println("====================================");
    Serial.println("UART link: Serial2 RX=23 TX=4");
    Serial.println("Waiting for commands from Primary...");
}

void loop() {
    if (uart_comm.dataAvailable()) {
        blinkStatusLed();
    }

    while (uart_comm.dataAvailable()) {
        int packet_type = uart_comm.peekType();

        if (packet_type == UART_Comm::CMD_MOVE_TO_POSITION) {
            UART_Comm::PositionCommand pos_cmd;
            if (!uart_comm.receivePositionCommand(pos_cmd)) {
                break;
            }

            blinkStatusLed();
            startPositionMove(pos_cmd.motor_id, pos_cmd.target_position, pos_cmd.tolerance);
            Serial.print("Motor ");
            Serial.print(pos_cmd.motor_id);
            Serial.print(" target: ");
            Serial.print(pos_cmd.target_position);
            Serial.print(" +/- ");
            Serial.println(pos_cmd.tolerance);
        } else if (packet_type == UART_Comm::CMD_SET_MOTOR_SPEED ||
                   packet_type == UART_Comm::CMD_GET_ENCODER_DATA ||
                   packet_type == UART_Comm::CMD_STOP_MOTOR ||
                   packet_type == UART_Comm::CMD_RESET_ENCODER ||
                   packet_type == UART_Comm::CMD_CALIBRATE_ENCODER_DIRECTION ||
                   packet_type == UART_Comm::CMD_RUN_UNTIL_STALL ||
                   packet_type == UART_Comm::CMD_ACK) {
            UART_Comm::MotorCommand cmd;
            if (!uart_comm.receiveCommand(cmd)) {
                break;
            }

            blinkStatusLed();

            switch (cmd.type) {
                case UART_Comm::CMD_SET_MOTOR_SPEED: {
                    Motor* local_motor = getLocalMotor(cmd.motor_id);
                    if (local_motor != nullptr) {
                        clearPositionMove(cmd.motor_id);
                        setMotorCommandSpeed(cmd.motor_id, local_motor, cmd.speed);
                        Serial.print("Motor ");
                        Serial.print(cmd.motor_id);
                        Serial.print(" set to: ");
                        Serial.println(cmd.speed);
                    }
                    break;
                }

                case UART_Comm::CMD_STOP_MOTOR:
                    if (cmd.motor_id == 0) {
                        motor1.stop();
                        motor2.stop();
                        motor3.stop();
                        motor4.stop();
                        for (uint8_t i = 0; i < 4; i++) {
                            position_moves[i].active = false;
                        }
                        Serial.println("All motors stopped");
                    } else {
                        Motor* local_motor = getLocalMotor(cmd.motor_id);
                        if (local_motor != nullptr) {
                            local_motor->stop();
                            clearPositionMove(cmd.motor_id);
                            Serial.print("Motor ");
                            Serial.print(cmd.motor_id);
                            Serial.println(" stopped");
                        }
                    }
                    break;

                case UART_Comm::CMD_RESET_ENCODER:
                    if (cmd.motor_id == 0) {
                        motor1.resetPosition();
                        motor2.resetPosition();
                        motor3.resetPosition();
                        motor4.resetPosition();
                        travel_limits_enabled = true;
                        for (uint8_t i = 0; i < 4; i++) {
                            position_moves[i].active = false;
                        }
                        Serial.println("All encoders reset");
                    } else {
                        Motor* local_motor = getLocalMotor(cmd.motor_id);
                        if (local_motor != nullptr) {
                            local_motor->resetPosition();
                            travel_limits_enabled = true;
                            clearPositionMove(cmd.motor_id);
                            Serial.print("Motor ");
                            Serial.print(cmd.motor_id);
                            Serial.println(" encoder reset");
                        }
                    }
                    break;

                case UART_Comm::CMD_GET_ENCODER_DATA:
                    sendMotorFeedback(1, motor1);
                    sendMotorFeedback(2, motor2);
                    sendMotorFeedback(3, motor3);
                    sendMotorFeedback(4, motor4);
                    break;

                case UART_Comm::CMD_CALIBRATE_ENCODER_DIRECTION: {
                    int16_t result = calibrateMotorDirection(cmd.motor_id, abs(cmd.speed));
                    UART_Comm::MotorCommand ack;
                    ack.type = UART_Comm::CMD_ACK;
                    ack.motor_id = cmd.motor_id;
                    ack.speed = result;
                    uart_comm.sendCommand(ack);
                    break;
                }

                case UART_Comm::CMD_RUN_UNTIL_STALL: {
                    bool success = runMotorForwardUntilStall(cmd.motor_id, abs(cmd.speed));
                    UART_Comm::MotorCommand ack;
                    ack.type = UART_Comm::CMD_ACK;
                    ack.motor_id = cmd.motor_id;
                    ack.speed = success ? 1 : 0;
                    uart_comm.sendCommand(ack);
                    break;
                }

                default:
                    break;
            }
        } else {
            uart_comm.discardByte();
        }
    }

    if (led_on_until != 0 && millis() >= led_on_until) {
        digitalWrite(STATUS_LED_PIN, LOW);
        led_on_until = 0;
    }

    updatePositionMoves();
    updateMotorSafety();

    if (millis() - last_feedback_time >= 100) {
        last_feedback_time = millis();

        sendMotorFeedback(1, motor1);
        sendMotorFeedback(2, motor2);
        sendMotorFeedback(3, motor3);
        sendMotorFeedback(4, motor4);

        Serial.print("M1: ");
        Serial.print(motor1.getPosition());
        Serial.print(" | M2: ");
        Serial.print(motor2.getPosition());
        Serial.print(" | M3: ");
        Serial.print(motor3.getPosition());
        Serial.print(" | M4: ");
        Serial.println(motor4.getPosition());
    }
}
