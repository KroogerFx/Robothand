// Primary ESP32 - Main Controller
// Receives USB serial input and controls local motors
// Communicates with Secondary ESP32 via UART

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

// UART Communication to Secondary ESP32
#define UART_RX_PIN 4
#define UART_TX_PIN 23
#define UART_BAUD 115200
#define JOG_SPEED 180
#define STALL_TEST_SPEED 128
#define JOG_DURATION_MS 37
#define POSITION_MOVE_SPEED 170
#define POSITION_SETTLE_TIME_MS 200
#define RETURN_TO_ZERO_TOLERANCE 5
#define CALIBRATION_SETTLE_MS 75
#define CALIBRATION_TIMEOUT_MS 1500
#define STALL_CALIBRATION_TIMEOUT_MS 100
#define STALL_CALIBRATION_MIN_MOVEMENT_COUNTS 50
#define STALL_CALIBRATION_MAX_RUNTIME_MS 5000
#define STALL_CALIBRATION_BACKOFF_COUNTS 50
#define STALL_CALIBRATION_BACKOFF_TOLERANCE 5
#define CAL2_STALL_SPEED 128
#define CAL2_STALL_WINDOW_MS 100
#define CAL2_STALL_MIN_MOVING_COUNTS 60
#define CAL2_STALL_DROP_PERCENT 50
#define CAL2_STALL_MIN_WINDOW_COUNTS 25
#define CAL2_FINAL_BACKOFF_COUNTS 2000
#define CAL2_SETTLE_MS 100

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

struct RemoteMotorState {
    int32_t position = 0;
    int32_t speed = 0;
};

RemoteMotorState secondary_motors[4];

struct JogState {
    bool active = false;
    unsigned long stop_time = 0;
};

JogState jog_states[8];

struct PositionMoveState {
    bool active = false;
    bool hold_enabled = true;
    int32_t target = 0;
    int32_t tolerance = 0;
    unsigned long in_tolerance_since = 0;
    int32_t last_abs_error = 0;
    unsigned long last_progress_time = 0;
};

PositionMoveState position_moves[8];

struct PositionCommandInput {
    uint8_t motor_id = 0;
    int32_t target = 0;
    int32_t tolerance = 0;
};

struct JogMoveInput {
    uint8_t motor_id = 0;
    bool forward = true;
    bool has_position_target = false;
    int32_t counts = 0;
    int32_t tolerance = 0;
};

void startPositionMove(
    uint8_t motor_id,
    int32_t target,
    int32_t tolerance,
    bool hold_enabled = true);
int32_t getMotorPositionForCalibration(uint8_t motor_id);

Motor* getLocalMotor(uint8_t motor_id) {
    switch (motor_id) {
        case 1: return &motor1;
        case 2: return &motor2;
        case 3: return &motor3;
        case 4: return &motor4;
        default: return nullptr;
    }
}

uint8_t toSecondaryMotorId(uint8_t motor_id) {
    if (motor_id >= 5 && motor_id <= 8) {
        return motor_id - 4;
    }
    return 0;
}

RemoteMotorState* getRemoteMotor(uint8_t motor_id) {
    if (motor_id >= 1 && motor_id <= 4) {
        return &secondary_motors[motor_id - 1];
    }
    return nullptr;
}

void sendStopCommand(uint8_t motor_id) {
    UART_Comm::MotorCommand cmd_data;
    cmd_data.type = UART_Comm::CMD_STOP_MOTOR;
    cmd_data.motor_id = motor_id;
    cmd_data.speed = 0;
    uart_comm.sendCommand(cmd_data);
}

void sendMotorSpeedCommand(uint8_t motor_id, int16_t speed) {
    UART_Comm::MotorCommand cmd_data;
    cmd_data.type = UART_Comm::CMD_SET_MOTOR_SPEED;
    cmd_data.motor_id = motor_id;
    cmd_data.speed = speed;
    uart_comm.sendCommand(cmd_data);
}

void requestEncoderData() {
    UART_Comm::MotorCommand cmd_data;
    cmd_data.type = UART_Comm::CMD_GET_ENCODER_DATA;
    cmd_data.motor_id = 0;
    cmd_data.speed = 0;
    uart_comm.sendCommand(cmd_data);
}

void sendResetEncoderCommand(uint8_t motor_id) {
    UART_Comm::MotorCommand cmd_data;
    cmd_data.type = UART_Comm::CMD_RESET_ENCODER;
    cmd_data.motor_id = motor_id;
    cmd_data.speed = 0;
    uart_comm.sendCommand(cmd_data);
}

void sendPositionMoveCommand(uint8_t motor_id, int32_t target_position, int32_t tolerance) {
    UART_Comm::PositionCommand cmd_data;
    cmd_data.motor_id = motor_id;
    cmd_data.target_position = target_position;
    cmd_data.tolerance = tolerance;
    uart_comm.sendPositionCommand(cmd_data);
}

void sendCalibrateEncoderCommand(uint8_t motor_id) {
    UART_Comm::MotorCommand cmd_data;
    cmd_data.type = UART_Comm::CMD_CALIBRATE_ENCODER_DIRECTION;
    cmd_data.motor_id = motor_id;
    cmd_data.speed = JOG_SPEED;
    uart_comm.sendCommand(cmd_data);
}

void updateRemoteEncoderStates() {
    while (uart_comm.dataAvailable()) {
        int packet_type = uart_comm.peekType();

        if (packet_type == UART_Comm::RESP_ENCODER_DATA) {
            UART_Comm::EncoderData feedback;
            if (!uart_comm.receiveEncoderData(feedback)) {
                break;
            }

            RemoteMotorState* remote_motor = getRemoteMotor(feedback.motor_id);
            if (remote_motor != nullptr) {
                remote_motor->position = feedback.position;
                remote_motor->speed = feedback.speed;
            }
        } else {
            break;
        }
    }
}

bool waitForCalibrationAck(uint8_t secondary_motor_id, int16_t& ack_value) {
    unsigned long start_time = millis();

    while (millis() - start_time < CALIBRATION_TIMEOUT_MS) {
        if (!uart_comm.dataAvailable()) {
            delay(5);
            continue;
        }

        int packet_type = uart_comm.peekType();

        if (packet_type == UART_Comm::RESP_ENCODER_DATA) {
            updateRemoteEncoderStates();
        } else if (packet_type == UART_Comm::CMD_ACK ||
                   packet_type == UART_Comm::CMD_SET_MOTOR_SPEED ||
                   packet_type == UART_Comm::CMD_GET_ENCODER_DATA ||
                   packet_type == UART_Comm::CMD_STOP_MOTOR ||
                   packet_type == UART_Comm::CMD_RESET_ENCODER ||
                   packet_type == UART_Comm::CMD_CALIBRATE_ENCODER_DIRECTION) {
            UART_Comm::MotorCommand ack;
            if (uart_comm.receiveCommand(ack) &&
                ack.type == UART_Comm::CMD_ACK &&
                ack.motor_id == secondary_motor_id) {
                ack_value = ack.speed;
                return true;
            }
        } else {
            uart_comm.discardByte();
        }
    }

    return false;
}

void stopMotorById(uint8_t motor_id) {
    Motor* local_motor = getLocalMotor(motor_id);
    uint8_t secondary_motor_id = toSecondaryMotorId(motor_id);

    if (local_motor != nullptr) {
        local_motor->stop();
    } else if (secondary_motor_id != 0) {
        sendStopCommand(secondary_motor_id);
    }
}

void startJog(uint8_t motor_id, bool forward) {
    Motor* local_motor = getLocalMotor(motor_id);
    uint8_t secondary_motor_id = toSecondaryMotorId(motor_id);
    int16_t speed = forward ? JOG_SPEED : -JOG_SPEED;

    if (local_motor != nullptr) {
        local_motor->setSpeed(speed);
    } else if (secondary_motor_id != 0) {
        sendMotorSpeedCommand(secondary_motor_id, speed);
    } else {
        Serial.println("Invalid motor ID. Use 1-8.");
        return;
    }

    jog_states[motor_id - 1].active = true;
    jog_states[motor_id - 1].stop_time = millis() + JOG_DURATION_MS;

    Serial.print("Jogging motor ");
    Serial.print(motor_id);
    Serial.println(forward ? " forward" : " backward");
}

bool refreshRemoteMotorStates() {
    requestEncoderData();
    delay(20);
    updateRemoteEncoderStates();
    return true;
}

void refreshRemoteMotorStateIfNeeded(uint8_t motor_id) {
    if (motor_id >= 5 && motor_id <= 8) {
        refreshRemoteMotorStates();
    }
}

bool parseJogCommand(const String& input, JogMoveInput& command) {
    if (input.length() < 3 || tolower(input.charAt(0)) != 'j') {
        return false;
    }

    command.motor_id = input.charAt(1) - '0';
    char direction = tolower(input.charAt(2));
    if (command.motor_id < 1 || command.motor_id > 8 ||
        (direction != 'f' && direction != 'b')) {
        return false;
    }

    command.forward = (direction == 'f');

    if (input.length() == 3) {
        command.has_position_target = false;
        return true;
    }

    if (input.charAt(3) != ',') {
        return false;
    }

    int first_comma = input.indexOf(',', 3);
    int second_comma = input.indexOf(',', first_comma + 1);
    if (first_comma != 3 || second_comma <= first_comma) {
        return false;
    }

    command.counts = input.substring(first_comma + 1, second_comma).toInt();
    command.tolerance = input.substring(second_comma + 1).toInt();
    command.has_position_target = true;

    return command.counts > 0 && command.tolerance >= 0;
}

void startJogMove(uint8_t motor_id, bool forward, int32_t counts, int32_t tolerance) {
    if (motor_id < 1 || motor_id > 8 || counts <= 0 || tolerance < 0) {
        Serial.println("Invalid jog move command.");
        return;
    }

    int32_t current_position = 0;
    Motor* local_motor = getLocalMotor(motor_id);
    if (local_motor != nullptr) {
        current_position = local_motor->getPosition();
    } else {
        if (!refreshRemoteMotorStates()) {
            Serial.println("Unable to refresh remote encoder data.");
            return;
        }

        uint8_t secondary_motor_id = toSecondaryMotorId(motor_id);
        RemoteMotorState* remote_motor = getRemoteMotor(secondary_motor_id);
        if (remote_motor == nullptr) {
            Serial.println("Invalid motor ID. Use 1-8.");
            return;
        }
        current_position = remote_motor->position;
    }

    int32_t target = current_position + (forward ? counts : -counts);
    startPositionMove(motor_id, target, tolerance);
}

void updateJogStops() {
    unsigned long now = millis();

    for (uint8_t i = 0; i < 8; i++) {
        if (jog_states[i].active && now >= jog_states[i].stop_time) {
            stopMotorById(i + 1);
            jog_states[i].active = false;
        }
    }
}

void clearPositionMove(uint8_t motor_id) {
    if (motor_id >= 1 && motor_id <= 8) {
        position_moves[motor_id - 1].active = false;
        position_moves[motor_id - 1].hold_enabled = true;
        position_moves[motor_id - 1].target = 0;
        position_moves[motor_id - 1].tolerance = 0;
        position_moves[motor_id - 1].in_tolerance_since = 0;
        position_moves[motor_id - 1].last_abs_error = 0;
        position_moves[motor_id - 1].last_progress_time = 0;
    }
}

void startPositionMove(
    uint8_t motor_id,
    int32_t target,
    int32_t tolerance,
    bool hold_enabled) {
    Motor* local_motor = getLocalMotor(motor_id);
    uint8_t secondary_motor_id = toSecondaryMotorId(motor_id);

    if (motor_id < 1 || motor_id > 8 || tolerance < 0) {
        Serial.println("Invalid move command.");
        return;
    }

    if (local_motor != nullptr) {
        position_moves[motor_id - 1].active = true;
        position_moves[motor_id - 1].hold_enabled = hold_enabled;
        position_moves[motor_id - 1].target = target;
        position_moves[motor_id - 1].tolerance = tolerance;
        position_moves[motor_id - 1].in_tolerance_since = 0;
        position_moves[motor_id - 1].last_abs_error = 0x7fffffff;
        position_moves[motor_id - 1].last_progress_time = millis();
    } else if (secondary_motor_id != 0) {
        sendPositionMoveCommand(secondary_motor_id, target, tolerance);
        position_moves[motor_id - 1].active = true;
        position_moves[motor_id - 1].hold_enabled = hold_enabled;
        position_moves[motor_id - 1].target = target;
        position_moves[motor_id - 1].tolerance = tolerance;
        position_moves[motor_id - 1].in_tolerance_since = 0;
        position_moves[motor_id - 1].last_abs_error = 0x7fffffff;
        position_moves[motor_id - 1].last_progress_time = millis();
    } else {
        Serial.println("Invalid motor ID. Use 1-8.");
        return;
    }

    Serial.print("Moving motor ");
    Serial.print(motor_id);
    Serial.print(" to ");
    Serial.print(target);
    Serial.print(" +/- ");
    Serial.println(tolerance);
}

bool parsePositionCommandSegment(const String& segment, PositionCommandInput& command) {
    String trimmed = segment;
    trimmed.trim();

    if (trimmed.startsWith("p") || trimmed.startsWith("P")) {
        trimmed = trimmed.substring(1);
        trimmed.trim();
    }

    int first_comma = trimmed.indexOf(',');
    int second_comma = trimmed.indexOf(',', first_comma + 1);

    if (first_comma <= 0 || second_comma <= first_comma) {
        return false;
    }

    command.motor_id = trimmed.substring(0, first_comma).toInt();
    command.target = trimmed.substring(first_comma + 1, second_comma).toInt();
    command.tolerance = trimmed.substring(second_comma + 1).toInt();

    return command.motor_id >= 1 && command.motor_id <= 8 && command.tolerance >= 0;
}

bool handlePositionCommands(const String& input) {
    String payload = input.substring(1);
    payload.trim();

    if (payload.length() == 0) {
        return false;
    }

    PositionCommandInput commands[8];
    uint8_t command_count = 0;
    int start_index = 0;
    bool parsed_all_segments = false;

    while (start_index < payload.length() && command_count < 8) {
        int separator_index = payload.indexOf(';', start_index);
        String segment = separator_index == -1
            ? payload.substring(start_index)
            : payload.substring(start_index, separator_index);

        if (!parsePositionCommandSegment(segment, commands[command_count])) {
            return false;
        }

        command_count++;

        if (separator_index == -1) {
            parsed_all_segments = true;
            break;
        }

        start_index = separator_index + 1;
    }

    if (command_count == 0 || !parsed_all_segments) {
        return false;
    }

    for (uint8_t i = 0; i < command_count; i++) {
        startPositionMove(commands[i].motor_id, commands[i].target, commands[i].tolerance);
    }

    return true;
}

void updateLocalPositionMoves() {
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
        motor->setSpeed(error > 0 ? POSITION_MOVE_SPEED : -POSITION_MOVE_SPEED);
    }
}

void updateLocalMotorSafety() {
    for (uint8_t motor_id = 1; motor_id <= 4; motor_id++) {
        Motor* motor = getLocalMotor(motor_id);
        if (motor != nullptr && motor->updateSafety()) {
            jog_states[motor_id - 1].active = false;
            clearPositionMove(motor_id);
            Serial.print("Safety stop: motor ");
            Serial.print(motor_id);
            Serial.println(" encoder stalled");
        }
    }
}

void printAllEncoderCounts() {
    requestEncoderData();
    delay(20);
    updateRemoteEncoderStates();

    Serial.println("Encoder counts:");
    Serial.print("M1: ");
    Serial.print(motor1.getPosition());
    Serial.print(" | M2: ");
    Serial.print(motor2.getPosition());
    Serial.print(" | M3: ");
    Serial.print(motor3.getPosition());
    Serial.print(" | M4: ");
    Serial.println(motor4.getPosition());

    Serial.print("M5: ");
    Serial.print(secondary_motors[0].position);
    Serial.print(" | M6: ");
    Serial.print(secondary_motors[1].position);
    Serial.print(" | M7: ");
    Serial.print(secondary_motors[2].position);
    Serial.print(" | M8: ");
    Serial.println(secondary_motors[3].position);
}

void printCalibrationResult(uint8_t motor_id, int32_t delta, bool inverted) {
    Serial.print("Motor ");
    Serial.print(motor_id);
    Serial.print(" raw delta: ");
    Serial.print(delta);
    Serial.print(" -> encoder ");
    Serial.println(inverted ? "inverted" : "normal");
}

void calibrateLocalMotorDirection(uint8_t motor_id) {
    Motor* motor = getLocalMotor(motor_id);
    if (motor == nullptr) {
        return;
    }

    motor->setEncoderInverted(false);
    delay(CALIBRATION_SETTLE_MS);

    int32_t start_position = motor->getRawPosition();
    motor->setSpeed(JOG_SPEED);
    delay(JOG_DURATION_MS);
    motor->stop();
    delay(CALIBRATION_SETTLE_MS);

    int32_t delta = motor->getRawPosition() - start_position;
    bool inverted = delta < 0;

    motor->setEncoderInverted(inverted);
    printCalibrationResult(motor_id, delta, inverted);
}

void calibrateRemoteMotorDirection(uint8_t motor_id) {
    uint8_t secondary_motor_id = toSecondaryMotorId(motor_id);
    if (secondary_motor_id == 0) {
        return;
    }

    sendCalibrateEncoderCommand(secondary_motor_id);

    int16_t ack_value = 0;
    if (waitForCalibrationAck(secondary_motor_id, ack_value)) {
        Serial.print("Motor ");
        Serial.print(motor_id);
        Serial.print(" calibration: ");

        if (ack_value < 0) {
            Serial.println("no encoder movement detected");
        } else if (ack_value > 0) {
            Serial.println("encoder inverted");
        } else {
            Serial.println("encoder normal");
        }
    } else {
        Serial.print("Motor ");
        Serial.print(motor_id);
        Serial.println(" calibration timed out");
    }
}

void calibrateEncoderDirections() {
    Serial.println("Calibrating encoder directions...");

    for (uint8_t motor_id = 1; motor_id <= 4; motor_id++) {
        calibrateLocalMotorDirection(motor_id);
    }

    for (uint8_t motor_id = 5; motor_id <= 8; motor_id++) {
        calibrateRemoteMotorDirection(motor_id);
    }
}

void moveAllMotorsToZero() {
    for (uint8_t motor_id = 1; motor_id <= 8; motor_id++) {
        startPositionMove(motor_id, 0, RETURN_TO_ZERO_TOLERANCE);
    }

    Serial.println("Moving all motors to encoder position 0 +/- 5");
}

int32_t getMotorPositionForCalibration(uint8_t motor_id) {
    Motor* local_motor = getLocalMotor(motor_id);
    if (local_motor != nullptr) {
        return local_motor->getPosition();
    }

    uint8_t secondary_motor_id = toSecondaryMotorId(motor_id);
    RemoteMotorState* remote_motor = getRemoteMotor(secondary_motor_id);
    if (secondary_motor_id != 0 && remote_motor != nullptr) {
        return remote_motor->position;
    }

    return 0;
}

void requestRemoteFeedbackIfNeeded(uint8_t motor_id, unsigned long& last_request_time) {
    if (motor_id < 5 || motor_id > 8) {
        return;
    }

    unsigned long now = millis();
    if (now - last_request_time >= 20) {
        requestEncoderData();
        last_request_time = now;
    }
}

void setCalibrationMotorSpeed(uint8_t motor_id, int16_t speed) {
    Motor* motor = getLocalMotor(motor_id);
    uint8_t secondary_motor_id = toSecondaryMotorId(motor_id);

    if (motor != nullptr) {
        motor->setSpeed(speed);
    } else if (secondary_motor_id != 0) {
        sendMotorSpeedCommand(secondary_motor_id, speed);
    }
}

bool waitForMotorHold(uint8_t motor_id, int32_t target, int32_t tolerance) {
    unsigned long start_time = millis();
    unsigned long in_tolerance_since = 0;
    unsigned long last_remote_request_time = 0;

    while (millis() - start_time < STALL_CALIBRATION_MAX_RUNTIME_MS) {
        updateRemoteEncoderStates();
        updateJogStops();
        updateLocalPositionMoves();
        updateLocalMotorSafety();
        requestRemoteFeedbackIfNeeded(motor_id, last_remote_request_time);

        int32_t current_position = getMotorPositionForCalibration(motor_id);
        if (abs(target - current_position) <= tolerance) {
            if (in_tolerance_since == 0) {
                in_tolerance_since = millis();
            } else if (millis() - in_tolerance_since >= CAL2_SETTLE_MS) {
                return true;
            }
        } else {
            in_tolerance_since = 0;
        }

        delay(5);
    }

    return false;
}

bool runMotorUntilRateDropStall(uint8_t motor_id, int32_t& stall_position) {
    if (motor_id != 1 && motor_id != 2 &&
        motor_id != 3 && motor_id != 4 &&
        motor_id != 5 && motor_id != 6 &&
        motor_id != 7 && motor_id != 8) {
        return false;
    }

    stopMotorById(motor_id);
    clearPositionMove(motor_id);
    jog_states[motor_id - 1].active = false;
    delay(CALIBRATION_SETTLE_MS);

    refreshRemoteMotorStateIfNeeded(motor_id);
    int32_t window_start_position = getMotorPositionForCalibration(motor_id);
    unsigned long window_start_time = millis();
    unsigned long start_time = window_start_time;
    unsigned long last_remote_request_time = 0;
    int32_t peak_window_counts = 0;
    bool saw_meaningful_motion = false;

    setCalibrationMotorSpeed(motor_id, CAL2_STALL_SPEED);

    while (millis() - start_time < STALL_CALIBRATION_MAX_RUNTIME_MS) {
        unsigned long now = millis();
        updateRemoteEncoderStates();
        requestRemoteFeedbackIfNeeded(motor_id, last_remote_request_time);

        if (now - window_start_time < CAL2_STALL_WINDOW_MS) {
            delay(5);
            continue;
        }

        int32_t current_position = getMotorPositionForCalibration(motor_id);
        int32_t window_counts = abs(current_position - window_start_position);

        if (window_counts > peak_window_counts) {
            peak_window_counts = window_counts;
        }

        if (window_counts >= CAL2_STALL_MIN_MOVING_COUNTS) {
            saw_meaningful_motion = true;
        }

        int32_t dynamic_threshold =
            max(CAL2_STALL_MIN_WINDOW_COUNTS,
                (peak_window_counts * CAL2_STALL_DROP_PERCENT) / 100);

        if (saw_meaningful_motion && window_counts <= dynamic_threshold) {
            stopMotorById(motor_id);
            refreshRemoteMotorStateIfNeeded(motor_id);
            stall_position = getMotorPositionForCalibration(motor_id);
            Serial.print("C2 M");
            Serial.print(motor_id);
            Serial.print(" stall=");
            Serial.print(stall_position);
            Serial.print(" window=");
            Serial.print(window_counts);
            Serial.print(" peak=");
            Serial.println(peak_window_counts);
            return true;
        }

        window_start_position = current_position;
        window_start_time = now;
    }

    stopMotorById(motor_id);
    Serial.print("Motor ");
    Serial.print(motor_id);
    Serial.println(" stall detect timed out");
    return false;
}

bool calibrateLocalMotorToBackoffHold(uint8_t motor_id, int32_t& post50_position) {
    int32_t stall_position = 0;
    if (!runMotorUntilRateDropStall(motor_id, stall_position)) {
        return false;
    }

    int32_t target_position = stall_position - STALL_CALIBRATION_BACKOFF_COUNTS;
    startPositionMove(motor_id, target_position, STALL_CALIBRATION_BACKOFF_TOLERANCE, true);

    if (!waitForMotorHold(motor_id, target_position, STALL_CALIBRATION_BACKOFF_TOLERANCE)) {
        Serial.print("Motor ");
        Serial.print(motor_id);
        Serial.println(" 50-count hold timed out");
        return false;
    }

    post50_position = getMotorPositionForCalibration(motor_id);
    Serial.print("C2 M");
    Serial.print(motor_id);
    Serial.print(" hold50=");
    Serial.println(post50_position);
    return true;
}

void calibrateMotors21ToStallAndHold() {
    int32_t motor2_post50 = 0;
    int32_t motor1_post50 = 0;
    int32_t motor6_post50 = 0;
    int32_t motor5_post50 = 0;
    int32_t motor4_post50 = 0;
    int32_t motor3_post50 = 0;
    int32_t motor8_post50 = 0;
    int32_t motor7_post50 = 0;

    Serial.println("C2 starting: motors 2,1 then 6,5 then 4,3 then 8,7");

    if (!calibrateLocalMotorToBackoffHold(2, motor2_post50)) {
        Serial.println("C2 aborted on motor 2");
        return;
    }

    if (!calibrateLocalMotorToBackoffHold(1, motor1_post50)) {
        Serial.println("C2 aborted on motor 1");
        return;
    }

    int32_t motor2_final_target = motor2_post50 - CAL2_FINAL_BACKOFF_COUNTS;
    int32_t motor1_final_target = motor1_post50 - CAL2_FINAL_BACKOFF_COUNTS;

    startPositionMove(2, motor2_final_target, STALL_CALIBRATION_BACKOFF_TOLERANCE, true);
    startPositionMove(1, motor1_final_target, STALL_CALIBRATION_BACKOFF_TOLERANCE, true);

    bool motor2_ok =
        waitForMotorHold(2, motor2_final_target, STALL_CALIBRATION_BACKOFF_TOLERANCE);
    bool motor1_ok =
        waitForMotorHold(1, motor1_final_target, STALL_CALIBRATION_BACKOFF_TOLERANCE);

    if (!motor2_ok || !motor1_ok) {
        Serial.println("C2 final hold timed out on motors 2/1");
        return;
    }

    Serial.print("C2 M2 final=");
    Serial.println(getMotorPositionForCalibration(2));
    Serial.print("C2 M1 final=");
    Serial.println(getMotorPositionForCalibration(1));

    if (!calibrateLocalMotorToBackoffHold(6, motor6_post50)) {
        Serial.println("C2 aborted on motor 6");
        return;
    }

    if (!calibrateLocalMotorToBackoffHold(5, motor5_post50)) {
        Serial.println("C2 aborted on motor 5");
        return;
    }

    int32_t motor6_final_target = motor6_post50 - CAL2_FINAL_BACKOFF_COUNTS;
    int32_t motor5_final_target = motor5_post50 - CAL2_FINAL_BACKOFF_COUNTS;

    startPositionMove(6, motor6_final_target, STALL_CALIBRATION_BACKOFF_TOLERANCE, true);
    startPositionMove(5, motor5_final_target, STALL_CALIBRATION_BACKOFF_TOLERANCE, true);

    bool motor6_ok =
        waitForMotorHold(6, motor6_final_target, STALL_CALIBRATION_BACKOFF_TOLERANCE);
    bool motor5_ok =
        waitForMotorHold(5, motor5_final_target, STALL_CALIBRATION_BACKOFF_TOLERANCE);

    if (!motor6_ok || !motor5_ok) {
        Serial.println("C2 final hold timed out on motors 6/5");
        return;
    }

    Serial.print("C2 M6 final=");
    Serial.println(getMotorPositionForCalibration(6));
    Serial.print("C2 M5 final=");
    Serial.println(getMotorPositionForCalibration(5));

    if (!calibrateLocalMotorToBackoffHold(4, motor4_post50)) {
        Serial.println("C2 aborted on motor 4");
        return;
    }

    if (!calibrateLocalMotorToBackoffHold(3, motor3_post50)) {
        Serial.println("C2 aborted on motor 3");
        return;
    }

    int32_t motor4_final_target = motor4_post50 - CAL2_FINAL_BACKOFF_COUNTS;
    int32_t motor3_final_target = motor3_post50 - CAL2_FINAL_BACKOFF_COUNTS;

    startPositionMove(4, motor4_final_target, STALL_CALIBRATION_BACKOFF_TOLERANCE, true);
    startPositionMove(3, motor3_final_target, STALL_CALIBRATION_BACKOFF_TOLERANCE, true);

    bool motor4_ok =
        waitForMotorHold(4, motor4_final_target, STALL_CALIBRATION_BACKOFF_TOLERANCE);
    bool motor3_ok =
        waitForMotorHold(3, motor3_final_target, STALL_CALIBRATION_BACKOFF_TOLERANCE);

    if (!motor4_ok || !motor3_ok) {
        Serial.println("C2 final hold timed out on motors 4/3");
        return;
    }

    Serial.print("C2 M4 final=");
    Serial.println(getMotorPositionForCalibration(4));
    Serial.print("C2 M3 final=");
    Serial.println(getMotorPositionForCalibration(3));

    if (!calibrateLocalMotorToBackoffHold(8, motor8_post50)) {
        Serial.println("C2 aborted on motor 8");
        return;
    }

    if (!calibrateLocalMotorToBackoffHold(7, motor7_post50)) {
        Serial.println("C2 aborted on motor 7");
        return;
    }

    int32_t motor8_final_target = motor8_post50 - CAL2_FINAL_BACKOFF_COUNTS;
    int32_t motor7_final_target = motor7_post50 - CAL2_FINAL_BACKOFF_COUNTS;

    startPositionMove(8, motor8_final_target, STALL_CALIBRATION_BACKOFF_TOLERANCE, true);
    startPositionMove(7, motor7_final_target, STALL_CALIBRATION_BACKOFF_TOLERANCE, true);

    bool motor8_ok =
        waitForMotorHold(8, motor8_final_target, STALL_CALIBRATION_BACKOFF_TOLERANCE);
    bool motor7_ok =
        waitForMotorHold(7, motor7_final_target, STALL_CALIBRATION_BACKOFF_TOLERANCE);

    if (!motor8_ok || !motor7_ok) {
        Serial.println("C2 final hold timed out on motors 8/7");
        return;
    }

    Serial.print("C2 M8 final=");
    Serial.println(getMotorPositionForCalibration(8));
    Serial.print("C2 M7 final=");
    Serial.println(getMotorPositionForCalibration(7));
    Serial.println("C2 complete");
}

void printHelp() {
    Serial.println("Available commands:");
    Serial.println("  h             - Show this help");
    Serial.println("  cf            - Calibrate encoder direction on motors 1-8");
    Serial.println("  c2            - Calibrate motors 2 then 1 with stall/backoff/hold");
    Serial.println("  j<id>f        - Jog motor 1-8 forward");
    Serial.println("  j<id>b        - Jog motor 1-8 backward");
    Serial.println("  j<id><f|b>,<counts>,<tol> - Move motor relative by encoder counts");
    Serial.println("  p<id>,<pos>,<tol> - Move motor to encoder position");
    Serial.println("  p<id>,<pos>,<tol>;<id>,<pos>,<tol> - Move multiple motors together");
    Serial.println("  s<id>         - Stop motor 1-8");
    Serial.println("  sa            - Stop all motors");
    Serial.println("  e             - Show encoder counts for motors 1-8");
    Serial.println("  ra            - Reset all encoder counts to 0");
    Serial.println("  mz            - Move all motors to encoder position 0 +/- 5");
}

void setup() {
    Serial.begin(115200);

    driver1.init();
    driver2.init();
    motor1.init();
    motor2.init();
    motor3.init();
    motor4.init();

    uart_comm.init();

    Serial.println("====================================");
    Serial.println("PRIMARY ESP32 - Robot Hand Control");
    Serial.println("====================================");
    Serial.println("UART link: Serial2 RX=4 TX=23");
    printHelp();
}

void loop() {
    updateRemoteEncoderStates();
    updateJogStops();
    updateLocalPositionMoves();
    updateLocalMotorSafety();

    if (Serial.available() > 0) {
        String input = Serial.readStringUntil('\n');
        input.trim();

        if (input.length() > 0) {
            char cmd = input.charAt(0);

            if (cmd == 'j' && input.length() > 2) {
                JogMoveInput jog_command;
                if (parseJogCommand(input, jog_command)) {
                    if (jog_command.has_position_target) {
                        startJogMove(
                            jog_command.motor_id,
                            jog_command.forward,
                            jog_command.counts,
                            jog_command.tolerance);
                    } else {
                        startJog(jog_command.motor_id, jog_command.forward);
                    }
                } else {
                    Serial.println("Use j<id>f, j<id>b, or j<id><f|b>,<counts>,<tolerance>");
                }
            } else if (cmd == 'p') {
                if (!handlePositionCommands(input)) {
                    Serial.println("Use p<id>,<position>,<tolerance> or separate multiple moves with ;");
                }
            } else if (cmd == 's' && input.length() > 1) {
                if (input[1] == 'a') {
                    motor1.stop();
                    motor2.stop();
                    motor3.stop();
                    motor4.stop();
                    sendStopCommand(0);
                    for (uint8_t i = 0; i < 8; i++) {
                        jog_states[i].active = false;
                        position_moves[i].active = false;
                    }
                    Serial.println("All motors stopped");
                } else {
                    uint8_t motor_id = input.substring(1).toInt();
                    if (motor_id >= 1 && motor_id <= 8) {
                        stopMotorById(motor_id);
                        jog_states[motor_id - 1].active = false;
                        clearPositionMove(motor_id);
                        Serial.print("Motor ");
                        Serial.print(motor_id);
                        Serial.println(" stopped");
                    } else {
                        Serial.println("Invalid motor ID. Use 1-8.");
                    }
                }
            } else if (cmd == 'r' && input.length() > 1 && input[1] == 'a') {
                motor1.resetPosition();
                motor2.resetPosition();
                motor3.resetPosition();
                motor4.resetPosition();
                for (uint8_t i = 0; i < 8; i++) {
                    jog_states[i].active = false;
                    position_moves[i].active = false;
                }

                secondary_motors[0].position = 0;
                secondary_motors[0].speed = 0;
                secondary_motors[1].position = 0;
                secondary_motors[1].speed = 0;
                secondary_motors[2].position = 0;
                secondary_motors[2].speed = 0;
                secondary_motors[3].position = 0;
                secondary_motors[3].speed = 0;

                sendResetEncoderCommand(0);
                Serial.println("All encoder counts reset to 0");
            } else if (cmd == 'm' && input.length() > 1 && input[1] == 'z') {
                moveAllMotorsToZero();
            } else if (cmd == 'e') {
                printAllEncoderCounts();
            } else if (cmd == 'c' && input.length() > 1 && input[1] == 'f') {
                calibrateEncoderDirections();
            } else if (cmd == 'c' && input.length() > 1 && input[1] == '2') {
                calibrateMotors21ToStallAndHold();
            } else if (cmd == 'h') {
                printHelp();
            }
        }
    }
}
