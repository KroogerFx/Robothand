#include "Motor.h"

Motor::Motor(DRV8833* driver, Encoder* encoder, uint8_t motor_index)
    : driver(driver), encoder(encoder), motor_index(motor_index),
      encoder_inverted(false), target_speed(0), target_position(0),
      position_control_enabled(false), stall_window_start_position(0),
      stall_window_start_time(0), stall_detected(false),
      stalled_direction(0) {
}

void Motor::init() {
    encoder->init();
    resetStallDetection();
}

void Motor::setSpeed(int16_t speed) {
    int8_t requested_direction = getDirection(speed);
    int8_t current_direction = getDirection(target_speed);
    bool was_stopped = (target_speed == 0);
    bool direction_changed =
        current_direction != 0 && requested_direction != 0 &&
        current_direction != requested_direction;

    if (stall_detected && requested_direction != 0 &&
        requested_direction == stalled_direction) {
        updateMotorSpeed(0);
        return;
    }

    target_speed = speed;

    if (speed == 0) {
        resetStallDetection();
    } else if (was_stopped || direction_changed) {
        resetStallDetection();
    }

    updateMotorSpeed(speed);
}

int16_t Motor::getSpeed() const {
    return target_speed;
}

int32_t Motor::getPosition() const {
    return applyEncoderDirection(encoder->getPulseCount());
}

int32_t Motor::getRawPosition() const {
    return encoder->getPulseCount();
}

void Motor::resetPosition() {
    encoder->resetPulseCount();
    resetStallDetection();
}

int32_t Motor::getEncoderSpeed() {
    return applyEncoderDirection(encoder->getSpeed());
}

int32_t Motor::getRawEncoderSpeed() {
    return encoder->getSpeed();
}

void Motor::stop() {
    setSpeed(0);
}

void Motor::setEncoderInverted(bool inverted) {
    encoder_inverted = inverted;
}

bool Motor::isEncoderInverted() const {
    return encoder_inverted;
}

bool Motor::updateSafety() {
    int32_t current_position = encoder->getPulseCount();
    int32_t movement = abs(current_position - stall_window_start_position);

    if (target_speed == 0) {
        resetStallDetection();
        return false;
    }

    if (movement >= STALL_MIN_MOVEMENT_COUNTS) {
        stall_window_start_position = current_position;
        stall_window_start_time = millis();
        stall_detected = false;
        stalled_direction = 0;
        return false;
    }

    if (stall_detected) {
        updateMotorSpeed(0);
        return false;
    }

    if (millis() - stall_window_start_time < STALL_TIMEOUT_MS) {
        return false;
    }

    stall_detected = true;
    stalled_direction = getDirection(target_speed);
    target_speed = 0;
    updateMotorSpeed(0);
    return true;
}

bool Motor::isStalled() const {
    return stall_detected;
}

void Motor::updateMotorSpeed(int16_t speed) {
    if (motor_index == 1) {
        driver->setMotor1Speed(speed);
    } else if (motor_index == 2) {
        driver->setMotor2Speed(speed);
    }
}

int32_t Motor::applyEncoderDirection(int32_t value) const {
    return encoder_inverted ? -value : value;
}

void Motor::resetStallDetection() {
    stall_window_start_position = encoder->getPulseCount();
    stall_window_start_time = millis();
    stall_detected = false;
    stalled_direction = 0;
}

int8_t Motor::getDirection(int16_t speed) {
    if (speed > 0) {
        return 1;
    }
    if (speed < 0) {
        return -1;
    }
    return 0;
}
