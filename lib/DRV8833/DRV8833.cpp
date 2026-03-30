#include "DRV8833.h"

uint8_t DRV8833::next_pwm_channel = 0;

DRV8833::DRV8833(uint8_t motor1_a, uint8_t motor1_b,
                 uint8_t motor2_a, uint8_t motor2_b)
    : motor1_pin_a(motor1_a), motor1_pin_b(motor1_b),
      motor2_pin_a(motor2_a), motor2_pin_b(motor2_b),
      pwm_channel_1a(next_pwm_channel++), pwm_channel_1b(next_pwm_channel++),
      pwm_channel_2a(next_pwm_channel++), pwm_channel_2b(next_pwm_channel++),
      current_speed_motor1(0), current_speed_motor2(0) {
}

void DRV8833::init() {
    // Configure PWM channels for ESP32
    pinMode(motor1_pin_a, OUTPUT);
    pinMode(motor1_pin_b, OUTPUT);
    pinMode(motor2_pin_a, OUTPUT);
    pinMode(motor2_pin_b, OUTPUT);
    
    ledcSetup(pwm_channel_1a, PWM_FREQ, PWM_RESOLUTION);
    ledcSetup(pwm_channel_1b, PWM_FREQ, PWM_RESOLUTION);
    ledcSetup(pwm_channel_2a, PWM_FREQ, PWM_RESOLUTION);
    ledcSetup(pwm_channel_2b, PWM_FREQ, PWM_RESOLUTION);
    
    ledcAttachPin(motor1_pin_a, pwm_channel_1a);
    ledcAttachPin(motor1_pin_b, pwm_channel_1b);
    ledcAttachPin(motor2_pin_a, pwm_channel_2a);
    ledcAttachPin(motor2_pin_b, pwm_channel_2b);
    
    stopAll();
}

void DRV8833::setMotor1Speed(int16_t speed) {
    speed = constrain(speed, -255, 255);
    current_speed_motor1 = speed;
    setPWM(motor1_pin_a, motor1_pin_b, pwm_channel_1a, pwm_channel_1b, speed);
}

void DRV8833::setMotor2Speed(int16_t speed) {
    speed = constrain(speed, -255, 255);
    current_speed_motor2 = speed;
    setPWM(motor2_pin_a, motor2_pin_b, pwm_channel_2a, pwm_channel_2b, speed);
}

void DRV8833::stopMotor1() {
    setMotor1Speed(0);
}

void DRV8833::stopMotor2() {
    setMotor2Speed(0);
}

void DRV8833::stopAll() {
    stopMotor1();
    stopMotor2();
}

int16_t DRV8833::getMotor1Speed() const {
    return current_speed_motor1;
}

int16_t DRV8833::getMotor2Speed() const {
    return current_speed_motor2;
}

void DRV8833::setPWM(uint8_t pin_a, uint8_t pin_b, uint8_t channel_a,
                     uint8_t channel_b, int16_t speed) {
    if (speed > 0) {
        // Forward
        ledcWrite(channel_a, speed);
        ledcWrite(channel_b, 0);
    } else if (speed < 0) {
        // Reverse
        ledcWrite(channel_a, 0);
        ledcWrite(channel_b, -speed);
    } else {
        // Stop
        ledcWrite(channel_a, 0);
        ledcWrite(channel_b, 0);
    }
}
