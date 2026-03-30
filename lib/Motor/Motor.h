#ifndef MOTOR_H
#define MOTOR_H

#include <Arduino.h>
#include "../DRV8833/DRV8833.h"
#include "../Encoder/Encoder.h"

/**
 * Single N20 Motor with Encoder Feedback
 * Manages speed, position, and encoder data
 */
class Motor {
private:
    static const unsigned long STALL_TIMEOUT_MS = 100;
    static const int32_t STALL_MIN_MOVEMENT_COUNTS = 50;

    DRV8833* driver;
    Encoder* encoder;
    uint8_t motor_index;  // 1 or 2
    bool encoder_inverted;
    
    int16_t target_speed;
    int32_t target_position;
    bool position_control_enabled;
    int32_t stall_window_start_position;
    unsigned long stall_window_start_time;
    bool stall_detected;
    int8_t stalled_direction;

public:
    Motor(DRV8833* driver, Encoder* encoder, uint8_t motor_index);
    
    void init();
    
    /**
     * Set motor speed (-255 to 255)
     */
    void setSpeed(int16_t speed);
    
    /**
     * Get current speed
     */
    int16_t getSpeed() const;
    
    /**
     * Get encoder pulse count
     */
    int32_t getPosition() const;
    int32_t getRawPosition() const;
    
    /**
     * Reset encoder position
     */
    void resetPosition();
    
    /**
     * Get speed in pulses per second
     */
    int32_t getEncoderSpeed();
    int32_t getRawEncoderSpeed();
    
    /**
     * Stop the motor
     */
    void stop();
    void setEncoderInverted(bool inverted);
    bool isEncoderInverted() const;
    bool updateSafety();
    bool isStalled() const;

private:
    void updateMotorSpeed(int16_t speed);
    int32_t applyEncoderDirection(int32_t value) const;
    void resetStallDetection();
    static int8_t getDirection(int16_t speed);
};

#endif
