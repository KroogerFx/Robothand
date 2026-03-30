#ifndef DRV8833_H
#define DRV8833_H

#include <Arduino.h>

/**
 * DRV8833 Dual Motor Driver Controller
 * Controls 2 motors using PWM signals
 */
class DRV8833 {
private:
    static uint8_t next_pwm_channel;

    uint8_t motor1_pin_a;
    uint8_t motor1_pin_b;
    uint8_t motor2_pin_a;
    uint8_t motor2_pin_b;
    
    uint8_t pwm_channel_1a;
    uint8_t pwm_channel_1b;
    uint8_t pwm_channel_2a;
    uint8_t pwm_channel_2b;
    
    const uint16_t PWM_FREQ = 5000;  // 5kHz PWM frequency
    const uint8_t PWM_RESOLUTION = 8; // 8-bit resolution (0-255)

public:
    DRV8833(uint8_t motor1_a, uint8_t motor1_b, 
            uint8_t motor2_a, uint8_t motor2_b);
    
    void init();
    
    /**
     * Set motor speed and direction
     * speed: -255 to 255 (negative = reverse, positive = forward)
     */
    void setMotor1Speed(int16_t speed);
    void setMotor2Speed(int16_t speed);
    
    void stopMotor1();
    void stopMotor2();
    void stopAll();
    
    /**
     * Get current speed setting
     */
    int16_t getMotor1Speed() const;
    int16_t getMotor2Speed() const;

private:
    void setPWM(uint8_t pin_a, uint8_t pin_b, uint8_t channel_a, 
                uint8_t channel_b, int16_t speed);
    
    int16_t current_speed_motor1;
    int16_t current_speed_motor2;
};

#endif
