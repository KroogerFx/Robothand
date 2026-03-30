#ifndef ENCODER_H
#define ENCODER_H

#include <Arduino.h>

/**
 * Rotary Encoder Reader
 * Reads encoder feedback from motors
 */
class Encoder {
private:
    uint8_t pin_a;
    uint8_t pin_b;
    volatile int32_t pulse_count;
    unsigned long last_read_time;
    int32_t last_pulse_count;
    
    // Static array to track encoder instances (max 4)
    static Encoder* instances[4];
    uint8_t instance_id;
    
    // Private interrupt handlers
    void handleInterruptA();
    
public:
    Encoder(uint8_t pin_a, uint8_t pin_b);
    
    void init();
    
    /**
     * Get the current pulse count
     */
    int32_t getPulseCount() const;
    
    /**
     * Reset the pulse count
     */
    void resetPulseCount();
    
    /**
     * Get speed in pulses per second (call periodically)
     */
    int32_t getSpeed();

private:
    // Static ISR wrappers - one for each possible encoder
    static void ISR_EncoderA_0();
    static void ISR_EncoderA_1();
    static void ISR_EncoderA_2();
    static void ISR_EncoderA_3();
};

#endif
