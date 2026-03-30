#include "Encoder.h"

// Static array initialization
Encoder* Encoder::instances[4] = {nullptr, nullptr, nullptr, nullptr};

Encoder::Encoder(uint8_t pin_a, uint8_t pin_b)
    : pin_a(pin_a), pin_b(pin_b), pulse_count(0),
      last_read_time(millis()), last_pulse_count(0), instance_id(0) {
}

void Encoder::init() {
    // GPIO34-39 are input-only on ESP32 and do not support internal pull-ups.
    // Use plain inputs here so the same code works across valid encoder pins;
    // provide external pull-ups on boards that need them.
    pinMode(pin_a, INPUT);
    pinMode(pin_b, INPUT);
    
    // Find an available instance slot
    for (uint8_t i = 0; i < 4; i++) {
        if (instances[i] == nullptr) {
            instances[i] = this;
            instance_id = i;
            break;
        }
    }
    
    // Attach interrupts using appropriate static ISR based on instance_id
    void (*isr_func)() = nullptr;
    switch (instance_id) {
        case 0: isr_func = ISR_EncoderA_0; break;
        case 1: isr_func = ISR_EncoderA_1; break;
        case 2: isr_func = ISR_EncoderA_2; break;
        case 3: isr_func = ISR_EncoderA_3; break;
    }
    
    if (isr_func) {
        attachInterrupt(digitalPinToInterrupt(pin_a), isr_func, CHANGE);
    }
}

void Encoder::handleInterruptA() {
    if (digitalRead(pin_b) == digitalRead(pin_a)) {
        pulse_count++;
    } else {
        pulse_count--;
    }
}

// Static ISR wrappers
void Encoder::ISR_EncoderA_0() {
    if (instances[0]) instances[0]->handleInterruptA();
}

void Encoder::ISR_EncoderA_1() {
    if (instances[1]) instances[1]->handleInterruptA();
}

void Encoder::ISR_EncoderA_2() {
    if (instances[2]) instances[2]->handleInterruptA();
}

void Encoder::ISR_EncoderA_3() {
    if (instances[3]) instances[3]->handleInterruptA();
}

int32_t Encoder::getPulseCount() const {
    return pulse_count;
}

void Encoder::resetPulseCount() {
    pulse_count = 0;
}

int32_t Encoder::getSpeed() {
    unsigned long now = millis();
    unsigned long dt = now - last_read_time;
    
    if (dt >= 100) {  // Update speed every 100ms
        int32_t delta = pulse_count - last_pulse_count;
        last_pulse_count = pulse_count;
        last_read_time = now;
        
        // Return pulses per second
        return (delta * 1000) / dt;
    }
    
    return 0;
}
