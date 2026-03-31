#ifndef FINGER_H
#define FINGER_H

#include <Arduino.h>

typedef void (*FingerMoveCallback)(uint8_t motor_id, int32_t target, int32_t tolerance);

class Finger {
private:
    uint8_t finger_id;
    uint8_t proximal_motor_id;
    uint8_t distal_motor_id;
    FingerMoveCallback move_callback;

public:
    Finger(
        uint8_t finger_id,
        uint8_t proximal_motor_id,
        uint8_t distal_motor_id,
        FingerMoveCallback move_callback);

    uint8_t getFingerId() const;
    uint8_t getProximalMotorId() const;
    uint8_t getDistalMotorId() const;
    void moveToPositions(int32_t proximal_target, int32_t distal_target, int32_t tolerance) const;
};

#endif
