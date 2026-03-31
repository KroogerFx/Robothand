#include "Finger.h"

Finger::Finger(
    uint8_t finger_id,
    uint8_t proximal_motor_id,
    uint8_t distal_motor_id,
    FingerMoveCallback move_callback)
    : finger_id(finger_id),
      proximal_motor_id(proximal_motor_id),
      distal_motor_id(distal_motor_id),
      move_callback(move_callback) {
}

uint8_t Finger::getFingerId() const {
    return finger_id;
}

uint8_t Finger::getProximalMotorId() const {
    return proximal_motor_id;
}

uint8_t Finger::getDistalMotorId() const {
    return distal_motor_id;
}

void Finger::moveToPositions(int32_t proximal_target, int32_t distal_target, int32_t tolerance) const {
    if (move_callback == nullptr) {
        return;
    }

    move_callback(proximal_motor_id, proximal_target, tolerance);
    move_callback(distal_motor_id, distal_target, tolerance);
}
