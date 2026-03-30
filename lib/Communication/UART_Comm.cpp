#include "UART_Comm.h"

UART_Comm::UART_Comm(uint8_t rx_pin, uint8_t tx_pin, uint32_t baud_rate,
                     HardwareSerial* port)
    : rx_pin(rx_pin), tx_pin(tx_pin), baud_rate(baud_rate),
      uart_port(port), rx_index(0) {
}

void UART_Comm::init() {
    uart_port->begin(baud_rate, SERIAL_8N1, rx_pin, tx_pin);
}

void UART_Comm::sendCommand(const MotorCommand& cmd) {
    uint8_t packet[5];
    packet[0] = cmd.type;
    packet[1] = cmd.motor_id;
    packet[2] = (cmd.speed >> 8) & 0xFF;
    packet[3] = cmd.speed & 0xFF;
    packet[4] = calculateChecksum(packet, 4);
    
    uart_port->write(packet, 5);
}

bool UART_Comm::receiveCommand(MotorCommand& cmd) {
    if (!dataAvailable()) return false;

    if (rx_index == 0 && !isShortCommandType(uart_port->peek())) {
        return false;
    }
    
    while (uart_port->available()) {
        uint8_t byte = uart_port->read();
        rx_buffer[rx_index++] = byte;
         
        if (rx_index >= 5) {
            if (verifyChecksum(rx_buffer, 5) && isShortCommandType(rx_buffer[0])) {
                cmd.type = (CommandType)rx_buffer[0];
                cmd.motor_id = rx_buffer[1];
                cmd.speed = ((int16_t)rx_buffer[2] << 8) | rx_buffer[3];
                rx_index = 0;
                return true;
            }

            for (uint8_t i = 1; i < 5; i++) {
                rx_buffer[i - 1] = rx_buffer[i];
            }
            rx_index = 4;
        }
    }
    
    return false;
}

void UART_Comm::sendPositionCommand(const PositionCommand& cmd) {
    uint8_t packet[11];
    packet[0] = CMD_MOVE_TO_POSITION;
    packet[1] = cmd.motor_id;
    packet[2] = (cmd.target_position >> 24) & 0xFF;
    packet[3] = (cmd.target_position >> 16) & 0xFF;
    packet[4] = (cmd.target_position >> 8) & 0xFF;
    packet[5] = cmd.target_position & 0xFF;
    packet[6] = (cmd.tolerance >> 24) & 0xFF;
    packet[7] = (cmd.tolerance >> 16) & 0xFF;
    packet[8] = (cmd.tolerance >> 8) & 0xFF;
    packet[9] = cmd.tolerance & 0xFF;
    packet[10] = calculateChecksum(packet, 10);
    
    uart_port->write(packet, 11);
}

bool UART_Comm::receivePositionCommand(PositionCommand& cmd) {
    if (!dataAvailable()) return false;

    if (rx_index == 0 && uart_port->peek() != CMD_MOVE_TO_POSITION) {
        return false;
    }
    
    while (uart_port->available()) {
        uint8_t byte = uart_port->read();
        rx_buffer[rx_index++] = byte;
        
        if (rx_index >= 11) {
            if (verifyChecksum(rx_buffer, 11) &&
                rx_buffer[0] == CMD_MOVE_TO_POSITION) {
                cmd.motor_id = rx_buffer[1];
                cmd.target_position = ((int32_t)rx_buffer[2] << 24) |
                                      ((int32_t)rx_buffer[3] << 16) |
                                      ((int32_t)rx_buffer[4] << 8) |
                                      rx_buffer[5];
                cmd.tolerance = ((int32_t)rx_buffer[6] << 24) |
                                ((int32_t)rx_buffer[7] << 16) |
                                ((int32_t)rx_buffer[8] << 8) |
                                rx_buffer[9];
                rx_index = 0;
                return true;
            }

            for (uint8_t i = 1; i < 11; i++) {
                rx_buffer[i - 1] = rx_buffer[i];
            }
            rx_index = 10;
        }
    }

    return false;
}

void UART_Comm::sendEncoderData(const EncoderData& data) {
    uint8_t packet[11];
    packet[0] = RESP_ENCODER_DATA;
    packet[1] = data.motor_id;
    packet[2] = (data.position >> 24) & 0xFF;
    packet[3] = (data.position >> 16) & 0xFF;
    packet[4] = (data.position >> 8) & 0xFF;
    packet[5] = data.position & 0xFF;
    packet[6] = (data.speed >> 24) & 0xFF;
    packet[7] = (data.speed >> 16) & 0xFF;
    packet[8] = (data.speed >> 8) & 0xFF;
    packet[9] = data.speed & 0xFF;
    packet[10] = calculateChecksum(packet, 10);
    
    uart_port->write(packet, 11);
}

bool UART_Comm::receiveEncoderData(EncoderData& data) {
    if (!dataAvailable()) return false;

    if (rx_index == 0 && uart_port->peek() != RESP_ENCODER_DATA) {
        return false;
    }
    
    while (uart_port->available()) {
        uint8_t byte = uart_port->read();
        rx_buffer[rx_index++] = byte;
         
        if (rx_index >= 11) {
            if (verifyChecksum(rx_buffer, 11) &&
                rx_buffer[0] == RESP_ENCODER_DATA) {
                data.motor_id = rx_buffer[1];
                data.position = ((int32_t)rx_buffer[2] << 24) |
                               ((int32_t)rx_buffer[3] << 16) |
                               ((int32_t)rx_buffer[4] << 8) |
                               rx_buffer[5];
                data.speed = ((int32_t)rx_buffer[6] << 24) |
                             ((int32_t)rx_buffer[7] << 16) |
                             ((int32_t)rx_buffer[8] << 8) |
                             rx_buffer[9];
                rx_index = 0;
                return true;
            }

            for (uint8_t i = 1; i < 11; i++) {
                rx_buffer[i - 1] = rx_buffer[i];
            }
            rx_index = 10;
        }
    }
    
    return false;
}

bool UART_Comm::dataAvailable() const {
    return uart_port->available() > 0;
}

int UART_Comm::peekType() const {
    if (!dataAvailable()) {
        return -1;
    }
    return uart_port->peek();
}

void UART_Comm::discardByte() {
    if (dataAvailable()) {
        uart_port->read();
    }
}

uint8_t UART_Comm::calculateChecksum(const uint8_t* data, uint8_t length) const {
    uint8_t checksum = 0;
    for (uint8_t i = 0; i < length; i++) {
        checksum ^= data[i];
    }
    return checksum;
}

bool UART_Comm::verifyChecksum(const uint8_t* data, uint8_t length) const {
    uint8_t checksum = 0;
    for (uint8_t i = 0; i < length - 1; i++) {
        checksum ^= data[i];
    }
    return checksum == data[length - 1];
}

bool UART_Comm::isValidCommandType(uint8_t type) const {
    return type == CMD_SET_MOTOR_SPEED ||
           type == CMD_GET_ENCODER_DATA ||
           type == CMD_STOP_MOTOR ||
           type == CMD_MOVE_TO_POSITION ||
           type == CMD_CALIBRATE_ENCODER_DIRECTION ||
           type == CMD_RUN_UNTIL_STALL ||
           type == CMD_RESET_ENCODER ||
           type == CMD_ACK;
}

bool UART_Comm::isShortCommandType(uint8_t type) const {
    return type == CMD_SET_MOTOR_SPEED ||
           type == CMD_GET_ENCODER_DATA ||
           type == CMD_STOP_MOTOR ||
           type == CMD_CALIBRATE_ENCODER_DIRECTION ||
           type == CMD_RUN_UNTIL_STALL ||
           type == CMD_RESET_ENCODER ||
           type == CMD_ACK;
}
