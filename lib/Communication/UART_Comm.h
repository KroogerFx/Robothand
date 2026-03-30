#ifndef UART_COMM_H
#define UART_COMM_H

#include <Arduino.h>

/**
 * UART Communication Protocol
 * Handles communication between Primary and Secondary ESP32
 */
class UART_Comm {
private:
    uint8_t rx_pin;
    uint8_t tx_pin;
    uint32_t baud_rate;
    HardwareSerial* uart_port;
    
    static const uint8_t BUFFER_SIZE = 128;
    uint8_t rx_buffer[BUFFER_SIZE];
    uint8_t rx_index;

public:
    // Command types
    enum CommandType {
        CMD_SET_MOTOR_SPEED = 0x01,
        CMD_GET_ENCODER_DATA = 0x02,
        CMD_STOP_MOTOR = 0x03,
        CMD_RESET_ENCODER = 0x04,
        CMD_MOVE_TO_POSITION = 0x05,
        CMD_CALIBRATE_ENCODER_DIRECTION = 0x06,
        CMD_RUN_UNTIL_STALL = 0x07,
        RESP_ENCODER_DATA = 0x81,
        CMD_ACK = 0xFF
    };

    struct MotorCommand {
        CommandType type;
        uint8_t motor_id;
        int16_t speed;
    };

    struct EncoderData {
        uint8_t motor_id;
        int32_t position;
        int32_t speed;
    };

    struct PositionCommand {
        uint8_t motor_id;
        int32_t target_position;
        int32_t tolerance;
    };

    UART_Comm(uint8_t rx_pin, uint8_t tx_pin, uint32_t baud_rate, 
             HardwareSerial* port = &Serial1);
    
    void init();
    
    /**
     * Send motor command
     */
    void sendCommand(const MotorCommand& cmd);
    
    /**
     * Receive motor command
     */
    bool receiveCommand(MotorCommand& cmd);
    void sendPositionCommand(const PositionCommand& cmd);
    bool receivePositionCommand(PositionCommand& cmd);
    
    /**
     * Send encoder data
     */
    void sendEncoderData(const EncoderData& data);
    
    /**
     * Receive encoder data
     */
    bool receiveEncoderData(EncoderData& data);
    
    /**
     * Check if data is available
     */
    bool dataAvailable() const;
    int peekType() const;
    void discardByte();

private:
    uint8_t calculateChecksum(const uint8_t* data, uint8_t length) const;
    bool verifyChecksum(const uint8_t* data, uint8_t length) const;
    bool isValidCommandType(uint8_t type) const;
    bool isShortCommandType(uint8_t type) const;
};

#endif
