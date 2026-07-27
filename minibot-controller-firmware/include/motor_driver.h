#ifndef MOTOR_DRIVER_H
#define MOTOR_DRIVER_H

#include <Arduino.h>

class MotorDriver
{
public:

    MotorDriver(
        uint8_t pwmPin,
        uint8_t dirPin,
        uint8_t pwmChannel,
        bool forwardLevel,
        bool reverseLevel
    );

    void begin();

    void setSpeed(int speed);

    void stop();

    int getSpeed() const;

private:

    uint8_t pwmPin_;
    uint8_t dirPin_;
    uint8_t pwmChannel_;

    bool forwardLevel_;
    bool reverseLevel_;

    int currentSpeed_;
};

#endif