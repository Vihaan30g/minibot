#ifndef WHEEL_H
#define WHEEL_H

#include "motor_driver.h"
#include "encoder.h"
#include "pid_controller.h"

class Wheel
{
public:

    Wheel(
        MotorDriver& motor,
        Encoder& encoder,
        PIDController& pid
    );

    void begin();

    // dt (seconds) is now passed in explicitly by the caller (main.cpp's
    // fixed-period scheduler), rather than assumed internally, so the PID's
    // notion of dt always matches whatever period actually elapsed.
    void update(double dt);

    void stop();

    void enable();

    void disable();

    bool isEnabled() const;

    void setTargetVelocity(double targetVelocity);

    double getTargetVelocity() const;

    double getCurrentVelocity() const;

    double getCurrentPosition() const;

    // Current signed PWM duty actually being sent to the motor right now
    // (-255..255). Forwards straight through to MotorDriver::getSpeed().
    int getCurrentPwm() const;

private:

    MotorDriver& motor_;

    Encoder& encoder_;

    PIDController& pid_;

    double targetVelocity_;

    bool enabled_;
};

#endif
