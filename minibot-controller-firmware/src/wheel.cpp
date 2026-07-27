#include "wheel.h"

Wheel::Wheel(
    MotorDriver& motor,
    Encoder& encoder,
    PIDController& pid)
    :
      motor_(motor),
      encoder_(encoder),
      pid_(pid),
      targetVelocity_(0.0),
      enabled_(true)
{
}

void Wheel::begin()
{
    targetVelocity_ = 0.0;
    enabled_ = true;
    pid_.reset();
}

void Wheel::update(double dt)
{
    encoder_.update();

    if (!enabled_)
    {
        motor_.stop();
        return;
    }

    const double currentVelocity = encoder_.getVelocity();

    const double pwm =
        pid_.compute(
            targetVelocity_,
            currentVelocity,
            dt);

    motor_.setSpeed(static_cast<int>(pwm));
}

void Wheel::stop()
{
    targetVelocity_ = 0.0;

    pid_.reset();

    motor_.stop();
}

void Wheel::enable()
{
    pid_.reset();

    targetVelocity_ = 0.0;

    enabled_ = true;
}

void Wheel::disable()
{
    enabled_ = false;

    stop();
}

bool Wheel::isEnabled() const
{
    return enabled_;
}

void Wheel::setTargetVelocity(double targetVelocity)
{
    targetVelocity_ = targetVelocity;
}

double Wheel::getTargetVelocity() const
{
    return targetVelocity_;
}

double Wheel::getCurrentVelocity() const
{
    return encoder_.getVelocity();
}

double Wheel::getCurrentPosition() const
{
    return encoder_.getPosition();
}

int Wheel::getCurrentPwm() const
{
    return motor_.getSpeed();
}
