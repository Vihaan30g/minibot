#include "motor_driver.h"
#include "config.h"

MotorDriver::MotorDriver(
    uint8_t pwmPin,
    uint8_t dirPin,
    uint8_t pwmChannel,
    bool forwardLevel,
    bool reverseLevel
)
    : pwmPin_(pwmPin),
      dirPin_(dirPin),
      pwmChannel_(pwmChannel),
      forwardLevel_(forwardLevel),
      reverseLevel_(reverseLevel),
      currentSpeed_(0)
{
}

void MotorDriver::begin()
{
    pinMode(dirPin_, OUTPUT);

    ledcSetup(
        pwmChannel_,
        PWM_FREQUENCY,
        PWM_RESOLUTION);

    ledcAttachPin(
        pwmPin_,
        pwmChannel_);

    stop();
}

void MotorDriver::setSpeed(int speed)
{
    speed = constrain(speed, -MAX_PWM, MAX_PWM);

    currentSpeed_ = speed;

    if (speed >= 0)
    {
        digitalWrite(dirPin_, forwardLevel_);
        ledcWrite(pwmChannel_, speed);
    }
    else
    {
        digitalWrite(dirPin_, reverseLevel_);
        ledcWrite(pwmChannel_, -speed);
    }
}

void MotorDriver::stop()
{
    currentSpeed_ = 0;
    ledcWrite(pwmChannel_, 0);
}

int MotorDriver::getSpeed() const
{
    return currentSpeed_;
}