#include "pid_controller.h"
#include "config.h"

PIDController::PIDController(
    double kp,
    double ki,
    double kd)
    :
      kp_(kp),
      ki_(ki),
      kd_(kd),
      integral_(0.0),
      previousMeasurement_(0.0),
      outputMin_(MOTOR_PWM_MIN),
      outputMax_(MOTOR_PWM_MAX),
      integralMin_(PID_INTEGRAL_MIN),
      integralMax_(PID_INTEGRAL_MAX)
{
}

void PIDController::reset()
{
    integral_ = 0.0;
    previousMeasurement_ = 0.0;
}

double PIDController::compute(
    double setpoint,
    double measurement,
    double dt)
{
    if (dt <= 0.0)
    {
        return 0.0;
    }

    // Compute error
    const double error = setpoint - measurement;

    // Integral term
    integral_ += error * dt;

    // Clamp integral (anti-windup)
    if (integral_ > integralMax_)
    {
        integral_ = integralMax_;
    }
    else if (integral_ < integralMin_)
    {
        integral_ = integralMin_;
    }

    // Derivative on measurement
    const double derivative =
        -(measurement - previousMeasurement_) / dt;

    // PID output
    double output =
          kp_ * error
        + ki_ * integral_
        + kd_ * derivative;

    // Clamp output
    if (output > outputMax_)
    {
        output = outputMax_;
    }
    else if (output < outputMin_)
    {
        output = outputMin_;
    }

    previousMeasurement_ = measurement;

    return output;
}

void PIDController::setTunings(
    double kp,
    double ki,
    double kd)
{
    kp_ = kp;
    ki_ = ki;
    kd_ = kd;
}

void PIDController::setOutputLimits(
    double outputMin,
    double outputMax)
{
    outputMin_ = outputMin;
    outputMax_ = outputMax;
}

void PIDController::setIntegralLimits(
    double integralMin,
    double integralMax)
{
    integralMin_ = integralMin;
    integralMax_ = integralMax;
}
