#ifndef PID_CONTROLLER_H
#define PID_CONTROLLER_H

class PIDController
{
public:

    PIDController(
        double kp,
        double ki,
        double kd
    );

    void reset();

    double compute(
        double setpoint,
        double measurement,
        double dt
    );

    void setTunings(
        double kp,
        double ki,
        double kd
    );

    void setOutputLimits(
        double outputMin,
        double outputMax
    );

    void setIntegralLimits(
        double integralMin,
        double integralMax
    );

private:

    double kp_;
    double ki_;
    double kd_;

    double integral_;

    double previousMeasurement_;

    double outputMin_;
    double outputMax_;

    double integralMin_;
    double integralMax_;
};

#endif