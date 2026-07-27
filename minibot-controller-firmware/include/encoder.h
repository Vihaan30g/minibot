#ifndef ENCODER_H
#define ENCODER_H

#include <Arduino.h>
#include <Wire.h>

class Encoder
{
public:

    // direction corrects for mirrored motor mounting: on a diff-drive
    // robot with the two motors facing each other, a rotation that drives
    // the robot straight forward spins the two motor shafts in opposite
    // handedness as seen by each motor's own encoder. Pass +1 or -1 so
    // that getPosition()/getVelocity() come out positive-for-forward for
    // BOTH wheels, in a consistent robot-frame convention - matching what
    // MotorDriver's forwardLevel_/reverseLevel_ already does for output.
    Encoder(
        TwoWire& wire,
        uint8_t sdaPin,
        uint8_t sclPin,
        double countsPerWheelRev,
        int8_t direction = 1
    );

    void begin();

    void update();

    void reset();

    double getPosition() const;

    double getVelocity() const;

    int64_t getCounts() const;

    uint16_t getRawAngle() const;

private:

    uint16_t readRawAngle();

    TwoWire& wire_;

    uint8_t sdaPin_;
    uint8_t sclPin_;

    double countsPerWheelRev_;

    int8_t direction_;

    uint16_t previousRaw_;

    int64_t totalCounts_;

    double position_;

    double previousPosition_;

    double velocity_;

    uint64_t previousTime_;
};

#endif