#include "encoder.h"
#include "config.h"

#include <math.h>

Encoder::Encoder(
    TwoWire& wire,
    uint8_t sdaPin,
    uint8_t sclPin,
    double countsPerWheelRev,
    int8_t direction)
    : wire_(wire),
      sdaPin_(sdaPin),
      sclPin_(sclPin),
      countsPerWheelRev_(countsPerWheelRev),
      direction_(direction),
      previousRaw_(0),
      totalCounts_(0),
      position_(0.0),
      previousPosition_(0.0),
      velocity_(0.0),
      previousTime_(0)
{
}

void Encoder::begin()
{
    wire_.begin(sdaPin_, sclPin_);

    // 400 kHz instead of the 100 kHz default: roughly 4x faster per I2C
    // transaction. Matters here because the control loop's whole budget is
    // 1ms (CONTROL_FREQUENCY = 1000 Hz) and both encoders get read every
    // cycle - this buys back headroom in that budget.
    wire_.setClock(400000);

    previousRaw_ = readRawAngle();

    // Define current wheel position as zero
    totalCounts_ = 0;

    position_ = 0.0;
    previousPosition_ = 0.0;
    velocity_ = 0.0;

    previousTime_ = micros();
}

uint16_t Encoder::readRawAngle()
{
    wire_.beginTransmission(AS5600_ADDRESS);
    wire_.write(AS5600_RAW_ANGLE_REGISTER);

    if (wire_.endTransmission(false) != 0)
    {
        return previousRaw_;
    }

    if (wire_.requestFrom(AS5600_ADDRESS, (uint8_t)2) != 2)
    {
        return previousRaw_;
    }

    uint8_t high = wire_.read();
    uint8_t low = wire_.read();

    return ((high << 8) | low) & (AS5600_RESOLUTION - 1);
}

void Encoder::update()
{
    uint16_t raw = readRawAngle();

    int32_t diff = static_cast<int32_t>(raw) -
                   static_cast<int32_t>(previousRaw_);

    // Unwrap the 0-4095 rollover FIRST, on the raw circular value - this
    // must happen before applying direction_, since the unwrap math only
    // makes sense in the sensor's own native raw coordinate system.
    if (diff > AS5600_HALF_RESOLUTION)
    {
        diff -= AS5600_RESOLUTION;
    }
    else if (diff < -AS5600_HALF_RESOLUTION)
    {
        diff += AS5600_RESOLUTION;
    }

    previousRaw_ = raw;

    // Now apply the mounting-direction correction, so totalCounts_ (and
    // everything derived from it) is already in the consistent
    // positive-is-forward robot-frame convention.
    totalCounts_ += diff * direction_;

    position_ =
        (TWO_PI * static_cast<double>(totalCounts_)) /
        countsPerWheelRev_;

    uint64_t currentTime = micros();

    double dt =
        static_cast<double>(currentTime - previousTime_) /
        1000000.0;

    if (dt > 0.0)
    {
        velocity_ =
            (position_ - previousPosition_) / dt;
    }

    previousPosition_ = position_;
    previousTime_ = currentTime;
}

void Encoder::reset()
{
    previousRaw_ = readRawAngle();

    totalCounts_ = 0;

    position_ = 0.0;
    previousPosition_ = 0.0;
    velocity_ = 0.0;

    previousTime_ = micros();
}

double Encoder::getPosition() const
{
    return position_;
}

double Encoder::getVelocity() const
{
    return velocity_;
}

int64_t Encoder::getCounts() const
{
    return totalCounts_;
}

uint16_t Encoder::getRawAngle() const
{
    return previousRaw_;
}