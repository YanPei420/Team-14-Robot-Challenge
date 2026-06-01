#pragma once

#include <Arduino.h>

class Encoder
{
public:
    Encoder(uint8_t pin1, uint8_t pin2)
        : pin1_(pin1),
          pin2_(pin2),
          position_(0)
    {
        pinMode(pin1_, INPUT_PULLUP);
        pinMode(pin2_, INPUT_PULLUP);
        state_ = readState();
    }

    long read()
    {
        update();
        return position_;
    }

    void write(long position)
    {
        position_ = position;
        state_ = readState();
    }

private:
    uint8_t pin1_;
    uint8_t pin2_;
    uint8_t state_;
    long position_;

    uint8_t readState() const
    {
        return (digitalRead(pin1_) << 1) | digitalRead(pin2_);
    }

    void update()
    {
        const int8_t table[] =
        {
            0, -1, 1, 0,
            1, 0, 0, -1,
            -1, 0, 0, 1,
            0, 1, -1, 0
        };
        const uint8_t now = readState();
        position_ += table[(state_ << 2) | now];
        state_ = now;
    }
};
