#pragma once
#include <Arduino.h>
#include <Motoron.h>

class MotoronDrive
{
public:
    MotoronDrive(uint8_t Front_Board, uint8_t Rear_Board);

    void begin();
    void stop();

    void set_front_left(int16_t speed);
    void set_front_right(int16_t speed);
    void set_rear_left(int16_t speed);
    void set_rear_right(int16_t speed);

    void set_all(int16_t fl, int16_t fr, int16_t rl, int16_t rr);
    void get_wheel_speeds(int16_t& fl, int16_t& fr, int16_t& rl, int16_t& rr) const;

    void drive(float vx, float vy, float w);
    void set_max_speed(int16_t maxSpeed);
    int16_t get_max_speed() const;

    void forward(int16_t speed);
    void backward(int16_t speed);
    void left(int16_t speed);
    void right(int16_t speed);

    void rotate_left(int16_t speed);
    void rotate_right(int16_t speed);

    void stop_all(); 
    void raw_front(int16_t motor1, int16_t motor2);
    void raw_rear(int16_t motor1, int16_t motor2);
    void raw_front_motor(uint8_t motor, int16_t speed, bool immediate);
    void raw_rear_motor(uint8_t motor, int16_t speed, bool immediate);
    void resend_current_speeds();
    void clear_status_flags();
    void print_status(Stream& output);

private:
    MotoronI2C front_;
    MotoronI2C rear_;

    int16_t maxSpeed_;
    int16_t currentFrontLeft_;
    int16_t currentFrontRight_;
    int16_t currentRearLeft_;
    int16_t currentRearRight_;

    void setup_controller(MotoronI2C& controller);
    void configure_motor_limits(MotoronI2C& controller, uint8_t motor);
    void clear_controller_status_flags(MotoronI2C& controller);
    void print_controller_status(Stream& output, const char* name, MotoronI2C& controller);
    void print_motor_status(Stream& output, MotoronI2C& controller, uint8_t motor);
    int16_t clamp_speed(int16_t speed) const;
};
