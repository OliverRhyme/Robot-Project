#ifndef MOTOR_H
#define MOTOR_H

#include "pin_assign.h"
#include "pins.h"

class Motor {
public:
    Motor();
    void forward();
    void backward();
    void turnLeft();
    void turnRight();
    void stop();

    void leftWheel(uint8_t speed);
    void rightWheel(uint8_t speed);

    void setSpeed(uint8_t speed);

    // Call this in setup() function
    void init(int pwmLeft, int pwmRight, int dirLeft, int dirRight);

private:
    int pwmLeft_;
    int pwmRight_;
    int dirLeft_;
    int dirRight_;

    uint8_t speed_;
};

#endif  // MOTOR_H
