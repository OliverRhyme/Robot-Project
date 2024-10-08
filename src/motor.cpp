#include "motor.h"

#include <Arduino_FreeRTOS.h>

#include "math_util.h"

Motor::Motor(){};

void Motor::init(int pwmLeft, int pwmRight, int dirLeft, int dirRight) {
    pwmLeft_ = pwmLeft;
    pwmRight_ = pwmRight;
    dirLeft_ = dirLeft;
    dirRight_ = dirRight;

    setupPin(pwmLeft_, OUTPUT);
    setupPin(pwmRight_, OUTPUT);
    setupPin(dirLeft_, OUTPUT);
    setupPin(dirRight_, OUTPUT);

    speed_ = 128;  // 50% duty cycle
}

void Motor::forward() {
    pwmWrite(pwmLeft_, speed_);
    pwmWrite(pwmRight_, speed_);

    writePin(dirLeft_, HIGH);
    writePin(dirRight_, LOW);
}

void Motor::backward() {
    pwmWrite(pwmLeft_, speed_);
    pwmWrite(pwmRight_, speed_);

    writePin(dirLeft_, LOW);
    writePin(dirRight_, HIGH);
}

void Motor::turnLeft() {
    pwmWrite(pwmLeft_, speed_);
    pwmWrite(pwmRight_, speed_);

    writePin(dirLeft_, LOW);
    writePin(dirRight_, LOW);
}

void Motor::turnRight() {
    pwmWrite(pwmLeft_, speed_);
    pwmWrite(pwmRight_, speed_);

    writePin(dirLeft_, HIGH);
    writePin(dirRight_, HIGH);
}

void Motor::stop() {
    pwmWrite(pwmLeft_, 0);
    pwmWrite(pwmRight_, 0);
}

void Motor::setSpeed(uint8_t speed) {
    speed_ = speed;
}

void Motor::leftWheel(uint8_t speed) {
    if (speed > 0) {
        pwmWrite(pwmLeft_, speed);
        writePin(dirLeft_, HIGH);
    } else {
        pwmWrite(pwmLeft_, -speed);
        writePin(dirLeft_, LOW);
    }
}

void Motor::rightWheel(uint8_t speed) {
    if (speed > 0) {
        pwmWrite(pwmRight_, speed);
        writePin(dirRight_, LOW);
    } else {
        pwmWrite(pwmRight_, -speed);
        writePin(dirRight_, HIGH);
    }
}

void Motor::turn(int8_t angle) {
    if (angle > 0) {
        turnRight();
    } else if (angle < 0) {
        turnLeft();
    }
    if (angle != 0) {
        delay(abs(angle) * speed_ * 0.107);
    }

    stop();
}

void Motor::forward(uint8_t distance) {
    forward();
    delay(distance * 10);
    stop();
}

void Motor::backward(uint8_t distance) {
    backward();
    delay(distance * 10);
    stop();
}