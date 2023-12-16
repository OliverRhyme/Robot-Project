#include <Arduino_FreeRTOS.h>
#include <PID_v1.h>
#include <avr/interrupt.h>
#include <util/delay.h>

typedef bool boolean;

#include "Servo.h"
#include "Ultrasonic.h"
#include "init.h"
#include "motor.h"
#include "pin_assign.h"
#include "pins.h"

Motor motor;
Servo servo;

Ultrasonic ultrasonic(TRIGGER_PIN, ECHO_PIN);

// Main function
int main(void) {
    init();

    initVariant();

    setup();

    for (;;) loop();

    return 0;
}

void sensorLoop(void *pvParameters);
void motorLoop(void *pvParameters);

double currentDistance;
double pidOut;

double targetDistance = 30.0;
double Kp = 5.0;  // Proportional gain
double Ki = 1.4;  // Integral gain
double Kd = 0.6;  // Derivative gain

const uint8_t defaultSpeed = 64;

double leftDistance;
double frontDistance;

PID myPID(&leftDistance, &pidOut, &targetDistance, Kp, Ki, Kd, DIRECT);

void setup(void) {
    setupPin(TRIGGER_PIN, OUTPUT);
    setupPin(ECHO_PIN, INPUT);

    motor.init(LEFT_MOTOR_PWM_PIN, RIGHT_MOTOR_PWM_PIN, LEFT_MOTOR_DIR_PIN, RIGHT_MOTOR_DIR_PIN);
    motor.setSpeed(defaultSpeed);

    servo.attach(SERVO_PIN);

    servo.write(180);

    // sensorLoop - loop that will be called
    // "Sensor Loop" - name of the task (for debugging)
    // 128 - Stack size in words
    // NULL - parameter to pass
    // 2 - priority
    // NULL - task handle
    xTaskCreate(sensorLoop, "Sensor Loop", 128, NULL, 2, NULL);
    xTaskCreate(motorLoop, "Motor Loop", 128, NULL, 1, NULL);

    myPID.SetOutputLimits(-(defaultSpeed - 40), (defaultSpeed - 40));
    myPID.SetMode(AUTOMATIC);
}

void loop(void) {
    // threadController.run();
}

unsigned int checkdistance() {
    return ultrasonic.read();
}

const float alpha = 0.8;  // Exponential filter coefficient

void sensorLoop(void *pvParameters) {
    for (;;) {
        if (servo.read() == 180) {
            leftDistance = alpha * checkdistance() + (1 - alpha) * leftDistance;
            servo.write(90);
            vTaskDelay(1);
        } else if (servo.read() == 90) {
            frontDistance = alpha * checkdistance() + (1 - alpha) * frontDistance;
            servo.write(180);
            vTaskDelay(1);
        }
    }
}

void motorLoop(void *pvParameters) {
    for (;;) {
        myPID.Compute();

        if (pidOut != 0) {
            motor.leftWheel(defaultSpeed + pidOut);
            motor.rightWheel(defaultSpeed);
        } else {
            motor.forward();
        }
    }
}
