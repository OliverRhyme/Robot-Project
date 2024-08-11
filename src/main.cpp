#include <Arduino_FreeRTOS.h>
#include <PID_v1.h>
#include <avr/interrupt.h>
// #include <util/delay.h>

typedef bool boolean;

#include "HCSR04.h"

#include "Servo.h"
#include "init.h"
#include "motor.h"
#include "pin_assign.h"
#include "pins.h"

Motor motor;
Servo servo;

UltraSonicDistanceSensor ultrasonic(TRIGGER_PIN, ECHO_PIN);

void sensorLoop(void *pvParameters);
void motorLoop(void *pvParameters);

// Main function
int main(void) {
    init();

    initVariant();

    setup();

    for (;;) loop();

    return 0;
}

double targetDistance = 10.0;
double Kp = 2.0;  // Proportional gain
double Ki = 0.4;  // Integral gain
double Kd = 0.6;  // Derivative gain

const uint8_t defaultSpeed = 64;

double leftDistance;
double frontDistance;

double leftPIDOut;
double frontPIDOut;

PID leftPID(&leftDistance, &leftPIDOut, &targetDistance, Kp, Ki, Kd, DIRECT);
PID frontPID(&frontDistance, &frontPIDOut, &targetDistance, Kp, Ki, Kd, DIRECT);

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
    // xTaskCreate(motorLoop, "Motor Loop", 128, NULL, 1, NULL);

    leftPID.SetOutputLimits(-(defaultSpeed - 40), (defaultSpeed - 40));
    frontPID.SetOutputLimits(-(defaultSpeed - 50), (defaultSpeed - 50));
    leftPID.SetMode(AUTOMATIC);
    frontPID.SetMode(AUTOMATIC);

    Serial.begin(9600);
}

unsigned int checkdistance() {
    unsigned int value = ultrasonic.measureDistanceCm();
    return value;
}

void loop(void) {
    // threadController.run();
}

// Exponential filter function
void exponentialFilter(double *value, double newValue, double alpha = 0.8) {
    *value = alpha * (newValue) + (1 - alpha) * (*value);
}

void sensorLoop(void *pvParameters) {
    for (;;) {
        if (servo.read() == 180) {
            exponentialFilter(&leftDistance, checkdistance());

            servo.write(90);
            delay(1000);

        } else if (servo.read() == 90) {
            exponentialFilter(&frontDistance, checkdistance());

            servo.write(180);
            del(1000);
        }
    }
}

void motorLoop(void *pvParameters) {
    leftPID.Compute();
    frontPID.Compute();

    if (frontDistance < 10) {
        motor.turn(90);
    } else {
        if (leftPIDOut != 0) {
            motor.leftWheel(defaultSpeed + leftPIDOut);
            motor.rightWheel(defaultSpeed);
        } else {
            motor.forward();
        }
    }
}
