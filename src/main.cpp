#include <Arduino.h>
#include <Arduino_FreeRTOS.h>
#include <PID_v1.h>
#include <avr/interrupt.h>
// #include <util/delay.h>

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
void getSensorData();

void testAlgo(void *pvParameters);

void motorLoop(void *pvParameters);

void motorDemo(void *pvParameters);

// Main function
int main(void) {
    init();

    initVariant();

    setup();

    for (;;) loop();

    return 0;
}

double targetDistance = 20.0;
const double Kp = 5.0;  // Proportional gain
const double Ki = 0.1;  // Integral gain
const double Kd = 3.5;  // Derivative gain

const uint8_t defaultSpeed = 60;

double leftDistance;
double frontDistance;

double leftPIDOut;
double frontPIDOut;

PID leftPID(&leftDistance, &leftPIDOut, &targetDistance, Kp, Ki, Kd, DIRECT);
PID frontPID(&frontDistance, &frontPIDOut, &targetDistance, Kp, Ki, Kd, DIRECT);

void setup(void) {
    Serial.begin(9600);

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
    // xTaskCreate(sensorLoop, "Sensor Loop", 128, NULL, 3, NULL);
    // xTaskCreate(motorLoop, "Motor Loop", 128, NULL, 2, NULL);
    // xTaskCreate(motorDemo, "Motor DEmo", 128, NULL, 2, NULL);
    xTaskCreate(testAlgo, "Test Algo", 128, NULL, 1, NULL);

    double leftOutMax = defaultSpeed * 0.4;

    leftPID.SetOutputLimits(-leftOutMax, leftOutMax);
    frontPID.SetOutputLimits(-(defaultSpeed - 50), (defaultSpeed - 50));
    leftPID.SetMode(AUTOMATIC);
    frontPID.SetMode(AUTOMATIC);
}

unsigned int checkdistance() {
    unsigned int value = ultrasonic.measureDistanceCm();
    return value;
}

void loop(void) {
    // threadController.run();
}

// Exponential filter function
void exponentialFilter(double *value, double newValue, double alpha = 0.9) {
    *value = alpha * (newValue) + (1 - alpha) * (*value);
}

void getSensorData() {
    if (servo.read() == 180) {
        leftDistance = checkdistance();
        // exponentialFilter(&leftDistance, checkdistance());
        leftPID.Compute();
        delay(100);

        servo.write(90);
        delay(500);

    } else if (servo.read() == 90) {
        frontDistance = checkdistance();
        // exponentialFilter(&frontDistance, checkdistance());
        frontPID.Compute();
        delay(100);

        servo.write(180);
        delay(500);
    }
}

void sensorLoop(void *pvParameters) {
    for (;;) {
        getSensorData();
        yield();
    }
}

void motorLoop(void *pvParameters) {
    for (;;) {
        Serial.print("Left PID: ");
        Serial.println(leftPIDOut);

        Serial.print("Left Distance: ");
        Serial.println(leftDistance);

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
        yield();
    }
}

void motorDemo(void *pvParameters) {
    Serial.println("Motor Demo");

    Serial.println("Forward");
    motor.forward();
    delay(2000);
    motor.stop();

    delay(1000);

    Serial.println("Backward");
    motor.backward();
    delay(2000);
    motor.stop();

    delay(1000);

    Serial.println("Turn Left");
    motor.turn(-40);
    delay(1000);

    Serial.println("Turn Right");
    motor.turn(40);
    delay(1000);

    Serial.println("Large Turn Left");
    motor.turn(-100);
    delay(1000);

    Serial.println("Large Turn Right");
    motor.turn(100);
    delay(1000);

    delay(2000);
}

void testAlgo2();

void testAlgo(void *pvParameters) {
    for (;;) {
        // twice to read left and front distance
        getSensorData();
        getSensorData();

        testAlgo2();
        continue;

        double leftDistanceDiff = targetDistance - leftDistance;
        double frontDistanceDiff = 15.0 - frontDistance;

        Serial.print("Front:");
        Serial.println(frontDistance);

        Serial.print("Front diff:");
        Serial.println(frontDistanceDiff);

        Serial.print("Left:");
        Serial.println(leftDistance);

        Serial.print("Left diff:");
        Serial.println(leftDistanceDiff);

        if (frontDistanceDiff > 0) {
            motor.turn(80);
            motor.forward(100);
            continue;
        }

        // motor.leftWheel(defaultSpeed);
        // motor.rightWheel(defaultSpeed);
        // continue;;

        // generate adjustment ladder logic based on leftDistanceDiff
        if (leftDistanceDiff > 40) {
            motor.turn(60);
        } else if (leftDistanceDiff > 20) {
            Serial.println("Turning Right 20");
            // motor.leftWheel(defaultSpeed + 20);
            // motor.rightWheel(defaultSpeed);
            motor.turn(45);
        } else if (leftDistanceDiff > 10) {
            Serial.println("Turning Right 10");
            // motor.leftWheel(defaultSpeed + 10);
            // motor.rightWheel(defaultSpeed);
            motor.turn(20);
        } else if (leftDistanceDiff > 0) {
            Serial.println("Turning Right 5");
            // motor.leftWheel(defaultSpeed + 5);
            // motor.rightWheel(defaultSpeed);
            motor.turn(10);
        } else if (leftDistanceDiff == 0) {
            motor.forward();
        } else if (leftDistanceDiff > -10) {
            Serial.println("Turning left 10");
            // motor.leftWheel(defaultSpeed - 10);
            // motor.rightWheel(defaultSpeed);
            motor.turn(-20);
        } else if (leftDistanceDiff > -20) {
            Serial.println("Turning left 20");
            // motor.leftWheel(defaultSpeed - 20);
            // motor.rightWheel(defaultSpeed);
            motor.turn(-45);
        } else {
            motor.forward(50);
            motor.turn(-60);
            motor.forward(50);
            // motor.leftWheel(defaultSpeed);
            // motor.rightWheel(defaultSpeed);
        }

        motor.forward(50);
    }
}

void testAlgo2() {
    if (frontDistance < targetDistance - 5) {
        Serial.println("Front blockage");
        motor.backward(20);
        delay(100);
        motor.turn(70);
        return;
    }

    if (leftDistance < 10) {
        Serial.println("Reorient, left too close");
        motor.backward(60);
        delay(100);
        motor.turn(25);
        delay(100);
        motor.forward(40);
        // return;
    }

    double diff = min(abs(targetDistance - leftDistance + 10), 40);
    // double diff = 10;

    Serial.print("Diff:");
    Serial.println(diff);
    if (leftDistance > targetDistance) {
        motor.leftWheel(defaultSpeed - diff);
        motor.rightWheel(defaultSpeed + diff * 0.1);
        // motor.turn(-20);
    } else {
        motor.leftWheel(defaultSpeed + diff);
        motor.rightWheel(defaultSpeed - diff * 0.1);
        // motor.turn(20);
    }

    delay(400);
    motor.stop();

    if (leftDistance > 60) {
        Serial.println("Left too far");
        motor.forward(60);
        delay(100);
        motor.turn(-50);
        delay(500);
        motor.forward(100);
        return;
    }

    // motor.forward(50);
}