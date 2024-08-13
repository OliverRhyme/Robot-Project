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

void getSensorData();

void mainLoop(void *pvParameters);

// Main function
int main(void) {
    init();

    initVariant();

    setup();

    for (;;) loop();

    return 0;
}

double targetDistance = 20.0;

const uint8_t defaultSpeed = 60;

double leftDistance;
double frontDistance;

void setup(void) {
    Serial.begin(9600);

    setupPin(TRIGGER_PIN, OUTPUT);
    setupPin(ECHO_PIN, INPUT);

    motor.init(LEFT_MOTOR_PWM_PIN, RIGHT_MOTOR_PWM_PIN, LEFT_MOTOR_DIR_PIN, RIGHT_MOTOR_DIR_PIN);
    motor.setSpeed(defaultSpeed);

    servo.attach(SERVO_PIN);

    servo.write(180);

    xTaskCreate(mainLoop, "Main loop", 128, NULL, 1, NULL);
}

unsigned int checkdistance() {
    unsigned int value = ultrasonic.measureDistanceCm();
    return value;
}

void loop(void) {
    // threadController.run();
}

void getSensorData() {
    if (servo.read() == 180) {
        leftDistance = checkdistance();
        delay(100);

        servo.write(90);
        delay(500);

    } else if (servo.read() == 90) {
        frontDistance = checkdistance();
        delay(100);

        servo.write(180);
        delay(500);
    }
}
void mainLoop(void *pvParameters) {
    for (;;) {
        // twice to read left and front distance
        getSensorData();
        getSensorData();
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
    }
}