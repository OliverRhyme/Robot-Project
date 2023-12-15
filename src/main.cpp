#include <PID_v1.h>
#include <avr/interrupt.h>
#include <util/delay.h>

#include "Servo.h"
#include "StaticThreadController.h"
#include "Thread.h"
#include "ThreadController.h"
#include "init.h"
#include "motor.h"
#include "pin_assign.h"
#include "pins.h"

Motor motor;
Servo servo;

Thread sensorThread;
Thread motorThread;

StaticThreadController<2> threadController(&motorThread, &sensorThread);

void sensorLoop();
void motorLoop();

double currentDistance;
double pidOut;

double targetDistance = 40.0;
double Kp = 4.0;  // Proportional gain
double Ki = 1.4;  // Integral gain
double Kd = 0.6;  // Derivative gain

PID myPID(&currentDistance, &pidOut, &targetDistance, Kp, Ki, Kd, DIRECT);

const uint8_t defaultSpeed = 100;

double leftDistance;
double frontDistance;

void setup() {
    setupPin(TRIGGER_PIN, OUTPUT);
    setupPin(ECHO_PIN, INPUT);

    Serial.begin(9600);
    motor.init(LEFT_MOTOR_PWM_PIN, RIGHT_MOTOR_PWM_PIN, LEFT_MOTOR_DIR_PIN, RIGHT_MOTOR_DIR_PIN);
    motor.setSpeed(defaultSpeed);

    servo.attach(SERVO_PIN);

    servo.write(180);

    sensorThread.onRun(sensorLoop);
    motorThread.onRun(motorLoop);

    myPID.SetOutputLimits(-64, 64);
    myPID.SetMode(AUTOMATIC);
}

void loop() {
    threadController.run();
}

float checkdistance() {
    writePin(TRIGGER_PIN, LOW);
    _delay_us(2);
    writePin(TRIGGER_PIN, HIGH);
    _delay_us(10);
    writePin(TRIGGER_PIN, LOW);
    float distance = pulseIn(ECHO_PIN, HIGH) / 58.00;
    _delay_ms(10);
    return distance;
}

const float alpha = 0.8;  // Exponential filter coefficient

void sensorLoop() {
    float rawDistance = checkdistance();

    // Exponential filter
    currentDistance = alpha * rawDistance + (1 - alpha) * currentDistance;
}

void motorLoop() {
    myPID.Compute();

    if (pidOut != 0) {
        motor.leftWheel(defaultSpeed + pidOut);
        motor.rightWheel(defaultSpeed);
    } else {
        motor.forward();
    }

    Serial.println(pidOut);
}

// Main function
int main(void) {
    init();

    setup();

    for (;;) loop();

    return 0;
}
