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

const double targetDistance = 30.0;
const double Kp = 4.0;  // Proportional gain
const double Ki = 1.0;  // Integral gain
const double Kd = 0.2;  // Derivative gain

PID myPID(&currentDistance, &pidOut, &targetDistance, Kp, Ki, Kd, DIRECT);


void setup() {
    setupPin(TRIGGER_PIN, OUTPUT);
    setupPin(ECHO_PIN, INPUT);


    Serial.begin(9600);
    motor.init(LEFT_MOTOR_PWM_PIN, RIGHT_MOTOR_PWM_PIN, LEFT_MOTOR_DIR_PIN, RIGHT_MOTOR_DIR_PIN);

    servo.attach(SERVO_PIN);

    sensorThread.onRun(sensorLoop);
    motorThread.onRun(motorLoop);

    myPID.SetOutputLimits(-128, 128);
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

    if (pidOut > 0) {
        motor.backward();
        motor.setSpeed(pidOut);
    } else {
        motor.forward();
        motor.setSpeed(-pidOut);
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
