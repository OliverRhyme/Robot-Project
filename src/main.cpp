#include <avr/interrupt.h>
#include <util/delay.h>

#include "init.h"
#include "pins.h"

#include "Thread.h"

#define LEFT_MOTOR_PWM_PIN 5
#define RIGHT_MOTOR_PWM_PIN 6

#define LEFT_MOTOR_DIR_PIN 2
#define RIGHT_MOTOR_DIR_PIN 4

void setup();
void loop();

void setup() {
    setupPin(LEFT_MOTOR_DIR_PIN, OUTPUT);
    setupPin(RIGHT_MOTOR_DIR_PIN, OUTPUT);

    setupPin(LEFT_MOTOR_PWM_PIN, OUTPUT);
    setupPin(RIGHT_MOTOR_PWM_PIN, OUTPUT);
}

void loop() {
    pwmWrite(LEFT_MOTOR_PWM_PIN, 255);
    pwmWrite(RIGHT_MOTOR_PWM_PIN, 255);

    writePin(LEFT_MOTOR_DIR_PIN, HIGH);
    writePin(RIGHT_MOTOR_DIR_PIN, LOW);
}


// Main function
int main(void) {
    init();

    setup();

    for (;;) loop();

    return 0;
}
