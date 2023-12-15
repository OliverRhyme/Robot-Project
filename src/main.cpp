#include <avr/interrupt.h>
#include <util/delay.h>

#include "init.h"
#include "pins.h"

#include "pin_assign.h"

#include "motor.h"

Motor motor;

void setup() {
    motor.init(LEFT_MOTOR_PWM_PIN, RIGHT_MOTOR_PWM_PIN, LEFT_MOTOR_DIR_PIN, RIGHT_MOTOR_DIR_PIN);
}

void loop() {
    motor.forward();
    _delay_ms(1000);

    motor.backward();
    _delay_ms(1000);

    motor.turnLeft();
    _delay_ms(1000);

    motor.turnRight();
    _delay_ms(1000);

    motor.stop();
    _delay_ms(1000);
}


// Main function
int main(void) {
    init();

    setup();

    for (;;) loop();

    return 0;
}
