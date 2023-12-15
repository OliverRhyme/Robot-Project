#include "init.h"


// initializes timers and ADC and enables interrupts
void init() {
    sei();  // enable interrupts

    TCCR0A = 0; // clear timer 0 control register A
    TCCR0B = 0; // clear timer 0 control register B
    TCCR1A = 0; // clear timer 1 control register A
    TCCR1B = 0; // clear timer 1 control register B
    TCCR2A = 0; // clear timer 2 control register A
    TCCR2B = 0; // clear timer 2 control register B

    // set timer 0 to fast pwm mode
    TCCR0A |= (1 << WGM00) | (1 << WGM01);
    // set timer 0 prescaler to 64, default on arduino uno atmega328p
    TCCR0B |= (1 << CS00) | (1 << CS01);

    // set timer 1 to phase correct pwm mode
    TCCR1A |= (1 << WGM10);
    // set timer 1 prescaler to 64, default on arduino uno atmega328p
    TCCR1B |= (1 << CS10) | (1 << CS11);

    // set timer 2 to phase correct pwm mode
    TCCR2A |= (1 << WGM20);
    // set timer 2 prescaler to 64, default on arduino uno atmega328p
    TCCR2B |= (1 << CS22);

    // enable timer 0 overflow interrupt
    // TIMSK0 |= (1 << TOIE0);

    // ADC
    ADCSRA |= (1 << ADPS2) | (1 << ADPS1) | (1 << ADPS0); // set ADC prescaler to 128
    ADCSRA |= (1 << ADEN); // enable ADC

}