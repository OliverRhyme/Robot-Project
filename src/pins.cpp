#include "pins.h"

void setupPin(uint8_t pin, uint8_t mode) {
    if (pin >= 0 && pin <= 7) {
        if (mode == INPUT) {
            DDRD &= ~(1 << pin);
            PORTD &= ~(1 << pin);
        } else if (mode == OUTPUT) {
            DDRD |= (1 << pin);
        } else if (mode == INPUT_PULLUP) {
            DDRD &= ~(1 << pin);
            PORTD |= (1 << pin);
        }
    } else if (pin >= 8 && pin <= 13) {
        if (mode == INPUT) {
            DDRB &= ~(1 << (pin - 8));
            PORTB &= ~(1 << (pin - 8));
        } else if (mode == OUTPUT) {
            DDRB |= (1 << (pin - 8));
        } else if (mode == INPUT_PULLUP) {
            DDRB &= ~(1 << (pin - 8));
            PORTB |= (1 << (pin - 8));
        }
    }
}

void writePin(uint8_t pin, uint8_t value) {
    if (pin >= 0 && pin <= 7) {
        if (value == HIGH) {
            PORTD |= (1 << pin);
        } else if (value == LOW) {
            PORTD &= ~(1 << pin);
        }
    } else if (pin >= 8 && pin <= 13) {
        if (value == HIGH) {
            PORTB |= (1 << (pin - 8));
        } else if (value == LOW) {
            PORTB &= ~(1 << (pin - 8));
        }
    }
}


void pwmWrite(uint8_t pin, uint8_t value) {

    setupPin(pin, OUTPUT);

    if (pin == 5 || pin == 6) {
        uint8_t comValue;

        if (pin == 5) {
            comValue = COM0B1;
        } else if (pin == 6) {
            comValue = COM0A1;
        }
        TCCR0A |= (1 << comValue); // connect timer 0 to pin 5 or 6
        
        if (pin == 5) {
            OCR0B = value; // set duty cycle
        } else if (pin == 6) {
            OCR0A = value; // set duty cycle
        }


    } else if (pin == 9 || pin == 10) {
        uint8_t comValue;

        if (pin == 9) {
            comValue = COM1A1;
        } else if (pin == 10) {
            comValue = COM1B1;
        }
        TCCR1A |= (1 << comValue); // connect timer 1 to pin 9 or 10
        
        if (pin == 9) {
            OCR1A = value; // set duty cycle
        } else if (pin == 10) {
            OCR1B = value; // set duty cycle
        }

    } else if (pin == 11 || pin == 3) {
        uint8_t comValue;

        if (pin == 11) {
            comValue = COM2A1;
        } else if (pin == 3) {
            comValue = COM2B1;
        }
        TCCR2A |= (1 << comValue); // connect timer 2 to pin 11 or 3
        
        if (pin == 11) {
            OCR2A = value; // set duty cycle
        } else if (pin == 3) {
            OCR2B = value; // set duty cycle
        }
    } else { // if pin is not a PWM pin, fall back to digital write
        if (value >= 128) {
            writePin(pin, HIGH);
        } else {
            writePin(pin, LOW);
        }
    }
}

