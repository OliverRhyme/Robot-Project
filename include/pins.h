#ifndef PINS_H
#define PINS_H

#include <avr/io.h>

#define INPUT 0
#define OUTPUT 1
#define INPUT_PULLUP 2

#define HIGH 1
#define LOW 0

void setupPin(uint8_t pin, uint8_t mode);
void writePin(uint8_t pin, uint8_t value);
// void readPin(uint8_t pin);

void pwmWrite(uint8_t pin, uint8_t value);



#endif // PINS_H
