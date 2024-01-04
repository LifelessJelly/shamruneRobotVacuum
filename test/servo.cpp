#include <Arduino.h>
#define servoPin 13

unsigned int servoPos;

void setup()
{
    pinMode(servoPin, OUTPUT);
    servoPos = 1450; // Pulses duration: 600:0deg; 1450:90deg; 2300:180deg
                     // Try change this number
}

void loop()
{
    // A pulse each 20ms
    digitalWrite(servoPin, HIGH);
    delayMicroseconds(servoPos); // Duration of the pusle in microseconds
    digitalWrite(servoPin, LOW);
    delayMicroseconds(20000 - servoPos); // 20ms - duration of the pusle. 18550
}
