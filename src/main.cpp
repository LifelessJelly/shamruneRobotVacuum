#include <Arduino.h>
#define leftMotorForward PD6
#define leftMotorBackward PD5
#define rightMotorForward PB2
#define rightMotorBackward PB1
#define servoMotor PB5
#define leftmostSensor PC5
#define leftSensor PC4
#define middleSensor PC3
#define rightSensor PC2
#define rightmostSensor PC1

void robotStop(){
    digitalWrite(leftMotorForward, HIGH);
    digitalWrite(rightMotorForward, HIGH);
    digitalWrite(leftMotorBackward, HIGH);
    digitalWrite(rightMotorBackward, HIGH);
}

void setup() {
    Serial.begin(115200);
    pinMode(leftmostSensor, INPUT);
    pinMode(leftSensor, INPUT);
    pinMode(middleSensor, INPUT);
    pinMode(rightSensor, INPUT);

    pinMode(rightmostSensor, INPUT);
    pinMode(leftMotorForward, OUTPUT);
    pinMode(leftMotorBackward, OUTPUT);
    pinMode(rightMotorForward, OUTPUT);
    pinMode(rightMotorBackward, OUTPUT);

    pinMode(servoMotor, OUTPUT);

}

void loop() {

}