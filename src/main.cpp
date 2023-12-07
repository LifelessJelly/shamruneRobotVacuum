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
    digitalWrite(leftMotorForward, 255);
    digitalWrite(rightMotorForward, 255);
    digitalWrite(leftMotorBackward, 255);
    digitalWrite(rightMotorBackward, 255);
}

void robotForward(){
    digitalWrite(leftMotorForward, 255);
    digitalWrite(rightMotorForward, 255);
    digitalWrite(leftMotorBackward, 0);
    digitalWrite(rightMotorBackward, 0);
}

void robotLeft(char turnRate){ //use char for memory saving
    digitalWrite(leftMotorForward, 255);
    digitalWrite(rightMotorForward, 255-turnRate);
    digitalWrite(leftMotorBackward, 0);
    digitalWrite(rightMotorBackward, 0);
}
void robotRight(char turnRate){ //use char for memory saving
    digitalWrite(leftMotorForward, 255-turnRate);
    digitalWrite(rightMotorForward, 255);
    digitalWrite(leftMotorBackward, 0);
    digitalWrite(rightMotorBackward, 0);
}

void slapObstacle(){

}
void sendServoUpdate(){ //send update every 20ms

}
bool* readSensor() { //store sensor data in a bool array
    static bool sensorData[]
            {static_cast<bool>(digitalRead(leftmostSensor)),
             static_cast<bool>(digitalRead(leftSensor)),
             static_cast<bool>(digitalRead(middleSensor)),
             static_cast<bool>(digitalRead(rightSensor)),
             static_cast<bool>(digitalRead(rightmostSensor))};
    return sensorData;
}

void decideMove(bool* sensorReadings){

}

inline void setup() {
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
    robotStop();
}

void loop() {



}