#include <Arduino.h>
#include <bitset>
#include <cstdint>
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

//Tweak the turn values here
#define SLOW_TURN 64
#define FAST_TURN 128
#define FASTER_TURN 192
#define SLOW_ACCEL 1
#define SLOW_DECEL (-1)
#define MED_ACCEL 2
#define MED_DECEL (-2)
#define FAST_ACCEL 4
#define FAST_DECEL (-4)

static std::uint8_t currentTurnSpeed {0};
static std::uint8_t previousTurnSpeed {0};

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

void robotLeft(std::uint8_t turnRate, std::uint8_t turnCap){
    digitalWrite(leftMotorForward, 255);
    digitalWrite(rightMotorForward, 255-(turnRate*(turnRate < turnCap) + turnCap*(turnRate >= turnCap)));
    digitalWrite(leftMotorBackward, 0);
    digitalWrite(rightMotorBackward, 0);
}
void robotRight(std::uint8_t turnRate, std::uint8_t turnCap){
    digitalWrite(leftMotorForward, 255-(turnRate*(turnRate < turnCap) + turnCap*(turnRate >= turnCap)));
    digitalWrite(rightMotorForward, 255);
    digitalWrite(leftMotorBackward, 0);
    digitalWrite(rightMotorBackward, 0);
}

void slapObstacle(){

}
void sendServoUpdate(){ //send update every 20ms

}
std::bitset<5> readSensor() { //store sensor data in a bool array
    std::bitset<5> sensorBits;
    sensorBits[0] = static_cast<bool>(digitalRead(rightmostSensor));
    sensorBits[1] = static_cast<bool>(digitalRead(rightSensor));
    sensorBits[2] = static_cast<bool>(digitalRead(middleSensor));
    sensorBits[3] = static_cast<bool>(digitalRead(leftSensor));
    sensorBits[4] = static_cast<bool>(digitalRead(leftmostSensor));
    return sensorBits;
}

void decideMove(std::bitset<5> sensorReadings){
    static std::uint8_t turnRate {};
    static std::int8_t turnFasterOrSlower {};

// btw I just copy-pasted the .to_ulong function and replaced the return value to std::uint8 for performance reasons
    std::uint8_t data {sensorReadings.to_uint8()};
    switch (data) {

        case 4: {

            currentTurnSpeed = 0;
            robotForward();
            break;
        }   // Go forward
        case 31: {

            currentTurnSpeed = 0;
            robotStop();
            slapObstacle();

            break;
        }  // A wild obstacle has appeared!
        case 6: {
            currentTurnSpeed = SLOW_ACCEL;
            if (currentTurnSpeed != previousTurnSpeed) {
                if (currentTurnSpeed > previousTurnSpeed) {
                    turnFasterOrSlower = SLOW_ACCEL;
                } else if (currentTurnSpeed < previousTurnSpeed) {
                    turnFasterOrSlower = MED_DECEL;
                }

            }
            turnRate += turnFasterOrSlower;
            robotLeft(turnRate, SLOW_TURN);
            break;
        }   // Slowly accelerate/decelerate going left
        case 12: {
            currentTurnSpeed = SLOW_ACCEL;
            if (currentTurnSpeed != previousTurnSpeed) {
                if (currentTurnSpeed > previousTurnSpeed) {
                    turnFasterOrSlower = SLOW_ACCEL;
                } else if (currentTurnSpeed < previousTurnSpeed) {
                    turnFasterOrSlower = MED_DECEL;
                }

            }
            turnRate += turnFasterOrSlower;
            robotRight(turnRate, SLOW_TURN);
            break;
        }  // Slowly accelerate/decelerate going right
        case 2:
            [[fallthrough]];
        case 7: {
            currentTurnSpeed = MED_ACCEL;
            if (currentTurnSpeed != previousTurnSpeed) {
                if (currentTurnSpeed > previousTurnSpeed) {
                    turnFasterOrSlower = MED_ACCEL;
                } else if (currentTurnSpeed < previousTurnSpeed) {
                    turnFasterOrSlower = FAST_DECEL;
                }

            }
            turnRate += turnFasterOrSlower;
            robotLeft(turnRate, FAST_TURN);
            break;
        }   // Moderately accelerate/decelerate going left
        case 8:
            [[fallthrough]];
        case 28: {
            currentTurnSpeed = MED_ACCEL;
            if (currentTurnSpeed != previousTurnSpeed) {
                if (currentTurnSpeed > previousTurnSpeed) {
                    turnFasterOrSlower = MED_ACCEL;
                } else if (currentTurnSpeed < previousTurnSpeed) {
                    turnFasterOrSlower = FAST_DECEL;
                }

            }
            turnRate += turnFasterOrSlower;
            robotRight(turnRate, FAST_TURN);
            break;
        }  // Moderately accelerate/decelerate going right
        case 3: {
            currentTurnSpeed = FAST_ACCEL;
            if (currentTurnSpeed > previousTurnSpeed) {
                turnFasterOrSlower = FAST_ACCEL;
            }
            turnRate += turnFasterOrSlower;
            robotLeft(turnRate, FASTER_TURN);
            break;
        }   // Rapidly accelerate/decelerate (shouldn't be the case) going left
        case 24: {
            currentTurnSpeed = FAST_ACCEL;
            if (currentTurnSpeed > previousTurnSpeed) {
                turnFasterOrSlower = FAST_ACCEL;
            }
            turnRate += turnFasterOrSlower;
            robotRight(turnRate, FASTER_TURN);
            break;
        }
        default: {
            robotStop();
            // slapObstacle();
        }  // Something has really gone wrong or the sensors didn't line up with the stop line properly
                        // If that's the case just comment in the slapObstacle function
    }
    previousTurnSpeed = currentTurnSpeed;
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
    decideMove(readSensor());
}