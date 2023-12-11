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
#define LOOKAHEAD_TIME_MICROSEC 1

static std::uint8_t currentTurnSpeed {0};
static std::uint8_t previousTurnSpeed {0};
static std::uint32_t previousTime {0};
std::uint16_t servoUptime {0};


// sends an update to the servo motor (every 20 milliseconds should either tell it to do something or don't do anything)
void servoUpdate(){ //send update every 20ms
    digitalWrite(servoMotor, HIGH);
    delayMicroseconds(servoUptime);
    digitalWrite(servoMotor, LOW);
    //some code here
    servoUptime = 0;
}

/*
checkIfServoTime checks whether 20ms (or the respective servo downtime) has elapsed, then updates the servo.
This function should be placed generously around the program to rapidly check whether the enough time has passed,
just like coming back and forth from the microwave to check if your raw egg has finished microwaving whilst going to do
other stuff
 */
void checkIfServoTime(){
    // use micros() to check whether it has been 20ms
    std::uint32_t time {micros()};
    if (time - previousTime > 20000-servoUptime){
        previousTime = time;
        servoUpdate();
    }
}

// tells the robot to stop
void robotStop(){
    checkIfServoTime();
    digitalWrite(leftMotorForward, 255);
    digitalWrite(rightMotorForward, 255);
    digitalWrite(leftMotorBackward, 255);
    digitalWrite(rightMotorBackward, 255);
    checkIfServoTime();
}

// tells the robot to move forward
void robotForward(){
    checkIfServoTime();
    digitalWrite(leftMotorForward, 255);
    digitalWrite(rightMotorForward, 255);
    digitalWrite(leftMotorBackward, 0);
    digitalWrite(rightMotorBackward, 0);
    checkIfServoTime();
}

// tells the robot to move left with a dynamic turn rate cap, so it doesn't overturn
void robotLeft(std::uint8_t turnRate, std::uint8_t turnCap){
    checkIfServoTime();
    digitalWrite(leftMotorForward, 255);
    digitalWrite(rightMotorForward, 255-(turnRate*(turnRate < turnCap) + turnCap*(turnRate >= turnCap)));
    digitalWrite(leftMotorBackward, 0);
    digitalWrite(rightMotorBackward, 0);
    checkIfServoTime();
}

// basically the same thing but for the right-facing direction
void robotRight(std::uint8_t turnRate, std::uint8_t turnCap){
    checkIfServoTime();
    digitalWrite(leftMotorForward, 255-(turnRate*(turnRate < turnCap) + turnCap*(turnRate >= turnCap)));
    digitalWrite(rightMotorForward, 255);
    digitalWrite(leftMotorBackward, 0);
    digitalWrite(rightMotorBackward, 0);
    checkIfServoTime();
}

// slaps the obstacle when it reaches a black stop line
void slapObstacle(){
    robotStop();
    checkIfServoTime();
    servoUptime = 2300;
    delayMicroseconds((20000-servoUptime) - (previousTime - micros()) - LOOKAHEAD_TIME_MICROSEC);
    checkIfServoTime();
}

// reads the sensor and returns a byte containing 5 bits
std::bitset<5> readSensor() { 
    std::bitset<5> sensorBits;
    sensorBits[0] = static_cast<bool>(digitalRead(rightmostSensor));
    sensorBits[1] = static_cast<bool>(digitalRead(rightSensor));
    sensorBits[2] = static_cast<bool>(digitalRead(middleSensor));
    sensorBits[3] = static_cast<bool>(digitalRead(leftSensor));
    sensorBits[4] = static_cast<bool>(digitalRead(leftmostSensor));
    checkIfServoTime();
    return sensorBits;
}

// decides all the moves based on the sensor readings
void decideMove(const std::bitset<5>& sensorReadings){
    static std::uint8_t turnRate {};
    static std::int8_t turnFasterOrSlower {};
    checkIfServoTime();

// btw I just copy-pasted the .to_ulong function and replaced the return value to std::uint8 for performance reasons
    std::uint8_t data {sensorReadings.to_uint8()};
    checkIfServoTime();

    //FIXME this part needs testing (especially all the speed increment parameters)
    switch (data) {

        case 27: {
            currentTurnSpeed = 0;
            turnRate = 0;
            robotForward();
            break;
        }  // Go forward                                    [sensor: 11011]
        case 0: {
            currentTurnSpeed = 0;
            turnRate = 0;
            robotStop();
            slapObstacle();
            break;
        }   // A wild obstacle has appeared!                 [sensor: 00000]
        case 19: {
            currentTurnSpeed = SLOW_ACCEL;
            if (currentTurnSpeed != previousTurnSpeed) {
                if (currentTurnSpeed > previousTurnSpeed) {
                    turnFasterOrSlower = SLOW_ACCEL;
                } else if (currentTurnSpeed < previousTurnSpeed) {
                    turnFasterOrSlower = MED_DECEL;
                }
                checkIfServoTime();
            }
            turnRate += turnFasterOrSlower;
            robotLeft(turnRate, SLOW_TURN);
            break;
        }  // Slowly accelerate/decelerate going left       [sensor: 10011]
        case 25: {
            currentTurnSpeed = SLOW_ACCEL;
            if (currentTurnSpeed != previousTurnSpeed) {
                if (currentTurnSpeed > previousTurnSpeed) {
                    turnFasterOrSlower = SLOW_ACCEL;
                } else if (currentTurnSpeed < previousTurnSpeed) {
                    turnFasterOrSlower = MED_DECEL;
                }
                checkIfServoTime();
            }
            turnRate += turnFasterOrSlower;
            robotRight(turnRate, SLOW_TURN);
            break;
        }  // Slowly accelerate/decelerate going right      [sensor: 11001]
        case 23: {
            [[fallthrough]];
        }  //                                               [sensor: 10111]
        case 3: {
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
        }   // Moderately accelerate/decelerate going left   [sensor: 00011]
        case 29: {
            [[fallthrough]];
        }  //                                               [sensor: 11101]
        case 24: {
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
        }  // Moderately accelerate/decelerate going right  [sensor: 11000]
        case 7: {
            currentTurnSpeed = FAST_ACCEL;
            turnFasterOrSlower = FAST_ACCEL;
            turnRate += turnFasterOrSlower;
            robotLeft(turnRate, FASTER_TURN);
            break;
        }   // Rapidly accelerate going left                 [sensor: 00111]
        case 28: {
            currentTurnSpeed = FAST_ACCEL;
            turnFasterOrSlower = FAST_ACCEL;
            turnRate += turnFasterOrSlower;
            robotRight(turnRate, FASTER_TURN);
            break;
        }  // Rapidly accelerate going right                [sensor: 11100]
        default: {
            robotStop();
        }  // Robot either went off course or reached the end
    }
    previousTurnSpeed = currentTurnSpeed;
}

// inline because funny optimisation trick
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

    delay(20);
}

void loop() {
    checkIfServoTime();
    decideMove(readSensor());
}
