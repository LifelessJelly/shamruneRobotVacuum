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
std::uint16_t nextServoUptime {1450};
std::uint16_t currentServoUptime {};

void checkIfServoTime();
void robotStop();
void robotForward();
void robotLeft(std::uint8_t turnRate, std::uint8_t turnCap);
void robotRight(std::uint8_t turnRate, std::uint8_t turnCap);
void slapObstacle();
std::bitset<5> readSensor();
void waitForServoTime();



/*
checkIfServoTime checks whether 20ms (or the respective servo downtime) has elapsed, then updates the servo.
This function should be placed generously around the program to rapidly check whether the enough time has passed,
just like coming back and forth from the microwave to check if your raw egg has finished microwaving whilst going to do
other stuff
 */
void checkIfServoTime(){
    Serial.println("checking for update");
    static bool On {true};
    // use micros() to check whether it has been 20ms
    std::uint32_t timeElapsed {micros()};
    if (On) {
        if (timeElapsed - previousTime > 20000 - currentServoUptime) {
            Serial.println("servo going LOW");
            digitalWrite(servoMotor, LOW);
            On = false;
        }
    }
    else {
        if (timeElapsed - previousTime > 20000){
            previousTime = timeElapsed;
            Serial.println("servo going HIGH");
            digitalWrite(servoMotor, HIGH);
            On = true;
            return;
        }
    }
}


void waitForServoTime(){
    Serial.println("waiting for next servo instruction");
    static std::uint32_t thisTime = previousTime;
    while (thisTime == previousTime){
        checkIfServoTime();
    }
}

// tells the robot to stop
void robotStop(){
    checkIfServoTime();
    Serial.println("stopping");
    digitalWrite(leftMotorForward, 255);
    digitalWrite(rightMotorForward, 255);
    digitalWrite(leftMotorBackward, 255);
    digitalWrite(rightMotorBackward, 255);
    checkIfServoTime();
}

// tells the robot to move forward
void robotForward(){
    checkIfServoTime();
    Serial.println("moving forward");
    digitalWrite(leftMotorForward, 255);
    digitalWrite(rightMotorForward, 255);
    digitalWrite(leftMotorBackward, 0);
    digitalWrite(rightMotorBackward, 0);
    checkIfServoTime();
}

// tells the robot to move left with a dynamic turn rate cap, so it doesn't overturn
void robotLeft(std::uint8_t turnRate, std::uint8_t turnCap){
    checkIfServoTime();
    Serial.println("turning left");
    digitalWrite(leftMotorForward, 255);
    // funny branchless statement (minus by value if value less than turncap, if not minus by turncap)
    digitalWrite(rightMotorForward, 255-(turnRate*(turnRate < turnCap) + turnCap*(turnRate >= turnCap)));
    digitalWrite(leftMotorBackward, 0);
    digitalWrite(rightMotorBackward, 0);
    checkIfServoTime();
}

// basically the same thing but for the right-facing direction
void robotRight(std::uint8_t turnRate, std::uint8_t turnCap){
    checkIfServoTime();
    Serial.println("turning right");
    // another funny branchless statement
    digitalWrite(leftMotorForward, 255-(turnRate*(turnRate < turnCap) + turnCap*(turnRate >= turnCap)));
    digitalWrite(rightMotorForward, 255);
    digitalWrite(leftMotorBackward, 0);
    digitalWrite(rightMotorBackward, 0);
    checkIfServoTime();
}

// slaps the obstacle when it reaches a black stop line
void slapObstacle(){
    Serial.println("slapping obstacle sequence");
    checkIfServoTime();
    Serial.println("servo going 0 degrees");
    nextServoUptime = 600;
    waitForServoTime();                     // waits until 1.45ms pulse width is done
    currentServoUptime = nextServoUptime;   // currentServoUptime = 600
    checkIfServoTime();                     // calls the check here just for fun I don't think it does anything
    Serial.println("servo going 180 degrees");
    nextServoUptime = 2300;                 // queue the next servo uptime to 2.3ms
    waitForServoTime();                     // waits until the 0.6ms pulse width is done
    currentServoUptime = nextServoUptime;   // currentServoUptime = 2300
    checkIfServoTime();                     // also doesn't do much I think
    Serial.println("servo going 90 degrees");
    nextServoUptime = 1450;                 // queue the next servo uptime back to 1.45ms (normal operation)
    waitForServoTime();                     // waits until the 2.3ms pulse width is done
    currentServoUptime = nextServoUptime;
    // goes back to robotForward() then to loop()
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
            robotForward();
            // if bot doesn't move fully out of the black line, add a delay() here
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
            break;
        }  // Robot either went off course or reached the end
    }
    previousTurnSpeed = currentTurnSpeed;
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
    robotStop();
    delay(20);
    previousTime = micros();
    digitalWrite(servoMotor, HIGH);
}

void loop() {
    uint32_t startTime = micros();
    currentServoUptime = nextServoUptime;
    decideMove(readSensor());
    uint32_t totalTime = micros() - startTime;
    totalTime /= 1000;
    Serial.print("Total time for one loop = ");
    Serial.println(totalTime);
}
