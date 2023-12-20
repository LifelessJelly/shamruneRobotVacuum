#include <Arduino.h>
#include <bitset>
#include <cstdint>

#define DEBUG_PRINT_SENSOR false
#define DEBUG_PRINT_MOTORVAL false

#define leftMotorForward 10
#define leftMotorBackward 9
#define rightMotorForward 6
#define rightMotorBackward 5
#define servoMotor 13
#define leftmostSensor A1
#define leftSensor A2
#define middleSensor A3
#define rightSensor A4
#define rightmostSensor A5

//Tweak the turn values here
#define SLOW_TURN 160
#define MED_TURN 200
#define FAST_TURN 240
#define SLOW_ACCEL 10
#define SLOW_DECEL -SLOW_ACCEL
#define MED_ACCEL 15
#define MED_DECEL -MED_ACCEL
#define FAST_ACCEL 20
#define FAST_DECEL -FAST_ACCEL
#define LOOKAHEAD_TIME_MICROSEC 1

#define DEG0_SLAP 600
#define DEG90_SLAP 1450
#define DEG180_SLAP 2300

std::uint16_t servoUptime{0};
std::uint8_t turnRateL {0};
std::uint8_t turnRateR {0};

std::bitset<5> readSensor();
void motorPin(std::uint8_t LMF, std::uint8_t RMF, std::uint8_t LMB, std::uint8_t RMB);

void robotStop();
void robotForward();
void robotLeft(std::uint8_t turnRate, std::uint8_t turnCap);
void robotRight(std::uint8_t turnRate, std::uint8_t turnCap);

std::uint16_t servoSignal(std::uint16_t deg);

int accelTo(int current, int goal, int accel, int decel) {
    return current < goal ? 
                std::min(current + accel, goal) : 
           current > goal ?
                std::max(current + decel, goal) : 
           current;
}

bool Please_Slap_Thank_You = false;
// decides all the moves based on the sensor readings
int decideMove(const std::bitset<5>& sensorReadings){
    std::uint8_t data {sensorReadings.to_uint8()};
    std::uint16_t delta = servoSignal(DEG0_SLAP);

    switch (data) {
        case 17: case 27: { 
            // Go forward [sensor: 10001] [sensor: 11011]
            robotForward();
            break;
        }
        case 0: {
            // A wild obstacle has appeared! [sensor: 00000]
            int elapsed = 0;
            robotStop();                
            delayMicroseconds(delta);
            while (elapsed < 250) {
                std::uint16_t delta2 = servoSignal(elapsed < 55 ? DEG0_SLAP : DEG90_SLAP);
                robotStop();
                delayMicroseconds(delta2);
                elapsed++;
            }
            elapsed = 0;
            while (elapsed < 175) {
                std::uint16_t delta2 = servoSignal(DEG0_SLAP);
                if (elapsed > 50)
                    robotForward();
                delayMicroseconds(delta2);
                elapsed++;
            }
            delta = servoSignal(DEG0_SLAP);
            // if bot doesn't move out of the black line, add a delay() here
            break;
        }  
        case 19: {
            // Slowly accelerate/decelerate going left       [sensor: 10011]
            turnRateL = accelTo(turnRateL, SLOW_TURN, SLOW_ACCEL, MED_DECEL);
            robotLeft(turnRateL, SLOW_TURN);
            break;
        }  
        case 25: {
            // Slowly accelerate/decelerate going right      [sensor: 11001]
            turnRateR = accelTo(turnRateR, SLOW_TURN, SLOW_ACCEL, MED_DECEL);
            robotRight(turnRateR, SLOW_TURN);
            break;
        }  
        case 23: case 3: {
            // Moderately accelerate/decelerate going left [sensor: 10111] [sensor: 00011]
            turnRateL = accelTo(turnRateL, MED_TURN, MED_ACCEL, FAST_DECEL);
            robotLeft(turnRateL, MED_TURN);
            break;
        }   
        case 29: case 24: {
            // Moderately accelerate/decelerate going right  [sensor: 11101] [sensor: 11000]
            turnRateR = accelTo(turnRateR, MED_TURN, MED_ACCEL, FAST_DECEL);
            robotRight(turnRateR, MED_TURN);
            break;
        }  
        case 15: case 7: {
            // Rapidly accelerate going left [sensor: 01111] [sensor: 00111]
            turnRateL = accelTo(turnRateL, FAST_TURN, FAST_ACCEL, FAST_DECEL);
            robotLeft(turnRateL, FAST_TURN);
            break;
        }   
        case 30: case 28: {
            // Rapidly accelerate going right [sensor: 11110] [sensor: 11100]
            turnRateR = accelTo(turnRateR, FAST_TURN, FAST_ACCEL, FAST_DECEL);
            robotRight(turnRateR, FAST_TURN);
            break;
        }  
        case 1: case 16: {
            // edge case, just continue forward [00001][10000]
            robotForward();
        }
        default: {
            // Robot either went off course or reached the end
            robotStop();
        }  
    }
    delayMicroseconds(delta);
    return data;
}

void setup() {
    Serial.begin(9600);
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
    decideMove(readSensor());
}

// tells the robot to stop
void robotStop(){
    turnRateL = 0; turnRateR = 0;
    motorPin(0, 0, 0, 0);    
}
// tells the robot to move forward
void robotForward(){
    turnRateL = 0; turnRateR = 0;
    motorPin(255, 255, 0, 0);
}
// tells the robot to move left with a dynamic turn rate cap, so it doesn't overturn
void robotLeft(std::uint8_t turnRate, std::uint8_t turnCap){
    turnRateR = 0;
    motorPin(255, (255-std::min(turnRate, turnCap)), 0, 0);
}
// basically the same thing but for the right-facing direction
void robotRight(std::uint8_t turnRate, std::uint8_t turnCap){
    turnRateL = 0;
    motorPin((255-std::min(turnRate, turnCap)), 255, 0, 0);
}

// function to set the motor pins
void motorPin(std::uint8_t LMF, std::uint8_t RMF, std::uint8_t LMB, std::uint8_t RMB) {
    analogWrite(leftMotorForward, LMF);
    analogWrite(rightMotorForward, RMF);
    analogWrite(leftMotorBackward, LMB);
    analogWrite(rightMotorBackward, RMB);
    #if DEBUG_PRINT_MOTORVAL
        Serial.print(LMF); Serial.print(RMF); Serial.print(LMB); Serial.println(RMB);
    #endif
}

// slaps the obstacle when it reaches a black stop line
std::uint16_t servoSignal(std::uint16_t deg){
    digitalWrite(servoMotor, HIGH);
    delayMicroseconds(deg);
    digitalWrite(servoMotor, LOW);

    return 20000-deg;
}

// reads the sensor and returns a byte containing 5 bits
std::bitset<5> readSensor() { 
    std::bitset<5> sensorBits;
    sensorBits[0] = static_cast<bool>(digitalRead(rightmostSensor));
    sensorBits[1] = static_cast<bool>(digitalRead(rightSensor));
    sensorBits[2] = static_cast<bool>(digitalRead(middleSensor));
    sensorBits[3] = static_cast<bool>(digitalRead(leftSensor));
    sensorBits[4] = static_cast<bool>(digitalRead(leftmostSensor));

    #if DEBUG_PRINT_SENSOR
        Serial.println(static_cast<unsigned char>(sensorBits.to_uint8()), BIN);
    #endif
    
    return sensorBits;
}