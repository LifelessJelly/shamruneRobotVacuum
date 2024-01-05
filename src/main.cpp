#include <Arduino.h>
#include <bitset> // for std::bitset<size_t>

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
#define SLOW_TURN 200
#define MED_TURN 235
#define FAST_TURN 255
#define SLOW_ACCEL 30 
#define SLOW_DECEL (-SLOW_ACCEL)
#define MED_ACCEL 50
#define MED_DECEL (-MED_ACCEL)
#define FAST_ACCEL 100 
#define FAST_DECEL (-FAST_ACCEL)
#define LOOKAHEAD_TIME_MICROSEC 1

#define DEG0_SLAP 550
#define DEG90_SLAP 1400
#define DEG180_SLAP 2300

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

std::uint8_t previousData;
int followThrough {0};
int volatile restingDirection{DEG90_SLAP};
int slapDirection {DEG180_SLAP};

int getOppDirection(int dir);
void alternateSlap();
int phase = 0, elapsed = 0, debounce = 0;
// decides all the moves based on the sensor readings
int decideMove(const std::bitset<5>& sensorReadings){
    int final_direction = phase == 2 ? slapDirection : restingDirection;
    std::uint8_t data {sensorReadings.to_uint8()};
    std::uint16_t delta = servoSignal(final_direction);

    elapsed--;
    debounce--;
    switch (data) {
        case 17: case 27: { 
            // Go forward [sensor: 10001] [sensor: 11011]
            robotForward();
            break;
        }
        case 0: {
            // A wild obstacle has appeared! [sensor: 00000]
            if (debounce > 0) {robotForward(); break;}
            if (phase == 0) {
                phase = 1;
                elapsed = 15;
                robotForward();
            } else if (phase == 1) {
                robotStop();
                
                if (elapsed <= 0) {
                    phase = 2;
                    elapsed = 30;
                }
            } else if (phase == 2) {
                robotStop();
                if (elapsed <= 0) {
                    phase = 3;
                    elapsed = 60;
                }
            } else if (phase == 3) {
                if (elapsed <= 0) {
                    phase = 0;
                    alternateSlap();
                    //debounce = 10;
                }
                robotForward();
            }
            break;
        }  
        case 19: case 23: {
            // Slowly accelerate/decelerate going left       [sensor: 10011] [10111]
            turnRateL = accelTo(turnRateL, SLOW_TURN, SLOW_ACCEL, FAST_DECEL);
            robotLeft(turnRateL, SLOW_TURN);
            break;
        }  
        case 25: case 29: {
            // Slowly accelerate/decelerate going right      [sensor: 11001] [11101]
            turnRateR = accelTo(turnRateR, SLOW_TURN, SLOW_ACCEL, FAST_DECEL);
            robotRight(turnRateR, SLOW_TURN);
            break;
        }  
        case 3: case 11: {
            // mid accel left
            turnRateL = accelTo(turnRateL, FAST_TURN, MED_ACCEL, FAST_DECEL);
            robotLeft(turnRateL, SLOW_TURN);
            break;
        }  
        case 24: case 26: {
            // mid accel right
            turnRateR = accelTo(turnRateR, FAST_TURN, MED_ACCEL, FAST_DECEL);
            robotRight(turnRateR, SLOW_TURN);
            break;
        }  
        case 15: case 7: case 1: {
            // Rapidly accelerate going left [sensor: 01111] [sensor: 00111] [00001]
            turnRateL = accelTo(turnRateL, FAST_TURN, FAST_ACCEL, FAST_DECEL);
            robotLeft(turnRateL, FAST_TURN);
            break;
        }   
        case 30: case 28: case 16: {
            // Rapidly accelerate going right [11110] [11100] [10000]
            turnRateR = accelTo(turnRateR, FAST_TURN, FAST_ACCEL, FAST_DECEL);
            robotRight(turnRateR, FAST_TURN);
            break;
        }  
        case 31: // [11111]
        {
            robotForward();
            break;
        }
        default: {
            // Robot either went off course or reached the end
            robotForward();
            break;
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

    delayMicroseconds(20000);
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

void alternateSlap() {
    slapDirection = ((slapDirection == DEG0_SLAP) ? DEG180_SLAP : DEG0_SLAP);
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
    sensorBits[0] = digitalRead(rightmostSensor);
    sensorBits[1] = digitalRead(rightSensor);
    sensorBits[2] = digitalRead(middleSensor);
    sensorBits[3] = digitalRead(leftSensor);
    sensorBits[4] = digitalRead(leftmostSensor);

    #if DEBUG_PRINT_SENSOR
        Serial.println(static_cast<unsigned char>(sensorBits.to_uint8()), BIN);
    #endif
    
    return sensorBits;
}