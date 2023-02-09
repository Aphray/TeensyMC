#pragma once
#include <Arduino.h>

// turn on command echoing
#define ECHO_CMDS

// switch to S-curve acceleration (from linear acceleration)
#define S_CURVE_ACCELERATION

// delay periods for status reporting
const uint16_t REPORT_MILLIS_IDLE = 500;
const uint16_t REPORT_MILLIS_ACTIVE = 50;

// microsecond delay b/w HIGH and LOW pulses on the step pin
const uint8_t PULSE_WIDTH_MICROS = 4;

// number of steppers to initialize
const uint8_t NUM_STEPPERS = 3;

// conversion from mm to steps
const float MM_PER_STEP[] = {
    0.0015625,   // X
    0.0015625,   // Y
    0.0015625    // Z
};

// travel limits (in mm)
const float TRAVEL_LIMITS[][2] = {
    {0, 200},    
    {0, 200},     
    {0, 200}
};

// speed limits (in mm/sec)
const float SPEED_LIMITS[][2] = {
    {0.01, 50},
    {0.01, 50},
    {0.01, 50}
};

// default and maximum acceleration (in mm/s/s)
const uint32_t DEFAULT_ACCEL = 200;
const uint32_t MAX_ACCEL[] = {
    3000,
    3000,
    3000
};

// invert the direction of travel
const bool INVERT_DIRECTION[] = {
    false,
    false,
    false
};

// step pins and states
const uint8_t STEP_PINS[][2] = {
    {4, HIGH},
    {5, HIGH},
    {6, HIGH}
};

// dir pins
const uint8_t DIR_PINS[] = {
    8,
    9,
    10
};


// probe pins and states (-1 disables probing)
const int8_t PROBE_PINS[][2] = {
    {-1, HIGH},
    {-1, HIGH},
    {12, HIGH},
};