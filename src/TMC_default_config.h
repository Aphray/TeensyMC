#pragma once

#if __has_include("TMC_user_config.h")
    #include "TMC_user_config.h"
#endif


#ifndef SERIAL_STREAM
    #define SERIAL_STREAM Serial
#endif

#ifndef SERIAL_BAUDRATE
    #define SERIAL_BAUDRATE 115200
#endif

#ifndef ACTIVE_REPORT_MILLIS
    #define ACTIVE_REPORT_MILLIS 50
#endif

#ifndef IDLE_REPORT_MILLIS
    #define IDLE_REPORT_MILLIS 500
#endif

#ifndef RX_BUFFER_SIZE
    #define RX_BUFFER_SIZE 256
#endif

#ifndef MESSAGE_BUFFER_SIZE
    #define MESSAGE_BUFFER_SIZE 256
#endif

#ifndef ARG_BUFFER_SIZE
    #define ARG_BUFFER_SIZE 16
#endif

#ifndef CMD_CHAR_MAX
    #define CMD_CHAR_MAX 5
#endif

#ifndef MAX_USER_COMMANDS
    #define MAX_USER_COMMANDS 25
#endif

#ifndef MAX_USER_CALLBACKS
    #define MAX_USER_CALLBACKS 10
#endif

#ifndef CMD_MAX_ARGS
    #define CMD_MAX_ARGS 10
#endif

#ifndef CMD_DELIMITER
    #define CMD_DELIMITER ":"
#endif

#ifndef ARG_DELIMITER_CHAR
    #define ARG_DELIMITER_CHAR ','
#endif

#ifndef ARG_SKIP_CHAR
    #define ARG_SKIP_CHAR '*'
#endif

#ifndef MAX_STEPPERS
    #define MAX_STEPPERS 6
#endif

#ifndef HOME_STEPPERS_FIRST
    #define HOME_STEPPERS_FIRST false
#endif

#ifndef SIN_CURVE_ACCELERATION
    #define SIN_CURVE_ACCELERATION false
#endif

#ifndef PULSE_WIDTH_US
    #define PULSE_WIDTH_US 6
#endif

#ifndef HOMING_OVERSHOOT
    #define HOMING_OVERSHOOT 2.0
#endif

#ifndef PROBING_OVERSHOOT
    #define PROBING_OVERSHOOT 2.0
#endif

#ifndef MAX_EVENT_CALLBACKS
    #define MAX_EVENT_CALLBACKS 20
#endif