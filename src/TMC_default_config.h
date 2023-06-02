#pragma once

// serial port for printing
#define SERIAL_STREAM Serial

// serial port baud
#define SERIAL_BAUDRATE 115200

// report period when steppers active
#define ACTIVE_REPORT_MILLIS 50

// report period when steppers idle
#define IDLE_REPORT_MILLIS 500

// report period when in hold condition
#define HOLD_REPORT_MILLIS 250

// buffer size for the serial RX data
#define RX_BUFFER_SIZE 1024

// max size of the message queue
#define MSG_QUEUE_SIZE 10

// max size of the command queue
#define CMD_QUEUE_SIZE  10

// max size of the move planner queue
#define PLANNER_QUEUE_SIZE 100

// buffer size for message agent
#define MESSAGE_BUFFER_SIZE 1024

// buffer size for arguments
#define ARG_BUFFER_SIZE 16

// max number of chars for the command tags
#define CMD_CHAR_MAX 5

// maximum number of registered user commands
#define MAX_USER_COMMANDS 25

// maximum number of registered user callbacks (per command)
#define MAX_USER_CALLBACKS 10

// maximum number of registered user callbacks (for all events, in total)
#define MAX_EVENT_CALLBACKS 20

// max arguments (per command)
#define CMD_MAX_ARGS 10

// delimiter b/w command and arguments
#define CMD_DELIMITER ":"

// delimiter to separate arguments
#define ARG_DELIMITER_CHAR ','

// char for skipping an argument
#define ARG_SKIP_CHAR '*'

#define MAX_STEPPERS 6

#define PULSE_WIDTH_US 4

#define HOMING_OVERSHOOT 2.0

#define PROBING_OVERSHOOT 2.0

// time of acceleration in milliseconds
#define ACCELERATION_TIME 50.0

// to enable/disable homing steppers before movement (of after any fault condition)
// #define HOME_STEPPERS_FIRST

// to enable/disable sin curve acceleration profile (disable to use linear acceleration)
#define SIGMOID_CURVE_ACCELERATION