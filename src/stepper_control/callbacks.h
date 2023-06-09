#pragma once

#include "../communication/serial_comm.h"

using namespace TeensyMC::SerialComm;

#define CALLBACK(name) void name##__cb(char* cmd, ArgList* args)

// enable stepper command
CALLBACK(ENABL);

// move command
CALLBACK(MOVE);

// probe command
CALLBACK(PROBE);

// home command
CALLBACK(HOME);

// clear fault command
CALLBACK(CFAULT);

// set zero command
// CALLBACK(ZERO);

// controlled stop command
CALLBACK(STOP);

// halt/e-stop command
CALLBACK(HALT);

// set limits command
CALLBACK(LIMIT);

// jog begin command
CALLBACK(JOG);

// jog cancel command
CALLBACK(JOGC);

// hold command
CALLBACK(HOLD);

// hold cancel/clear command
CALLBACK(HOLDC);

// store position command
CALLBACK(STORE);

// recall position command
CALLBACK(RECALL);

// set position command
CALLBACK(SETPOS);
