#pragma once

#include "../communication/serial_command.h"

// move command
CALLBACK(MVE);

// probe command
CALLBACK(PRB);

// home command
CALLBACK(HME);

// clear fault command
CALLBACK(FLT);

// set zero command
CALLBACK(ZRO);

// controlled stop command
CALLBACK(STP);

// halt/e-stop command
CALLBACK(HLT);

// set limits command
CALLBACK(LIM);