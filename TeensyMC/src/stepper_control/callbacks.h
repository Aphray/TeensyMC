#pragma once

#include "../communication/message_agent.h"
#include "../communication/serial_command.h"

#define CALLBACK(name) inline void name##__cb(char* cmd, ArgList* args)


CALLBACK(MVE) {
    // move command
    TMCMessageAgent.post_message(DEBUG, cmd);

}

CALLBACK(PRB) {
    // probe command
    TMCMessageAgent.post_message(DEBUG, cmd);

}

CALLBACK(HME) {
    // home command

}

CALLBACK(FLT) {
    // clear fault command

}

CALLBACK(ZRO) {
    // set zero command

}

CALLBACK(STP) {
    // stop command

}

CALLBACK(HLT) {
    // halt/e-stop command

}