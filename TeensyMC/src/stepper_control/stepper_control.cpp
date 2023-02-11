#include "stepper_control.h"
#include "../communication/message_agent.h"
#include "../communication/serial_command.h"


void MVE_cb(char* cmd, ArgList* args) {
    TMCMessageAgent.post_message(DEBUG, cmd);
}

void PRB_cb(char* cmd, ArgList* args) {
    TMCMessageAgent.post_message(DEBUG, cmd);
}


void setup_stepper_callbacks() {
    TMCSerialCommand.new_command("MVE", STEPPERS + 2);
    TMCSerialCommand.new_command("PRB", 1);
    TMCSerialCommand.new_command("HME", 1);
    TMCSerialCommand.new_command("STP", 0);
    TMCSerialCommand.new_command("HLT", 0);
    TMCSerialCommand.new_command("FLT", 0);

    TMCSerialCommand.add_callback("MVE", &MVE_cb);
    TMCSerialCommand.add_callback("PRB", &PRB_cb);
}

