#include "../TeensyMC.h"
#include "src/stepper_control/stepper_control.h"

#ifndef SERIAL_STREAM
    #define SERIAL_STREAM Serial
#endif


_message_agent TMCMessageAgent(&SERIAL_STREAM);
_serial_command TMCSerialCommand(&SERIAL_STREAM);


void post_realtime_status() {
    static uint32_t last_post = 0;
}

void TMCSetup() {
    setup_stepper_callbacks();
}

void TMCRun() {
    TMCSerialCommand.poll();
    TMCMessageAgent.post_queued_messages();
    post_realtime_status();
}

