#include "../TeensyMC.h"
#include "src/stepper_control/stepper_control.h"

#ifndef SERIAL_STREAM
    #define SERIAL_STREAM Serial
#endif

_message_agent MessageAgent(&SERIAL_STREAM);
_serial_command SerialCommand(&SERIAL_STREAM);


void post_realtime_status() {
    static uint32_t last_post = 0;
}

void TMCSetup() {
    setup_stepper_callbacks();
}

void TMCRun() {
    SerialCommand.poll();
    MessageAgent.post_queued_messages();
    post_realtime_status();
}

