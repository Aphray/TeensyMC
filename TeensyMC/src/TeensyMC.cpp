#include "../TeensyMC.h"
#include "communication/serial_command.h"
#include "communication/message_agent.h"

#ifndef SERIAL_STREAM
    #define SERIAL_STREAM Serial
#endif


_serial_command SerialCommand(&SERIAL_STREAM);
_message_agent MessageAgent(&SERIAL_STREAM);


void post_realtime_status() {
    static uint32_t last_post = 0;
}


void run_TMC() {
    SerialCommand.poll();
    MessageAgent.post_queued_messages();
    post_realtime_status();
}

