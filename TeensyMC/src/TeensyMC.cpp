#include "../TeensyMC.h"
#include "TMC_default_config.h"
#include "stepper_control/callbacks.h"
#include "src/stepper_control/stepper_control.h"


_message_agent TMCMessageAgent = _message_agent(&SERIAL_STREAM);
_serial_command TMCSerialCommand = _serial_command(&SERIAL_STREAM);
_stepper_control TMCStepperControl = _stepper_control();


void post_realtime_status() {
    static uint32_t last_millis = 0;

    uint32_t period = TMCStepperControl.steppers_active() ? ACTIVE_REPORT_MILLIS : IDLE_REPORT_MILLIS;

    if ((millis() - last_millis) >= period) {
        TMCStepperControl.post_steppers_status(false);  // 'false' to post directly, no need to queue
        last_millis = millis();
    }
}


void TMC_Begin() {
    // setup serial channel
    SERIAL_STREAM.begin(SERIAL_BAUDRATE);

    // initialize the stepper control
    TMCStepperControl.begin();
}

void TMC_Run() {
    TMCSerialCommand.poll();
    TMCMessageAgent.post_queued_messages();
    post_realtime_status();
}

