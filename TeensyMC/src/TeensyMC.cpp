#include "../TeensyMC.h"
// #include "config.h"
#include "stepper_control/callbacks.h"
#include "src/stepper_control/stepper_control.h"


_message_agent TMCMessageAgent = _message_agent(&SERIAL_STREAM);
_serial_command TMCSerialCommand = _serial_command(&SERIAL_STREAM);
_stepper_control TMCStepperControl = _stepper_control();


void post_realtime_status() {
    static uint32_t last_millis = 0;

    uint32_t period = TMCStepperControl.steppers_active() ? ACTIVE_REPORT_MILLIS : IDLE_REPORT_MILLIS;

    if ((millis() - last_millis) >= period) {
        TMCStepperControl.post_steppers_status();
        last_millis = millis();
    }
}


void TMCSetup() {

    SERIAL_STREAM.begin(SERIAL_BAUDRATE);

    // setup step and pulse timers
    TMCStepperControl.setup_timers();

    // add the serial commands
    TMCSerialCommand.new_command("MVE", STEPPERS + 2);
    TMCSerialCommand.new_command("PRB", 1);
    TMCSerialCommand.new_command("HME", 1);
    TMCSerialCommand.new_command("STP", 0);
    TMCSerialCommand.new_command("HLT", 0);
    TMCSerialCommand.new_command("FLT", 0);

    // attach callbacks
    TMCSerialCommand.add_callback("MVE", &MVE__cb);
    TMCSerialCommand.add_callback("PRB", &PRB__cb);
    TMCSerialCommand.add_callback("HME", &HME__cb);
    TMCSerialCommand.add_callback("STP", &STP__cb);
    TMCSerialCommand.add_callback("HLT", &HLT__cb);
    TMCSerialCommand.add_callback("FLT", &FLT__cb);
}

void TMCRun() {
    TMCSerialCommand.poll();
    TMCMessageAgent.post_queued_messages();
    // post_realtime_status();
}

