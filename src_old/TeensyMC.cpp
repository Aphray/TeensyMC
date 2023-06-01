#include <Arduino.h>
#include "TeensyMC.h"
#include "config.h"
#include "stepper_control/callbacks.h"
#include "stepper_control/stepper_control.h"
#include "event_handling/event_manager.h"


_message_agent TMCMessageAgent = _message_agent(&SERIAL_STREAM);
_serial_command TMCSerialCommand = _serial_command(&SERIAL_STREAM);
_stepper_control TMCStepperControl = _stepper_control();
_event_manager TMCEventManager = _event_manager();


void post_realtime_status() {
    static uint32_t last_millis = 0;

    uint32_t period = IDLE_REPORT_MILLIS;
    
    if (TMCStepperControl.steppers_active()) {
        period = ACTIVE_REPORT_MILLIS;
    } else if (TMCStepperControl.steppers_holding()) {
        period = HOLD_REPORT_MILLIS;
    }

    if ((millis() - last_millis) >= period) {
        TMCStepperControl.post_steppers_status(); 
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

    TMCEventManager.process_queued_events();
    TMCMessageAgent.post_queued_messages();

    TMCSerialCommand.process_command_queue();
    TMCSerialCommand.poll();
    
    post_realtime_status();
}

