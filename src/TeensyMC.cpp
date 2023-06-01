#include "TeensyMC.h"
#include "config.h"
#include "./communication/serial_comm.h"
#include "./stepper_control/stepper_control.h"

using namespace TeensyMC;

void post_realtime_status() {
    static uint32_t last_millis = 0;

    uint32_t period = IDLE_REPORT_MILLIS;
    
    if (StepperControl::steppers_active()) {
        period = ACTIVE_REPORT_MILLIS;
    } else if (StepperControl::check_state(StepperControl::HOLDING)) {
        period = HOLD_REPORT_MILLIS;
    }

    if ((millis() - last_millis) >= period) {
        StepperControl::internal::post_steppers_status(); 
        last_millis = millis();
    }
}


void TeensyMC::init() {
    SERIAL_STREAM.begin(SERIAL_BAUDRATE);

    StepperControl::internal::begin();
}

void TeensyMC::run() {
    SerialComm::internal::process_command_queue();
    SerialComm::internal::post_queued_messages();
    SerialComm::internal::poll_serial();

    post_realtime_status();
}