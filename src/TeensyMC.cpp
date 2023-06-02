#include "TeensyMC.h"
#include "config.h"
#include "./stepper_control/callbacks.h"
#include "./communication/serial_comm.h"
#include "./stepper_control/stepper_control.h"

using namespace TeensyMC;

bool init_complete = false;
bool (*external_init)() = nullptr;

CALLBACK(INIT) {
    if (init_complete) {
        TMCSerialComm::post_message(TMCSerialComm::ERROR, "Already initialized");
        return;
    }

    TMC::init(external_init);
}

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

void TeensyMC::init(bool (*ext_init)() = nullptr) {
    static bool first_call = true;

    if (first_call) {
        SERIAL_STREAM.begin(SERIAL_BAUDRATE);
        StepperControl::internal::begin();
        TMCSerialComm::register_command("INIT", 0);
        TMCSerialComm::add_callback("INIT", &INIT__cb);
    }

    init_complete = (ext_init != nullptr) ? ext_init() : true;

    if (!init_complete) {
        external_init = ext_init;
        TMCSerialComm::queue_message(TMCSerialComm::ERROR, "Initialization failed; run <INIT> command to try again");
    } else {
        TMCSerialComm::queue_message(TMCSerialComm::INFO, "Initialization complete");
    }

    first_call = false;
}

bool TeensyMC::initialized() {
    return init_complete;
}

void TeensyMC::run() {
    if (init_complete) {
        StepperControl::internal::process();
        post_realtime_status();
    }
    
    SerialComm::internal::process_command_queue();
    SerialComm::internal::post_queued_messages();
    SerialComm::internal::poll_serial();
}