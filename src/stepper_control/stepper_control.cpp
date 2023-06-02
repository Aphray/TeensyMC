#include <TeensyTimerTool.h>
#include <algorithm>
#include "stepper.h"
#include "stepper_control.h"
#include "accelerator.h"
#include "callbacks.h"
#include "../config.h"
#include "../communication/serial_comm.h"

#define ASSERT_IDLE if (state != IDLE) return

#define NO_INTERRUPTS(body) { noInterrupts() \
                              body \
                              interrupts() }

using namespace TeensyTimerTool;

using namespace TeensyMC;
using namespace SerialComm;
using namespace StepperControl;

StepperState state = IDLE;
StepperState prev_state = IDLE;

Stepper* fault_stepper = nullptr;
Stepper* master_stepper = nullptr;
Stepper* steppers[MAX_STEPPERS + 1] = { nullptr };
Stepper* steppers_sort[MAX_STEPPERS + 1] = { nullptr };

OneShotTimer pulse_timer;
PeriodicTimer step_timer;
PeriodicTimer hold_timer;

uint8_t num_steppers = 0;
uint32_t hold_ms = 0;


void step_ISR();    // stepper ISR
void pulse_ISR();   // step-pulse ISR
void hold_ISR();    // hold timer ISR


inline void change_state(StepperState state_) {
    if (state == FAULT) SerialComm::clear_command_queue();

    prev_state = state;
    state = state_;
}

inline void restore_state() {
    state = prev_state;
}

void run_steppers(float speed) {

    if (master_stepper->get_delta_steps() == 0) { 
        restore_state();
        return; 
    }

    float start_speed = 1;

    speed = master_stepper->cvt_to_steps(speed);
    float accel = (speed - start_speed) * 1000.0 / ACCELERATION_TIME;
    // accel = master_stepper->cvt_to_steps(accel);

    Stepper** stepper = steppers;
    while (*stepper) {
        (*stepper)->constrain_speed_accel(master_stepper, &start_speed, &speed, &accel);
        stepper++;
    }

    prepare_accelerator((state == JOGGING ? std::numeric_limits<uint32_t>::max() : master_stepper->get_delta_steps()), start_speed, speed, accel);
    step_timer.setPeriod(1);

    switch (state) {
        case PROBING:
            // TMCEventManager.trigger_event(PROBING_STARTED);
            SerialComm::post_message(INFO, "Probing: started on axis %i", master_stepper->get_axis_id());
            break;
        case HOMING:
            // TMCEventManager.trigger_event(HOMING_STARTED);
            SerialComm::post_message(INFO, "Homing: started on axis %i", master_stepper->get_axis_id());
            break;
        case JOGGING:
            // TMCEventManager.trigger_event(JOG_STARTED);
            SerialComm::post_message(INFO, "Jog: started");
            break;
        case MOVING:
            // TMCEventManager.trigger_event(MOVE_STARTED);
            SerialComm::post_message(INFO, "Move: started");
            break;
    }

    step_timer.start();
}

inline void finish_move() {
    step_timer.stop();
    hold_timer.stop();
    reset_accelerator();
    StepperControl::internal::post_steppers_status(true);

    Stepper** stepper = steppers;
    while (*stepper) {
        (*stepper)->finish_move();
        stepper++;
    }
}

inline bool do_bresenham_step() {
    Stepper** stepper = steppers_sort;

    while (*stepper) {
        if (!(*stepper)->step(master_stepper)) {
            fault_stepper = *stepper;
            return false;
        }
        stepper++;
    }

    return true;
}


void StepperControl::internal::begin() {

    pulse_timer.begin([]{ pulse_ISR(); });
    step_timer.begin([]{ step_ISR(); }, 1000, false);
    hold_timer.begin([]{ hold_ISR(); }, 1000, false);

    SerialComm::register_command("ENABL", 2, false);
    SerialComm::register_command("MOVE", 1, &num_steppers);
    SerialComm::register_command("PROBE", 3);
    SerialComm::register_command("HOME", 2);
    SerialComm::register_command("FAULT", 0, false);
    SerialComm::register_command("ZERO", 1);
    SerialComm::register_command("STOP", 0, false);
    SerialComm::register_command("HALT", 0, false);
    SerialComm::register_command("LIMIT", 3);
    SerialComm::register_command("JOG", 1, false, &num_steppers);
    SerialComm::register_command("JOGC", 0, false);
    SerialComm::register_command("HOLD", 1);
    SerialComm::register_command("HOLDC", 0, false);

    // attach callbacks
    SerialComm::add_callback("ENABL", &ENABL__cb);
    SerialComm::add_callback("MOVE", &MOVE__cb);
    SerialComm::add_callback("PROBE", &PROBE__cb);
    SerialComm::add_callback("HOME", &HOME__cb);
    SerialComm::add_callback("FAULT", &CFAULT__cb);
    SerialComm::add_callback("ZERO", &ZERO__cb);
    SerialComm::add_callback("STOP", &STOP__cb);
    SerialComm::add_callback("HALT", &HALT__cb);
    SerialComm::add_callback("LIMIT", &LIMIT__cb);
    SerialComm::add_callback("JOG", &JOG__cb);
    SerialComm::add_callback("JOGC", &JOGC__cb);
    SerialComm::add_callback("HOLD", &HOLD__cb);
    SerialComm::add_callback("HOLDC", &HOLDC__cb);
}

void StepperControl::internal::sort_steppers() {
    std::sort(steppers_sort, steppers_sort + num_steppers, Stepper::cmp_delta);
    master_stepper = steppers_sort[0];
    for (uint8_t n = 0; n < num_steppers; n++) {
        steppers_sort[n]->init_delta_rem(master_stepper);
    }
}

void StepperControl::internal::post_steppers_status(bool queue = false) {
    static const char* const STEPPER_STATE_STRINGS[] = { STEPPER_STATES(MAKE_STRINGS) };

    char message[MESSAGE_BUFFER_SIZE];
    sprintf(message, "%s ", STEPPER_STATE_STRINGS[state]);

    if (state == HOLDING) {

        uint32_t msec = hold_ms;

        uint8_t sec = msec / 1000;
        msec %= 1000;
        uint16_t min = sec / 60;
        sec %= 60;

        sprintf(message + strlen(message), "Hold: %i:%i:%i (M:S:MS)", min, sec, msec);

    } else {

        char pos_buffer[7 * MAX_STEPPERS + MAX_STEPPERS];
        char speed_buffer[7 * MAX_STEPPERS + MAX_STEPPERS];

        for (uint8_t n = 0; n < num_steppers; n++) {
            Stepper* stepper = steppers[n];

            sprintf(pos_buffer + ((n == 0) ? 0 : strlen(pos_buffer)), "%f%s", stepper->get_position(), (n < (num_steppers - 1) ? "," : ""));
            sprintf(speed_buffer + ((n == 0) ? 0 : strlen(speed_buffer)), "%f%s", stepper->get_speed(), (n < (num_steppers - 1) ? "," : ""));
        }

        sprintf(message + strlen(message), "Pos:[%s] Speed:[%s]", pos_buffer, speed_buffer);
    }

    if (queue) { 
        SerialComm::queue_message(STATUS, message); 
    } else { 
        SerialComm::post_message(STATUS, message); 
    }
}

void StepperControl::internal::process() {
    switch (state) {
        case HOMING:
            switch (master_stepper->homing_status()) {
                case 1: // homing complete
                    NO_INTERRUPTS(
                        master_stepper->homing_complete();
                        change_state(steppers_homed() ? IDLE : HOME_FIRST);
                        finish_move();
                        SerialComm::post_message(INFO, "Homing: complete on axis %i", master_stepper->get_axis_id());
                    )
                    return;
                    
                case -1: // homing error
                    NO_INTERRUPTS(
                        change_state(FAULT);
                        finish_move();
                        SerialComm::queue_message(CRITICAL, "Homing: failed on axis %i", master_stepper->get_axis_id());
                    )
                    return;

                default:
                    return;
            }

        case PROBING:
            switch (master_stepper->probing_status()) {
                case 1: // probing complete
                    NO_INTERRUPTS(
                        change_state(IDLE);
                        finish_move();
                        SerialComm::queue_message(INFO, "Probing: complete on axis %i", master_stepper->get_axis_id());
                    )
                    return;

                case -1: // probing error
                    NO_INTERRUPTS(
                        change_state(FAULT);
                        finish_move();
                        SerialComm::queue_message(CRITICAL, "Probing: failed on axis %i", master_stepper->get_axis_id());
                    )
                    return;
            }

        default:
            return;
    }
}

bool StepperControl::check_state(StepperState state_) {
    return state_ == state;
}

void StepperControl::add_stepper(Stepper& stepper) {

    if (++num_steppers > MAX_STEPPERS) {
            SerialComm::post_message(ERROR, "Too many steppers initialized; 'MAX_STEPPERS' is set to %i", MAX_STEPPERS);
            abort();
        }

    steppers[num_steppers - 1] = &stepper;
    steppers_sort[num_steppers - 1] = &stepper;

    if (!stepper.homed()) change_state(HOME_FIRST); 
}

void StepperControl::start_move(float speed) {
    ASSERT_IDLE;
    change_state(MOVING);
    run_steppers(speed);
}

void StepperControl::start_home(Stepper* stepper, float speed) {
    if (stepper == nullptr) return;
    if (state != HOME_FIRST) {
        ASSERT_IDLE;
    }

    change_state(HOMING);
    stepper->prepare_homing();
    run_steppers(speed);
}

void StepperControl::start_home(uint8_t axis, float speed) {
    if (axis > num_steppers) return;
    start_home(steppers[axis], speed);
}

void StepperControl::start_probe(Stepper* stepper, float speed, int8_t dir) {
    if (stepper == nullptr) return;
    ASSERT_IDLE;
    change_state(PROBING);
    stepper->prepare_probing(dir);
    run_steppers(speed);
}

void StepperControl::start_probe(uint8_t axis, float speed, int8_t dir) {
    if (axis > num_steppers) return;
    start_probe(steppers[axis], speed, dir);
}

void StepperControl::start_jogging(float* unit_vectors, float speed) {
    ASSERT_IDLE;
    change_state(JOGGING);
    for (uint8_t n = 0; n < num_steppers; n++) {
        steppers[n]->prepare_jogging(unit_vectors[n]);
    }
    run_steppers(speed);
}

void StepperControl::stop_jogging() {
    if (state == JOGGING) stop();
}

void StepperControl::zero_stepper(Stepper* stepper) {
    ASSERT_IDLE;
    stepper->set_zero();
}

void StepperControl::zero_stepper(uint8_t axis) {
    if (axis > num_steppers) return;
    zero_stepper(steppers[axis]);
}

void StepperControl::stop() {
    if (steppers_active()) decelerate_now();
}

void StepperControl::halt() {
    if (steppers_active()) change_state(FAULT);
}

void StepperControl::clear_fault() {
    if (state != FAULT) return;
    fault_stepper = nullptr;

    bool home_first = false;

    Stepper** stepper = steppers;
    while (*stepper) {
        (*stepper)->reset_home();
        if (!(*stepper)->homed()) home_first = true;
        stepper++;
    }

    change_state(home_first ? HOME_FIRST : IDLE);
}

void StepperControl::hold(uint32_t milliseconds) {
    if (milliseconds == 0) clear_hold();
    ASSERT_IDLE;
    hold_ms = milliseconds;
    change_state(HOLDING);
    SerialComm::post_message(INFO, "Hold: started");
    hold_timer.start();
}

void StepperControl::clear_hold() {
    if (state != HOLDING) return;
    hold_timer.stop();
    hold_ms = 0;
    change_state(IDLE);
    SerialComm::post_message(INFO, "Hold: manually cleared");
}

uint32_t StepperControl::get_hold_time() {
    return hold_ms;
}

bool StepperControl::steppers_active() {
    return (state == MOVING || state == PROBING || state == HOMING || state == JOGGING);
}

bool StepperControl::steppers_homed() {
    Stepper** stepper = steppers;
    while (*stepper) {
        if (!(*stepper)->homed()) return false;
        stepper++;
    }
    return true;
}

float StepperControl::get_accelerator_speed() {
    return accelerator_speed();
}

bool StepperControl::steppers_accelerating() {
    return accelerating();
}

uint8_t StepperControl::get_num_steppers() {
    return num_steppers;
}

Stepper* StepperControl::get_master_stepper() {
    return master_stepper;
}

Stepper* StepperControl::get_faulted_stepper() {
    return fault_stepper;
}

Stepper* StepperControl::get_stepper(uint8_t axis) {
    return (axis < num_steppers) ? steppers[axis] : nullptr;
}

Stepper** StepperControl::get_all_steppers() {
    return steppers;
}

// ISRs

void step_ISR() {

    switch (state) {
        case HOME_FIRST:
        case FAULT:
        case IDLE:
            finish_move();
            break;

        case PROBING:
        case HOMING:
        case JOGGING:
        case MOVING:
        {
            float period = compute_next_step_period();
            step_timer.setPeriod(period); 

            if (!do_bresenham_step()) {
                change_state(FAULT);
                finish_move();
                SerialComm::queue_message(CRITICAL, "Fault: axis %i", fault_stepper->get_axis_id());

            } else if (master_stepper->move_complete() || (period < 0)) {

                if (state == JOGGING) { 
                    // jog complete
                    SerialComm::queue_message(INFO, "Jog: complete");
                } else {
                    // move complete
                    SerialComm::queue_message(INFO, "Move: complete");
                }

                change_state(IDLE);
                finish_move();
            }

            pulse_timer.trigger(PULSE_WIDTH_US);
        }
    }
}

// void step_ISR() {

//     switch (state) {
//         case HOME_FIRST:
//         case FAULT:
//         case IDLE:
//             finish_move();
//             break;

//         case PROBING:
//         case HOMING:
//         case JOGGING:
//         case MOVING:
//         {
//             float period = compute_next_step_period();
//             step_timer.setPeriod(period); 

//             if (!do_bresenham_step()) {
//                 change_state(FAULT);
//                 finish_move();
//                 SerialComm::clear_command_queue();
//                 // TMCEventManager.queue_event(FAULT_OCCURED);
//                 SerialComm::queue_message(CRITICAL, "Fault: axis %i", fault_stepper->get_axis_id());

//             } else if (state == HOMING) {

//                 switch (master_stepper->homing_status()) {
//                     case 1:
//                         // homing complete
//                         master_stepper->homing_complete();

//                         change_state(steppers_homed() ? IDLE : HOME_FIRST);
//                         finish_move();
//                         // TMCEventManager.queue_event(HOMING_COMPLETE);
//                         SerialComm::queue_message(INFO, "Homing: complete on axis %i", master_stepper->get_axis_id());
//                         break;
//                     case -1:
//                         // homing failure
//                         change_state(FAULT);
//                         finish_move();
//                         SerialComm::clear_command_queue();
//                         // TMCEventManager.queue_event(HOMING_FAILED);
//                         SerialComm::queue_message(CRITICAL, "Homing: failed on axis %i", master_stepper->get_axis_id());
//                         break;
//                     default:
//                         // no action
//                         break;
//                 }

//             } else if (state == PROBING) {

//                 switch (master_stepper->probing_status()) {
//                     case 1:
//                         // probing complete
//                         change_state(IDLE);
//                         finish_move();
//                         // TMCEventManager.queue_event(PROBING_COMPLETE);
//                         SerialComm::queue_message(INFO, "Probing: complete on axis %i", master_stepper->get_axis_id());
//                         break;
//                     case -1:
//                         // probing failure
//                         change_state(FAULT);
//                         finish_move();
//                         SerialComm::clear_command_queue();
//                         // TMCEventManager.queue_event(PROBING_FAILED);
//                         SerialComm::queue_message(CRITICAL, "Probing: failed on axis %i", master_stepper->get_axis_id());
//                         break;
//                     default:
//                         // no action
//                         break;
//                 }

//             } else if (master_stepper->move_complete() || (period < 0)) {

//                 if (state == JOGGING) { 

//                     // jog complete
//                     // TMCEventManager.queue_event(JOG_COMPLETE);
//                     SerialComm::queue_message(INFO, "Jog: complete");
//                 } else {
//                     // move complete
//                     // TMCEventManager.queue_event(MOVE_COMPLETE);
//                     SerialComm::queue_message(INFO, "Move: complete");
//                 }

//                 change_state(IDLE);
//                 finish_move();
//             }

//             pulse_timer.trigger(PULSE_WIDTH_US);
//         }
//     }
// }

void pulse_ISR() {
    Stepper** stepper = steppers_sort;

    while (*stepper) {
        (*stepper)->clear_step();
        stepper++;
    }
}

void hold_ISR() {
    if ((--hold_ms) == 0) {
        hold_timer.stop();
        change_state(IDLE);
        SerialComm::queue_message(INFO, "Hold: complete");
    }
}