#include <algorithm>

#include "stepper_control.h"
#include "callbacks.h"
#include "../communication/serial_command.h"
#include "../event_handling/event_manager.h"

#define ASSERT_IDLE if (state != IDLE) return

_stepper_control::_stepper_control() {
    num_steppers = 0;
    
    #ifdef HOME_STEPPERS_FIRST
    prev_state = state = HOME_FIRST;
    #else
    prev_state = state = IDLE;
    #endif
}

void _stepper_control::begin() {

    // setup the timers
    pulse_timer.begin([this]{ this->pulse_ISR(); });
    step_timer.begin([this]{ this->step_ISR(); }, 1000, false);
    hold_timer.begin([this] {this->hold_ISR(); }, 1000, false);

    // add the serial commands
    TMCSerialCommand.register_command("ENABL", 2, false);
    TMCSerialCommand.register_command("MOVE", 2, &num_steppers);
    TMCSerialCommand.register_command("PROBE", 4);
    TMCSerialCommand.register_command("HOME", 3);
    TMCSerialCommand.register_command("FAULT", 0, false);
    TMCSerialCommand.register_command("ZERO", 0);
    TMCSerialCommand.register_command("STOP", 0, false);
    TMCSerialCommand.register_command("HALT", 0, false);
    TMCSerialCommand.register_command("LIMIT", 3);
    TMCSerialCommand.register_command("JOG", 2, &num_steppers);
    TMCSerialCommand.register_command("JOGC", 0, false);
    TMCSerialCommand.register_command("HOLD", 1);
    TMCSerialCommand.register_command("HOLDC", 0, false);
   

    // attach callbacks
    TMCSerialCommand.add_callback("ENABL", &ENABL__cb);
    TMCSerialCommand.add_callback("MOVE", &MOVE__cb);
    TMCSerialCommand.add_callback("PROBE", &PROBE__cb);
    TMCSerialCommand.add_callback("HOME", &HOME__cb);
    TMCSerialCommand.add_callback("FAULT", &CFAULT__cb);
    TMCSerialCommand.add_callback("ZERO", &ZERO__cb);
    TMCSerialCommand.add_callback("STOP", &STOP__cb);
    TMCSerialCommand.add_callback("HALT", &HALT__cb);
    TMCSerialCommand.add_callback("LIMIT", &LIMIT__cb);
    TMCSerialCommand.add_callback("JOG", &JOG__cb);
    TMCSerialCommand.add_callback("JOGC", &JOGC__cb);
    TMCSerialCommand.add_callback("HOLD", &HOLD__cb);
    TMCSerialCommand.add_callback("HOLDC", &HOLDC__cb);
}


void _stepper_control::sort_steppers() {
    std::sort(steppers_sort, steppers_sort + num_steppers, Stepper::cmp_delta);
    master_stepper = steppers_sort[0];
}

void _stepper_control::add_stepper(Stepper& stepper) {

    if (++num_steppers > MAX_STEPPERS) {
        TMCMessageAgent.post_message(ERROR, "Too many steppers initialized; 'MAX_STEPPERS' is set to %i", MAX_STEPPERS);
        abort();
    }

    steppers[num_steppers - 1] = &stepper;
    steppers_sort[num_steppers - 1] = &stepper;
}

void _stepper_control::start_move(float speed, float accel) {
    ASSERT_IDLE;
    change_state(MOVING);
    run_steppers(speed, accel);
}

void _stepper_control::start_home(uint8_t axis, float speed, float accel) {
    if (axis > num_steppers) return;
    _stepper_control::start_home(steppers[axis], speed, accel);
}

void _stepper_control::start_home(Stepper* stepper, float speed, float accel) {
    if (stepper == nullptr) return;
    if (state != HOME_FIRST) {
        ASSERT_IDLE;
    }

    change_state(HOMING);
    stepper->prepare_homing();
    run_steppers(speed, accel);
}

void _stepper_control::start_probe(uint8_t axis, float speed, float accel, int8_t dir) {
    if (axis > num_steppers) return;
    _stepper_control::start_probe(steppers[axis], speed, accel, dir);
}

void _stepper_control::start_probe(Stepper* stepper, float speed, float accel, int8_t dir) {
    if (stepper == nullptr) return;
    ASSERT_IDLE;
    change_state(PROBING);
    stepper->prepare_probing(dir);
    run_steppers(speed, accel);
}

void _stepper_control::start_jogging(float* unit_vectors, float speed, float accel) {
    ASSERT_IDLE;
    change_state(JOGGING);
    for (uint8_t n = 0; n < num_steppers; n++) {
        steppers[n]->prepare_jogging(unit_vectors[n]);
    }
    run_steppers(speed, accel);
}

void _stepper_control::stop_jogging() {
    if (state == JOGGING) stop();
}

void _stepper_control::zero_stepper(uint8_t axis) {
    if (axis > num_steppers) return;
    _stepper_control::zero_stepper(steppers[axis]);
}

void _stepper_control::zero_stepper(Stepper* stepper) {
    ASSERT_IDLE;
    stepper->set_zero();
}

void _stepper_control::stop() {
    if (steppers_active()) accelerator.decelerate_now();
}

void _stepper_control::halt() {
    if (steppers_active()) change_state(FAULT);
}

void _stepper_control::clear_fault() {
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

void _stepper_control::hold(uint32_t milliseconds) {
    if (milliseconds == 0) clear_hold();
    ASSERT_IDLE;
    hold_ms = milliseconds;
    change_state(HOLDING);
    TMCMessageAgent.post_message(INFO, "Hold: started");
    hold_timer.start();
}

void _stepper_control::clear_hold() {
    if (state != HOLDING) return;
    hold_timer.stop();
    hold_ms = 0;
    change_state(IDLE);
    TMCMessageAgent.post_message(INFO, "Hold: manually cleared");
}

bool _stepper_control::steppers_homed() {
    Stepper** stepper = steppers;
    while (*stepper) {
        if (!(*stepper)->homed()) return false;
        stepper++;
    }
    return true;
}

void _stepper_control::post_steppers_status(bool queue = false) {
    static const char* const STEPPER_STATE_STRINGS[] = { STEPPER_STATES(MAKE_STRINGS) };

    char message[MESSAGE_BUFFER_SIZE];
    sprintf(message, "(%s)", STEPPER_STATE_STRINGS[state]);

    if (state == HOLDING) {

        uint32_t msec = hold_ms;

        uint8_t sec = msec / 1000;
        msec %= 1000;
        uint16_t min = sec / 60;
        sec %= 60;

        sprintf(message + strlen(message), "Hold: %i:%i:%i (M:S:MS)", min, sec, msec);

    } else {
        Stepper** stepper = steppers;
        while (*stepper) {
            sprintf(message + strlen(message), " AX%i:(%f,%f)", 
                (*stepper)->get_axis_id(), (*stepper)->get_position(), (*stepper)->get_speed());

            stepper++;
        }
    }

    if (queue) { 
        TMCMessageAgent.queue_message(STATUS, message); 
    } else { 
        TMCMessageAgent.post_message(STATUS, message); 
    }
}

float _stepper_control::get_accelerator_speed() {
    return accelerator.get_speed();
}

uint8_t _stepper_control::get_num_steppers() {
    return num_steppers;
}

Stepper* _stepper_control::get_master_stepper() {
    return master_stepper;
}

Stepper* _stepper_control::get_faulted_stepper() {
    return fault_stepper;
}

Stepper* _stepper_control::get_stepper(uint8_t axis) {
    return (axis < num_steppers) ? steppers[axis] : nullptr;
}

Stepper* _stepper_control::get_next_stepper() {
    static uint8_t idx = 0;
    if (idx == num_steppers) {
        idx = 0;
        return nullptr;
    } else {
        return steppers[idx++];
    }
}

Stepper** _stepper_control::get_all_steppers() {
    return steppers;
}

void _stepper_control::step_ISR() {

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

            float period = accelerator.compute_next_step_period();
            step_timer.setPeriod(period); 

            if (!do_bresenham_step()) {
                change_state(FAULT);
                finish_move();
                TMCSerialCommand.clear_queue();
                TMCEventManager.queue_event(FAULT_OCCURED);
                TMCMessageAgent.queue_message(CRITICAL, "Fault: axis %i", fault_stepper->get_axis_id());

            } else if (state == HOMING) {

                switch (master_stepper->homing_status()) {
                    case 1:
                        // homing complete
                        master_stepper->homing_complete();

                        change_state(IDLE);
                        finish_move();
                        TMCEventManager.queue_event(HOMING_COMPLETE);
                        TMCMessageAgent.queue_message(INFO, "Homing: complete on axis %i", master_stepper->get_axis_id());
                        break;
                    case -1:
                        // homing failure
                        change_state(FAULT);
                        finish_move();
                        TMCSerialCommand.clear_queue();
                        TMCEventManager.queue_event(HOMING_FAILED);
                        TMCMessageAgent.queue_message(CRITICAL, "Homing: failed on axis %i", master_stepper->get_axis_id());
                        break;
                    default:
                        // no action
                        break;
                }

            } else if (state == PROBING) {

                switch (master_stepper->probing_status()) {
                    case 1:
                        // probing complete
                        change_state(IDLE);
                        finish_move();
                        TMCEventManager.queue_event(PROBING_COMPLETE);
                        TMCMessageAgent.queue_message(INFO, "Probing: complete on axis %i", master_stepper->get_axis_id());
                        break;
                    case -1:
                        // probing failure
                        change_state(FAULT);
                        finish_move();
                        TMCSerialCommand.clear_queue();
                        TMCEventManager.queue_event(PROBING_FAILED);
                        TMCMessageAgent.queue_message(CRITICAL, "Probing: failed on axis %i", master_stepper->get_axis_id());
                        break;
                    default:
                        // no action
                        break;
                }

            } else if (master_stepper->move_complete() || (period < 0)) {

                if (state == JOGGING) { 

                    // jog complete
                    TMCEventManager.queue_event(JOG_COMPLETE);
                    TMCMessageAgent.queue_message(INFO, "Jog: complete");
                } else {
                    // move complete
                    TMCEventManager.queue_event(MOVE_COMPLETE);
                    TMCMessageAgent.queue_message(INFO, "Move: complete");
                }

                change_state(IDLE);
                finish_move();
            }

            pulse_timer.trigger(PULSE_WIDTH_US);
        }
    }
}

void _stepper_control::pulse_ISR() {
    Stepper** stepper = steppers_sort;

    while (*stepper) {
        (*stepper)->clear_step();
        stepper++;
    }
}

void _stepper_control::hold_ISR() {
    if ((--hold_ms) == 0) {
        hold_timer.stop();
        change_state(IDLE);
        TMCMessageAgent.queue_message(INFO, "Hold: complete");
    }
}

void _stepper_control::run_steppers(float speed, float accel) {

    if (master_stepper->get_delta_steps() == 0) { 
        restore_state();
        return; 
    }

    float start_speed = 0;
    speed = master_stepper->cvt_to_steps(speed);
    accel = master_stepper->cvt_to_steps(accel);

    Stepper** stepper = steppers;
    while (*stepper) {
        (*stepper)->constrain_speed_accel(master_stepper, &start_speed, &speed, &accel);
        stepper++;
    }

    accelerator.prepare((state == JOGGING ? std::numeric_limits<uint32_t>::max() : master_stepper->get_delta_steps()), start_speed, speed, accel);
    step_timer.setPeriod(1);

    switch (state) {
        case PROBING:
            TMCEventManager.trigger_event(PROBING_STARTED);
            TMCMessageAgent.post_message(INFO, "Probing: started on axis %i", master_stepper->get_axis_id());
            break;
        case HOMING:
            TMCEventManager.trigger_event(HOMING_STARTED);
            TMCMessageAgent.post_message(INFO, "Homing: started on axis %i", master_stepper->get_axis_id());
            break;
        case JOGGING:
            TMCEventManager.trigger_event(JOG_STARTED);
            TMCMessageAgent.post_message(INFO, "Jog: started");
            break;
        case MOVING:
            TMCEventManager.trigger_event(MOVE_STARTED);
            TMCMessageAgent.post_message(INFO, "Move: started");
            break;
    }

    step_timer.start();
}