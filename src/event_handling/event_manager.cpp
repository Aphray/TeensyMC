#include "event_manager.h"
#include "../communication/message_agent.h"


_event_manager::_event_manager() {
    n_callbacks = 0;
}

void _event_manager::queue_event(StepperEvents event) {
    event_queue.push(event);
}

void _event_manager::process_queued_events() {
    if (event_queue.empty()) return;

    StepperEvents event = event_queue.pop();
    run_event_callbacks(event);
}

void _event_manager::trigger_event(StepperEvents event) {
    run_event_callbacks(event);
}

void _event_manager::attach_callback(StepperEvents event, void (*function)()) {
    if (n_callbacks == MAX_EVENT_CALLBACKS) {
        TMCMessageAgent.post_message(ERROR, "Reached max event callbacks (%i); increase 'MAX_EVENT_CALLBACKS' in the config file", MAX_EVENT_CALLBACKS);
        return;
    }

    _event_callback* callback = &(callbacks[n_callbacks++]);
    callback->event = event;
    callback->function = function;
}

void _event_manager::run_event_callbacks(StepperEvents event) {
    for (uint8_t n = 0; n < n_callbacks; n++) {
        if (callbacks[n].event == event) {
            callbacks[n].function();
        }
    }
}