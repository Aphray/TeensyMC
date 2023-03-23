#pragma once

#include <Arduino.h>

#include "events.h"
#include "../config.h"
#include "../utility/fixed_queue.h"


struct _event_callback {
    StepperEvents event;
    void (*function)();
};


class _event_manager {

    public:
        
        _event_manager();

        void process_queued_events();

        void queue_event(StepperEvents event);

        void trigger_event(StepperEvents event);

        void attach_callback(StepperEvents event, void (*callback)());

    private:

        uint8_t n_callbacks;

        FixedQueue<StepperEvents, 5> event_queue;
        _event_callback callbacks[MAX_EVENT_CALLBACKS];

        void run_event_callbacks(StepperEvents event);
};


extern _event_manager TMCEventManager;