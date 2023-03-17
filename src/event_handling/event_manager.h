#pragma once

#include <map>
#include <queue>
#include <Arduino.h>

#include "events.h"
#include "../config.h"


struct _event_callback {
    TMCEvents event;
    void (*callback)();
};


class _event_manager {

    public:
        
        _event_manager();

        void process_events();

        void queue_event(TMCEvents event);

        void attach_callback(TMCEvents event, void (*callback)());

    private:

        uint8_t num_callbacks;

        std::queue<TMCEvents> event_queue;
        _event_callback callbacks[MAX_EVENT_CALLBACKS];
};


extern _event_manager TMCEventManager;