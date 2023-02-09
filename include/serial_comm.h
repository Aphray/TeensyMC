#pragma once

#include <queue>
#include "enum_factory.h"

#define MESSAGE_MAX_LEN 256

#define MESSAGE_LEVELS(X)   \
    X(STATUS)               \
    X(INFO)                 \
    X(DEBUG)                \
    X(ERROR)                \
    X(WARNING)              \
    X(CRITICAL)

enum MessageLevel {
    MESSAGE_LEVELS(MAKE_ENUM)
};

enum Command {
    UNKNOWN,
    START_MOVE,
    START_HOME,
    START_PROBE,
    CLEAR_FAULT,
    SET_ZERO,
    STOP,
    HALT,
    RESET
};

struct Message {
    char buffer[MESSAGE_MAX_LEN];

    template<typename... Args>
    Message(MessageLevel level, char* format, Args... args) {
        static const char* const MESSAGE_LEVEL_STRINGS[] = { MESSAGE_LEVELS(MAKE_STRINGS) };

        sprintf(buffer, "[%s] ", MESSAGE_LEVEL_STRINGS[level]);
        sprintf(buffer + strlen(buffer), format, args...);
    }

    inline void post() {
        Serial.println(buffer);
    }
};

namespace SerialCommunication {

    extern std::queue<Message> message_queue;

    void read_serial();

    void post_queued_messages();

    void report_realtime_status();

    void queue_realtime_status();

    template<typename... Args>
    void post_message(MessageLevel level, char* format, Args... args) {
        Message message(level, format, args...);
        message.post();
    }

    template<typename... Args>
    void queue_message(MessageLevel level, char* format, Args... args) {
        Message message(level, format, args...);
        message_queue.push(message);
    }
}