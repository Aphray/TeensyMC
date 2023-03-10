#pragma once

#include <queue>
#include "Stream.h"
#include "enum_factory.h"
#include "../TMC_default_config.h"

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


struct _message {
    char buffer[MESSAGE_BUFFER_SIZE];

    template<typename... Args>
    _message(MessageLevel level, char* format, Args... args) {
        static const char* const MESSAGE_LEVEL_STRINGS[] = { MESSAGE_LEVELS(MAKE_STRINGS) };

        sprintf(buffer, "[%s] ", MESSAGE_LEVEL_STRINGS[level]);
        sprintf(buffer + strlen(buffer), format, args...);
    }
};


class _message_agent {

    public:
        _message_agent(Stream* stream);

        template<typename... Args>
        void post_message(MessageLevel level, char* format, Args... args) {
            _message message(level, format, args...);
            stream->println(message.buffer);
        }

        template<typename... Args>
        void queue_message(MessageLevel level, char* format, Args... args) {
            _message message(level, format, args...);
            message_queue.push(message);
        }

        void post_queued_messages();

    private:

        Stream* stream;

        std::queue<_message> message_queue;
};


extern _message_agent TMCMessageAgent;