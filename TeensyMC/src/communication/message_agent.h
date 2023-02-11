#pragma once

#include <queue>
#include "Stream.h"
#include "../enum_factory.h"

#ifndef MESSAGE_BUFFER_SIZE
    #define MESSAGE_BUFFER_SIZE 256
#endif

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


struct Message {
    char buffer[MESSAGE_BUFFER_SIZE];

    template<typename... Args>
    Message(MessageLevel level, char* format, Args... args) {
        static const char* const MESSAGE_LEVEL_STRINGS[] = { MESSAGE_LEVELS(MAKE_STRINGS) };

        sprintf(buffer, "[%s] ", MESSAGE_LEVEL_STRINGS[level]);
        sprintf(buffer + strlen(buffer), format, args...);
    }
};


class MessageAgent {

    public:

        MessageAgent(Stream* stream);

        template<typename... Args>
        void post_message(MessageLevel level, char* format, Args... args) {
            Message message(level, format, args...);
            stream->println(message.buffer);
        }

        template<typename... Args>
        void queue_message(MessageLevel level, char* format, Args... args) {
            Message message(level, format, args...);
            message_queue.push(message);
        }

        void post_queued_messages();

    private:

        Stream* stream;

        std::queue<Message> message_queue;
};