#pragma once

#include "Stream.h"
#include "arg_list.h"
#include "../config.h"
#include "enum_factory.h"


#define MESSAGE_LEVELS(X)   \
    X(STATUS)               \
    X(INFO)                 \
    X(DEBUG)                \
    X(ERROR)                \
    X(WARNING)              \
    X(CRITICAL)


namespace TeensyMC { namespace SerialComm {

    typedef void (*CommandCallback)(char*, ArgList*);

    enum MessageLevel {
        MESSAGE_LEVELS(MAKE_ENUM)
    };

    struct Message {
        char buffer[MESSAGE_BUFFER_SIZE];

        Message() {};

        template<typename... Args>
        Message(MessageLevel level, char* format, Args... args) {
            static const char* const MESSAGE_LEVEL_STRINGS[] = { MESSAGE_LEVELS(MAKE_STRINGS) };

            sprintf(buffer, "[%s] ", MESSAGE_LEVEL_STRINGS[level]);
            sprintf(buffer + strlen(buffer), format, args...);
        }

        // print message
        void print();

        // queue message
        void queue();
    };

    struct CommandEntry {
        char name[CMD_CHAR_MAX + 1];

        bool queue_cmd;

        uint8_t n_args;
        uint8_t* n_var_args;
        uint8_t n_callbacks;

        CommandCallback callbacks[MAX_USER_CALLBACKS];
    };

    struct Command {
        ArgList args;
        CommandEntry* entry;
        char* name = entry->name;

        // run the command
        void run();
    };


    namespace internal {

        // print the queued messages to the serial
        void post_queued_messages();

        // poll the stream for incoming data
        void poll_serial();

        // run any commands sitting in the queue
        void process_command_queue();

    } // namespace internal

    template<typename... Args>
    void post_message(MessageLevel level, char* format, Args... args) {
        Message message(level, format, args...);
        message.print();
    }

    template<typename... Args>
    void queue_message(MessageLevel level, char* format, Args... args) {
        Message message(level, format, args...);
        message.queue();
    }

    // add a user-defined command and callback that can be executed via serial commands
    void register_command(char* cmd_name, uint8_t args);
    // add a user-defined command and callback that can be executed via serial commands
    void register_command(char* cmd_name, uint8_t args, bool queue);
    // add a user-defined command and callback that can be executed via serial commands
    void register_command(char* cmd_name, uint8_t args, uint8_t* variable_args);
    // add a user-defined command and callback that can be executed via serial commands
    void register_command(char* cmd_name, uint8_t args, bool queue, uint8_t* variable_args);

    // attach callback to the specified command
    void add_callback(char* cmd_name, CommandCallback callback);

    // clears the command queue
    void clear_command_queue();

    // call a command internally
    void run_command(char* cmd);

}} // namespace TeensyMC::SerialComm