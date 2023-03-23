#pragma once

#include <Arduino.h>
#include <Stream.h>
#include <queue>

#include "message_agent.h"
#include "../config.h"
#include "../utility/fixed_queue.h"

#define CALLBACK(name) void name##__cb(char* cmd, ArgList* args)

inline void ARG_ERROR(char* arg) {
    TMCMessageAgent.post_message(ERROR, "Command error; invalid argument (%s)", arg);
}

bool argtoi(char* arg, int* res);

bool argtof(char* arg, float* res);


class ArgList {
    public:
        ArgList() {};
        ArgList(char* args);

        // get the next argument in the list
        char* next();

        // reset the indexer
        void reset();

        // returns the number of arguments 
        uint8_t get_num_args();

        // copy from another argument list
        void copy(ArgList* arg_list);
    
    private:
        uint8_t arg_idx;
        uint8_t num_args;

        char args[CMD_MAX_ARGS][ARG_BUFFER_SIZE];

        // split the args c-string into 
        void split_args(char* args);
};


typedef void (*CommandCallback)(char*, ArgList*);


struct _command {
    char name[CMD_CHAR_MAX + 1];

    bool queue;

    uint8_t n_args;
    uint8_t* n_var_args;
    uint8_t n_callbacks;

    ArgList args;
    CommandCallback callbacks[MAX_USER_CALLBACKS];
};


class _serial_command {

    public:
        _serial_command(Stream* stream);

        // poll the stream for incoming data
        void poll();

        // run any commands sitting in the queue
        void process_command_queue();

        // add a user-defined command and callback that can be executed via serial commands
        void register_command(char* cmd_name, uint8_t args, bool queue = true, uint8_t* variable_args = nullptr);

        // attach callback to the specified command
        void add_callback(char* cmd_name, CommandCallback callback);

        // clears the command queue
        void clear_queue();

    private:
        Stream* stream;

        uint8_t n_cmds;
        _command cmd_registry[MAX_USER_COMMANDS];
        FixedQueue<_command*, CMD_QUEUE_SIZE> cmd_queue;

        char rx_buffer[RX_BUFFER_SIZE];

        // parse the recieved data and execute any attached commands
        void parse(char* data);

        // runs the command with the arguments
        void run_cmd(_command* cmd);
};

extern _serial_command TMCSerialCommand;