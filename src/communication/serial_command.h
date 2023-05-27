#pragma once

#include <Arduino.h>
#include <Stream.h>
#include <queue>

#include "message_agent.h"
#include "arg_list.h"
#include "../config.h"
#include "../utility/fixed_queue.h"


bool argtoi(char* arg, int* res);

bool argtof(char* arg, float* res);

typedef void (*CommandCallback)(char*, ArgList*);


struct _command_entry {
    char name[CMD_CHAR_MAX + 1];

    bool queue_cmd;

    uint8_t n_args;
    uint8_t* n_var_args;
    uint8_t n_callbacks;

    CommandCallback callbacks[MAX_USER_CALLBACKS];
};


struct _command {

    ArgList args;
    _command_entry* entry;

    // char* name;

    // bool queue_cmd;

    // uint8_t n_args;
    // uint8_t* n_var_args;
    // uint8_t n_callbacks;

    
    // CommandCallback callbacks[MAX_USER_CALLBACKS];

    // CommandCallback* callbacks;
};


class _serial_command {

    public:
        _serial_command(Stream* stream);

        // poll the stream for incoming data
        void poll();

        // run any commands sitting in the queue
        void process_command_queue();

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
        void clear_queue();

        // call a command internally
        void run_cmd(char* cmd);

    private:
        Stream* stream;

        uint8_t n_cmds;
        _command_entry cmd_registry[MAX_USER_COMMANDS];
        FixedQueue<_command, CMD_QUEUE_SIZE> cmd_queue;

        char rx_buffer[RX_BUFFER_SIZE];

        // parse the recieved data and execute any attached commands
        void parse(char* data);

        // runs the command with the arguments
        void run_cmd(_command cmd);
};

extern _serial_command TMCSerialCommand;