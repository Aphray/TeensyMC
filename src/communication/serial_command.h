#pragma once

#include <Arduino.h>
#include <Stream.h>
#include "message_agent.h"
#include "../TMC_default_config.h"

#define CALLBACK(name) void name##__cb(char* cmd, ArgList* args)

inline void ARG_ERROR(char* arg) {
    TMCMessageAgent.post_message(ERROR, "Command error; invalid argument (%s)", arg);
}

bool argtoi(char* arg, int* res);

bool argtof(char* arg, float* res);


class ArgList {
    public:
        ArgList(char* args);

        // get the next argument in the list
        char* next();

        // reset the indexer
        void reset();

        // returns the number of arguments 
        uint8_t get_num_args();
    
    private:
        uint8_t arg_idx;
        uint8_t num_args;

        char args[CMD_MAX_ARGS][ARG_BUFFER_SIZE];

        // split the args c-string into 
        void split_args(char* args);
};


typedef void (*CommandCallback)(char*, ArgList*);


struct _user_command {
    uint8_t num_cbs;
    uint8_t static_args;
    uint8_t* dynamic_args;

    char cmd[CMD_CHAR_MAX + 1];

    CommandCallback callbacks[MAX_USER_CALLBACKS];
};


class _serial_command {

    public:
        _serial_command(Stream* stream);

        // poll the stream for incoming data
        void poll();

        // add a user-defined command and callback that can be executed via serial commands
        void register_command(char* cmd, uint8_t static_args, uint8_t* dynamic_args = nullptr);

        // void register_command_dynamic(char* cmd, uint8_t* num_args);

        // attach callback to the specified command
        void add_callback(char* cmd, CommandCallback callback);

    private:
        Stream* stream;

        uint8_t num_cmds;
        _user_command commands[MAX_USER_COMMANDS];

        char rx_buffer[RX_BUFFER_SIZE];

        // parse the recieved data and execute any attached commands
        void parse(char* data);
};

extern _serial_command TMCSerialCommand;