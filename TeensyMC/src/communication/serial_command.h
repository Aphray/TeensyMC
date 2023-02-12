#pragma once

#include <Arduino.h>
#include <Stream.h>
#include "../default_config.h"


class ArgList {
    public:
        uint8_t count;

        ArgList(char* args);

        // get the next argument in the list
        char* next();

        // reset the indexer
        void reset();
    
    private:
        uint8_t arg_idx;
        char args[CMD_MAX_ARGS][ARG_BUFFER_SIZE];

        // split the args c-string into 
        void split_args(char* args);
};


typedef void (*CommandCallback)(char*, ArgList*);


struct _user_command {
    uint8_t num_args;
    uint8_t num_cbs;
    char cmd[CMD_CHAR_MAX + 1];

    CommandCallback callbacks[MAX_USER_CALLBACKS];
};


class _serial_command {

    public:
        _serial_command(Stream* stream);

        // poll the stream for incoming data
        void poll();

        // add a user-defined command and callback that can be executed via serial commands
        void new_command(char* cmd, uint8_t num_args);

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