#pragma once

#include <Arduino.h>
#include <Stream.h>


#ifndef RX_BUFFER_SIZE
    #define RX_BUFFER_SIZE 256
#endif

#ifndef CMD_CHAR_MAX
    #define CMD_CHAR_MAX 5
#endif

#ifndef MAX_USER_COMMANDS
    #define MAX_USER_COMMANDS 20
#endif

#ifndef CMD_MAX_ARGS
    #define CMD_MAX_ARGS 10
#endif

#ifndef ARG_BUFFER_SIZE
    #define ARG_BUFFER_SIZE 12
#endif


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


struct _command {
    char cmd[CMD_CHAR_MAX + 1];
    uint8_t num_args;
    CommandCallback callback;
};


class _serial_command {

    public:
        _serial_command(Stream* stream);

        // poll the stream for incoming data
        void poll();

        // add a user-defined command and callback that can be executed via serial commands
        void add_command(char* cmd, uint8_t num_args, CommandCallback callback);

    private:
        Stream* stream;

        uint8_t command_idx;
        _command commands[MAX_USER_COMMANDS];

        char rx_buffer[RX_BUFFER_SIZE];

        // parse the recieved data and execute any attached commands
        void parse(char* data);
};

extern _serial_command SerialCommand;