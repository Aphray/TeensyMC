#pragma once

#include <Arduino.h>
#include <Stream.h>

#ifndef CMD_BUFFER_SIZE
    #define CMD_BUFFER_SIZE 256
#endif

#ifndef CMD_MAX_ARGS
    #define CMD_MAX_ARGS 10
#endif

class ArgList {
    public:
        uint8_t count;

        ArgList(char* args);

        char* next();
    
    private:
        char *args;
};

class SerialCommand {

    public:
        SerialCommand(Stream* stream);

        void poll();

    private:
        Stream* stream;

        char rx_buffer[CMD_BUFFER_SIZE];

        void parse(char* data);

};