#pragma once

#include <Arduino.h>
#include "../config.h"


class ArgList {
    public:
        ArgList();
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