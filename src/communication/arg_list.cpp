#include "arg_list.h"

ArgList::ArgList() {
    num_args = 0;
    arg_idx = 0;
}

ArgList::ArgList(char* args_) {
    num_args = 0;
    arg_idx = 0;

    if (args_ != nullptr) {
        split_args(args_);
    }
}

void ArgList::split_args(char* args_) {
    char* ptr = args[0];

    do {
        switch (*args_) {
            case 0:
                *ptr = 0;
                if (args[num_args][0] != 0) { num_args++; }
                break;
            case ARG_DELIMITER_CHAR:
                *ptr = 0;
                ptr = args[(args[num_args][0] != 0 ? ++num_args : num_args)];
                break;
            default:
                *ptr++ = *args_;
                break;
        }
    } while (*args_++ && num_args < CMD_MAX_ARGS);
}

char* ArgList::next() {
    return (arg_idx < num_args) ? args[arg_idx++] : nullptr;
}

void ArgList::reset() {
    arg_idx = 0;
}

uint8_t ArgList::get_num_args() {
    return num_args;
}

void ArgList::copy(ArgList* arg_list) {
    num_args = arg_list->num_args;
    for (uint8_t n = 0; n < num_args; n++) {
        strcpy(args[n], arg_list->args[n]);
    }
}