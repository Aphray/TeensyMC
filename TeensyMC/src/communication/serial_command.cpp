#include "serial_command.h"

#ifndef CMD_DELIMITER
    #define CMD_DELIMITER ":"
#endif

#ifndef ARG_DELIMITER_CHAR
    #define ARG_DELIMITER_CHAR ','
#endif

ArgList::ArgList(char* args_) {
    if (args == nullptr) count = 0;
    args = args_;
}

char* ArgList::next() {
    if (count == 0) return nullptr;

    static char* ptr = &(args[0]);
    while (*ptr != ARG_DELIMITER_CHAR) {

    }
}


SerialCommand::SerialCommand(Stream* stream_) {
    stream = stream_;
}

void SerialCommand::poll() {
    static char* rx_ptr = &rx_buffer[0];

    while (stream->available() > 0) {
        char c = stream->read();
        switch (c) {
            case '\r':
                // ignore carriage return 
                break;
            case '\n':
                // end of command
                // null-terminate the buffer and parse the data
                *rx_ptr = 0;
                parse(rx_buffer);

                // zero the buffer and reset the pointer
                rx_buffer[0] = 0;
                rx_ptr = &rx_buffer[0];
                break;
            default:
                // add character to buffer
                *rx_ptr++ = c;
        }
    }
}

void SerialCommand::parse(char* data) {
    char* cmd = strtok(data, CMD_DELIMITER);
    ArgList arg_list(strtok(NULL, CMD_DELIMITER));
}