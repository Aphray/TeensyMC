#include "serial_command.h"
#include "message_agent.h"

#define STR_CMP(a, b) (strcmp(a, b) == 0)

#ifndef CMD_DELIMITER
    #define CMD_DELIMITER ":"
#endif

#ifndef ARG_DELIMITER_CHAR
    #define ARG_DELIMITER_CHAR ','
#endif

#ifndef ARG_SKIP_CHAR
    #define ARG_SKIP_CHAR '*'
#endif

ArgList::ArgList(char* args_) {
    count = 0;
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
                if (args[count][0] != 0) { count++; }
                break;
            case ARG_DELIMITER_CHAR:
                *ptr = 0;
                ptr = args[(args[count][0] != 0 ? ++count : count)];
                break;
            default:
                *ptr++ = *args_;
                break;
        }
    } while (*args_++ && count < CMD_MAX_ARGS);
}


char* ArgList::next() {
    return args[arg_idx++];
}

void ArgList::reset() {
    arg_idx = 0;
}

_serial_command::_serial_command(Stream* stream_) {
    num_cmds = 0;
    stream = stream_;
}

void _serial_command::poll() {
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


void _serial_command::new_command(char* cmd, uint8_t num_args) {
    // checks
    if (strlen(cmd) > CMD_CHAR_MAX) { return; }
    if (num_cmds == MAX_USER_COMMANDS) { return; }

    // check to see if the command exists
    for (uint8_t n = 0; n < num_cmds; n ++) {
        // do nothing if the command already exists
        if (STR_CMP(cmd, commands[n].cmd)) { return; }
    }

    // build the command if it doesn't already exist
    _user_command* command = &commands[num_cmds++];
    command->num_args = num_args;
    strcpy(command->cmd, cmd);
}


void _serial_command::add_callback(char* cmd, CommandCallback callback) {
    // check to see if the command is exists
    for (uint8_t n = 0; n < num_cmds; n ++) {
        _user_command* command = &commands[n];

        if (STR_CMP(cmd, command->cmd)) {
            if (command->num_cbs == MAX_USER_CALLBACKS) { return; }

            // add the new callback to the callbacks list
            command->callbacks[command->num_cbs++] = callback;
            return;
        }
    }
}


void _serial_command::parse(char* data) {
    // get the command and make the arguments list
    char* cmd = strtok(data, CMD_DELIMITER);
    ArgList arg_list(strtok(NULL, CMD_DELIMITER));

    // loop through all the stored commands
    for (uint8_t n = 0; n < num_cmds; n++) {
        _user_command* command = &commands[n];

        // check if the parsed command matches the stored command
        if (STR_CMP(cmd, command->cmd)) {

            // check if the argument count matches
            if (arg_list.count == command->num_args) {

                // execute the callbacks and pass the arguments
                for (uint8_t i = 0; i < command->num_cbs; i ++) {
                    (*command->callbacks[i])(cmd, &arg_list);
                    arg_list.reset();
                }
                
            } else {
                // post error about argument mismatch
                TMCMessageAgent.post_message(ERROR, "Command <%s> requires (exactly) %i arg%s, but given %i", 
                        cmd, command->num_args, (command->num_args > 1) ? "s" : "", arg_list.count);
            }
            return;
        }
    }
    // command wasn't found...
    TMCMessageAgent.post_message(ERROR, "Command <%s> is unrecognized", cmd);
}