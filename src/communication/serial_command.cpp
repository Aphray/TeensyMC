#include "serial_command.h"
#include "../stepper_control/stepper_control.h"

#define STR_CMP(a, b) (strcmp(a, b) == 0)

bool argtoi(char* arg, int* res) {
    int8_t sign = 1;

    if (arg == nullptr) { return false; }

    for (uint8_t n = 0; n < strlen(arg); n++) {
        switch (arg[n]) {
            case '-':
            case '+':
                if (n != 0) { return false; }
                sign = (arg[n] == '+') ? 1 : -1;
                break;
            case '.':
                return false;
            default:
                if (!isdigit(arg[n])) { return false; }
                break;
        }
    }

    *res = atoi(arg);
    return true;

}

bool argtof(char* arg, float* res) {
    int8_t sign = 1;
    uint8_t decimals = 0;

    if (arg == nullptr) { return false; }

    for (uint8_t n = 0; n < strlen(arg); n++) {
        switch (arg[n]) {
            case '-':
            case '+':
                if (n != 0) { return false; }
                sign = (arg[n] == '+') ? 1 : -1;
                break;
            case '.':
                if (++decimals > 1) { return false; }
                break;
            default:
                if (!isdigit(arg[n])) { return false; }
                break;
        }
    }

    *res = atof(arg);
    return true;
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

_serial_command::_serial_command(Stream* stream_) {
    n_cmds = 0;
    stream = stream_;
}

void _serial_command::poll() {
    static char* rx_ptr = rx_buffer;

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
                rx_ptr = rx_buffer;
                break;
            default:
                // add character to buffer
                *rx_ptr++ = c;
        }
    }
}

void _serial_command::process_command_queue() {
    // if (TMCStepperControl.steppers_active() || TMCStepperControl.steppers_holding() || cmd_queue.empty()) return;
    if (!TMCStepperControl.check_state(IDLE, HOME_FIRST) || cmd_queue.empty()) return;

    _command* cmd = cmd_queue.pop();
    run_cmd(cmd);
}

void _serial_command::register_command(char* cmd_name, uint8_t args, bool queue = true, uint8_t* variable_args = nullptr) {
    // checks
    if (strlen(cmd_name) > CMD_CHAR_MAX) return;
    if (n_cmds == MAX_USER_COMMANDS) return;

    // check to see if the command exists
    for (uint8_t n = 0; n < n_cmds; n ++) {

        if (STR_CMP(cmd_name, cmd_registry[n].name)) { 
            // if command exists, report an error and return
            TMCMessageAgent.post_message(ERROR, "Command <%s> already registered", cmd_name);
            return; 
        }
    }

    // register the command when it doesn't exist in the registry
    _command* reg_cmd = &cmd_registry[n_cmds++];

    // assign the command info
    strcpy(reg_cmd->name, cmd_name);
    reg_cmd->queue = queue;
    reg_cmd->n_args = args;
    reg_cmd->n_var_args = variable_args;
}


void _serial_command::add_callback(char* cmd_name, CommandCallback callback) {
    // check to see if the command is exists
    for (uint8_t n = 0; n < n_cmds; n ++) {
        // _user_command* command = &user_cmds[n];
        _command* reg_cmd = &(cmd_registry[n]);

        if (STR_CMP(cmd_name, reg_cmd->name)) {
            if (reg_cmd->n_callbacks == MAX_USER_CALLBACKS) { 
                TMCMessageAgent.post_message(ERROR, "Command <%s> reached user callback limit (%i)", cmd_name, MAX_USER_CALLBACKS);
                return; 
            }

            // add the new callback to the callbacks list
            reg_cmd->callbacks[reg_cmd->n_callbacks++] = callback;
            return;
        }
    }

    // command wasn't found...
    TMCMessageAgent.post_message(ERROR, "Command <%s> unrecognized; register command before adding callbacks", cmd_name);
}

void _serial_command::clear_queue() {
    cmd_queue.clear();
}

void _serial_command::parse(char* data) {
    // get the command and make the arguments list
    char* cmd_name = strtok(data, CMD_DELIMITER);
    ArgList arg_list(strtok(NULL, CMD_DELIMITER));

    // loop through all the stored commands
    for (uint8_t n = 0; n < n_cmds; n++) {
        _command* cmd = &(cmd_registry[n]);

        // check if the parsed command matches a stored command
        if (STR_CMP(cmd_name, cmd->name)) {

            // number of arguments needed
            uint8_t n_args = (cmd->n_var_args != nullptr) ? *(cmd->n_var_args) + cmd->n_args : cmd->n_args;

            if (cmd->n_var_args) {
                Serial.println(*(cmd->n_var_args));
            }

            // check if the argument count matches
            if (arg_list.get_num_args() == n_args) {
                cmd->args.copy(&arg_list);
                
                if (!cmd->queue) {
                    run_cmd(cmd);
                } else if (!cmd_queue.full()) {
                    cmd_queue.push(cmd);

                    if (cmd_queue.full()) {
                        TMCMessageAgent.post_message(INFO, "Command queue: full");
                    } else {
                        TMCMessageAgent.post_message(INFO, "Command queue: %i available", cmd_queue.available());
                    }

                } else {
                    TMCMessageAgent.post_message(ERROR, "Cannot queue cmd <%s>; command queue is full", data);
                }
                
            } else {
                // post error about argument mismatch
                TMCMessageAgent.post_message(ERROR, "Command <%s> requires (exactly) %i arg%s, but given %i", 
                        cmd_name, n_args, ((n_args > 1) ? "s" : ""), arg_list.get_num_args());
            }
            return;
        }
    }
    
    // command wasn't found...
    TMCMessageAgent.post_message(ERROR, "Command <%s> unrecognized", cmd_name);
}

void _serial_command::run_cmd(_command* cmd) {
    // execute the callbacks and pass the arguments
    for (uint8_t i = 0; i < cmd->n_callbacks; i ++) {
        (*cmd->callbacks[i])(cmd->name, &cmd->args);
        cmd->args.reset();
    }
}