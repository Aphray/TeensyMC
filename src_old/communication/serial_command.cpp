#include "serial_command.h"
#include "message_agent.h"
#include "arg_list.h"
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
                if (rx_buffer[0] != 0) parse(rx_buffer);

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

    Command cmd = cmd_queue.pop();
    run_cmd(cmd);
    TMCMessageAgent.post_message(INFO, "Command queue: %i / %i", cmd_queue.size(), cmd_queue.max_size());
}

void _serial_command::register_command(char* cmd_name, uint8_t args) {
    register_command(cmd_name, args, true, nullptr);
}
void _serial_command::register_command(char* cmd_name, uint8_t args, bool queue) {
    register_command(cmd_name, args, queue, nullptr);
}

void _serial_command::register_command(char* cmd_name, uint8_t args, uint8_t* variable_args) {
    register_command(cmd_name, args, true, variable_args);
}

void _serial_command::register_command(char* cmd_name, uint8_t args, bool queue, uint8_t* variable_args) {
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
    CommandEntry* reg_cmd = &cmd_registry[n_cmds++];

    // assign the command info
    strcpy(reg_cmd->name, cmd_name);
    reg_cmd->queue_cmd = queue;
    reg_cmd->n_args = args;
    reg_cmd->n_var_args = variable_args;
}


void _serial_command::add_callback(char* cmd_name, CommandCallback callback) {
    // check to see if the command is exists
    for (uint8_t n = 0; n < n_cmds; n ++) {
        CommandEntry* cmd_entry = &(cmd_registry[n]);

        if (STR_CMP(cmd_name, cmd_entry->name)) {
            if (cmd_entry->n_callbacks == MAX_USER_CALLBACKS) { 
                TMCMessageAgent.post_message(ERROR, "Command <%s> reached user callback limit (%i)", cmd_name, MAX_USER_CALLBACKS);
                return; 
            }

            // add the new callback to the callbacks list
            cmd_entry->callbacks[cmd_entry->n_callbacks++] = callback;
            return;
        }
    }

    // command wasn't found...
    TMCMessageAgent.post_message(ERROR, "Command <%s> unrecognized; register command before adding callbacks", cmd_name);
}

void _serial_command::clear_queue() {
    cmd_queue.clear();
    TMCMessageAgent.queue_message(INFO, "Command queue: %i / %i", cmd_queue.size(), cmd_queue.max_size());
}

void _serial_command::parse(char* data) {
    
    TMCMessageAgent.post_message(DEBUG, "Echo: <%s>", data);

    // get the command and make the arguments list
    char* cmd_name = strtok(data, CMD_DELIMITER);
    ArgList arg_list(strtok(NULL, CMD_DELIMITER));

    // loop through all the stored commands
    for (uint8_t n = 0; n < n_cmds; n++) {
        CommandEntry* cmd_entry = &(cmd_registry[n]);

        // check if the parsed command matches a stored command
        if (STR_CMP(cmd_name, cmd_entry->name)) {

            // number of arguments needed
            uint8_t n_args = (cmd_entry->n_var_args != nullptr) ? *(cmd_entry->n_var_args) + cmd_entry->n_args : cmd_entry->n_args;

            // check if the argument count matches
            if (arg_list.get_num_args() == n_args) {

                Command cmd;
                cmd.entry = cmd_entry;
                // cmd.name = cmd_entry->name;
                // cmd.callbacks = cmd_entry->callbacks;
                cmd.args.copy(&arg_list);

                // cmd_entry->args.copy(&arg_list);
                
                if (!cmd_entry->queue_cmd || (cmd_queue.empty() && TMCStepperControl.check_state(IDLE, HOME_FIRST))) {
                    run_cmd(cmd);
                } else if (!cmd_queue.full()) {
                    cmd_queue.push(cmd);
                    TMCMessageAgent.post_message(INFO, "Command queue: %i / %i", cmd_queue.size(), cmd_queue.max_size());
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

void _serial_command::run_cmd(char* cmd) {
    if (cmd == nullptr) return;
    parse(cmd);
}

void _serial_command::run_cmd(Command cmd) {
    // execute the callbacks and pass the arguments

    // while (cmd.callbacks != nullptr) {
    //     CommandCallback callback = *(cmd.callbacks);
    //     callback(cmd.name, &(cmd.args));
    //     cmd.callbacks++;
    // }

    for (uint8_t i = 0; i < cmd.entry->n_callbacks; i ++) {
        (*cmd.entry->callbacks[i])(cmd.entry->name, &cmd.args);
        cmd.args.reset();
    }
}