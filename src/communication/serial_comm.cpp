
#include "serial_comm.h"
#include "../config.h"
#include "../utility/fixed_queue.h"
#include "../stepper_control/stepper_control.h"

#define STR_CMP(a, b) (strcmp(a, b) == 0)

using namespace TeensyMC;
using namespace SerialComm;
using namespace StepperControl;


Stream* serial = &SERIAL_STREAM;

uint8_t num_commands = 0;

CommandEntry cmd_registry[MAX_USER_COMMANDS];

FixedQueue<Command, CMD_QUEUE_SIZE> cmd_queue;
FixedQueue<Message, MSG_QUEUE_SIZE> message_queue;


// parse the recieved data and execute any attached commands
void parse(char* data) {
    post_message(DEBUG, "Echo: <%s>", data);

    // get the command and make the arguments list
    char* cmd_name = strtok(data, CMD_DELIMITER);
    ArgList arg_list(strtok(NULL, CMD_DELIMITER));

    for (uint8_t n = 0; n < num_commands; n++) {
        CommandEntry* cmd_entry = &(cmd_registry[n]);

        if (STR_CMP(cmd_name, cmd_entry->name)) {

            // number of arguments needed
            uint8_t n_args = (cmd_entry->n_var_args != nullptr) ? *(cmd_entry->n_var_args) + cmd_entry->n_args : cmd_entry->n_args;

            // check if the argument count matches
            if (arg_list.get_num_args() == n_args) {

                Command cmd;
                cmd.entry = cmd_entry;
                cmd.args.copy(&arg_list);
                
                if (!cmd_entry->queue_cmd) {
                    cmd.run();
                } else if (!cmd_queue.full()) {
                    cmd_queue.push(cmd);
                    post_message(INFO, "Command queue: %i / %i", cmd_queue.size(), cmd_queue.max_size());
                } else {
                    post_message(ERROR, "Cannot queue cmd <%s>; command queue is full", data);
                }
                
            } else {
                // post error about argument mismatch
                post_message(ERROR, "Command <%s> requires (exactly) %i arg%s, but given %i", 
                        cmd_name, n_args, ((n_args > 1) ? "s" : ""), arg_list.get_num_args());
            }
            return;
        }
    }

    // command wasn't found...
    post_message(ERROR, "Command <%s> unrecognized", cmd_name);
}


void Message::print() {
    serial->println(buffer);
}

void Message::queue() {
    message_queue.push(*this);
}

// runs the command with the arguments
void Command::run() {
    for (uint8_t i = 0; i < entry->n_callbacks; i ++) {
        (*entry->callbacks[i])(entry->name, &args);
        args.reset();
    }
}

void SerialComm::internal::post_queued_messages() {
    while (message_queue.size()) {
        Message message = message_queue.pop();
        message.print();
    }
}

void SerialComm::internal::poll_serial() {
    static char rx_buffer[RX_BUFFER_SIZE];
    static char* rx_ptr = rx_buffer;

    while (serial->available() > 0) {
        char c = serial->read();
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

void SerialComm::internal::process_command_queue() {
    if (!StepperControl::check_state(IDLE, HOME_FIRST) || cmd_queue.empty()) return;

    Command cmd = cmd_queue.pop();
    cmd.run();
    post_message(INFO, "Command queue: %i / %i", cmd_queue.size(), cmd_queue.max_size());
}

void SerialComm::register_command(char* cmd_name, uint8_t args) {
    register_command(cmd_name, args, true, nullptr);
}
void SerialComm::register_command(char* cmd_name, uint8_t args, bool queue) {
    register_command(cmd_name, args, queue, nullptr);
}

void SerialComm::register_command(char* cmd_name, uint8_t args, uint8_t* variable_args) {
    register_command(cmd_name, args, true, variable_args);
}

void SerialComm::register_command(char* cmd_name, uint8_t args, bool queue, uint8_t* variable_args) {

    // checks
    if (strlen(cmd_name) > CMD_CHAR_MAX) return;
    if (num_commands == MAX_USER_COMMANDS) return;

    // check to see if the command exists
    for (uint8_t n = 0; n < num_commands; n ++) {

        if (STR_CMP(cmd_name, cmd_registry[n].name)) { 
            // if command exists, report an error and return
            post_message(ERROR, "Command <%s> already registered", cmd_name);
            return; 
        }
    }

    // register the command when it doesn't exist in the registry
    CommandEntry* reg_cmd = &cmd_registry[num_commands++];

    // assign the command info
    strcpy(reg_cmd->name, cmd_name);
    reg_cmd->queue_cmd = queue;
    reg_cmd->n_args = args;
    reg_cmd->n_var_args = variable_args;
}

void SerialComm::add_callback(char* cmd_name, CommandCallback callback) {
    CommandEntry* cmd_entry = cmd_registry;

    while (cmd_entry != nullptr) {

        if (STR_CMP(cmd_name, cmd_entry->name)) {
            if (cmd_entry->n_callbacks == MAX_USER_CALLBACKS) { 
                post_message(ERROR, "Command <%s> reached user callback limit (%i)", cmd_name, MAX_USER_CALLBACKS);
                return; 
            }

            // add the new callback to the callbacks list
            cmd_entry->callbacks[cmd_entry->n_callbacks++] = callback;
            return;
        }

        cmd_entry++;
    }

    // command wasn't found...
    post_message(ERROR, "Command <%s> unrecognized; register command before adding callbacks", cmd_name);
}

void SerialComm::clear_command_queue() {
    cmd_queue.clear();
    queue_message(INFO, "Command queue: %i / %i", cmd_queue.size(), cmd_queue.max_size());
}