#include <queue>
#include <string>
#include "stepper.h"
#include "serial_comm.h"
#include "accelerator.h"
#include "configuration.h"

#define STR_CMP(a, b) strcmp(a, b) == 0

#define ARG_BUF_SIZE 12
#define MAX_ARGS (NUM_STEPPERS + 2)

#define CMD_DELIM " "
#define ARG_DELIM ','
#define SKIP_CHAR '*'

namespace SerialCommunication {

    std::queue<Message> message_queue;

    // try to convert argument cstring to int, returns true on success
    bool argtoi(const char* str, int* result) {
        for (uint8_t n = 0; n < strlen(str); n++) {
            char c = str[n];
            switch(c) {
                case '-':
                    if (n > 0) return false;
                    break;
                default:
                    if (!isdigit(c)) return false;
                    break;
            }
        }

        *result = atoi(str);
        return true;
    }

    // try to convert argument cstring to float, return true on success
    bool argtof(const char* str, float* result) {
        bool decimal = false;
        
        for (uint8_t n = 0; n < strlen(str); n++) {
            char c = str[n];
            switch(c) {
                case '-':
                    if (n > 0) return false;
                    break;
                case '.':
                    if (decimal) return false;
                    decimal = true;
                    break;
                default:
                    if (!isdigit(c)) return false;
                    break;
            }
        }

        *result = strtof(str, nullptr);
        return true;
    }

    // execute the parsed command with the arguments
    void run_cmd(Command command, char (*arg_list)[ARG_BUF_SIZE]) {
        #define CHECK_ACTIVE if (Stepper::state == ACTIVE || Stepper::state == HOMING || Stepper::state == PROBING) { post_message(ERROR, "Steppers active, cannot run command"); return; }
        #define CHECK_HOMED if (Stepper::state == NEEDS_HOMING) { post_message(ERROR, "Home the steppers first"); return; }
        #define ARG_ERROR post_message(ERROR, "Argument error")

        switch (command) {
            case START_MOVE:
            CHECK_ACTIVE;
            CHECK_HOMED;
            {
                char* speed_c = arg_list[NUM_STEPPERS];
                char* accel_c = arg_list[NUM_STEPPERS + 1];

                for (uint8_t n = 0; n < NUM_STEPPERS; n++) {
                    char* pos_c = arg_list[n];
                    float pos;

                    switch (*pos_c) {
                        case 'A':
                            if (!argtof(pos_c + 1, &pos)) {
                                ARG_ERROR;
                                return;
                            }
                            Stepper::steppers[n]->set_target_abs(pos / MM_PER_STEP[n]);
                            break;
                        case 'R':
                            if (!argtof(pos_c + 1, &pos)) {
                                ARG_ERROR;
                                return;
                            }
                            Stepper::steppers[n]->set_target_rel(pos / MM_PER_STEP[n]);
                            break;
                        case '*':
                            Stepper::steppers[n]->set_target_rel(0);
                            break;
                        default:
                            ARG_ERROR;
                            return;
                    }
                }

                float speed;
                float accel;

                if (*accel_c == SKIP_CHAR) {
                    accel = DEFAULT_ACCEL / MM_PER_STEP[Stepper::master->axis];
                } else if (argtof(accel_c, &accel)) {
                    accel /= MM_PER_STEP[Stepper::master->axis];
                } else {
                    ARG_ERROR;
                    return;
                }

                if (!argtof(speed_c, &speed)) {
                    ARG_ERROR;
                    return;
                }

                speed /= MM_PER_STEP[Stepper::master->axis];
                Stepper::start_move(speed, accel);

                break;
            }
            
            case SET_ZERO:
            case START_HOME:
            case START_PROBE:
            CHECK_ACTIVE;
            {
                char* ax_c = arg_list[0];

                int ax;
                if (!argtoi(ax_c, &ax)) return;
                if (ax >= NUM_STEPPERS || ax < 0) return;

                if (command == SET_ZERO) {
                    CHECK_HOMED
                    Stepper::steppers[ax]->zero();
                    
                } else if (command == START_HOME) {
                    Stepper::steppers[ax]->home();

                } else if (command == START_PROBE) {
                    CHECK_HOMED
                    Stepper::steppers[ax]->probe();
                }
                
                break;
            }
            case RESET:
                SCB_AIRCR = 0x05FA0004;
                break;

            default:
                break;
        }
    }

    void parse_serial(char* str) {

        char* cmd = strtok(str, CMD_DELIM);
        char* args = strtok(NULL, CMD_DELIM);
        char arg_list[MAX_ARGS][ARG_BUF_SIZE];        

        int8_t num_args = 0;
        Command command = UNKNOWN;

        #define IS_CMD(s, c, n) if (STR_CMP(cmd, s)) { command = c; num_args = n; }

        IS_CMD("MVE", START_MOVE, NUM_STEPPERS + 2)
        else IS_CMD("HME", START_HOME, 1)
        else IS_CMD("PRB", START_PROBE, 1)
        else IS_CMD("CLF", CLEAR_FAULT, 0)
        else IS_CMD("ZRO", SET_ZERO, 1)
        else IS_CMD("STP", STOP, 0)
        else IS_CMD("HLT", HALT, 0)
        else IS_CMD("RST", RESET, 0)
        else {
            post_message(ERROR, "Unknown command: <%s>", cmd);
            return;
        }

        uint8_t i = 0;
        char* tok = strtok(args, ",");

        while (tok != NULL) {
            num_args--;

            if (num_args < 0 || i == MAX_ARGS) {
                post_message(ERROR, "Too many args!");
                return;
            }

            strcpy(arg_list[i++], tok);
            tok = strtok(NULL, ",");
        }

        if (num_args != 0) {
            post_message(ERROR, "Not enough args!");
            return;
        }

        run_cmd(command, arg_list);
    }



    void echo(const char* message) {
        char buffer[strlen(message) + 10];
        sprintf(buffer, "echo: <%s>", message);
        post_message(DEBUG, buffer);
    }

    void read_serial() {
        static uint8_t idx = 0;
        static char rx_buffer[256];
        static uint8_t blocks = 1;

        while (Serial.available() > 0) {
            char c = Serial.read();

            switch (c) {
                case '\r':
                    break;

                case '\n':
                    rx_buffer[idx] = 0;

                    if (rx_buffer[0] == 0) continue;

                    #ifdef ECHO_CMDS
                    echo(rx_buffer);
                    #endif

                    parse_serial(rx_buffer);
                    rx_buffer[0] = 0;
                    idx = 0;
                    blocks = 1;

                    break;

                case ' ':
                    blocks++;
                default:
                    rx_buffer[idx++] = c;
            }
        }
    }

    void post_queued_messages() {
        while (message_queue.size() > 0) {
            Message message = message_queue.front();
            message.post();
            message_queue.pop();
        }
    }


    void _report_realtime_status(bool queue) {
        static const char* const STEPPER_STATE_STRINGS[] = {STEPPER_STATES(MAKE_STRINGS)};

        char status_message[256];
        float speed = Accelerator::current_speed;

        sprintf(status_message, "(%s)", STEPPER_STATE_STRINGS[Stepper::state]);
        for (uint8_t n = 0; n < Stepper::stepper_count; n++) {
            sprintf(status_message + strlen(status_message), " AX%i:<%f,%f>", n, Stepper::steppers[n]->current_position * MM_PER_STEP[n], Stepper::steppers[n]->map_speed(speed) * MM_PER_STEP[n]);
        }

        if (!queue) post_message(STATUS, status_message);
        else queue_message(STATUS, status_message);
    }

    void report_realtime_status() {
        static uint32_t last_report = 0;

        uint32_t period = ( Stepper::state == ACTIVE     
                        ||  Stepper::state == PROBING    
                        ||  Stepper::state == HOMING) ? REPORT_MILLIS_ACTIVE : REPORT_MILLIS_IDLE;
        
        if (millis() - last_report >= period) {
            _report_realtime_status(false);
            last_report = millis();
        }
    }

    void queue_realtime_status() {
        _report_realtime_status(true);
    }
}