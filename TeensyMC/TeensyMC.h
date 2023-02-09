#pragma once

#define MAX_STEPPERS 6

#ifndef REPORT_MILLIS_IDLE
    #define REPORT_MILLIS_IDLE 500
#endif

#ifndef REPORT_MILLIS_ACTIVE
    #define REPORT_MILLIS_ACTIVE 50
#endif

// #include "src/serial_comm.h"
// #include "src/stepper.h"
#include "src/stepper_control.h"
// #include "src/accelerator/accelerator.h"