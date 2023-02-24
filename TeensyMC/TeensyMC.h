#pragma once

#include <Arduino.h>
#include "src/stepper_control/stepper.h"
#include "src/communication/serial_command.h"
#include "src/communication/message_agent.h"
#include "src/stepper_control/stepper_control.h"


void TMC_Begin();

void TMC_Run();