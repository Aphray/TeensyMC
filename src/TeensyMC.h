#pragma once

#include "stepper_control/stepper.h"
#include "communication/serial_command.h"
#include "communication/message_agent.h"
#include "stepper_control/stepper_control.h"
#include "event_handling/event_manager.h"


void TMC_Begin();

void TMC_Run();
