#include <Arduino.h>
#include "stepper.h"
#include "accelerator.h"
#include "serial_comm.h"
#include "configuration.h"

Stepper* steppers[NUM_STEPPERS];

void setup() {

  Serial.begin(115200);
  Stepper::initialize();

  for (uint8_t n = 0; n < NUM_STEPPERS; n++) {
    Stepper* stepper = new Stepper(STEP_PINS[n][0], DIR_PINS[n], PROBE_PINS[n][0]);

    stepper->set_step_polarity(STEP_PINS[n][1]);
    stepper->set_probe_polarity(PROBE_PINS[n][1]);
    stepper->set_inverse_direction(INVERT_DIRECTION[n]);
    stepper->set_max_accel(MAX_ACCEL[n] / MM_PER_STEP[n]);
    stepper->set_speed_limits(SPEED_LIMITS[n][0] / MM_PER_STEP[n], SPEED_LIMITS[n][1] / MM_PER_STEP[n]);
    stepper->set_travel_limits(TRAVEL_LIMITS[n][0] / MM_PER_STEP[n], TRAVEL_LIMITS[n][1] / MM_PER_STEP[n]);

    steppers[n] = stepper;
  }
}


void loop() {
  SerialCommunication::read_serial();
  SerialCommunication::post_queued_messages();
  SerialCommunication::report_realtime_status();
}