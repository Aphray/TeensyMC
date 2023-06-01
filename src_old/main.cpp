#include <Arduino.h>
#include "TeensyMC.h"

#define X_STEP_PIN      10
#define X_DIR_PIN       11
#define X_EN_PIN        3
#define X_DIAG_PIN      24

#define X_SCREW_LEAD    2.0f

#define X_MICROSTEPS    16U
#define X_MM_PER_STEP   (X_SCREW_LEAD / (200UL * X_MICROSTEPS))

#define X_MIN_TRAVEL    -200.0f
#define X_MAX_TRAVEL    200.0f

#define X_MAX_ACCEL     1000.0f
#define X_MIN_SPEED     0.1f
#define X_MAX_SPEED     20.0f

#define X_INVERT_DIR    false
#define X_INVERT_HOME   false

#define X_DRIVER_RS             0.11f
#define X_DRIVER_ADDR           0b00
#define X_DRIVER_RMS_CURRENT    700 // milliamps

#define X_DRIVER_SGVAL  50
#define X_DRIVER_SG_SKIP

Stepper xstepper(X_DIR_PIN, X_STEP_PIN, X_EN_PIN);
Stepper ystepper(X_STEP_PIN, X_DIR_PIN, X_EN_PIN);
// Stepper zstepper(Z_STEP_PIN, Z_DIR_PIN);

void setup() {

  TMC_Begin();

  xstepper.begin();
  xstepper.set_units_per_step(X_MM_PER_STEP);
  xstepper.set_min_max_travel(X_MIN_TRAVEL, X_MAX_TRAVEL);
  xstepper.set_max_accel(X_MAX_ACCEL);
  xstepper.set_speed_limits(X_MIN_SPEED, X_MAX_SPEED);
  xstepper.invert_dir_polarity(X_INVERT_DIR);
  xstepper.invert_home_dir(X_INVERT_HOME);
  TMCStepperControl.add_stepper(xstepper);

  ystepper.begin();
  TMCStepperControl.add_stepper(ystepper);
}

void loop() {
  TMC_Run();

  // stallguard_debug();
  // Serial.println(TMCStepperControl.steppers_accelerating());
}