#include "TeensyMC.h"

void foo(char* cmd, ArgList* args) {
  Serial.println(cmd);
}

CommandCallback foo_cb = &foo;


void setup() {

  SerialCommand.add_command("FOO", 1, foo_cb);  // creates the command
  SerialCommand.add_command("BAR", 2, foo_cb);  // command already exists, this does nothing
}


void loop() {
  // do all the serial communication stuff
  run_TMC();  

  // do other __non-blocking__ stuff here...
}