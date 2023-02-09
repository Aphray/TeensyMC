# TeensyMC

(NOTE: This project is still in an infant stage and may change frequently)

TeensyMC (Teensy Motion Control) is a standalone motion control software designed for the Teensy4.1 MCU. This software heavily relies on the FPU (floating point unit) on the Teensy4.1 for speed/acceleration calculations. As a result, performance may suffer on older versions of the Teensy (e.g., 3.X).

## Configuration

All configuration is done through "configuration.h"

## Communication

Communication between the host (PC) and the controller is done entirely through USB/Serial. 

### Commands 

Command strings sent from the host are parsed on the controller and execute the desired actions (move, home, probe, etc.). Each command is composed of 3 capitol letters, and some commands require arguments to be passed. Below is the format for every command:
```
CMD:ARG1,ARG2,...\n
```
Commands that need arguments must have the exact number of arguments, otherwise an error is printed back to the host. Below is a list of the currently supported commands with their required arguments:
```
MVE:POS_0,POS_1,...,POS_N,SPEED,ACCEL     (Linear Move; POS_0 -> POS_N are the target positions of axes 0 - N)
HME:AX                                    (Home; AX=axis number to home)
PRB:AX                                    (Probe)
ZRO:AX                                    (Zero Axis)
STP                                       (Controlled stop; no arguments)
HLT                                       (Halt/Emergency Stop; no arguments)
CLF                                       (Clear Fault; no arguments)
RST                                       (SW Reset MCU)
```
For the move (MVE) command, the position (POS_X) arguments can be defined in relative (R) or absolute (A) coordinates. For example, the following command will move to axis 0 to relative position 10, and axis 1 to absolute position 20 (simultaneously):
```
MVE:R10,A20,10,200
```
Additionally, an asterisk character (*) can be used to "skip" any of the position arguments. For example, to move just the first axis of a 3-axis system, the following command can sent:
```
MVE:R20,*,*,10,200
```
Which is equivalent to:
```
MVE:R20,R0,R0,10,200
```

### Status Reporting

Periodically, the controlled will print status messages to the host. The status messages have the following form:
```
[STATUS] (ACTIVE) AX0:9.595313,2.500000 AX1:9.189063,5.000000 AX2:2.756250,1.500000
```
[STATUS] defines the message level. Other levels are used for different types of messages (e.g., errors, information). The string in the parenthesis (ACTIVE) provides the current motion state. Followed by a section for each axis:
```
AX__:POSITION,SPEED
```

### Other Messages

Other message levels are used to communicate information to the host:
```
INFO      (General information)
DEBUG     (Debug information)
ERROR     (Non-critical errors)
WARNING   (Warnings)
CRITICAL  (Critical errors)
```
An example of an error message for incorrect number of arguments on a move (MVE) command:
```
>MVE:R10
[ERROR] Not enough args
```
And an example of a warning message for travel out of bounds/limits:
```
>MVE:R-20,*,*,10,200    (axis 0 lower bound set to 0; attempting to move out of bounds...)
[WARNING] Target out of bounds on axis 0, limiting travel within bounds
```
## Behind The Scenes

### Coordination

The Bresenham Line Algorithm is used to coordinate simultaneous movement among multiple axis. The algorithm used here was adopted from the [TeensyStep](https://github.com/luni64/TeensyStep) library.

### Acceleration

This software supports both linear and S-curve acceleration (by commenting/uncommenting a define flag in the configuration file). Linear acceleration uses simple linear interpolation between the start and end speeds. 

The S-curve acceleration implementation is based upon [this blog post](https://fightpc.blogspot.com/2018/04/how-to-get-sinusoidal-s-curve-for.html). The file "s_curve_speed_map.h" contains a lookup table for the sinusoidal v(t) profile. During the accel/decel phases, the speed is interpolated from the lookup table (which goes from 0 to 2*pi) and mapped to the stepper speed.

