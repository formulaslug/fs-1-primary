// Copyright (c) 2018 Formula Slug. All Rights Reserved.

# Primary Controls

This repository contains the software for our primary controller of the Formula Slug 2017 racecar, FS-1.

The style guide repository at https://github.com/wpilibsuite/styleguide contains our style guide for C and C++ code and formatting scripts.

## TODO
* Finish testing UART RX on interface UARD3
* Encapsulate all chibios subsystem code within a class subsystem and
  test
* Write serial print function, figure out how to set stdout to route
  to route to serial interface
* Implement mechanism for simulating state machine events when building
  in test mode
* Implement mechanism for simulating HAL inputs with ChibiOS simulator
  or another mechanism
