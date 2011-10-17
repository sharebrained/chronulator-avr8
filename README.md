chronulator-avr8
================

by ShareBrained Technology

The latest version of this repository can be found at:
<https://github.com/sharebrained/chronulator-avr8>

Description
===========

An analog panel meter clock implementation based on the
Atmel AVR8 microcontroller.

Time is displayed via two analog meters, one for hours
and one for minutes. The microcontroller keeps time via
a 32.768kHz crystal, and controls the meter needles
using pulse-width modulation of current sources.

The current sources are adjustable by variable
resistors, and the time is set by momentary buttons.

A simple set of commands are supported via the AVR8
serial port. Meter needles can be controlled directly
via commands, overriding the default time-based
control. The brightness of two LEDs can be controlled
by other commands.

Documentation
=============

See http://www.sharebrained.com/chronulator/ for
hardware documentation.

Requirements
============

Software
--------

* Arduino IDE (to program the Chronulator firmware)
* avrdude (to program the AVR8 bootloader)
* avr-gcc (to compile and link the bootloader)
* EAGLE 5.x (to edit the schematic or circuit board)

Hardware
--------

* A Chronulator or Chronulator-compatible device.
* A serial adaptor and/or an Atmel ISP programmer.

Usage
=====

Before compiling and uploading this code in the Arduino
IDE, please select "LilyPad Arduino w/ ATmega168" under
the "Tools->Board" menu.

Contact
=======

ShareBrained Technology, Inc.
<http://www.sharebrained.com/>
