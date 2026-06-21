/************************************************************************
  main.h

  Main functions
    RP2040 - USB to quadrature mouse converter
    Copyright (C) 2023 Darren Jones
    Copyright (C) 2017-2020 Simon Inns

  This file is part of RP2040 Mouse based on the original SmallyMouse from Simon Inns.

    RP2040 Mouse is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with this program.  If not, see <http://www.gnu.org/licenses/>.

  Email: nz.darren.jones@gmail.com

************************************************************************/

#ifndef _MAIN_H_
#define _MAIN_H_

// Hardware map

// Sam output pins
// the mouse pins are the bottom four GPIOs
#define SamMouseBit0_PIN 5
#define SamMouseBit1_PIN 6 
#define SamMouseBit2_PIN 7
#define SamMouseBit3_PIN 8
// MouseBit4 isn't actually used for the sam mouse, but if we want to use it
// later for joypad mapping to ctrl+up+down+left+right we'll need it
#define SamMouseBit4_PIN 9
#define SamMousePinsMask ((1<<SamMouseBit0_PIN)|(1<<SamMouseBit1_PIN)|(1<<SamMouseBit2_PIN)|(1<<SamMouseBit3_PIN)|(1<<SamMouseBit4_PIN))
// joystick pins
/*#define JoystickUp_PIN 25
#define JoystickDown_PIN 26
#define JoystickLeft_PIN 27
#define JoystickRight_PIN 28
#define JoystickFire_PIN 29
*/
// these for the final board
#define JoystickFire_PIN 25
#define JoystickUp_PIN 26
#define JoystickDown_PIN 27
#define JoystickLeft_PIN 28
#define JoystickRight_PIN 29



#define JoystickPinsMask ((1<<JoystickUp_PIN)|(1<<JoystickDown_PIN)|(1<<JoystickLeft_PIN)|(1<<JoystickRight_PIN)|(1<<JoystickFire_PIN))

// Function prototypes
void initialiseHardware(void);
void initialiseTimers(void);

void processMouseMovement(int8_t movementUnits, uint8_t axis);

#endif