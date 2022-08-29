/*
 * File: settings.cpp
 * Project: mazerunner
 *
 *  File Created: Tuesday, 2nd March 2021 2:41:08 pm
 *  MIT License
 *
 *  Copyright (c) 2020-2021 Rob Probin & Peter Harrison
 *  Copyright (c) 2019-2021 UK Micromouse and Robotics Society
 *
 *  Permission is hereby granted, free of charge, to any person obtaining a copy
 *  of this software and associated documentation files (the "Software"), to deal
 *  in the Software without restriction, including without limitation the rights
 *  to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 *  copies of the Software, and to permit persons to whom the Software is
 *  furnished to do so, subject to the following conditions:
 *
 *  The above copyright notice and this permission notice shall be included in all
 *  copies or substantial portions of the Software.
 *
 *  THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 *  IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 *  FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 *  AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 *  LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 *  OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 *  SOFTWARE.
 */

#include "settings.h"
#include "EEPROM.h"
#include <Arduino.h>

Settings settings;
/***
 * Now store a copy of all the default values in FLASH in case we need them.
 * These defaults are the values as given in the SETTINGS_PARAMETERS list in the
 * header file. They cannot be changed at run time
 */
const Settings defaults PROGMEM = {
    .fwdKP = FWD_KP,
    .fwdKD = FWD_KD,
    .rotKP = ROT_KP,
    .rotKD = ROT_KD,
    .steering_KP = STEERING_KP,
    .steering_KD = STEERING_KD,
    .mouseRadius = MOUSE_RADIUS,
    .left_calibration = LEFT_CALIBRATION,
    .front_calibration = FRONT_CALIBRATION,
    .right_calibration = RIGHT_CALIBRATION,
    .left_adjust = LEFT_SCALE,
    .front_adjust = FRONT_SCALE,
    .right_adjust = RIGHT_SCALE,
    .left_threshold = LEFT_THRESHOLD,
    .front_threshold = FRONT_THRESHOLD,
    .right_threshold = RIGHT_THRESHOLD,
    .left_nominal = LEFT_NOMINAL,
    .front_nominal = FRONT_NOMINAL,
    .right_nominal = RIGHT_NOMINAL,
};

/***
 * A simple, dumb binary copy is a safe way to restore values from flash
 * because the structures are guaranteed to be identical
 */
int restore_default_settings() {
  memcpy_P(&settings, &defaults, sizeof(defaults));
  return 0;
}
