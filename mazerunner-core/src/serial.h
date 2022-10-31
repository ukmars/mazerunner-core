/******************************************************************************
 * Project: mazerunner-core                                                   *
 * File:    serial.h                                                          *
 * File Created: Monday, 31st October 2022 12:42:31 pm                        *
 * Author: Peter Harrison                                                     *
 * -----                                                                      *
 * Last Modified: Monday, 31st October 2022 4:17:25 pm                        *
 * -----                                                                      *
 * Copyright 2022 - 2022 Peter Harrison, Micromouseonline                     *
 * -----                                                                      *
 * Licence:                                                                   *
 *     Use of this source code is governed by an MIT-style                    *
 *     license that can be found in the LICENSE file or at                    *
 *     https://opensource.org/licenses/MIT.                                   *
 ******************************************************************************/

#pragma once

#include "serial_null.h"
#include <Arduino.h>

extern HardwareSerial &console;
extern Stream &debug;

extern void redirectPrintf();
