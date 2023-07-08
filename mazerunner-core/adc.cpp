/******************************************************************************
 * Project: mazerunner-core                                                   *
 * File:    adc.cpp                                                           *
 * File Created: Thursday, 27th October 2022 12:12:56 pm                      *
 * Author: Peter Harrison                                                     *
 * -----                                                                      *
 * Last Modified: Monday, 31st October 2022 12:51:02 am                       *
 * -----                                                                      *
 * Copyright 2022 - 2022 Peter Harrison, Micromouseonline                     *
 * -----                                                                      *
 * Licence:                                                                   *
 *     Use of this source code is governed by an MIT-style                    *
 *     license that can be found in the LICENSE file or at                    *
 *     https://opensource.org/licenses/MIT.                                   *
 ******************************************************************************/
#include "adc.h"

// TODO: should these few lines go into some sort of global variables file?
// along with the maze, encoders and so on?
AnalogueConverter adc;

ISR(ADC_vect) {
  adc.isr();
}
