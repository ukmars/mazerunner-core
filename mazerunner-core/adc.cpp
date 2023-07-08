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

AnalogueConverter adc;

ISR(ADC_vect) {
  adc.isr();
}
