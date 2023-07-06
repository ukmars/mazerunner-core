/******************************************************************************
 * Project: mazerunner-core                                                   *
 * File:    acd_null.cpp                                                      *
 * File Created: Monday, 31st October 2022 1:03:40 pm                         *
 * Author: Peter Harrison                                                     *
 * -----                                                                      *
 * Last Modified: Tuesday, 1st November 2022 11:29:52 am                      *
 * -----                                                                      *
 * Copyright 2022 - 2022 Peter Harrison, Micromouseonline                     *
 * -----                                                                      *
 * Licence:                                                                   *
 *     Use of this source code is governed by an MIT-style                    *
 *     license that can be found in the LICENSE file or at                    *
 *     https://opensource.org/licenses/MIT.                                   *
 ******************************************************************************/

#include "adc_null.h"
#include <Arduino.h>

#if defined(ARDUINO_ARCH_NRF52840)
#warning Needs a proper implementation of adc class
adc_null adc; // TODO: this needs to get defineed in a hardware file not here
#endif
// ISR(ADC_vect) {
//   adc_isr(adc);
// }
