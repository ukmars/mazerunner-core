/******************************************************************************
 * Project: mazerunner-core                                                   * 
 * File:    adc_atmega328.cpp                                                 * 
 * File Created: Wednesday, 26th October 2022 12:11:36 am                     * 
 * Author: Peter Harrison                                                     * 
 * -----                                                                      * 
 * Last Modified: Saturday, 29th October 2022 3:10:13 pm                      * 
 * -----                                                                      * 
 * Copyright 2022 - 2022 Peter Harrison, Micromouseonline                     * 
 * -----                                                                      * 
 * Licence:                                                                   * 
 *     Use of this source code is governed by an MIT-style                    * 
 *     license that can be found in the LICENSE file or at                    * 
 *     https://opensource.org/licenses/MIT.                                   * 
 ******************************************************************************/




#ifdef ARDUINO_ARCH_MEGAAVR

#include "adc_atmega4809.h"

adc_atmega4809 adc; // TODO: this needs to get defined in a hardware file not here

ISR(ADC0_RESRDY_vect) {
  adc_isr(adc);
}

#endif
