/******************************************************************************
 * Project: mazerunner-core                                                   * 
 * File:    adc_atmega328.cpp                                                 * 
 * File Created: Wednesday, 26th October 2022 12:11:36 am                     * 
 * Author: Peter Harrison                                                     * 
 * -----                                                                      * 
 * Last Modified: Thursday, 27th October 2022 10:34:22 pm                     * 
 * -----                                                                      * 
 * Copyright 2022 - 2022 Peter Harrison, Micromouseonline                     * 
 * -----                                                                      * 
 * Licence:                                                                   * 
 *     Use of this source code is governed by an MIT-style                    * 
 *     license that can be found in the LICENSE file or at                    * 
 *     https://opensource.org/licenses/MIT.                                   * 
 ******************************************************************************/


#if defined(ARDUINO_ARCH_AVR)

#include "adc_atmega328.h"
adc_atmega328 adc; // TODO: this needs to get defineed in a hardware file not here

ISR(ADC_vect) {
  adc_isr(adc);
}
#endif
