/******************************************************************************
 * Project: mazerunner-core                                                   * 
 * File:    systick.cpp                                                       * 
 * File Created: Tuesday, 25th October 2022 10:59:56 am                       * 
 * Author: Peter Harrison                                                     * 
 * -----                                                                      * 
 * Last Modified: Thursday, 27th October 2022 10:35:16 pm                     * 
 * -----                                                                      * 
 * Copyright 2022 - 2022 Peter Harrison, Micromouseonline                     * 
 * -----                                                                      * 
 * Licence:                                                                   * 
 *     Use of this source code is governed by an MIT-style                    * 
 *     license that can be found in the LICENSE file or at                    * 
 *     https://opensource.org/licenses/MIT.                                   * 
 ******************************************************************************/


#include "systick.h"

#if defined(ARDUINO_ARCH_AVR)
ISR(TIMER2_COMPA_vect, ISR_NOBLOCK) {
  systick.update();
}
#endif
