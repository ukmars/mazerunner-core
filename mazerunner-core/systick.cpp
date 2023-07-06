/******************************************************************************
 * Project: mazerunner-core                                                   *
 * File:    systick.cpp                                                       *
 * File Created: Tuesday, 25th October 2022 10:59:56 am                       *
 * Author: Peter Harrison                                                     *
 * -----                                                                      *
 * Last Modified: Wednesday, 2nd November 2022 10:38:50 am                    *
 * -----                                                                      *
 * Copyright 2022 - 2022 Peter Harrison, Micromouseonline                     *
 * -----                                                                      *
 * Licence:                                                                   *
 *     Use of this source code is governed by an MIT-style                    *
 *     license that can be found in the LICENSE file or at                    *
 *     https://opensource.org/licenses/MIT.                                   *
 ******************************************************************************/

#include "systick.h"
#include "Arduino.h"

ISR(TIMER2_COMPA_vect, ISR_NOBLOCK) {
  systick.update();
}
