/******************************************************************************
 * Project: mazerunner-core                                                   * 
 * File:    encoders.cpp                                                      * 
 * File Created: Tuesday, 25th October 2022 10:58:35 am                       * 
 * Author: Peter Harrison                                                     * 
 * -----                                                                      * 
 * Last Modified: Sunday, 30th October 2022 12:12:45 am                       * 
 * -----                                                                      * 
 * Copyright 2022 - 2022 Peter Harrison, Micromouseonline                     * 
 * -----                                                                      * 
 * Licence:                                                                   * 
 *     Use of this source code is governed by an MIT-style                    * 
 *     license that can be found in the LICENSE file or at                    * 
 *     https://opensource.org/licenses/MIT.                                   * 
 ******************************************************************************/


#include "encoders.h"
/**
 * Measurements indicate that even at 1500mm/s the total load due to
 * the encoder interrupts is less than 3% of the available bandwidth.
 */

// ISR will respond to the XOR-ed pulse train from the encoder
// runs in constant time of around 3us per interrupt.
// would be faster with direct port access

// a bit of indirection for convenience
void callback_left(){
  encoders.left_input_change();
}

void callback_right(){
encoders.right_input_change();
}

