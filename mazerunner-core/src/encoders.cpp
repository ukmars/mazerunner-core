/******************************************************************************
 * Project: mazerunner-core                                                   * 
 * File:    encoders.cpp                                                      * 
 * File Created: Tuesday, 25th October 2022 10:58:35 am                       * 
 * Author: Peter Harrison                                                     * 
 * -----                                                                      * 
 * Last Modified: Tuesday, 1st November 2022 11:46:25 pm                      * 
 * -----                                                                      * 
 * Copyright 2022 - 2022 Peter Harrison, Micromouseonline                     * 
 * -----                                                                      * 
 * Licence:                                                                   * 
 *     Use of this source code is governed by an MIT-style                    * 
 *     license that can be found in the LICENSE file or at                    * 
 *     https://opensource.org/licenses/MIT.                                   * 
 ******************************************************************************/


#include "encoders.h"

// a bit of indirection for convenience because the encoder instance is
// unknown until the linker has done its thing
void callback_left(){
  encoders.left_input_change();
}

void callback_right(){
encoders.right_input_change();
}

