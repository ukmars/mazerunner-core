/******************************************************************************
 * Project: mazerunner-core                                                   *
 * File:    atomic.h                                                          *
 * File Created: Thursday, 27th October 2022 11:00:18 pm                      *
 * Author: Peter Harrison                                                     *
 * -----                                                                      *
 * Last Modified: Thursday, 27th October 2022 11:04:40 pm                     *
 * -----                                                                      *
 * Copyright 2022 - 2022 Peter Harrison, Micromouseonline                     *
 * -----                                                                      *
 * Licence:                                                                   *
 *     Use of this source code is governed by an MIT-style                    *
 *     license that can be found in the LICENSE file or at                    *
 *     https://opensource.org/licenses/MIT.                                   *
 ******************************************************************************/

#if defined(ARDUINO_ARCH_MEGAAVR) || defined(ARDUINO_ARCH_AVR)
#include <util/atomic.h>
#endif
#ifdef ARDUINO_ARCH_NRF52840
#define ATOMIC_BLOCK(ATOMIC_RESTORESTATE)
#endif
