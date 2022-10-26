/******************************************************************************
 * Project: mazerunner-core                                                   * 
 * File:    switches.h                                                        * 
 * File Created: Wednesday, 26th October 2022 12:11:36 am                     * 
 * Author: Peter Harrison                                                     * 
 * -----                                                                      * 
 * Last Modified: Wednesday, 26th October 2022 11:52:31 pm                    * 
 * -----                                                                      * 
 * Copyright 2022 - 2022 Peter Harrison, Micromouseonline                     * 
 * -----                                                                      * 
 * Licence:                                                                   * 
 *     Use of this source code is governed by an MIT-style                    * 
 *     license that can be found in the LICENSE file or at                    * 
 *     https://opensource.org/licenses/MIT.                                   * 
 ******************************************************************************/

#ifndef SWITCHES_H
#define SWITCHES_H

#include "../config.h"
#include "adc.h"
#include "digitalWriteFast.h"
#include <Arduino.h>
#include <util/atomic.h>
#include <wiring_private.h>

class Switches {
public:
  explicit Switches(uint8_t channel) : m_channel(channel){};

  void update() {
    m_switches_adc = adc[m_channel];
  }

  /**
   * The adc_thresholds may need adjusting for non-standard resistors.
   *
   * @brief  Convert the switch ADC reading into a switch reading.
   * @return integer in range 0..16 or -1 if there is an error
   */
  int read() {
    const int adc_thesholds[] = {660, 647, 630, 614, 590, 570, 545, 522, 461, 429, 385, 343, 271, 212, 128, 44, 0};

    if (m_switches_adc > 800) {
      return 16;
    }
    for (int i = 0; i < 16; i++) {
      if (m_switches_adc > (adc_thesholds[i] + adc_thesholds[i + 1]) / 2) {
        return i;
      }
    }
    return -1;
  }

  inline bool button_pressed() {
    return read() == 16;
  }

  void wait_for_button_press() {
    while (not(button_pressed())) {
      delay(10);
    };
  }

  void wait_for_button_release() {
    while (button_pressed()) {
      delay(10);
    };
  }

  void wait_for_button_click() {
    wait_for_button_press();
    wait_for_button_release();
    delay(250);
  }

private:
  uint8_t m_channel = 255;
  int m_switches_adc;
};

extern Switches switches;
#endif