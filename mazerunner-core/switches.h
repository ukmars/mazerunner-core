/******************************************************************************
 * Project: mazerunner-core                                                   *
 * File:    switches.h                                                        *
 * File Created: Wednesday, 26th October 2022 12:11:36 am                     *
 * Author: Peter Harrison                                                     *
 * -----                                                                      *
 * Last Modified: Tuesday, 1st November 2022 10:42:08 am                      *
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

#include "adc.h"
#include "config.h"
#include "digitalWriteFast.h"
#include <Arduino.h>
#include <util/atomic.h>
#include <wiring_private.h>

/***
 * The Switches class looks after the multifunction analogue input on UKMARSBOT.
 *
 * A single analogue channel lets you examine four dip switches and a pushbutton.
 *
 * The dip switches short out combinations of resistors in a potential divider chain
 * and thus cause a different voltage to be presented to the ADC input pin. The
 * maximum voltage is applied when all switches are open. That voltage will be
 * about 66% of the full range of the analogue channel with the resistors chosen.
 *
 * The top resistor in this chain has a pushbutton in parallel so that pressing
 * the button pulls the input all the way up to the positive supply giving a
 * full scale adc reading.
 *
 * There is no debounce on this circuit or in the software. None has yet proven
 * necessary. The simplest option would be to place a small capacitor between
 * the ADC input and ground.
 *
 * NOTE: The switches class relies upon the ADC being updated regularly in the
 *       systick event.
 */
class Switches {
public:
  explicit Switches(uint8_t channel) : m_channel(channel){};

  void update() {
    m_switches_adc = adc.get_dark(m_channel);
  }

  /**
   * The adc_thresholds may need adjusting for non-standard resistors.
   *
   * @brief  Convert the switch ADC reading into a switch reading.
   * @return integer in range 0..16 or -1 if there is an error
   */
  int read() {
    const int adc_thesholds[] = {660, 647, 630, 614, 590, 570, 545, 522, 461, 429, 385, 343, 271, 212, 128, 44, 0};
    update();

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

  // for testing
  int adc_reading() {
    update();
    return m_switches_adc;
  }

private:
  uint8_t m_channel = 255;
  int m_switches_adc = 0;
};

extern Switches switches;
#endif