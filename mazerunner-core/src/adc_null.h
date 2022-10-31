/******************************************************************************
 * Project: mazerunner-core                                                   *
 * File:    adc_null.h                                                        *
 * File Created: Monday, 31st October 2022 1:03:23 pm                         *
 * Author: Peter Harrison                                                     *
 * -----                                                                      *
 * Last Modified: Monday, 31st October 2022 4:10:26 pm                        *
 * -----                                                                      *
 * Copyright 2022 - 2022 Peter Harrison, Micromouseonline                     *
 * -----                                                                      *
 * Licence:                                                                   *
 *     Use of this source code is governed by an MIT-style                    *
 *     license that can be found in the LICENSE file or at                    *
 *     https://opensource.org/licenses/MIT.                                   *
 ******************************************************************************/
#pragma once

#include "adc.h"
#include "digitalWriteFast.h"

/***
 * This is a null, or empty ADC device as a placeholder and to show what needs
 * to be implemented in an actual harware adc driver.
 *
 */
class adc_null : public IAnalogueConverter {

public:
  adc_null() = default;

  void begin() {
    disable_emitters();
    for (int i = 0; i < MAX_GROUPS; i++) {
      if (m_emitter_pin[i] < 255) {
        pinMode(m_emitter_pin[i], OUTPUT);
      }
    }
    converter_init();
    m_configured = true;
  }

  //***************************************************************************//
  // START OF HARDWARE DEPENDENCY

  /**
   *  The default for the Arduino is to give a slow ADC clock for maximum
   *  SNR in the results. Conversions then take more
   *  than 100us to complete. In this application, we want to be able to
   *  perform about 16 conversions in around 500us. To do that the prescaler
   *  is reduced to a value of 32. This gives an ADC clock speed of
   *  500kHz and a single conversion in around 26us. SNR is still pretty good
   *  at these speeds:
   *  http://www.openmusiclabs.com/learning/digital/atmega-m_adc_reading/
   *
   */
  void converter_init() {
  }

  void start_conversion_cycle() {
    if (not m_configured) {
      return;
    }
    // enable the ADC interrupt
    m_phase = 0; // sync up the start of the sensor sequence
    // start_conversion(15); // begin a dummy conversion to get things started
  }

  void end_conversion_cycle() {
    // disable the ADC interrupt
  }

  void start_conversion(uint8_t channel) {

    // select the channel
    // start the conversion
  }
  int get_adc_result() {
    return 0;
  }
  // END OF HARDWARE DEPENDENCY
  //***************************************************************************//

  void emitter_on(uint8_t pin) {
    if (pin == 255 || not m_emitters_enabled) {
      return;
    }
    digitalWriteFast(pin, 1);
  }

  void emitter_off(uint8_t pin) {
    if (pin == 255) {
      return;
    }
    digitalWriteFast(pin, 0);
  }

private:
};

extern adc_null adc;