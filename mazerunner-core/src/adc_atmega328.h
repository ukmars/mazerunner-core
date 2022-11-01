/******************************************************************************
 * Project: mazerunner-core                                                   *
 * File:    adc_atmega328.h                                                   *
 * File Created: Wednesday, 26th October 2022 10:51:51 pm                     *
 * Author: Peter Harrison                                                     *
 * -----                                                                      *
 * Last Modified: Tuesday, 1st November 2022 11:14:12 am                      *
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
#include <Arduino.h>
#include <wiring_private.h>

class adc_atmega328 : public IAnalogueConverter {

public:
  adc_atmega328() = default;

  void begin() override {
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
    // Change the clock prescaler from 128 to 32 for a 500kHz clock
    bitSet(ADCSRA, ADPS2);
    bitClear(ADCSRA, ADPS1);
    bitSet(ADCSRA, ADPS0);
    // Set the reference to AVcc and right adjust the result
    ADMUX = DEFAULT << 6;
  }
  void start_conversion_cycle() override {
    if (not m_configured) {
      return;
    }
    bitSet(ADCSRA, ADIE); // enable the ADC interrupt
    m_phase = 0;          // sync up the start of the sensor sequence
    start_conversion(15); // begin a dummy conversion to get things started
  }

  void end_conversion_cycle() override {
    bitClear(ADCSRA, ADIE); // disable the ADC interrupt
  }

  void start_conversion(uint8_t channel) override {

    // select the channel
    ADMUX = (ADMUX & 0xF0) | (channel & 0x0F);
    // start the conversion
    sbi(ADCSRA, ADSC);
  }

  int get_adc_result() override {
    return ADC;
  }
  // END OF HARDWARE DEPENDENCY
  //***************************************************************************//

  void emitter_on(uint8_t pin) override {
    if (pin == 255 || not m_emitters_enabled) {
      return;
    }
    digitalWriteFast(pin, 1);
  }

  void emitter_off(uint8_t pin) override {
    if (pin == 255) {
      return;
    }
    digitalWriteFast(pin, 0);
  }

private:
};

extern adc_atmega328 adc;
