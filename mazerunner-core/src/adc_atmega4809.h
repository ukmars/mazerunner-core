/******************************************************************************
 * Project: mazerunner-core                                                   *
 * File:    adc_atmega328.h                                                   *
 * File Created: Wednesday, 26th October 2022 10:51:51 pm                     *
 * Author: Peter Harrison                                                     *
 * -----                                                                      *
 * Last Modified: Friday, 28th October 2022 4:17:29 pm                        *
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

class adc_atmega4809 : public IAnalogueConverter {

public:
  adc_atmega4809() = default;

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

  /**
   *  The default for the Arduino is to give a slow ADC clock for maximum
   *  SNR in the results. That typically means a prescale value of 128
   *  for the 16MHz ATMEGA328P running at 16MHz. Conversions then take more
   *  than 100us to complete. In this application, we want to be able to
   *  perform about 16 conversions in around 500us. To do that the prescaler
   *  is reduced to a value of 32. This gives an ADC clock speed of
   *  500kHz and a single conversion in around 26us. SNR is still pretty good
   *  at these speeds:
   *  http://www.openmusiclabs.com/learning/digital/atmega-m_adc_reading/
   *
   */
  void converter_init() {
    ADC0.CTRLA = ADC_RESSEL_10BIT_gc | ADC_ENABLE_bm; // ADC 10 Bit Resolution, ADC enabled
    ADC0.CTRLB = 0;                                   // No sample accumuation
    ADC0.CTRLC != ADC_SAMPCAP_bm;                     // reduced sampling capacitance See DS.29.5.3
    ADC0.CTRLC &= ~ADC_REFSEL_gm;                     // clear the reference selection
    ADC0.CTRLC |= ~ADC_REFSEL_VDDREF_gc;              // use VDD for the reference
    ADC0.CTRLC &= ADC_PRESC_gm;                       // Clear the prescaler bits
    ADC0.CTRLC |= ADC_PRESC_DIV32_gc;                 // Prescaler DIV 32 => 500kHz?
    ADC0.CTRLD = 0;                                   // no sample delays or variations
    ADC0.CTRLE = 0;                                   // No window comparator
    ADC0.SAMPCTRL = 0;                                // Do not extend the ADC sampling length
    ADC0.INTCTRL |= ADC_RESRDY_bm;                    /* Enable interrupts Gobal interrupt enable must be on*/
    // ADC0.EVCTRL |= ADC_STARTEI_bm; //NEED THIS?                   /* Enable event triggered conversion */
  }

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

  //***************************************************************************//

  void start_conversion_cycle() {
    if (not m_configured) {
      return;
    }
    m_phase = 0; // sync up the start of the sensor sequence
    // bitSet(ADCSRA, ADIE); // enable the ADC interrupt
    start_conversion(15); // begin a dummy conversion to get things started
  }

  void end_conversion_cycle() {
    ADC0.INTCTRL &= ~ADC_RESRDY_bm; /* Disable interrupts */
  }

  void start_conversion(uint8_t channel) {

    // select the channel - fix to channels 0-7
    ADC0.MUXPOS = channel & 0x0F; //
    // start the conversion
    ADC0.INTCTRL = ADC_RESRDY_bm; // Enable ADC result ready interrupt TODO: should this be in init?
    ADC0.INTFLAGS = 0xFF;         // Clear interrupt flags
    ADC0.COMMAND = ADC_STCONV_bm; // start the conversion (cleared on completion)
  }

  int get_adc_result() {
    ADC0.INTFLAGS = ADC_RESRDY_bm;
    // Check that compiler gets low byte then high byte
    return ADC0.RES; // also clears the result ready interrupt flag.
  }

private:
};

extern adc_atmega4809 adc;
