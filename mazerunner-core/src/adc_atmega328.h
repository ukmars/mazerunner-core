
#pragma once

#include "adc.h"
#include "adc_atmega328.h"
#include "digitalWriteFast.h"
#include <Arduino.h>

class adc_atmega328 : public IAnalogueConverter {

public:
  adc_atmega328() = default;

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
    // Change the clock prescaler from 128 to 32 for a 500kHz clock
    bitSet(ADCSRA, ADPS2);
    bitClear(ADCSRA, ADPS1);
    bitSet(ADCSRA, ADPS0);
    // Set the reference to AVcc and right adjust the result
    ADMUX = DEFAULT << 6;
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
    m_phase = 0;          // sync up the start of the sensor sequence
    bitSet(ADCSRA, ADIE); // enable the ADC interrupt
    start_conversion(15); // begin a dummy conversion to get things started
  }

  void end_sensor_cycle() {
    bitClear(ADCSRA, ADIE); // disable the ADC interrupt
  }

  void start_conversion(uint8_t channel) {

    // select the channel
    ADMUX = (ADMUX & 0xF0) | (channel & 0x0F);
    // start the conversion
    sbi(ADCSRA, ADSC);
  }

  // ADSC is cleared when the conversion finishes and
  // normally you might wait for the end of conversion
  // while (bit_is_set(ADCSRA, ADSC));
  // But - since the conversions are done through interrupts
  // there should always be a result waiting when
  // this function is called.
  //
  // You have to read ADCL first; doing so locks both ADCL
  // and ADCH until ADCH is read.  Reading ADCL second would
  // cause the results of each conversion to be discarded,
  // as ADCL and ADCH would be locked when it completed.
  // Fortunately, the compiler knows how to do that for you.
  int get_adc_result() {
    return ADC;
  }

  friend void adc_isr(adc_atmega328 &a);

private:
  bool m_configured = false;
  uint8_t m_phase = 0; // used in the isr
};

extern adc_atmega328 adc;