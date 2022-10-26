#ifndef ADC_H
#define ADC_H

#include "../config.h"
#include "digitalWriteFast.h"
#include <Arduino.h>
#include <util/atomic.h>
#include <wiring_private.h>

class AnalogueConverter {

public:
  AnalogueConverter() = default;

  void begin(const int emitter_a) {
    begin(emitter_a, emitter_a);
  }

  void begin(const int emitter_a, const int emitter_b) {
    m_emitter_a = emitter_a;
    m_emitter_b = emitter_b;
    pinMode(emitter_a, OUTPUT);
    pinMode(emitter_b, OUTPUT);
    disable_emitters();
    init();
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

  void init() {
    // Change the clock prescaler from 128 to 32 for a 500kHz clock
    bitSet(ADCSRA, ADPS2);
    bitClear(ADCSRA, ADPS1);
    bitSet(ADCSRA, ADPS0);
  }
  // void set_emitter_a(uint8_t pin) {
  //   m_emitter_a = pin;
  // };

  // void set_emitter_b(uint8_t pin) {
  //   m_emitter_b = pin;
  // };

  void enable_emitters() {
    m_emitters_enabled = true;
  }
  void disable_emitters() {
    m_emitters_enabled = false;
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

  volatile int operator[](int i) const { return m_adc_reading[i]; }
  volatile int &operator[](int i) { return m_adc_reading[i]; }
  //***************************************************************************//

  void start_sensor_cycle() {
    m_sensor_phase = 0;   // sync up the start of the sensor sequence
    bitSet(ADCSRA, ADIE); // enable the ADC interrupt
    start_conversion(0);  // begin a conversion to get things started
  }

  void end_sensor_cycle() {
    bitClear(ADCSRA, ADIE); // enable the ADC interrupt
  }

  /***
   * NOTE: Manual analogue conversions
   * All eight available ADC channels are automatically converted
   * by the sensor interrupt. Attempting to performa a manual ADC
   * conversion with the Arduino AnalogueIn() function will disrupt
   * that process so avoid doing that.
   */

  const uint8_t ADC_REF = DEFAULT;

  void start_conversion(uint8_t channel) {
    ADMUX = (ADC_REF << 6) | (channel & 0x0F);
    // start the conversion
    sbi(ADCSRA, ADSC);
  }

  int get_adc_result() {
    // ADSC is cleared when the conversion finishes
    // while (bit_is_set(ADCSRA, ADSC));

    // we have to read ADCL first; doing so locks both ADCL
    // and ADCH until ADCH is read.  reading ADCL second would
    // cause the results of each conversion to be discarded,
    // as ADCL and ADCH would be locked when it completed.
    uint8_t low = ADCL;
    uint8_t high = ADCH;

    // combine the two bytes
    return (high << 8) | low;
  }

  friend void adc_isr(AnalogueConverter a);

private:
  uint8_t m_sensor_phase = 0;
  bool m_emitters_enabled = false;
  uint8_t m_emitter_a = 255;
  uint8_t m_emitter_b = 255;
  volatile int m_adc_reading[8];
};

extern AnalogueConverter adc;
#endif