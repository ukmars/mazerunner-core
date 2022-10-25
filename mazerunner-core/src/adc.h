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
    if (pin == 255) {
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

  /***
   * NOTE: Manual analogue conversions
   * All eight available ADC channels are automatically converted
   * by the sensor interrupt. Attempting to performa a manual ADC
   * conversion with the Arduino AnalogueIn() function will disrupt
   * that process so avoid doing that.
   */

  const uint8_t ADC_REF = DEFAULT;

  void start_conversion(uint8_t pin) {
    if (pin >= 14)
      pin -= 14; // allow for channel or pin numbers
                 // set the analog reference (high two bits of ADMUX) and select the
                 // channel (low 4 bits).  Result is right-adjusted
    ADMUX = (ADC_REF << 6) | (pin & 0x07);
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

  /** @brief Sample all the sensor channels with and without the emitter on
   *
   * At the end of the 500Hz systick interrupt, the ADC interrupt is enabled
   * and a conversion started. After each ADC conversion the interrupt gets
   * generated and this ISR is called. The eight channels are read in turn with
   * the sensor emitter(s) off.
   * At the end of that sequence, the emiter(s) get turned on and a dummy ADC
   * conversion is started to provide a delay while the sensors respond.
   * After that, all channels are read again to get the lit values.
   * After all the channels have been read twice, the ADC interrupt is disabbled
   * and the sensors are idle until triggered again.
   *
   * The ADC service runs all th etime even with the sensors 'disabled'. In this
   * software, 'enabled' only means that the emitters are turned on in the second
   * phase. Without that, you might expect the sensor readings to be zero.
   *
   * Timing tests indicate that the sensor ISR consumes no more that 5% of the
   * available system bandwidth.
   *
   * There are actually 16 available channels and channel 8 is the internal
   * temperature sensor. Channel 15 is Gnd. If appropriate, a read of channel
   * 15 can be used to zero the ADC sample and hold capacitor.
   *
   * NOTE: All the channels are read even though only 5 are used for the maze
   * robot. This gives worst-case timing so there are no surprises if more
   * sensors are added.
   * If different types of sensor are used or the I2C is needed, there
   * will need to be changes here.
   */
  void update_channel() {
    switch (m_sensor_phase) {
      case 0:
        start_conversion(A0);
        break;
      case 1:
        m_adc_reading[0] = get_adc_result();
        start_conversion(1);
        break;
      case 2:
        m_adc_reading[1] = get_adc_result();
        start_conversion(A2);
        break;
      case 3:
        m_adc_reading[2] = get_adc_result();
        start_conversion(A3);
        break;
      case 4:
        m_adc_reading[3] = get_adc_result();
        start_conversion(A4);
        break;
      case 5:
        m_adc_reading[4] = get_adc_result();
        start_conversion(A5);
        break;
      case 6:
        m_adc_reading[5] = get_adc_result();
        start_conversion(A6);
        break;
      case 7:
        m_adc_reading[6] = get_adc_result();
        start_conversion(A7);
        break;
      case 8:
        m_adc_reading[7] = get_adc_result();
        if (m_emitters_enabled) {
          digitalWriteFast(m_emitter_a, 1);
          digitalWriteFast(m_emitter_b, 1);
        }
        start_conversion(A7); // dummy adc conversion to create a delay
        // wait at least one cycle for the detectors to respond
        break;
      case 9:
        start_conversion(A0);
        break;
      case 10:
        m_adc_reading[0] = get_adc_result() - m_adc_reading[0];
        start_conversion(A1);
        break;
      case 11:
        m_adc_reading[1] = get_adc_result() - m_adc_reading[1];
        start_conversion(A2);
        break;
      case 12:
        m_adc_reading[2] = get_adc_result() - m_adc_reading[2];
        start_conversion(A3);
        break;
      case 13:
        m_adc_reading[3] = get_adc_result() - m_adc_reading[3];
        start_conversion(A4);
        break;
      case 14:
        m_adc_reading[4] = get_adc_result() - m_adc_reading[4];
        start_conversion(A5);
        break;
      case 15:
        m_adc_reading[5] = get_adc_result() - m_adc_reading[5];
        if (m_emitters_enabled) {
          emitter_off(m_emitter_a);
          emitter_off(m_emitter_b);
        }
        _NOP();
        bitClear(ADCSRA, ADIE); // turn off the interrupt
        break;
      default:
        break;
    }
    m_sensor_phase++;
  }

private:
  AnalogueConverter() = default;
  uint8_t m_sensor_phase = 0;
  bool m_emitters_enabled = false;
  uint8_t m_emitter_a = 255;
  uint8_t m_emitter_b = 255;
  volatile int m_adc_reading[8];
};

extern AnalogueConverter adc;
#endif