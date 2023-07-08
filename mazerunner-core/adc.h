/******************************************************************************
 * Project: mazerunner-core                                                   *
 * File:    h                                                             *
 * File Created: Wednesday, 26th October 2022 10:51:51 pm                     *
 * Author: Peter Harrison                                                     *
 * -----                                                                      *
 * Last Modified: Wednesday, 2nd November 2022 12:58:56 pm                    *
 * -----                                                                      *
 * Copyright 2022 - 2022 Peter Harrison, Micromouseonline                     *
 * -----                                                                      *
 * Licence:                                                                   *
 *     Use of this source code is governed by an MIT-style                    *
 *     license that can be found in the LICENSE file or at                    *
 *     https://opensource.org/licenses/MIT.                                   *
 ******************************************************************************/

#ifndef ADC_H
#define ADC_H

#include "config.h"
#include "digitalWriteFast.h"
#include "list.h"
#include <Arduino.h>
#include <wiring_private.h>
/***
 * The AnalogueConverter class samples a fixed number of ADC from 0 to MAX_CHANNELS and
 * makes the readings available to the resto of the program.
 *
 * Each channel is samples once with the sensor emitters off and once with the emitters on.
 *
 * The first set of samples is stored in tyhe array m_adc_dark[].
 *
 * The second set, with the emitter on, is stored in m_adc_lit[].
 *
 * for wall sensors, you should use the get_difference() method to read the lit-dark values.
 *
 * The class does not care what is connected to the adc channel. It just gathers readings.
 *
 * The battery monitor and analogue button channels are also converted here to avoid conflict.
 * Your config file should declare the channel number for these as well as the actual sensors.
 *
 * This class is specific to UKMARSBOT with a ATmega328p processor. If you are using any other
 * processor, you will need to re-write this class.
 *
 * To save overhead, the class works by using the conversion complete interrupt to gather
 * results only after a successful conversion. The entire sequence is begun by enabling the
 * appropriate interrupt and then starting an ADC conversion. Each interrupt action takes
 * about 5us and conversions take about 30us. Since each channel gets converted twice, the
 * entire sequence takes about 620us to complete but only uses about 100us of processor time.
 *
 * Even if the emitters are disabled, the conversions are still performed but the lit
 * values and dark values will be very similar.
 *
 *
 * Although the code exists almost entirely in this header file, C++ requires the actual
 * instance of the class and its interrupt service routine to be in a .cpp file.
 *
 * TODO: The inclusion in the class of information about the emitters is unfortunate but this
 * is the simplest scheme I could envisage. If there are two emitters, both are turned on
 * together in this version. It would be better to use different enitters for each sensor type.
 * that is left as an exercise for the reader.
 *
 * The code assumes the use of the avanced wall sensor board where there are two emitters. It
 * will work just as well with the basic wall sensor if you simply use the same pin name for
 * both emitter entries.
 *
 * TODO: The A4 and A5 channels get converted in the sequence. If you are expecting to use
 * these for an I2C device, they may need to be skipped.
 *
 * TODO: If only four sensor channels are used, there are opportunities to optimise this
 * code if you are so inclined.
 *
 *
 */

class AnalogueConverter;

extern AnalogueConverter adc;
class AnalogueConverter {
public:
  enum {
    MAX_CHANNELS = 8,
  };

  void enable_emitters() {
    m_emitters_enabled = true;
  }

  void disable_emitters() {
    m_emitters_enabled = false;
  }

  // for convenience allow access in an array-like manner
  volatile int &operator[](int i) { return m_adc_dark[i]; }

  virtual void begin() {
    disable_emitters();
    set_front_emitter_pin(EMITTER_FRONT);
    set_side_emitter_pin(EMITTER_DIAGONAL);
    converter_init();
    m_configured = true;
  };

  void set_front_emitter_pin(uint8_t pin) {
    pinMode(pin, OUTPUT);
    m_emitter_front_pin = pin;
  };

  void set_side_emitter_pin(uint8_t pin) {
    pinMode(pin, OUTPUT);
    m_emitter_diagonal_pin = pin;
  };

  uint8_t emitter_front() { return m_emitter_front_pin; };
  uint8_t emitter_diagonal() { return m_emitter_diagonal_pin; };

  void converter_init() {
    // Change the clock prescaler from 128 to 32 for a 500kHz clock
    bitSet(ADCSRA, ADPS2);
    bitClear(ADCSRA, ADPS1);
    bitSet(ADCSRA, ADPS0);
    // Set the reference to AVcc and right adjust the result
    ADMUX = DEFAULT << 6;
  }

  void start_conversion_cycle() {
    if (not m_configured) {
      return;
    }

    m_phase = 1; // sync up the start of the sensor sequence
    m_channel = 0;
    bitSet(ADCSRA, ADIE);        // enable the ADC interrupt
    start_conversion(m_channel); // begin a conversion to get things started
  }

  void end_conversion_cycle() {
    bitClear(ADCSRA, ADIE); // disable the ADC interrupt
  }

  void start_conversion(uint8_t channel) {
    ADMUX = (ADMUX & 0xF0) | (channel & 0x0F); // select the channel
    sbi(ADCSRA, ADSC);                         // start the conversion
  }

  int get_adc_result() {
    return ADC;
  }

  int get_lit(int i) {
    return m_adc_lit[i];
  }

  int get_dark(int i) {
    return m_adc_dark[i];
  }

  int get_difference(int i) {
    return max(0, m_adc_lit[i] - m_adc_dark[i]);
  }

  /// Perform a 'manual' conversion of a channel
  /// should not be used if the interrupt-driven sequencer is on
  int do_conversion(uint8_t channel) {
    start_conversion(channel);
    while (ADCSRA & (1 << ADSC)) {
      // do nothing
    }
    return get_adc_result();
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

  void isr() {
    digitalWriteFast(LED_USER, 1);
    switch (m_phase) {
      case 1:
        m_adc_dark[m_channel] = get_adc_result();
        m_channel++;
        start_conversion(m_channel);
        if (m_channel > 7) {
          m_phase = 2;
        }
        break;
      case 2:
        get_adc_result(); // dummy read to clear flag
        if (m_emitters_enabled) {
          digitalWriteFast(emitter_diagonal(), 1);
          digitalWriteFast(emitter_front(), 1);
        }
        m_channel = 0;
        start_conversion(m_channel); // Start a conversion to generate the interrupt
        m_phase = 3;
        break;
      case 3:
        // skip one cycle for the detectors to respond
        get_adc_result();
        start_conversion(m_channel);
        m_phase = 4;
        break;
      case 4:
        m_adc_lit[m_channel] = get_adc_result() - m_adc_dark[m_channel];
        m_channel++;
        start_conversion(m_channel);
        if (m_channel > 7) {
          m_phase = 13;
        }
        break;
      case 13:
      default:
        get_adc_result(); // dummy read clears the flag
        // uncoditionally turn off emitters for safety
        digitalWriteFast(emitter_diagonal(), 0);
        digitalWriteFast(emitter_front(), 0);
        bitClear(ADCSRA, ADIE); // turn off the interrupt
        break;
    }
    digitalWriteFast(LED_USER, 0);
  }

private:
  volatile int m_adc_dark[MAX_CHANNELS] = {0};
  volatile int m_adc_lit[MAX_CHANNELS] = {0};
  uint8_t m_emitter_front_pin = -1;
  uint8_t m_emitter_diagonal_pin = -1;
  uint8_t m_index = 0;
  bool m_emitters_enabled = false;
  bool m_configured = false;
  uint8_t m_phase = 0; // used in the isr
  uint8_t m_channel = 0;
};

#endif