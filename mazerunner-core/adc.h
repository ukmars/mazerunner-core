/******************************************************************************
 * Project: mazerunner-core                                                   *
 * -----                                                                      *
 * Copyright 2022 - 2023 Peter Harrison, Micromouseonline                     *
 * -----                                                                      *
 * Licence:                                                                   *
 *     Use of this source code is governed by an MIT-style                    *
 *     license that can be found in the LICENSE file or at                    *
 *     https://opensource.org/licenses/MIT.                                   *
 ******************************************************************************/

#ifndef ADC_H
#define ADC_H

#include <Arduino.h>
#include <wiring_private.h>
#include "config.h"

/***
 * The AnalogueConverter class samples a fixed number of ADC channels from 0 to
 * MAX_CHANNELS and makes the readings available to the rest of the program.
 *
 * Each channel is sampled once with the sensor emitters off and once with the emitters on.
 *
 * The first set of samples is stored in the array m_adc_dark[].
 *
 * The second set, with the emitter on, is stored in m_adc_lit[].
 *
 * for wall sensors, you should use the get_raw() method to read the (lit-dark) values.
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
 * Although the code exists almost entirely in this header file, C++ requires the actual
 * instance of the class and its interrupt service routine to be somewhere in a .cpp file.
 * I have placed the instances in the main project file.
 *
 * TODO: some space and time could be saved by only storing the difference between the
 * lit and dark values but it complicates matters elsewhere if code tries to read the
 * values in the middle of the interrupt sequence. If you have all the channels read
 * in one go in a single interrupt service routine - or in systick - then that will not
 * be a problem.
 *
 * TODO: The inclusion in the class of information about the emitters is unfortunate but this
 * is the simplest scheme I could envisage. If there are two emitters, both are turned on
 * together in this version. It would be better to use different emitters for each sensor type.
 * that is left as an exercise for the reader.
 *
 * The code assumes the use of the advanced wall sensor board where there are two emitters. It
 * will work just as well with the basic wall sensor if you simply use the same pin name for
 * both emitter entries.
 *
 * TODO: The A4 and A5 channels get converted in the sequence. If you are expecting to use
 * these for an I2C device, they will need to be skipped.
 *
 * TODO: If only four sensor channels are used, there are opportunities to optimise this
 * code if you are so inclined.
 *
 * PORTING: A simulator may provide fake values as it sees fit and without the delays
 * or interrupts
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

  // call this or nothing will work
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

  uint8_t emitter_front() {
    return m_emitter_front_pin;
  };
  uint8_t emitter_diagonal() {
    return m_emitter_diagonal_pin;
  };

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

    m_phase = 1;  // sync up the start of the sensor sequence
    m_channel = 0;
    bitSet(ADCSRA, ADIE);         // enable the ADC interrupt
    start_conversion(m_channel);  // begin a conversion to get things started
  }

  void end_conversion_cycle() {
    bitClear(ADCSRA, ADIE);  // disable the ADC interrupt
  }

  void start_conversion(uint8_t channel) {
    ADMUX = (ADMUX & 0xF0) | (channel & 0x0F);  // select the channel
    sbi(ADCSRA, ADSC);                          // start the conversion
  }

  int get_adc_result() {
    return ADC;
  }

  int get_lit(const int i) const {
    return m_adc_lit[i];
  }

  int get_dark(const int i) const {
    return m_adc_dark[i];
  }

  int get_raw(const int i) const {
    int diff;
    ATOMIC {
      diff = max(1, m_adc_lit[i] - m_adc_dark[i]);
    }
    return diff;
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

  void callback_adc_isr() {
    switch (m_phase) {
      case 1:
        // cycle through all 8 channels with emitters off
        m_adc_dark[m_channel] = get_adc_result();
        m_channel++;
        start_conversion(m_channel);
        if (m_channel >= MAX_CHANNELS) {
          m_phase = 2;
        }
        break;
      case 2:
        get_adc_result();  // dummy read to clear interrupt flag
        if (m_emitters_enabled) {
          digitalWrite(emitter_diagonal(), 1);
          digitalWrite(emitter_front(), 1);
        }
        m_channel = 0;
        start_conversion(m_channel);  // Start a conversion to generate the interrupt
        m_phase = 3;
        break;
      case 3:
        // skip one cycle for the detectors to respond
        get_adc_result();  // dummy read clears the interrupt flag
        start_conversion(m_channel);
        m_phase = 4;
        break;
      case 4:
        // cycle through the channels again with the emitters on
        // avoid zero result so we know it is working
        m_adc_lit[m_channel] = max(1, get_adc_result());
        m_channel++;
        start_conversion(m_channel);
        if (m_channel >= MAX_CHANNELS) {
          m_phase = 13;
        }
        break;
      case 13:
      default:
        get_adc_result();  // dummy read clears the interrupt flag
        // unconditionally turn off emitters for safety
        digitalWrite(emitter_diagonal(), 0);
        digitalWrite(emitter_front(), 0);
        bitClear(ADCSRA, ADIE);  // turn off the interrupt
        break;
    }
  }

 private:
  volatile int m_adc_dark[MAX_CHANNELS];
  volatile int m_adc_lit[MAX_CHANNELS];
  uint8_t m_emitter_front_pin = -1;
  uint8_t m_emitter_diagonal_pin = -1;
  uint8_t m_index = 0;
  bool m_emitters_enabled = false;
  bool m_configured = false;
  uint8_t m_phase = 0;  // used in the isr
  uint8_t m_channel = 0;
};

#endif