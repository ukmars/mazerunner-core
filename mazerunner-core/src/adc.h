/******************************************************************************
 * Project: mazerunner-core                                                   *
 * File:    adc.h                                                             *
 * File Created: Wednesday, 26th October 2022 10:51:51 pm                     *
 * Author: Peter Harrison                                                     *
 * -----                                                                      *
 * Last Modified: Tuesday, 1st November 2022 11:32:24 am                      * 
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

#include "digitalWriteFast.h"
#include "list.h"
#include <Arduino.h>

/***
 * This is an interface class for an actual adc subsystem. It is a pure
 * virtual class and cannot be instantiated. Instead, you must create
 * a derived class with appropriate code for the target hardware.
 *
 * Currently there are 8 available channels and 2 groups.
 *
 * The channels currently refer to the hardware ADC channel numbers,
 * starting at zero.
 * TODO: A future development should allow the use of any subset of
 * hardware channels.
 *
 * It is expected that all the channels will be read at least once in a
 * conversion cycle. These are the 'dark' readings.
 *
 * This is done on the assumption that not all adc channels are associated
 * with active sensors. For example, on the basic UKMARSBOT, one of the
 * channels is used for the battery voltage measurement and another is
 * used for the analogue switches. Using this technique, the battery monitor
 * and switches just need to grab a reading for the adc converter. If
 * your platform has a different arrangement it should not be affected
 * by this ADC code.
 *
 * After the first read each group will be read again with its associated
 * emitter pin set high. The result for that channel will then be the
 * difference between 'lit' reading and the previous 'dark' reading.
 *
 * Pin numbers are in the range 0..255 and are not necessarily mapped
 * to specific hardware pins. That depends on the target hardware. For
 * An arduino target, pins 0..13 have their usual meaning. Pins 14..23
 * are the pins associated with the Arduino A0..A7 pins.
 *
 * A pin number of 255 is treated as a null value and will be ignored
 * for all operations.
 *
 * Any, or all, of the channels can be added to one (or both) of the
 * groups. At present the groups are hard-coded in sequence in the
 * hardware implementation.
 *
 * Before the converter is used, you should assign emitter pin numbers to
 * each group. If there is no pin number assigned, the channels in group
 * will still be read a second time and so are likely to return a value of
 * zero. TODO: this needs to be fixed
 *
 * Next, each group should have one or more channel numbers allocated to it.
 * These are the channels that will get a second read with the associated
 * emitter lit.
 *
 * Once the emitters and groups have been assigned, call the begin() method
 * to configure the underlying hardware ADC and allow conversion cycle to begin.
 *
 * It is assumed that the conversion cycle happens under interrupt control and
 * is triggered at the end of the systick function by calling the
 * start_conversion_cycle() method.
 *
 * If you have a suitably fast ADC or some other hardware that gathers the
 * sensor readings then the start_conversion_cycle() method should be used to
 * perform the complete cycle in one call.
 *
 * NOTE: Manual analogue conversions:
 * All ADC channels are automatically converted by the sensor interrupt.
 * Attempting to performa a manual ADC conversion will disrupt that
 * process so avoid doing that if you value your sanity.
 *
 * The use of virtual functions comes at some small cost. The VTABLE
 * takes up at least another 8 bytes and there is an overhead in the
 * function call. If this is a problem consider the use of CRTP techniques.
 */
class IAnalogueConverter {
public:
  enum {
    MAX_GROUPS = 2,
    MAX_CHANNELS = 8,
  };

  IAnalogueConverter() {
    for (int i = 0; i < MAX_GROUPS; i++) {
      m_emitter_pin[i] = 255; // pin 255 is a null, or invalid pin
    }
  }

  void set_emitter_for_group(uint8_t pin, uint8_t group) {
    if (group >= MAX_GROUPS) {
      return;
    }
    m_emitter_pin[group] = pin;
  };

  void add_channel_to_group(uint8_t channel, uint8_t group) {
    if (channel >= MAX_CHANNELS) {
      return;
    }
    if (group >= MAX_GROUPS) {
      return;
    }
    m_group[group].add(channel);
  };

  void enable_emitters() {
    m_emitters_enabled = true;
  }

  void disable_emitters() {
    m_emitters_enabled = false;
  }

  // for convenience allow access in an array-like manner
  volatile int &operator[](int i) { return m_adc_reading[i]; }

  // all of these MUST be defined in the derived hardware class
  virtual void begin() = 0;
  virtual void start_conversion_cycle() = 0;
  virtual void end_conversion_cycle() = 0;
  virtual void start_conversion(uint8_t channel) = 0;
  virtual int get_adc_result() = 0;
  virtual void emitter_on(uint8_t pin) = 0;
  virtual void emitter_off(uint8_t pin) = 0;

  // an external function called from the hardware end of conversion ISR
  friend void adc_isr(IAnalogueConverter &a);

protected:
  volatile int m_adc_reading[MAX_CHANNELS] = {0};

  uint8_t m_emitter_pin[MAX_GROUPS] = {0};

  List<uint8_t, MAX_CHANNELS> m_group[MAX_GROUPS];

  uint8_t m_index = 0;
  uint8_t m_group_index = 0;

  bool m_emitters_enabled = false;
  bool m_configured = false;
  uint8_t m_phase = 0; // used in the isr
};

#endif