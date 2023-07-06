/******************************************************************************
 * Project: mazerunner-core                                                   *
 * File:    adc.cpp                                                           *
 * File Created: Thursday, 27th October 2022 12:12:56 pm                      *
 * Author: Peter Harrison                                                     *
 * -----                                                                      *
 * Last Modified: Monday, 31st October 2022 12:51:02 am                       *
 * -----                                                                      *
 * Copyright 2022 - 2022 Peter Harrison, Micromouseonline                     *
 * -----                                                                      *
 * Licence:                                                                   *
 *     Use of this source code is governed by an MIT-style                    *
 *     license that can be found in the LICENSE file or at                    *
 *     https://opensource.org/licenses/MIT.                                   *
 ******************************************************************************/
#include "adc.h"

/***
 * This function should be independent of the hardware but is called from the
 * hardware end-of-conversion ISR.
 *
 * In the adc interface class, it is declared as a friend so that it can access
 * the methods and properties of the actual class instance
 *
 *
 * @brief Sample all the sensor channels with and without the emitter on
 *
 * At the end of the 500Hz systick interrupt, the ADC interrupt is enabled
 * and a conversion started. After each ADC conversion the interrupt gets
 * generated and this ISR is called. The eight channels are read in turn with
 * the sensor emitter(s) off.
 *
 * At the end of that sequence, the emitter(s) get turned on and a dummy ADC
 * conversion is started to provide a delay while the sensors respond.
 * After that, all channels are read again to get the lit values.
 *
 * The lit section handles the sensor channels in two groups so that several
 * channels can be illuminated by one emitters while the others use a different
 * emitter. This is a little clunky but essential to avoid crosstalk between the
 * forward- and side-looking sensor types
 *
 * After all the channels have been read twice, the ADC interrupt is disabbled
 * and the sensors are idle until triggered again.
 *
 * The ADC service runs all the time even with the sensors 'disabled'. In this
 * software, 'enabled' only means that the emitters are turned on in the second
 * phase. Without that, you might expect the sensor readings to be zero.
 *
 * Timing tests indicate that the sensor ISR consumes no more that 5% of the
 * available system bandwidth.
 *
 * There are actually 16 available channels on the ATMEGA328p and channel 8 is
 * the internal temperature sensor. Channel 15 is Gnd. If appropriate, a read of channel
 * 15 can be used to zero the ADC sample and hold capacitor.
 *
 * NOTE: All the channels are read even though only 5 are used for the maze
 * robot. This gives worst-case timing so there are no surprises if more
 * sensors are added.
 *
 * If different types of sensor are used or the I2C is needed, there
 * will need to be changes here.
 */
static int channel;
void adc_isr(IAnalogueConverter &adc) {
  switch (adc.m_phase) {
    case 0: { // initialisation
      adc.m_group_index = 0;
      adc.m_index = 0;
      channel = adc.m_index;
      adc.start_conversion(channel);
      adc.m_phase = 1;
    } break;
    case 1: { // all channels get read 'dark' first
      adc.m_adc_reading[adc.m_index] = adc.get_adc_result();
      adc.m_index += 1;
      if (adc.m_index >= adc.MAX_CHANNELS) {
        if (adc.m_emitters_enabled) {
          adc.m_index = 0;
          adc.m_phase = 2;
          adc.emitter_on(adc.m_emitter_pin[0]);
          adc.start_conversion(7); // dummy conversion to get to the next isr
        } else {
          adc.end_conversion_cycle(); // finish the cycle
        }
        break;
      }
      channel = adc.m_index;
      adc.start_conversion(adc.m_index);
    } break;
    case 2: { // start the first lit group
      channel = adc.m_group[0][adc.m_index];
      adc.start_conversion(channel);
      adc.m_phase = 3;
    } break;
    case 3: { // first group conversions
      adc.m_adc_reading[channel] = adc.get_adc_result() - adc.m_adc_reading[channel];
      adc.m_index += 1;
      if (adc.m_index >= adc.m_group[0].size()) {
        adc.m_index = 0;
        adc.m_phase = 4;
        adc.emitter_off(adc.m_emitter_pin[0]);
        adc.emitter_on(adc.m_emitter_pin[1]);
        adc.start_conversion(7); // dummy conversion to delay one cycle
        break;
      }
      channel = adc.m_group[0][adc.m_index];
      adc.start_conversion(channel);
    } break;
    case 4: { // start the second group
      adc.m_index = 0;
      channel = adc.m_group[1][adc.m_index];
      adc.start_conversion(channel);
      adc.m_phase = 5;
    } break;
    case 5: { // second group conversions
      adc.m_adc_reading[channel] = adc.get_adc_result() - adc.m_adc_reading[channel];
      adc.m_index += 1;
      if (adc.m_index >= adc.m_group[1].size()) {
        adc.m_index = 0;
        adc.emitter_off(adc.m_emitter_pin[1]);
        adc.end_conversion_cycle();
        break;
      }
      channel = adc.m_group[1][adc.m_index];
      adc.start_conversion(channel);
    } break;
  }
}