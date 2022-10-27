/******************************************************************************
 * Project: mazerunner-core                                                   * 
 * File:    adc.cpp                                                           * 
 * File Created: Thursday, 27th October 2022 12:12:56 pm                      * 
 * Author: Peter Harrison                                                     * 
 * -----                                                                      * 
 * Last Modified: Thursday, 27th October 2022 1:28:33 pm                      * 
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
*/
void adc_isr(IAnalogueConverter &adc) {
  switch (adc.m_phase) {
    case 0: { // initialisation
      adc.m_group_index = 0;
      adc.m_channel_index = 0;
      adc.m_phase = 1;
      adc.start_conversion(15);
    } break;
    case 1: { // all channels get read 'dark' first
      adc.m_adc_reading[adc.m_channel_index] = adc.get_adc_result();
      adc.m_channel_index += 1;
      if (adc.m_channel_index >= adc.MAX_CHANNELS) {
        if (adc.m_emitters_enabled) {
          adc.m_channel_index = 0;
          adc.m_phase = 2;
          adc.emitter_on(adc.m_emitter_pin[0]);
          adc.start_conversion(15); // dummy conversion to get to the next isr
        } else {
          adc.end_conversion_cycle(); // finish the cycle
        }
        break;
      }
      adc.start_conversion(adc.m_channel_index);
    } break;
    case 2: { // start the first lit group
      int channel = adc.m_group[0][adc.m_channel_index];
      adc.start_conversion(channel);
      adc.m_phase = 3;
    } break;
    case 3: { // first group conversions
      int channel = adc.m_group[0][adc.m_channel_index];
      adc.m_adc_reading[channel] = adc.get_adc_result() - adc.m_adc_reading[channel];
      adc.m_channel_index += 1;
      //            std::cout << " [" << int(adc.m_channel_index) << "] ";
      if (adc.m_channel_index >= adc.m_group[0].size()) {
        adc.m_channel_index = 0;
        adc.m_phase = 4;
        adc.emitter_off(adc.m_emitter_pin[0]);
        adc.emitter_on(adc.m_emitter_pin[1]);
        adc.start_conversion(15); // dummy conversion to delay one cycle
        break;
      }
      channel = adc.m_group[0][adc.m_channel_index];
      adc.start_conversion(channel);
    } break;
    case 4: { // start the second group
      int channel = adc.m_group[1][adc.m_channel_index];
      adc.start_conversion(channel);
      adc.m_phase = 5;
    } break;
    case 5: { // second group conversions
      int channel = adc.m_group[0][adc.m_channel_index];
      adc.m_adc_reading[channel] = adc.get_adc_result() - adc.m_adc_reading[channel];
      adc.m_channel_index += 1;
      if (adc.m_channel_index >= adc.m_group[0].size()) {
        adc.m_channel_index = 0;
        adc.emitter_off(adc.m_emitter_pin[1]);
        adc.end_conversion_cycle();
        break;
      }
      channel = adc.m_group[0][adc.m_channel_index];
      adc.start_conversion(channel);
    } break;
  }
}
