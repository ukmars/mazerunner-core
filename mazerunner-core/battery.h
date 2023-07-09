/******************************************************************************
 * Project: mazerunner-core                                                   *
 * File:    battery.h                                                        *
 * Last Modified: Tuesday, 1st November 2022 10:42:08 am                      *
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
#include "config.h"
#include <Arduino.h>

class Battery;

extern Battery battery;

/***
 *
 */

// TODO set BATTERY MULTIPLIER through the constructor
class Battery {
public:
  explicit Battery(uint8_t channel) : m_adc_channel(channel){};

  void update() {
    m_adc_value = adc.get_dark(m_adc_channel);
    m_battery_volts = BATTERY_MULTIPLIER * m_adc_value;
    m_battery_compensation = 255.0 / m_battery_volts;
  }

  float voltage() {
    return m_battery_volts;
  }

  float compensation() {
    return 255.0 / m_battery_volts;
  }

private:
  Battery(); // no instantiation without an adc channel
  int m_adc_value;
  int m_adc_channel;
  float m_battery_volts;
  float m_battery_compensation;
};
