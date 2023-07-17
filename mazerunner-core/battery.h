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

#pragma once

#include <Arduino.h>
#include "adc.h"
#include "config.h"

class Battery;

extern Battery battery;

/***
 * The Battery class monitors the battery voltage and provides a
 * correction factor that is used by the motors to ensure that
 * the actual voltage supplied to the motors will be correct even
 * if the battery is over or under its nominal voltage.
 *
 * The robot config file calculates the value of the constant
 * BATTERY_MULTIPLIER based on the values of the potential divider
 * used in the battery monitor circuit. This is stored as a constant
 * in the config because it saves storage and/or a calculation step.
 *
 */
class Battery {
 public:
  explicit Battery(uint8_t channel) : m_adc_channel(channel){};

  void update() {
    m_adc_value = adc.get_dark(m_adc_channel);
    m_battery_volts = BATTERY_MULTIPLIER * m_adc_value;
  }

  float voltage() {
    return m_battery_volts;
  }

 private:
  Battery();  // no instantiation without an adc channel
  int m_adc_value;
  int m_adc_channel;
  float m_battery_volts;
};
