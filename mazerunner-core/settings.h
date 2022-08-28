/*
 * File: settings.h
 * Project: vw-control
 *
 *  File Created: Tuesday, 2nd March 2021 2:41:08 pm
 *
 *  MIT License
 *
 *  Copyright (c) 2020-2021 Rob Probin & Peter Harrison
 *  Copyright (c) 2019-2021 UK Micromouse and Robotics Society
 *
 *  Permission is hereby granted, free of charge, to any person obtaining a copy
 *  of this software and associated documentation files (the "Software"), to deal
 *  in the Software without restriction, including without limitation the rights
 *  to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 *  copies of the Software, and to permit persons to whom the Software is
 *  furnished to do so, subject to the following conditions:
 *
 *  The above copyright notice and this permission notice shall be included in all
 *  copies or substantial portions of the Software.
 *
 *  THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 *  IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 *  FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 *  AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 *  LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 *  OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 *  SOFTWARE.
 */

/***********************************************************************
 *
 * Macros derived from
 * https://stackoverflow.com/questions/201593/is-there-a-simple-way-to-convert-c-enum-to-string/238157#238157
 *
 * Storing and reading stuff in FLASH from:
 * https://www.arduino.cc/reference/en/language/variables/utilities/progmem/
 *
 * SPOILERS: Some proper preprocessor arcana here.
 *           Look away now if you are squeamish
 *
 **************************************************************************/

#ifndef SETTINGS_H
#define SETTINGS_H

#include "config.h"
#include <stdint.h>

/***
 * The revision number can be used to indicate a change to the settings
 * structure. For example a type change for a variable or the addition or
 * deletion of a variable.
 *
 * Without some kind of indicator like this, the code on the target amy
 * try to load settings form EEPROM into a settings structure that is
 * different to the one used to store those settings.
 *
 * A change in the revision implies that the defaults have changed in some
 * significant manner. Normally this is a fundamental change, not just some
 * different default value for a variable.
 *
 * Another reason for a change in the revision might be for a codebase that
 * runs on a similar but different target.
 *
 * The only safe approach when that happens is to overwrite any settings
 * stored in EEPROM with the compiled defaults and load the working settings
 * with those values.
 *
 * NOTE: this means that any custom values in EEPROM will be lost.
 */

/***
 * The address of the copy stored in EEPROM must be fixed. Although the size of
 * the structure is known to the compiler, any program using the settings module
 * may be less aware of how much space is taken from EEPROM storage.
 *
 * That means that the user must keep track of the sizes of any objects held
 * in EEPROM
 */
const int SETTINGS_EEPROM_ADDRESS = 0x0000;
const int SETTING_MAX_SIZE = 64;

/***
 * First, list  all the types that will be used. Identifiers must all be of the
 * form T_xxxx where xxxx is any legal type name in C/C++
 */
enum TypeName : uint8_t {
  T_bool,
  T_int,
  T_uint16_t,
  T_uint32_t,
  T_float,
};

// clang-format off
/***
 * Now create a list of all the settings variables. For each variable add a line
 * of the form
 *    ACTION( type, name, default)  \
 * where
 *    type is any valid C/C++ type - but make sure there is an entry in TypeName
 *    name is a legal C/C++ identifier
 *    default is the value stored in flash as the default
 *
 * This list will be used to generate structures and populate them autumatically
 * at build time.
 *
 * NOTE: if the structure is changed, update SETTINGS_REVISION
 *
 * This is a multi-line macro. do not leave off the trailing backslash
 */
#define SETTINGS_PARAMETERS(ACTION)             \
    ACTION(int, revision, SETTINGS_REVISION)    \
    ACTION(uint16_t, flags,          0                    ) \
    ACTION(float, fwdKP ,            FWD_KP               ) \
    ACTION(float, fwdKD ,            FWD_KD               ) \
    ACTION(float, rotKP ,            ROT_KP               ) \
    ACTION(float, rotKD ,            ROT_KD               ) \
    ACTION(float, steering_KP,       STEERING_KP          ) \
    ACTION(float, steering_KD,       STEERING_KD          ) \
    ACTION(float, mouseRadius,       MOUSE_RADIUS         ) \
    ACTION(int,   left_calibration,  LEFT_CALIBRATION     ) \
    ACTION(int,   front_calibration, FRONT_CALIBRATION    ) \
    ACTION(int,   right_calibration, RIGHT_CALIBRATION    ) \
    ACTION(float, left_adjust,       LEFT_SCALE           ) \
    ACTION(float, front_adjust,      FRONT_SCALE          ) \
    ACTION(float, right_adjust,      RIGHT_SCALE          ) \
    ACTION(int,   left_threshold,    LEFT_THRESHOLD       ) \
    ACTION(int,   front_threshold,   FRONT_THRESHOLD      ) \
    ACTION(int,   right_threshold,   RIGHT_THRESHOLD      ) \
    ACTION(int,   left_nominal,      LEFT_NOMINAL         ) \
    ACTION(int,   front_nominal,     FRONT_NOMINAL        ) \
    ACTION(int,   right_nominal,     RIGHT_NOMINAL        ) \
\

/***
 * These macros are going to be used to generate individual entries in
 * the data structures that are created later on.
 *
 * The macro name will be substituted for the string 'ACTION' in the list above
 *
 */
#define MAKE_STRINGS(       CTYPE,  VAR,   VALUE) const PROGMEM char s_##VAR[] = #VAR;
#define MAKE_NAMES(         CTYPE,  VAR,   VALUE) s_##VAR,
#define MAKE_TYPES(         CTYPE,  VAR,   VALUE) T_##CTYPE,
#define MAKE_DEFAULTS(      CTYPE,  VAR,   VALUE) .VAR = VALUE,
#define MAKE_STRUCT(        CTYPE,  VAR,   VALUE) CTYPE VAR;
#define MAKE_POINTERS(      CTYPE,  VAR,   VALUE) reinterpret_cast<void *>(&settings.VAR),
#define MAKE_CONFIG_ENTRY(  CTYPE,  VAR,   VALUE) {#VAR,T_##CTYPE,reinterpret_cast<void *>(&config.VAR)},

// clang-format on

/***
 * define the structure that holds the settings
 *
 * Uses the MAKE_STRUCT macro to generate one line for each variable in the
 * parameters list. The result will be a basic struct definition like
 *
 * struct Settings {
 *   float mass;
 *   int quanity;
 *   bool loaded;
 * }
 */
struct Settings {
  SETTINGS_PARAMETERS(MAKE_STRUCT)
};

// Now declare the  global instances of the settings data
extern Settings settings;       // the global working copy in RAM
extern const Settings defaults; // The coded-in defaults in flash
// and the supprting structures
extern void *const variablePointers[] PROGMEM;
extern const TypeName variableType[] PROGMEM;

const int get_settings_count();

int get_setting_name(int i, char *s);
void print_setting_name(int i);
void print_setting_type(const int i);
void print_setting_value(const int i, const int dp = DEFAULT_DECIMAL_PLACES);
void print_setting_details(const int i, const int dp = DEFAULT_DECIMAL_PLACES);

// reading and writing EEPROM settings values and defaults
int restore_default_settings();
void save_settings_to_eeprom();
void load_settings_from_eeprom(bool verbose = false);

// send one setting to the serial device in the form '$n=xxx'
void print_setting(const int i, const int dp = DEFAULT_DECIMAL_PLACES);

// send all to the serial device. sets displayed decimals
void dump_settings(const int dp = DEFAULT_DECIMAL_PLACES);
void dump_settings_detail(const int dp = DEFAULT_DECIMAL_PLACES);

// write a value to a setting by index number
int write_setting(const int i, const char *valueString);
/***
 * The templated version executes much faster because there are
 * no calls to the ascii_to_xxx converters.
 *
 * If the string converting version is never called, you will
 * save about 1k of flash. Unless you use atof() or atoi() elsewhere.
 *
 */
template <class T>
int write_setting(const int i, const T value) {
  void *ptr = (void *)pgm_read_word_near(variablePointers + i);
  switch (pgm_read_byte_near(variableType + i)) {
    case T_float:
      *reinterpret_cast<float *>(ptr) = value;
      break;
    case T_bool:
      *reinterpret_cast<bool *>(ptr) = value;
      break;
    case T_uint32_t:
      *reinterpret_cast<uint32_t *>(ptr) = value;
      break;
    case T_uint16_t:
      *reinterpret_cast<uint16_t *>(ptr) = value;
      break;
    case T_int:
      *reinterpret_cast<int *>(ptr) = value;
      break;
    default:
      return -1;
  }
  return 0;
}
#endif // SETTINGS_H
