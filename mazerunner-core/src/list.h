/******************************************************************************
 * Project: mazerunner-core                                                   *
 * File:    list.h                                                            *
 * File Created: Wednesday, 26th October 2022 2:59:04 pm                      *
 * Author: Peter Harrison                                                     *
 * -----                                                                      *
 * Last Modified: Wednesday, 2nd November 2022 10:49:38 pm                    *
 * -----                                                                      *
 * Copyright 2022 - 2022 Peter Harrison, Micromouseonline                     *
 * -----                                                                      *
 * Licence:                                                                   *
 *     Use of this source code is governed by an MIT-style                    *
 *     license that can be found in the LICENSE file or at                    *
 *     https://opensource.org/licenses/MIT.                                   *
 ******************************************************************************/

#pragma once

/**
 * The List class is VERY basic! Minimal even.
 */
template <class item_t, int num_items = 8>
class List {
public:
  List() {
    clear();
  }

  int size() {
    return mTail;
  }

  void clear() {
    mTail = 0;
  }

  void add(item_t item) {
    if (mTail >= num_items) {
      return;
    }
    mData[mTail++] = item;
  }
  int operator[](int i) const { return mData[i]; }

protected:
  item_t mData[num_items];
  uint8_t mTail;

private:
  // while this is probably correct, prevent use of the copy constructor
  //    List(const List<item_t,num_items> &rhs) {}
};
