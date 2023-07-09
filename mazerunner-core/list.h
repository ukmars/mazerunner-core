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

  bool contains(item_t item) {
    for (int i = 0; i < num_items; i++) {
      if (mData[i] == item) {
        return true;
      }
    }
    return false;
  }

  int operator[](int i) const { return mData[i]; }

protected:
  item_t mData[num_items];
  uint8_t mTail;

private:
  // while this is probably correct, prevent use of the copy constructor
  //    List(const List<item_t,num_items> &rhs) {}
};
