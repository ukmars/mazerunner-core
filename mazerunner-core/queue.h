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
 * The Queue class is used to speed up flooding of the maze
 */
template <class item_t, int num_items = 64>
class Queue {
 public:
  Queue() {
    clear();
  }

  int size() {
    return mItemCount;
  }

  void clear() {
    mHead = 0;
    mTail = 0;
    mItemCount = 0;
  }

  void add(item_t item) {
    mData[mTail] = item;
    ++mTail;
    ++mItemCount;
    if (mTail > num_items) {
      mTail -= num_items;
    }
  }

  item_t head() {
    item_t result = mData[mHead];
    ++mHead;
    if (mHead > num_items) {
      mHead -= num_items;
    }
    --mItemCount;
    return result;
  }

 protected:
  item_t mData[num_items + 1];
  int mHead = 0;
  int mTail = 0;
  int mItemCount = 0;

 private:
  // while this is probably correct, prevent use of the copy constructor
  Queue(const Queue<item_t> &rhs) {
  }
};
