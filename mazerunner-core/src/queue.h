/******************************************************************************
 * Project: mazerunner-core                                                   *
 * File:    queue.h                                                           *
 * File Created: Tuesday, 25th October 2022 9:25:57 am                        *
 * Author: Peter Harrison                                                     *
 * -----                                                                      *
 * Last Modified: Tuesday, 1st November 2022 10:42:20 am                      *
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
 * The Queue class is used to speed up flooding of the maze
 */
template <class item_t>
class Queue {
public:
  explicit Queue(int maxSize = 64) : MAX_ITEMS(maxSize) {
    mData = new item_t[MAX_ITEMS + 1];
    clear();
  }

  ~Queue() {
    delete[] mData;
  };

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
    if (mTail > MAX_ITEMS) {
      mTail -= MAX_ITEMS;
    }
  }

  item_t head() {
    item_t result = mData[mHead];
    ++mHead;
    if (mHead > MAX_ITEMS) {
      mHead -= MAX_ITEMS;
    }
    --mItemCount;
    return result;
  }

protected:
  item_t *mData = nullptr;
  const int MAX_ITEMS = 10;
  int mHead = 0;
  int mTail = 0;
  int mItemCount = 0;

private:
  // while this is probably correct, prevent use of the copy constructor
  Queue(const Queue<item_t> &rhs) {}
};
