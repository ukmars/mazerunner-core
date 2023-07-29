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
 * The Queue class is used to speed up flooding of the maze. It is very
 * simple and so not very flexible but shouldbe enough for the flooding.
 *
 * On a larger processor, you might create a Queue of operations that
 * constitute the required moves in a fast run. there is not much room on
 * the Arduino Nano.
 *
 * In the code, queues are created dynamically on the stack so you should
 * have some confidence that there is always enough space. Do not create
 * large queues if there in any doubt. The results can be unpredictable and
 * without obvious warnings. The default size is 64 elements. It is not clear
 * what the minimum queue size should be to guarantee no errors.
 *
 * By only using queues as local variables in this way, there can be no memory
 * leaks or heap fragmentation.
 *
 * To define a Queue of 64 bytes use something like:
 *    Queue<uint8_t> q_bytes;
 *
 * To define a Queue of 100 integers use something like:
 *    Queue<int,100> g_ints;
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
