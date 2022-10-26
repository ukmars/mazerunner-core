//
// Created by peter on 26/10/2022.
//

#pragma once
/************************************************************************
 *
 * Copyright (C) 2017 by Peter Harrison. www.micromouseonline.com
 *
 * Permission is hereby granted, free of charge, to any person obtaining a
 * copy of this software and associated documentation files (the
 * "Software"), to deal in the Software without restriction, including
 * without l> imitation the rights to use, copy, modify, merge, publish,
 * distribute, sublicense, and/or sell copies of the Software, and to
 * permit persons to whom the Software is furnished to do so, subject to
 * the following conditions:
 *
 * The above copyright notice and this permission notice shall be included
 * in all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS
 * OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF
 * MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.
 * IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY
 * CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT,
 * TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE
 * SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
 *
 ************************************************************************/

#pragma once

/**
 * The List class is VERY basic!
 */
template <class item_t, int num_items = 8>
class List {
public:
  explicit List(int maxSize = num_items) : mTail(0) {
    mData = new item_t[num_items];
    clear();
  }

  ~List() {
    delete[] mData;
  };

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
  item_t *mData;
  //    const int MAX_ITEMS;
  uint8_t mTail;

private:
  // while this is probably correct, prevent use of the copy constructor
  //    List(const List<item_t,num_items> &rhs) {}
};
