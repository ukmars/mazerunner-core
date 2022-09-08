#ifndef UTILS_H
#define UTILS_H

// simple formatting functions for printing maze costs
inline void print_hex_2(unsigned char value) {
  if (value < 16) {
    Serial.print('0');
  }
  Serial.print(value, HEX);
}

inline void print_justified(int32_t value, int width) {
  int v = value;
  int w = width;
  w--;
  if (v < 0) {
    w--;
  }
  while (v /= 10) {
    w--;
  }
  while (w > 0) {
    Serial.write(' ');
    --w;
  }
  Serial.print(value);
}
inline void print_justified(int value, int width) {
  print_justified(int32_t(value), width);
}

#endif