/***********************************************************************
 * Created by Peter Harrison on 22/12/2017.
 * Copyright (c) 2017 Peter Harrison
 *
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
 **************************************************************************/

#ifndef MAZE_H
#define MAZE_H

#include <stdint.h>

#define MAZE_WIDTH 16
#define GOAL 0x22
#define START 0x00

// directions for mapping
#define NORTH 0
#define EAST 1
#define SOUTH 2
#define WEST 3

#define VISITED 0xF0

#define INVALID_DIRECTION (0)
#define MAX_COST 255

extern const uint8_t emptyMaze[];
extern const uint8_t japan2007[];

extern uint8_t cost[256];
extern uint8_t walls[256];

// tables give new direction from current heading and next turn
const unsigned char DtoR[] = {1, 2, 3, 0};
const unsigned char DtoB[] = {2, 3, 0, 1};
const unsigned char DtoL[] = {3, 0, 1, 2};

inline void mark_cell_visited(uint8_t cell) {
  walls[cell] |= VISITED;
}

inline bool cell_is_visited(uint8_t cell) {
  return (walls[cell] & VISITED) == VISITED;
}

inline bool is_exit(uint8_t cell, uint8_t direction) {
  return ((walls[cell] & (1 << direction)) == 0);
}

inline bool is_wall(uint8_t cell, uint8_t direction) {
  return ((walls[cell] & (1 << direction)) != 0);
}

void set_maze_goal(uint8_t goal_cell);
uint8_t maze_goal();

uint8_t cell_north(uint8_t cell);
uint8_t cell_east(uint8_t cell);
uint8_t cell_south(uint8_t cell);
uint8_t cell_west(uint8_t cell);
uint8_t neighbour(uint8_t cell, uint8_t direction);
uint8_t neighbour_cost(uint8_t cell, uint8_t direction);
uint8_t direction_to_smallest(uint8_t cell, uint8_t startDirection);

void set_wall_present(uint8_t cell, uint8_t direction);
void set_wall_absent(uint8_t cell, uint8_t direction);

void initialise_maze();
void flood_maze(uint8_t target);

#endif // MAZE_H
