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

#include "maze.h"
#include "queue.h"
#include <avr/pgmspace.h>

uint8_t cost[256];
uint8_t walls[256] __attribute__((section(".noinit"))); // the maze walls are preserved after a reset

static uint8_t s_goal = GOAL;

void set_maze_goal(uint8_t goal_cell) {
  s_goal = goal_cell;
}

uint8_t maze_goal() {
  return s_goal;
}

/***
 * Set a single wall in the maze. Each wall is set from two directions
 * so that it is consistent when seen from the neighbouring cell.
 *
 * The wall is set unconditionally regardless of whether there is
 * already a wall present
 *
 * No check is made on the provided value for direction
 */
void set_wall_present(uint8_t cell, uint8_t direction) {
  uint16_t nextCell = neighbour(cell, direction);
  switch (direction) {
    case NORTH:
      walls[cell] |= (1 << NORTH);
      walls[nextCell] |= (1 << SOUTH);
      break;
    case EAST:
      walls[cell] |= (1 << EAST);
      walls[nextCell] |= (1 << WEST);
      break;
    case SOUTH:
      walls[cell] |= (1 << SOUTH);
      walls[nextCell] |= (1 << NORTH);
      break;
    case WEST:
      walls[cell] |= (1 << WEST);
      walls[nextCell] |= (1 << EAST);
      break;
    default:; // do nothing - although this is an error
      break;
  }
}

/***
 * Clear a single wall in the maze. Each wall is cleared from two directions
 * so that it is consistent when seen from the neighbouring cell.
 *
 * The wall is cleared unconditionally regardless of whether there is
 * already a wall present
 *
 * No check is made on the provided value for direction. Take care not
 * to clear walls around maze boundary.
 */
void set_wall_absent(uint8_t cell, uint8_t direction) {
  uint16_t nextCell = neighbour(cell, direction);
  switch (direction) {
    case NORTH:
      walls[cell] &= ~(1 << NORTH);
      walls[nextCell] &= ~(1 << SOUTH);
      break;
    case EAST:
      walls[cell] &= ~(1 << EAST);
      walls[nextCell] &= ~(1 << WEST);
      break;
    case SOUTH:
      walls[cell] &= ~(1 << SOUTH);
      walls[nextCell] &= ~(1 << NORTH);
      break;
    case WEST:
      walls[cell] &= ~(1 << WEST);
      walls[nextCell] &= ~(1 << EAST);
      break;
    default:; // do nothing - although this is an error
      break;
  }
}

/***
 * Initialise a maze and the costs with border walls and the start cell
 *
 * If a test maze is provided, the walls will all be set up from that
 * No attempt is made to verufy the correctness of a test maze.
 *
 */
void initialise_maze(const uint8_t *testMaze = nullptr) {
  for (int i = 0; i < 256; i++) {
    cost[i] = 0;
    walls[i] = 0;
  }
  if (testMaze) {
    copy_walls_from_flash(testMaze);
    return;
  }
  // place the boundary walls.
  for (uint8_t i = 0; i < 16; i++) {
    set_wall_present(i, WEST);
    set_wall_present(15 * 16 + i, EAST);
    set_wall_present(i * 16, SOUTH);
    set_wall_present((16 * i + 15), NORTH);
  }
  // and the start cell walls.
  set_wall_present(START, EAST);
  set_wall_absent(START, NORTH);
}

uint8_t cell_north(uint8_t cell) {
  uint8_t nextCell = (cell + (1));
  return nextCell;
}

uint8_t cell_east(uint8_t cell) {
  uint8_t nextCell = (cell + (16));
  return nextCell;
}

uint8_t cell_south(uint8_t cell) {
  uint8_t nextCell = (cell + (255));
  return nextCell;
}

uint8_t cell_west(uint8_t cell) {
  uint8_t nextCell = (cell + (240));
  return nextCell;
}

uint8_t neighbour(uint8_t cell, uint8_t direction) {
  uint16_t next;
  switch (direction) {
    case NORTH:
      next = cell_north(cell);
      break;
    case EAST:
      next = cell_east(cell);
      break;
    case SOUTH:
      next = cell_south(cell);
      break;
    case WEST:
      next = cell_west(cell);
      break;
    default:
      next = MAX_COST;
  }
  return next;
}

/***
 * Assumes the maze has been flooded
 */
uint8_t neighbour_cost(uint8_t cell, uint8_t direction) {
  uint8_t result = MAX_COST;
  uint8_t wallData = walls[cell];
  switch (direction) {
    case NORTH:
      if ((wallData & (1 << NORTH)) == 0) {
        result = cost[cell_north(cell)];
      }
      break;
    case EAST:
      if ((wallData & (1 << EAST)) == 0) {
        result = cost[cell_east(cell)];
      }
      break;
    case SOUTH:
      if ((wallData & (1 << SOUTH)) == 0) {
        result = cost[cell_south(cell)];
      }
      break;
    case WEST:
      if ((wallData & (1 << WEST)) == 0) {
        result = cost[cell_west(cell)];
      }
      break;
    default:
      // TODO: this is an error. We should handle it.
      break;
  }
  return result;
}

/***
 * Very simple cell counting flood fills cost array with the
 * manhattan distance from every cell to the target.
 *
 * Although the queue looks complicated, this is a fast flood that
 * examines each accessible cell exactly once. Consequently, it runs
 * in fairly constant time, taking 5.3ms when there are no interrupts.
 *
 * @param target - the cell from which all distances are calculated
 */
void flood_maze(uint8_t target) {
  for (int i = 0; i < 256; i++) {
    cost[i] = MAX_COST;
  }
  Queue<uint8_t> queue;
  cost[target] = 0;
  queue.add(target);
  while (queue.size() > 0) {
    uint8_t here = queue.head();
    uint16_t newCost = cost[here] + 1;

    for (uint8_t direction = 0; direction < 4; direction++) {
      if (is_exit(here, direction)) {
        uint16_t nextCell = neighbour(here, direction);
        if (cost[nextCell] > newCost) {
          cost[nextCell] = newCost;
          queue.add(nextCell);
        }
      }
    }
  }
}

/***
 * Algorithm looks around the current cell and records the smallest
 * neighbour and its direction. By starting with the supplied direction,
 * then looking right, then left, the result will preferentially be
 * ahead if there are multiple neighbours with the same cost.
 *
 * @param cell
 * @param startDirection
 * @return
 */
uint8_t direction_to_smallest(uint8_t cell, uint8_t startDirection) {
  uint8_t nextDirection = startDirection;
  uint8_t smallestDirection = INVALID_DIRECTION;
  uint16_t nextCost;
  uint16_t smallestCost = cost[cell];
  nextCost = neighbour_cost(cell, nextDirection);
  if (nextCost < smallestCost) {
    smallestCost = nextCost;
    smallestDirection = nextDirection;
  };
  nextDirection = (startDirection + 1) % 4; // right
  nextCost = neighbour_cost(cell, nextDirection);
  if (nextCost < smallestCost) {
    smallestCost = nextCost;
    smallestDirection = nextDirection;
  };
  nextDirection = (startDirection + 3) % 4; // left
  nextCost = neighbour_cost(cell, nextDirection);
  if (nextCost < smallestCost) {
    smallestCost = nextCost;
    smallestDirection = nextDirection;
  };
  nextDirection = (startDirection + 2) % 4; // behind
  nextCost = neighbour_cost(cell, nextDirection);
  if (nextCost < smallestCost) {
    smallestCost = nextCost;
    smallestDirection = nextDirection;
  };
  if (smallestCost == MAX_COST) {
    smallestDirection = 0;
  }
  return smallestDirection;
}

/***
 * Since the sample mazes are in flash memory, we cannnot simply copy
 * them without using the PROGMEM stuff
 */
void copy_walls_from_flash(const uint8_t *src) {
  memcpy_P(walls, src, 256);
}

// some sample maze data
const PROGMEM uint8_t emptyMaze[] = {
    0x0E, 0x08, 0x08, 0x08, 0x08, 0x08, 0x08, 0x08, 0x08, 0x08, 0x08, 0x08, 0x08, 0x08, 0x08, 0x09,
    0x0C, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01,
    0x04, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01,
    0x04, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01,
    0x04, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01,
    0x04, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01,
    0x04, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01,
    0x04, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01,
    0x04, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01,
    0x04, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01,
    0x04, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01,
    0x04, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01,
    0x04, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01,
    0x04, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01,
    0x04, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01,
    0x06, 0x02, 0x02, 0x02, 0x02, 0x02, 0x02, 0x02, 0x02, 0x02, 0x02, 0x02, 0x02, 0x02, 0x02, 0x03};

const PROGMEM uint8_t japan2007[] = {
    0x0E, 0x08, 0x0A, 0x09, 0x0C, 0x09, 0x0C, 0x09, 0x0C, 0x0A, 0x0A, 0x09, 0x0D, 0x0C, 0x0A, 0x09,
    0x0C, 0x02, 0x09, 0x06, 0x03, 0x06, 0x03, 0x06, 0x01, 0x0C, 0x0A, 0x03, 0x04, 0x02, 0x09, 0x05,
    0x06, 0x08, 0x02, 0x08, 0x0B, 0x0C, 0x0A, 0x0A, 0x03, 0x06, 0x0A, 0x0A, 0x03, 0x0C, 0x03, 0x05,
    0x0C, 0x02, 0x09, 0x06, 0x0A, 0x03, 0x0C, 0x0A, 0x0A, 0x0A, 0x0A, 0x08, 0x0B, 0x06, 0x09, 0x05,
    //
    0x04, 0x0A, 0x00, 0x0B, 0x0E, 0x08, 0x03, 0x0C, 0x0A, 0x09, 0x0E, 0x00, 0x0B, 0x0C, 0x03, 0x05,
    0x04, 0x0A, 0x00, 0x0B, 0x0C, 0x01, 0x0E, 0x02, 0x09, 0x05, 0x0E, 0x00, 0x0B, 0x06, 0x09, 0x05,
    0x04, 0x0A, 0x00, 0x0B, 0x05, 0x04, 0x08, 0x0A, 0x03, 0x05, 0x0C, 0x03, 0x0C, 0x09, 0x05, 0x05,
    0x05, 0x0C, 0x03, 0x0C, 0x03, 0x05, 0x07, 0x0C, 0x09, 0x05, 0x06, 0x0A, 0x03, 0x05, 0x05, 0x05,
    //
    0x04, 0x02, 0x0A, 0x03, 0x0E, 0x00, 0x0B, 0x04, 0x03, 0x06, 0x0A, 0x0A, 0x09, 0x05, 0x05, 0x05,
    0x06, 0x0A, 0x0A, 0x0A, 0x0A, 0x03, 0x0C, 0x00, 0x0B, 0x0C, 0x0A, 0x0A, 0x03, 0x05, 0x05, 0x05,
    0x0C, 0x09, 0x0C, 0x0A, 0x0A, 0x0A, 0x03, 0x04, 0x0B, 0x06, 0x0A, 0x0A, 0x08, 0x03, 0x05, 0x05,
    0x05, 0x05, 0x05, 0x0C, 0x08, 0x0A, 0x0A, 0x02, 0x0B, 0x0C, 0x0A, 0x09, 0x06, 0x09, 0x05, 0x05,
    //
    0x05, 0x05, 0x05, 0x05, 0x06, 0x0A, 0x0A, 0x0A, 0x09, 0x06, 0x09, 0x06, 0x09, 0x06, 0x01, 0x05,
    0x05, 0x05, 0x05, 0x05, 0x0C, 0x09, 0x0C, 0x09, 0x06, 0x09, 0x06, 0x09, 0x06, 0x09, 0x05, 0x05,
    0x05, 0x06, 0x03, 0x06, 0x03, 0x06, 0x03, 0x06, 0x09, 0x06, 0x0A, 0x02, 0x0B, 0x06, 0x01, 0x05,
    0x06, 0x0A, 0x0A, 0x0A, 0x0A, 0x0A, 0x0A, 0x0A, 0x02, 0x0A, 0x0A, 0x0A, 0x0A, 0x0A, 0x02, 0x03};

//--------------------------------------------------------------------------
