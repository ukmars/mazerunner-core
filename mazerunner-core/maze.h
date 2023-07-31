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

#ifndef MAZE_H
#define MAZE_H

/***
 * The Maze class holds the map of the maze as the state of all four
 * walls in each cell.
 *
 * When looking for exits, you can set a view mask. The mask is:
 *      MASK_OPEN => unseen walls are treated as exits
 *    MASK_CLOSED => unseen walls are treated as walls
 *
 * Set the mask to OPEN while searching and CLOSED when calculating a speed run
 *
 * A cell is considered to have been visited if all of its walls have been seen.
 *
 */

#include <stdint.h>
#include "queue.h"
#include "reporting.h"

#define GOAL 0x22
#define START 0x00

#define VISITED 0xF0

enum MazeMask {
  MASK_OPEN = 0x01,    // open maze for search
  MASK_CLOSED = 0x03,  // closed maze for fast run
};

enum WallState {
  EXIT = 0,
  WALL = 1,
  UNKNOWN = 2,
  VIRTUAL = 3,
};

struct WallInfo {
  unsigned char north : 2;
  unsigned char east : 2;
  unsigned char south : 2;
  unsigned char west : 2;
};

enum Heading { NORTH, EAST, SOUTH, WEST, HEADING_COUNT, BLOCKED = 99 };

enum Direction { AHEAD, RIGHT, BACK, LEFT, DIRECTION_COUNT };

#define MAX_COST 255
#define MAZE_WIDTH 16
#define MAZE_CELL_COUNT (MAZE_WIDTH * MAZE_WIDTH)

// for the print function
#define POST 'o'
#define ERR '?'
#define GAP "   "
#define H_WALL F("---")
#define H_EXIT F("   ")
#define H_UNKN F("···")
#define H_VIRT F("###")
#define V_WALL '|'
#define V_EXIT ' '
#define V_UNKN ':'
#define V_VIRT '#'
enum MazeView { PLAIN, COSTS, DIRS };

struct Location {
  uint8_t x;
  uint8_t y;

  Location(uint8_t ix = 0, uint8_t iy = 0) : x(ix), y(iy){};

  bool is_in_maze() {
    return x < MAZE_WIDTH && y < MAZE_WIDTH;
  }

  bool operator==(const Location &obj) const {
    return x == obj.x && y == obj.y;
  }

  bool operator!=(const Location &obj) const {
    return x != obj.x || y != obj.y;
  }
  Location operator+(const Location &obj) const {
    return Location(x + obj.x, y + obj.y);
  }
  Location operator-(const Location &obj) const {
    return Location(x - obj.x, y - obj.y);
  }
  void operator+=(const Location &obj) {
    x += obj.x;
    y += obj.y;
  }

  // these operators prevent the user from exceeding the bounds of the maze
  Location north() const {
    uint8_t new_y = (y + 1) % MAZE_WIDTH;
    uint8_t new_x = x;
    return Location(new_x, new_y);
  }

  Location east() const {
    uint8_t new_y = y;
    uint8_t new_x = (x + 1) % MAZE_WIDTH;
    return Location(new_x, new_y);
  }

  Location south() const {
    uint8_t new_y = (y + MAZE_WIDTH - 1) % MAZE_WIDTH;
    uint8_t new_x = x;
    return Location(new_x, new_y);
  }

  Location west() const {
    uint8_t new_y = y;
    uint8_t new_x = (x + MAZE_WIDTH - 1) % MAZE_WIDTH;
    return Location(new_x, new_y);
  }

  Location neighbour(const Heading heading) const {
    switch (heading) {
      case NORTH:
        return north();
        break;
      case EAST:
        return east();
        break;
      case SOUTH:
        return south();
        break;
      case WEST:
        return west();
        break;
      default:
        return *this;
        break;
    }
  }
};

class Maze {
 public:
  Maze() {
  }

  uint8_t width() const {
    return m_width;
  }

  Location goal() const {
    return m_goal_loc;
  }

  void set_goal(const Location goal) {
    m_goal_loc = goal;
  }

  WallInfo walls(const Location loc) const {
    return m_walls[loc.x][loc.y];
  }

  bool has_unknown_walls(const Location cell) const {
    WallInfo walls_here = m_walls[cell.x][cell.y];
    if (walls_here.north == UNKNOWN || walls_here.east == UNKNOWN || walls_here.south == UNKNOWN || walls_here.west == UNKNOWN) {
      return true;
    } else {
      return false;
    }
  }

  bool cell_is_visited(const Location cell) const {
    return not has_unknown_walls(cell);
  }

  bool is_exit(const Location loc, const Heading direction) const {
    bool result = false;
    WallInfo walls = m_walls[loc.x][loc.y];
    switch (direction) {
      case NORTH:
        result = (walls.north & m_mask) == EXIT;
        break;
      case EAST:
        result = (walls.east & m_mask) == EXIT;
        break;
      case SOUTH:
        result = (walls.south & m_mask) == EXIT;
        break;
      case WEST:
        result = (walls.west & m_mask) == EXIT;
        break;
      default:
        result = false;
        break;
    }
    return result;
  }

  // Unconditionally set a wall state.
  // Normally you only use this in setting up the maze at the start
  void set_wall_state(const Location loc, const Heading direction, const WallState state) {
    switch (direction) {
      case NORTH:
        m_walls[loc.x][loc.y].north = state;
        m_walls[loc.north().x][loc.north().y].south = state;
        break;
      case EAST:
        m_walls[loc.x][loc.y].east = state;
        m_walls[loc.east().x][loc.east().y].west = state;
        break;
      case WEST:
        m_walls[loc.x][loc.y].west = state;
        m_walls[loc.west().x][loc.west().y].east = state;
        break;
      case SOUTH:
        m_walls[loc.x][loc.y].south = state;
        m_walls[loc.south().x][loc.south().y].north = state;
        break;
      default:
        // ignore any other direction (blocked)
        break;
    }
  }

  // only change a wall if it is unknown
  // This is what you use when exploring. Once seen, a wall should not be changed.
  void update_wall_state(const Location loc, const Heading direction, const WallState state) {
    switch (direction) {
      case NORTH:
        if ((m_walls[loc.x][loc.y].north & UNKNOWN) != UNKNOWN) {
          return;
        }
        break;
      case EAST:
        if ((m_walls[loc.x][loc.y].east & UNKNOWN) != UNKNOWN) {
          return;
        }
        break;
      case WEST:
        if ((m_walls[loc.x][loc.y].west & UNKNOWN) != UNKNOWN) {
          return;
        }
        break;
      case SOUTH:
        if ((m_walls[loc.x][loc.y].south & UNKNOWN) != UNKNOWN) {
          return;
        }
        break;
      default:
        // ignore any other direction (blocked)
        break;
    }
    set_wall_state(loc, direction, state);
  }

  /***
   * Initialise a maze and the costs with border m_walls and the start cell
   *
   * If a test maze is provided, the m_walls will all be set up from that
   * No attempt is made to verufy the correctness of a test maze.
   *
   */
  void initialise() {
    for (int x = 0; x < MAZE_WIDTH; x++) {
      for (int y = 0; y < MAZE_WIDTH; y++) {
        m_walls[x][y].north = UNKNOWN;
        m_walls[x][y].east = UNKNOWN;
        m_walls[x][y].south = UNKNOWN;
        m_walls[x][y].west = UNKNOWN;
      }
    }
    for (int x = 0; x < MAZE_WIDTH; x++) {
      m_walls[x][0].south = WALL;
      m_walls[x][MAZE_WIDTH - 1].north = WALL;
    }
    for (int y = 0; y < MAZE_WIDTH; y++) {
      m_walls[0][y].west = WALL;
      m_walls[MAZE_WIDTH - 1][y].east = WALL;
    }
    set_wall_state(Location(0, 0), NORTH, EXIT);
    set_wall_state(Location(0, 0), EAST, WALL);
    set_wall_state(Location(0, 0), SOUTH, WALL);
    set_wall_state(Location(0, 0), WEST, WALL);
    m_walls[0][0].north = EXIT;
    m_walls[0][0].east = WALL;
    m_walls[0][0].south = WALL;
    m_walls[0][0].west = WALL;

    // the open maze treats unknowns as exits
    set_mask(MASK_OPEN);
  }

  void set_mask(const MazeMask mask) {
    m_mask = mask;
  }

  MazeMask get_mask() const {
    return m_mask;
  }

  /***
   * Assumes the maze has been flooded
   */

  uint16_t neighbour_cost(const Location cell, const Heading direction) const {
    if (not is_exit(cell, direction)) {
      return MAX_COST;
    }
    Location next_cell = cell.neighbour(direction);
    return m_cost[next_cell.x][next_cell.y];
  }

  uint16_t cost(const Location cell) const {
    return m_cost[cell.x][cell.y];
  }

  /***
   * Very simple cell counting flood fills m_cost array with the
   * manhattan distance from every cell to the target.
   *
   * Although the queue looks complicated, this is a fast flood that
   * examines each accessible cell exactly once. Consequently, it runs
   * in fairly constant time, taking 5.3ms when there are no interrupts.
   *
   * @param target - the cell from which all distances are calculated
   */

  void flood(const Location target) {
    for (int x = 0; x < MAZE_WIDTH; x++) {
      for (int y = 0; y < MAZE_WIDTH; y++) {
        m_cost[x][y] = (uint8_t)MAX_COST;
      }
    }
    Queue<Location, 64> queue;
    m_cost[target.x][target.y] = 0;
    queue.add(target);
    while (queue.size() > 0) {
      Location here = queue.head();
      uint16_t newCost = m_cost[here.x][here.y] + 1;
      // the casting of enums is potentially problematic
      for (int h = NORTH; h < HEADING_COUNT; h++) {
        Heading heading = static_cast<Heading>(h);
        if (is_exit(here, heading)) {
          Location nextCell = here.neighbour(heading);
          if (m_cost[nextCell.x][nextCell.y] > newCost) {
            m_cost[nextCell.x][nextCell.y] = newCost;
            queue.add(nextCell);
          }
        }
      }
    }
  }

  Heading right_from(const Heading heading) const {
    return static_cast<Heading>((heading + 1) % HEADING_COUNT);
  }

  Heading left_from(const Heading heading) const {
    return static_cast<Heading>((heading + HEADING_COUNT - 1) % HEADING_COUNT);
  }

  Heading ahead_from(const Heading heading) const {
    return heading;
  }

  Heading behind_from(const Heading heading) const {
    return static_cast<Heading>((heading + 2) % HEADING_COUNT);
  }

  /***
   * Algorithm looks around the current cell and records the smallest
   * neighbour and its direction. By starting with the supplied direction,
   * then looking right, then left, the result will preferentially be
   * ahead if there are multiple neighbours with the same m_cost.
   *
   * This could be extended to look ahead then towards the goal but it
   * probably is not worth the effort
   *
   * @param cell
   * @param startDirection
   * @return
   */
  Heading direction_to_smallest(const Location cell, const Heading startDirection) const {
    Heading nextDirection = startDirection;
    uint8_t smallestDirection = BLOCKED;
    uint16_t nextCost;
    uint16_t smallestCost = cost(cell);
    nextCost = neighbour_cost(cell, nextDirection);
    if (nextCost < smallestCost) {
      smallestCost = nextCost;
      smallestDirection = nextDirection;
    };
    nextDirection = right_from(startDirection);
    nextCost = neighbour_cost(cell, nextDirection);
    if (nextCost < smallestCost) {
      smallestCost = nextCost;
      smallestDirection = nextDirection;
    };
    nextDirection = left_from(startDirection);
    nextCost = neighbour_cost(cell, nextDirection);
    if (nextCost < smallestCost) {
      smallestCost = nextCost;
      smallestDirection = nextDirection;
    };
    nextDirection = behind_from(startDirection);
    nextCost = neighbour_cost(cell, nextDirection);
    if (nextCost < smallestCost) {
      smallestCost = nextCost;
      smallestDirection = nextDirection;
    };
    if (smallestCost == MAX_COST) {
      smallestDirection = 0;
    }
    return static_cast<Heading>(smallestDirection);
  }

  /**
   * Maze printing.
   *
   * Included here for want of a better place.
   *
   *
   */

  void print_h_wall(uint8_t state) {
    if (state == EXIT) {
      Serial.print(H_EXIT);
    } else if (state == WALL) {
      Serial.print(H_WALL);
    } else if (state == VIRTUAL) {
      Serial.print(H_VIRT);
    } else {
      Serial.print(H_UNKN);
    }
  }
  void printNorthWalls(int row) {
    // for (int col = 0; col < MAZE_WIDTH; col++) {
    //   unsigned char cell = row + MAZE_WIDTH * col;
    //   Serial.print('o');
    //   // print_h_wall(m_walls[cell].north & m_mask);
    // }
    // Serial.println(POST);
  }

  void printSouthWalls(int row) {
    // for (int col = 0; col < MAZE_WIDTH; col++) {
    //   unsigned char cell = row + MAZE_WIDTH * col;
    //   Serial.print(POST);
    //   print_h_wall(m_walls[cell].south & m_mask);
    // }
    // Serial.println(POST);
  }

  void print(int style = PLAIN) {
    // const char dirChars[] = "^>v<*";
    // Serial.println();
    // flood_maze(maze_goal());
    // for (int row = 15; row >= 0; row--) {
    //   printNorthWalls(row);
    //   for (int col = 0; col < MAZE_WIDTH; col++) {
    //     unsigned char cell = row + MAZE_WIDTH * col;
    //     uint8_t state = m_walls[cell].west & m_mask;
    //     if (state == EXIT) {
    //       Serial.print(V_EXIT);
    //     } else if (state == WALL) {
    //       Serial.print(V_WALL);
    //     } else if (state == VIRTUAL) {
    //       Serial.print(V_VIRT);
    //     } else {
    //       Serial.print(V_UNKN);
    //     }
    //     if (style == COSTS) {
    //       print_justified(m_cost[cell], 3);
    //     } else if (style == DIRS) {
    //       unsigned char direction = direction_to_smallest(cell, NORTH);
    //       if (cell == maze_goal()) {
    //         direction = DIRECTION_COUNT;
    //       }
    //       Serial.print(' ');
    //       Serial.print(dirChars[direction]);
    //       Serial.print(' ');
    //     } else {
    //       Serial.print(GAP);
    //     }
    //   }
    //   Serial.println(V_WALL);
    // }
    // printSouthWalls(0);
    // Serial.println();
  }

 private:
  MazeMask m_mask = MASK_OPEN;
  uint16_t m_width = 16;
  uint8_t m_goal_cell = 0x077;
  Location m_goal_loc{7, 7};
  // on Arduino only use 8 bits for cost to save space
  uint8_t m_cost[MAZE_WIDTH][MAZE_WIDTH];
  WallInfo m_walls[MAZE_WIDTH][MAZE_WIDTH];
};

extern Maze maze;

#endif  // MAZE_H
