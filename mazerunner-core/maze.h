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
 * There are a number of supporting structures and types. These are described below.
 *
 */

#include <stdint.h>
#include "queue.h"

#define START Location(0, 0)

/***
 * Walls exist in the map in one of four states and so get recorded using
 * two bits in the map. There are various schemes for this but here the
 * state is held in a single entity for each wall. Symbolic names for each
 * state are listed in the WallState enum.
 *
 * Virtual walls are not used in this code but are labelled for completeness
 */
enum WallState {
  EXIT = 0,     // a wall that has been seen and confirmed absent
  WALL = 1,     // a wall that has been seen and confirmed present
  UNKNOWN = 2,  // a wall that has not yet been seen
  VIRTUAL = 3,  // a wall that has not yet been seen
};

//***************************************************************************//
/***
 * The state of all four walls in a cell are stored as a structure in a single
 * variable.
 *
 * Note that GCC for AVR and STM32 (and probably most other) targets should
 * recognise this as representing a single byte in memory but that is not
 * guaranteed.
 */
struct WallInfo {
  WallState north : 2;
  WallState east : 2;
  WallState south : 2;
  WallState west : 2;
};

/***
 * Since maze wall state can have one of four values, the MazeMask
 * is used to let you use or ignore the fact that a wall has been seen.
 *
 * When the mask is set to MASK_OPEN, any unseen walls are treated as
 * being absent. Setting it to MASK_CLOSED treats unseen walls as present.
 *
 * Practically, this means that fast runs should be calculated after flooding
 * the maze with the MASK_CLOSE option so that the route never passes through
 * unseen walls or cels.
 *
 * Searching should be performed with the MASK_OPEN option so that the flood
 * uses all the cels.
 */
enum MazeMask {
  MASK_OPEN = 0x01,    // open maze for search
  MASK_CLOSED = 0x03,  // closed maze for fast run
};

//***************************************************************************//

/***
 * A Heading represents one of the four cardinal compass headings. Normally
 * this is sufficient. If you are going to run diagonals, you might want to
 * expand the list.
 *
 * The compiler will number enums consecutively starting at zero so we can
 * use a couple of tricks to work out what the new value should be when
 * the robot turns.
 *
 * If you expand the list for diagonals, take care to modify the calculations
 * appropriately.
 */
enum Heading { NORTH, EAST, SOUTH, WEST, HEADING_COUNT, BLOCKED = 99 };

inline Heading right_from(const Heading heading) {
  return static_cast<Heading>((heading + 1) % HEADING_COUNT);
}

inline Heading left_from(const Heading heading) {
  return static_cast<Heading>((heading + HEADING_COUNT - 1) % HEADING_COUNT);
}

inline Heading ahead_from(const Heading heading) {
  return heading;
}

inline Heading behind_from(const Heading heading) {
  return static_cast<Heading>((heading + 2) % HEADING_COUNT);
}
//***************************************************************************//

/***
 * Directions are relative to the robot's orientation. They do not refer to
 * any particular heading and are just used to make code more readable where
 * decisions are made about which way to turn.
 */

enum Direction { AHEAD, RIGHT, BACK, LEFT, DIRECTION_COUNT };

//***************************************************************************//

/***
 * For the classic micromouse contest, the maze is 16x16 cells. That is
 * convenient because a single byte can be used to store the cost associated
 * with each cell when performing a simple flood.
 *
 * If you extend the code to cope with a half-size maze of up to 32x32 cells
 * you will need to make changes to the way the cost is stored.
 *
 * A more sophisticated method for flooding the maze might also require larger
 * values for cost and changes will be needed. the AVR has very limited RAM
 * so the simplest option is used here.
 */
#define MAZE_WIDTH 16
#define MAZE_HEIGHT 16
#define MAZE_CELL_COUNT (MAZE_WIDTH * MAZE_HEIGHT)
#define MAX_COST (MAZE_CELL_COUNT - 1)

//***************************************************************************//

/***
 * Location stores a position in the maze as an (x,y) coordinate pair. This is
 * probably a more intuitive representation than storing just a single index
 * into and array. The method takes a little more code to manage than historical
 * methods but they were devised for processors with very little program memory.
 *
 * This is a example of more processor power making is easier to represent data
 * in a more user-friendly manner.
 *
 * Locations have a number of supporting operations collected into the struct so
 * that you don't have to keep re-writing the same bits of code.
 *
 */
class Location {
 public:
  uint8_t x;
  uint8_t y;

  Location() : x(0), y(0){};
  Location(uint8_t ix, uint8_t iy) : x(ix), y(iy){};

  bool is_in_maze() {
    return x < MAZE_WIDTH && y < MAZE_HEIGHT;
  }

  bool operator==(const Location &obj) const {
    return x == obj.x && y == obj.y;
  }

  bool operator!=(const Location &obj) const {
    return x != obj.x || y != obj.y;
  }

  // these operators prevent the user from exceeding the bounds of the maze
  // by wrapping to the opposite edge
  Location north() const {
    return Location(x, (y + 1) % MAZE_HEIGHT);
  }

  Location east() const {
    return Location((x + 1) % MAZE_WIDTH, y);
  }

  Location south() const {
    return Location(x, (y + MAZE_HEIGHT - 1) % MAZE_HEIGHT);
  }

  Location west() const {
    return Location((x + MAZE_WIDTH - 1) % MAZE_WIDTH, y);
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
        return *this;  // this is actually an error and should be handled
        break;
    }
  }
};

//***************************************************************************//

/***
 * The Maze class is the heart of the micromouse data.
 *
 * In this basic version, the maze has a single goal location even though all
 * micromouse mazes can have a rectangular region that defines a goal area.
 *
 * The two main data blocks in the class store the wall state of every cell
 * and a cost associated with every cell after the maze is flooded.
 *
 * To keep code simple, you will note that each wall is stored twice - once
 * as seen from each side. The methods in the maze take care to ensure that
 * changes to a wall are recorded in both associated cells.
 *
 * Before flooding the maze, take care to set the maze mask as appropriate.
 * See the description above for details of the mask.
 *
 * When the robot searches and updates the map, you should call the method
 * update_wall_state() to record changes. The similar-looking method
 * set_wall_state() is private for a reason. It is unconditional and may
 * result in walls being changed after they were frst seen.
 *
 */
class Maze {
 public:
  Maze() {
  }

  Location goal() const {
    return m_goal;
  }

  /// @brief  changes the default goal. For example in a practice maze
  void set_goal(const Location goal) {
    m_goal = goal;
  }

  /// @brief  return the state of the walls in a cell
  WallInfo walls(const Location cell) const {
    return m_walls[cell.x][cell.y];
  }

  /// @brief return true if ANY walls in a cell have NOT been seen
  bool has_unknown_walls(const Location cell) const {
    WallInfo walls_here = m_walls[cell.x][cell.y];
    if (walls_here.north == UNKNOWN || walls_here.east == UNKNOWN || walls_here.south == UNKNOWN || walls_here.west == UNKNOWN) {
      return true;
    } else {
      return false;
    }
  }

  /// @brief  return true if ALL the walls in a cell have been seen
  bool cell_is_visited(const Location cell) const {
    return not has_unknown_walls(cell);
  }

  /// @brief  Use the current mask to test if a given wall is an exit
  bool is_exit(const Location cell, const Heading heading) const {
    bool result = false;
    WallInfo walls = m_walls[cell.x][cell.y];
    switch (heading) {
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

  /// @brief only change a wall if it is unknown
  // This is what you use when exploring. Once seen, a wall should not be changed again.
  void update_wall_state(const Location cell, const Heading heading, const WallState state) {
    switch (heading) {
      case NORTH:
        if ((m_walls[cell.x][cell.y].north & UNKNOWN) != UNKNOWN) {
          return;
        }
        break;
      case EAST:
        if ((m_walls[cell.x][cell.y].east & UNKNOWN) != UNKNOWN) {
          return;
        }
        break;
      case WEST:
        if ((m_walls[cell.x][cell.y].west & UNKNOWN) != UNKNOWN) {
          return;
        }
        break;
      case SOUTH:
        if ((m_walls[cell.x][cell.y].south & UNKNOWN) != UNKNOWN) {
          return;
        }
        break;
      default:
        // ignore any other heading (blocked)
        break;
    }
    set_wall_state(cell, heading, state);
  }

  /// @brief set empty maze with border walls and the start cell, zero costs
  void initialise() {
    for (int x = 0; x < MAZE_WIDTH; x++) {
      for (int y = 0; y < MAZE_HEIGHT; y++) {
        m_walls[x][y].north = UNKNOWN;
        m_walls[x][y].east = UNKNOWN;
        m_walls[x][y].south = UNKNOWN;
        m_walls[x][y].west = UNKNOWN;
      }
    }
    for (int x = 0; x < MAZE_WIDTH; x++) {
      m_walls[x][0].south = WALL;
      m_walls[x][MAZE_HEIGHT - 1].north = WALL;
    }
    for (int y = 0; y < MAZE_HEIGHT; y++) {
      m_walls[0][y].west = WALL;
      m_walls[MAZE_WIDTH - 1][y].east = WALL;
    }
    set_wall_state(START, EAST, WALL);
    set_wall_state(START, NORTH, EXIT);

    // the open maze treats unknowns as exits
    set_mask(MASK_OPEN);
    flood(goal());
  }

  void set_mask(const MazeMask mask) {
    m_mask = mask;
  }

  MazeMask get_mask() const {
    return m_mask;
  }

  /// @brief return cost for neighbour cell in supplied heading
  uint16_t neighbour_cost(const Location cell, const Heading heading) const {
    if (not is_exit(cell, heading)) {
      return MAX_COST;
    }
    Location next_cell = cell.neighbour(heading);
    return m_cost[next_cell.x][next_cell.y];
  }

  /// @brief  return the cost associated withthe supplied cell location
  uint16_t cost(const Location cell) const {
    return m_cost[cell.x][cell.y];
  }

  /***
   * @brief basic manhattan flood of the maze
   *
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
      for (int y = 0; y < MAZE_HEIGHT; y++) {
        m_cost[x][y] = (uint8_t)MAX_COST;
      }
    }
    /***
     * When the maze is being flooded, there is a queue of 'frontier'
     * cells. These are the cells that are waiting to be checked for
     * neighbours. I believe the maximum size that this queue can
     * possibly be for a classic maze is 64 (MAZE_CELL_COUNT/4) cells.
     * HOWEVER, this is unproven
     */
    Queue<Location, MAZE_CELL_COUNT / 4> queue;
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

  /***
   * Algorithm looks around the current cell and records the smallest
   * neighbour and its direction. By starting with the supplied direction,
   * then looking right, then left, the result will preferentially be
   * ahead if there are multiple neighbours with the same m_cost.
   *
   * This could be extended to look ahead then towards the goal but it
   * probably is not worth the effort
   * @brief get the geating to the lowest cost neighbour
   * @param cell
   * @param start_heading
   * @return
   */
  Heading heading_to_smallest(const Location cell, const Heading start_heading) const {
    Heading next_heading = start_heading;
    Heading best_heading = BLOCKED;
    uint16_t best_cost = cost(cell);
    uint16_t cost;
    cost = neighbour_cost(cell, next_heading);
    if (cost < best_cost) {
      best_cost = cost;
      best_heading = next_heading;
    };
    next_heading = right_from(start_heading);
    cost = neighbour_cost(cell, next_heading);
    if (cost < best_cost) {
      best_cost = cost;
      best_heading = next_heading;
    };
    next_heading = left_from(start_heading);
    cost = neighbour_cost(cell, next_heading);
    if (cost < best_cost) {
      best_cost = cost;
      best_heading = next_heading;
    };
    next_heading = behind_from(start_heading);
    cost = neighbour_cost(cell, next_heading);
    if (cost < best_cost) {
      best_cost = cost;
      best_heading = next_heading;
    };
    if (best_cost == MAX_COST) {
      best_heading = BLOCKED;
    }
    return best_heading;
  }

 private:
  // Unconditionally set a wall state.
  // use update_wall_state() when exploring
  void set_wall_state(const Location loc, const Heading heading, const WallState state) {
    switch (heading) {
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
        // ignore any other heading (blocked)
        break;
    }
  }
  MazeMask m_mask = MASK_OPEN;
  Location m_goal{7, 7};
  // on Arduino only use 8 bits for cost to save space
  uint8_t m_cost[MAZE_WIDTH][MAZE_HEIGHT];
  WallInfo m_walls[MAZE_WIDTH][MAZE_HEIGHT];
};

extern Maze maze;

#endif  // MAZE_H
