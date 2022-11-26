/******************************************************************************
 * Project: mazerunner-core                                                   *
 * File:    MazePrinter.h                                                     *
 * File Created: Saturday, 26th November 2022 10:44:24 pm                     *
 * Author: Peter Harrison                                                     *
 * -----                                                                      *
 * Last Modified: Saturday, 26th November 2022 11:11:15 pm                    *
 * -----                                                                      *
 * Copyright 2022 - 2022 Peter Harrison, Micromouseonline                     *
 * -----                                                                      *
 * Licence:                                                                   *
 *     Use of this source code is governed by an MIT-style                    *
 *     license that can be found in the LICENSE file or at                    *
 *     https://opensource.org/licenses/MIT.                                   *
 ******************************************************************************/

#pragma once

#include "../maze.h"
#include "serial.h"
#include <Arduino.h>

const char dirChars[] = "^>v<*";

enum { PLAIN,
       COSTS,
       DIRS };

class MazePrinter {
public:
  static void printNorthWalls(Maze &maze, int row) {
    for (int col = 0; col < 16; col++) {
      unsigned char cell = row + 16 * col;
      console.print('o');
      if (maze.is_exit(cell, NORTH)) {
        console.print(F("   "));
      } else {
        console.print(F("---"));
      }
    }
    console.println('o');
  }

  static void printSouthWalls(Maze &maze, int row) {
    for (int col = 0; col < 16; col++) {
      unsigned char cell = row + 16 * col;
      console.print('o');
      if (maze.is_exit(cell, SOUTH)) {
        console.print(F("   "));
      } else {
        console.print(F("---"));
      }
    }
    console.println('o');
  }

  static void print_maze(Maze &maze, int style) {
    console.println();
    maze.flood_maze(maze.maze_goal());
    for (int row = 15; row >= 0; row--) {
      printNorthWalls(maze, row);
      for (int col = 0; col < 16; col++) {
        unsigned char cell = row + 16 * col;
        if (maze.is_exit(cell, WEST)) {
          console.print(' ');
        } else {
          console.print('|');
        }
        if (style == COSTS) {
          print_justified(maze.m_cost[cell], 3);
        } else if (style == DIRS) {
          unsigned char direction = maze.direction_to_smallest(cell, NORTH);
          if (cell == maze.maze_goal()) {
            direction = 4;
          }
          console.print(' ');
          console.print(dirChars[direction]);
          console.print(' ');
        } else {
          console.print(F("   "));
        }
      }
      console.println('|');
    }
    printSouthWalls(maze, 0);
    console.println();
    ;
  }

  static void print_maze_wall_data(Maze &maze) {
    console.println();
    ;
    for (int row = 15; row >= 0; row--) {
      for (int col = 0; col < 16; col++) {
        int cell = row + 16 * col;
        print_hex_2(maze.m_walls[cell]);
        console.print(' ');
      }
      console.println();
      ;
    }
    console.println();
    ;
  }
};