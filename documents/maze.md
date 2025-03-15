# The Maze

Since this is a Maze running robot, you will need some code for storing and manipulating a map of the maze as well as the ability to decide on the best route to the goal. The `maze.cpp` file contains all this.

## Maze mapping

For simplicity, the code assumes a 16x16 cell classic maze and the wall data is stored in an array of bytes. Each byte represents the wall data for one cell and each wall is represented by a single bit in that byte.

Because the North wall of one cell is also the South wall of an adjacent cell, wall information is actually stored twice and care must be taken to update neighbour cells when the wall information is changed.

In this implementation a one-dimensional array is used to store the two-dimensional map. The first cell in the array (cell 0) is the start cell in the South West corner of the maze. The next cell in the array is the cell immediately to the North of that and so-on. After the first column, the array continues with the cell to the East of the start cell. this is cell 16.

## Directions

So that it is clear everywhere, the four cardinal directions are given numeric values:

    #define NORTH 0
    #define EAST 1
    #define SOUTH 2
    #define WEST 3


## Wall representation

In each byte, a single bit is used to represent the presence of a wall:

    00000000
        ||||
        |||` NORTH
        ||`- EAST
        |`-- SOUTH
         `-- WEST

Thus the value store in the start cell, which always has walls to the West, South and East is 00001110 = 0x0e = 14

Functions are provided that let you set and clear wall data as well as loading, printing and saving the maze data.

## Surviving reset

It is really frustrating if your robot explores a lot of the maze and then crashes. Without care, pressing the reset button will erase the processors memory and you have to start all over again. Even more annoying is the observation that connecting a serial lead to the Arduino causes a processor reset and memory gets wiped again. You can, and probably should, arrange to save the maze to the processor's on-board EEPROM. Not all processors have that option though.

In this code, the maze map is stored in a special section of RAM that will not be wiped after a reset. Note that a power-down _will_ clear even that memory though. You can now press the reset button - or connect a serial lead - and the maze data will be preserved.

## Maze solving

A lot of new builders get hung up on the business of 'solving' the maze. Practically speaking it is not too hard and, in any case, is almost literally the last thing you need to do for your robot. After exploring and mapping the maze walls, the robot needs to be able to find the shortest, or best, route from the start to the goal. This is done by a process called 'flooding'. This is not the place for a long description of the flooding algorithm - there are many resources online that describe how it is done. in essence, the aim is to produce a map of costs that let the robot choose the least-cost neighbour so that it can plan its next move accordingly. That map is another array of 256 bytes organized in the same way as the maze wall data. The cost for cell 0 is in the first element of the array, ad the cost for the cell to the North is in the second element and so on.

In this code, there is a simple and efficient flooding function that should be able to fully flood the maze in about 7 milliseconds. That is fast enough that you can afford to flood the maze at every cell when exploring so that your robot can perform an intelligent search, always trying to find the best route as it searches for the goal.

## The goal

In a full-sized, classic maze, there are 256 cells in a 16x16 square. The goal is one of the four cells in the centre. That is not practical at home so you will probably have a smaller maze and will want to have a goal somewhere that you can reach. in the file `maze.h` you will find a definition for the goal cell location that you can change. just don't forget to set it back to one of the contest cell locations when you run a full contest. More than one contestant has been surprised to find their robot searches for and runs quickly to some place other than the actual goal.

The goal cell location is given in hexadecimal just to help visualise where it is. A practice goal at 0x22 would be in the third column and third row. For the idle, you could set the practice goal to 0x10 which is the cell to the East of the start cell. then you don't even need to stretch out to collect the robot.

Contest goal cells are any one of 0x77, 0x78, 0x87, 0x88.
