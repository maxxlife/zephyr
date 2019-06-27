Line follower robot using BBC micro:bit board and DFRobot Maqueen chassis
#####################

Origin:
Robot2
https://github.com/maxxlife/zephyr/tree/robot2

Status:
Tested only using latest master Zephyr build
The latest commit was 68c389c1f82aa2c6fedb0d077de31d86cd36bce2

Purpose:
To create a new sample for the BBC micro:bit board in robotics.

Description:
An example that demonstrates how to use Zephyr OS to create line follower robot
with the DFRobot Maqueen robot chassis and BBC micro:bit board.

Building
********
To run this sample you need to have DFRobot Maqueen robot chassis (ROB0148)
and BBC micro:bit board. Use black tape to create a black line on the floor.
Upload a program to the BBC micro:bit board, turn on a robot, and put it
on the black line track.

This project creates a program for a line follower robot.
It can be built as follows:
.. zephyr-app-commands::
   :zephyr-app: samples/boards/microbit/line_follower_robot
   :board: bbc_microbit
   :goals: build
   :compact:

Sample Output
=============
Program doesn’t have a special output on the screen. Real robot will start follow the black line.
Please check the video https://youtu.be/tIvoHQjo8a4

Dependencies:
No special dependencies. To run that sample you must have a special hardware (robot).

URL:
https://github.com/maxxlife/zephyr/tree/robot2

commit:
[a92dd671fe1f0b56ed058d83254c4c7fd1767269]

Maintained-by:
Maksim Masalski <maxxliferobot@gmail.com>

License:
Apache-2.0

License Link:
https://spdx.org/licenses/Apache-2.0.html


