#!/bin/bash
./bin/delta_calculate -t 1 -u 1 -d 0 -B turtlebot_partial2/turtlebot_partial2_translate

./bin/delta_calculate -t 1 -u 1 -d 0 -B turtlebot_partial1/turtlebot_partial1_translate

./bin/delta_calculate -t 1 -u 1 -d 5 -B turtlebot_partial1/turtlebot_partial1_rotate

./bin/delta_calculate -t 1 -u 1 -d 5 -B turtlebot_partial2/turtlebot_partial2_rotate

./bin/delta_calculate -t 1 -u 0 -d 0 -B turtlebot_partial1/turtlebot_full_translate

./bin/delta_calculate -t 1 -u 0 -d 5 -B turtlebot_partial1/turtlebot_full_rotate