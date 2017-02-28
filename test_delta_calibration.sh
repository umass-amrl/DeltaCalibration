#!/bin/bash

# ./bin/test_delta_calibration -d .02
# ./bin/delta_calibration
#./bin/test_delta_calibration -d .02
#./bin/deltacal_post_process -B ../../ownCloud/jaholtz/shared/paired_kinect_data/kinectStick_test_box_gap -T test_transform.txt
#  ./bin/delta_calibration -B ../../ownCloud/jaholtz/shared/paired_kinect_data/rack_overlap/rack_overlap -d 5
#  matlab -nodisplay -nosplash -nodesktop -r "cd('scripts/matlab_scripts/delta_calibration');addpath('utils');calibrate('../../../../../ownCloud/jaholtz/shared/paired_kinect_data/rack_overlap/rack_overlap');exit"
#   ./bin/delta_calibration -B ../../ownCloud/jaholtz/shared/paired_kinect_data/rack_overlap3/rack_overlap3 -d 5
#  matlab -nodisplay -nosplash -nodesktop -r "cd('scripts/matlab_scripts/delta_calibration');addpath('utils');calibrate('../../../../../ownCloud/jaholtz/shared/paired_kinect_data/rack_overlap3/rack_overlap3');exit"
#   ./bin/delta_calibration -B ../../ownCloud/jaholtz/shared/paired_kinect_data/overlap_final/overlap_final -d 5
#  matlab -nodisplay -nosplash -nodesktop -r "cd('scripts/matlab_scripts/delta_calibration');addpath('utils');calibrate('../../../../../ownCloud/jaholtz/shared/paired_kinect_data/overlap_final/overlap_final');exit"
# ./bin/delta_calibration -B ../../ownCloud/jaholtz/shared/Results/slam_calibration/bag_files/kinect_overlap_1 -d 5
# ./bin/delta_calibration -B ../../ownCloud/jaholtz/shared/Results/slam_calibration/bag_files/kinect_l_1 -d 5
# ./bin/delta_calculate -B turtlebot_partial1_rot -M 1 -d 1
# ./bin/delta_calculate -B turtlebot_partial1_trans -M 1 -d 0
# ./bin/delta_calculate -B turtlebot_partial2_trans -M 1 -d 0
# ./bin/delta_calculate -B turtlebot_partial2_rot -M 1 -d 1
./bin/delta_calculate -B turtlebot_corner -M 1 -d 1

 #  ./bin/check_transform -B ../../ownCloud/jaholtz/shared/paired_kinect_data/rack_overlap/rack_overlap -T ../../ownCloud/jaholtz/shared/paired_kinect_data/rack_overlap/rack_overlap.txt
#./bin/delta_calibration -B ../../ownCloud/jaholtz/shared/paired_kinect_data/kinectStick_close/kinectStick_close -d 5
# ./bin/delta_calibration -B ../../ownCloud/jaholtz/shared/paired_kinect_data/wide_final/wide_final -d 5
# ./bin/delta_calibration -B ../../ownCloud/jaholtz/shared/paired_kinect_data/overlap_final/overlap_final -d 5
# ./bin/delta_calibration -B ../../ownCloud/jaholtz/shared/kinecStick_close/kinecStick_close -d 5
#  matlab -nodisplay -nosplash -nodesktop -r "cd('scripts/matlab_scripts/delta_calibration');addpath('utils');calibrate('../../../../../ownCloud/jaholtz/shared/paired_kinect_data/kinecStick_close/kinecStick_close');exit"
#  ./bin/check_transform -B ../../ownCloud/jaholtz/shared/paired_kinect_data/kinecStick_close/kinecStick_close -T ../../ownCloud/jaholtz/shared/paired_kinect_data/kinecStick_close/kinecStick_close.txt
#./bin/deltacaxl_post_process -B ../../ownCloud/jaholtz/shared/pairedxx_kinect_data/kinectStick_test_box_gap -T test_transform.txt