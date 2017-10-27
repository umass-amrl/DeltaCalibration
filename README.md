# DeltaCalibration

DeltaCalibration is an automatic extrinsic calibration method for depth cameras which calibrates from observed motion from each sensor. 
This package includes code for calibrating Microsoft Kinect sensors, and for recording data and calibrating Kinect Sensors with the Kobuki Turtlebot base. This code can be used to calibrate two Kinect sensors, or a sensor with odometry, and can be used to calibrate either combination from partially informative scenes.

Authors: Jarrett Holtz (jaholtz@cs.umass.edu), Joydeep Biswas

License: LGPL

### COMPILATION:

Modify .bashrc and include libraries and ros scripts as follows: WARNING: In ubuntu, make sure the lines are entered before the "interactive terminal" check!

```bash
source <ROS DIRECTORY>/setup.bash
export ROS_PACKAGE_PATH=~/DeltaCalibration:$ROS_PACKAGE_PATH
export LD_LIBRARY_PATH=/usr/local/lib:$LD_LIBRARY_PATH
```

If dependences are not installed

```bash
./InstallPackages

mkdir build
make calibration
```
### USAGE:
DeltaCalibration requires three steps for extrinsic calibration. Data collection, calculation of DeltaTransforms, and finally extrinsic calibration from DeltaTransforms. In this release we include two sample datasets from the Kobuki Turtlebot. One taken from a fully informative scene, and one from multiple partially informative scenes. Instructions for running the full calibratino stack with these datasets are below, as well as instructions for calibrating with two sensors.

#### Fully Informative Turtlebot Calibration
A fully informative Turtlebot dataset (or for any ground based robot) requires two bagfiles, each containing Kinect point clouds and odometry. One with rotation information, and one with pure translation data, both from fully informative scenes. A fully informative scene is defined as a scene with at least one normal that is not parallel to each axes of translational motion possible for the robot, andwith at least one normal that is not perpendicular to each axes of rotational motion possible for the robot.

Delta Calculation
```bash
./bin/delta_calculate -t 1 -d 5 -B turtlebot_full_rotate.bag
./bin/delta_calculate -t 1 -d 0 -B turtlebot_full_translate.bag
```
where -t 1 specifices turtlebot mode, -d 5 specifies a minimum rotational DeltaTransform size of 5 degrees, -d 0 specifices that the data to be extracted should be pure translations, and -B specifices the bagfile to use for calculating DeltaTransforms.

Calibration
```bash
./bin/delta_calibrate -t 1 -f 'path to rotation file' -g 'path to translation file'
```
Which will output an extrinsic calibration file with the extension '.extrinsics'

Visualizing Calibration
```bash
./bin/visualize_poses -B 'bagfile' -T 'transform file'
```

#### Partially Informative Turtlebot Calibration
A partially informative ground robot dataset requires multiple paired bagfiles, each containing depth clouds and odometry data. Each pair should be a rotation and translation bagfile as above, but each pair of bagfiles should be from a partially informative scene such that the full dataset made up of the pairs contains a fully informative set of scene normals.

Delta Calculation
```bash
./bin/delta_calculate -t 1 -u 1 -d 5 -B turtlebot_partial1/turtlebot_partial1_rotate

./bin/delta_calculate -t 1 -u 1 -d 5 -B turtlebot_partial2/turtlebot_partial2_rotate

./bin/delta_calculate -t 1 -u 1 -d 0 -B turtlebot_partial1/turtlebot_partial1_translate

./bin/delta_calculate -t 1 -u 1 -d 0 -B turtlebot_partial2/turtlebot_partial2_translate
```
where '-u 1' specifices that the transforms may have uncertainty.

Merge Pose and Uncertainty Files
```bash
./merge_deltas.sh 'path to folder with files to merge'
```

Calibration
```bash
./bin/delta_calibrate -t 1 -f 'path to rotation file' -g 'path to translation file'
```
Which will output an extrinsic calibration file with the extension '.extrinsics'

Visualizing Calibration
```bash
./bin/visualize_poses -B 'bagfile' -T 'transform file'
```

#### Calibrating data from two sensors with full 6-DOF motion
Calibrating data from two sensors requires either a series of partially informative datasets or a single fully informative dataset similar to what is required to calibrate a ground robot and a sensor. With full 6-DOF motion however, only one bagfile is needed per dataset, but it must contain rotations about two distinct axes of rotation.

Delta Calculation
```bash
./bin/delta_calculate -d 5 -B 'path to bagfile'
```
where -d 5 specifies a minimum rotational DeltaTransform size of 5 degrees, and -B specifices the bagfile to use for calculating DeltaTransforms.

Calibration
```bash
./bin/delta_calibrate -f 'path to pose file'
```
Which will output an extrinsic calibration file with the extension '.extrinsics'

Visualizing Calibration
```bash
./bin/check_transform -B 'bagfile' -T 'transform file'
```

#### Recording Turtlebot Data
These instructions will run a number of Turtlebot packages, and a single c++ executable which will direct the Turtlebot to perform frontier exploration, recording either a number of partially informative datasets, or a single fully informative dataset depending on what the Turtlebot identifies in it's environment.

Turtlebot Startup
Follow the Turtlebot minimal bringup instructions here found in the Turtlebot tutorials.
http://wiki.ros.org/Robots/TurtleBot

Setup the necessary utilities for frontier exploration.
```bash
roslaunch record_setup.launch
```

Recording Data
```bash
./bin/record_turtlebot_data
```

Output Files and Calibration
This will output a number of bagfiles, one pair for each partially informative scene viewed, and possibly one pair for any fully informative scene viewed. If a fully informative dataset was recorded this should be used for calibration, otherwise calibration must be performed using all of the partially informative datasets recorded.

### Publications
For more information on calibration from partially informative scenes and applying this work to your platform you can view the DeltaCalibration publication linked below.
https://www.joydeepb.com/Publications/delta_calibration.pdf
