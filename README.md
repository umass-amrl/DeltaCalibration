#DeltaCalibration

DeltaCalibration calculates delta-transforms given input bagfiles of kinect depth image data. The output pose files can be fed to the included matlab files to generate extrinsic calibration for input sensors. check_transform can then be used to visualize the quality of a transform between two sensors using the output transform file from the matlab scripts. find_z is used to identify the z-axis translation for a turtlebot sensor given the partial transform calculated by the turtlebot_cal matlab script. visualize_poses can be used to visualize the transform for a turtlebot base and a kinect sensor. 

###COMPILATION:

Modify .bashrc and include libraries and ros scripts as follows: WARNING: In ubuntu, make sure the lines are entered before the "interactive terminal" check!

```bash
source <ROS DIRECTORY>/setup.bash
export ROS_PACKAGE_PATH=~/cobot:$ROS_PACKAGE_PATH
export LD_LIBRARY_PATH=/usr/local/lib:$LD_LIBRARY_PATH
```

If dependences are not installed (still installs some unnecessary dependencies)

```bash
./InstallPackages
./InstallResearchPackages

mkdir build
make
```
USAGE:
Start calibration watcher

```bash
./bin/calibration_watcher
```

Calibration watcher publishes several topics with information about the current estimate of miscalibration. 

topic: /calibration/ground_plane_error type: std_msgs::Float32MultiArray content: [0] = rotational error in radians [1] = translation error in meters

topic: /calibration/calibration_error type: std_msgs::Float32MultiArray content [0] = rot error in radians [1] = rot_err / total_rotation [2] = translation error in meters [3] = trans_error / total_translation

to trigger recalibration
publish 'recalibrate' on /calibration/commands - (will eventually have two calibration options)
