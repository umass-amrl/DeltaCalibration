//========================================================================
//  This software is free: you can redistribute it and/or modify
//  it under the terms of the GNU Lesser General Public License Version 3,
//  as published by the Free Software Foundation.
//
//  This software is distributed in the hope that it will be useful,
//  but WITHOUT ANY WARRANTY; without even the implied warranty of
//  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
//  GNU Lesser General Public License for more details.
//
//  You should have received a copy of the GNU Lesser General Public License
//  Version 3 in the file COPYING that came with this distribution.
//  If not, see <http://www.gnu.org/licenses/>.
//========================================================================
//========================================================================

#include "delta_calibration/delta_calc.h"
// #include "delta_calibration/icp.h"

// Intialize empty publishers
ros::Publisher cloudx_pub_1;
ros::Publisher cloudx_pub_2;
ros::Publisher cloudx_pub_3;
ros::Publisher cloudx_pub_4;

ros::Publisher xmarker_pub;
ros::Publisher xmarkerArray_pub;

using namespace delta_calc;
int main(int argc, char **argv) {
  signal(SIGINT, HandleStop);
  signal(SIGALRM, HandleStop);

  int max_clouds = INT_MAX;
  int max_delta_degrees = 0;
  char *bag_file = (char *)"pair_upright.bag";
  int mode = 0;
  //   bool normal_mode = false;
  // Parse arguments.
  static struct poptOption options[] = {
      {"max-clouds", 'k', POPT_ARG_INT, &max_clouds, 0,
        "Max # of clouds to use from file.", "NUM"},
      {"delta", 'd', POPT_ARG_INT, &max_delta_degrees, 0,
        "Minimum rotation in degrees to accept a Delta-Transform. If 0 uses a \
        default translation threshold only. ",
       "NUM"},
      {"bag-file", 'B', POPT_ARG_STRING, &bag_file, 0,
        "Bag File to use for extracting deltas.",
       "STR"},
      {"turtlebot_mode", 't', POPT_ARG_NONE, &mode, 0,
        "Extract Delta-Transforms in TurtlebotMode", "NONE"},
      POPT_AUTOHELP{NULL, 0, 0, NULL, 0, NULL, NULL}};

  // parse options
  POpt popt(NULL, argc, (const char **)argv, options, 0);
  int c;
  while ((c = popt.getNextOpt()) >= 0) {
  }
  // Print option values
  printf("Max Frames: %d\nBagfile: %s\nDelta Size: %d deg\n", max_clouds,
         bag_file, max_delta_degrees);

  // Initialize Ros
  ros::init(argc, argv, "delta_calibration",
            ros::init_options::NoSigintHandler);
  ros::NodeHandle n;
  //----------  Setup Publishers ----------
  cloudx_pub_1 = n.advertise<sensor_msgs::PointCloud2>("cloud_pub_1", 1);
  cloudx_pub_2 = n.advertise<sensor_msgs::PointCloud2>("cloud_pub_2", 1);
  cloudx_pub_3 = n.advertise<sensor_msgs::PointCloud2>("cloud_pub_3", 1);
  cloudx_pub_4 = n.advertise<sensor_msgs::PointCloud2>("cloud_pub_4", 1);
  vector<ros::Publisher> publishers;
  publishers.push_back(cloudx_pub_1);
  publishers.push_back(cloudx_pub_2);
  publishers.push_back(cloudx_pub_3);
  publishers.push_back(cloudx_pub_4);
  xmarker_pub =
      n.advertise<visualization_msgs::Marker>("visualization_marker", 10);
  xmarkerArray_pub = n.advertise<visualization_msgs::MarkerArray>(
      "visualization_marker_array", 10);

  if (mode == 0) {
    DeltaCalculationOpenni(bag_file,
                           publishers,
                           max_delta_degrees,
                           max_clouds,
                           false);
  } else if (mode == 1) {
    cout << "Turtlebot Delta Calculation" << endl;
    DeltaCalculationOpenniOdom(bag_file,
                               publishers,
                               max_delta_degrees,
                               max_clouds,
                               true);
  }
  return 0;
}