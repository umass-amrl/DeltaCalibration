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

//----------- INCLUDES
#include "delta_calibration/icp.h"
#include <pcl/common/common.h>

using namespace icp;
ros::Publisher cloud_pub_1;
ros::Publisher cloud_pub_2;
ros::Publisher cloud_pub_3;
ros::Publisher cloud_pub_4;
ros::Publisher marker_pub;
ros::Publisher markerArray_pub;

void TransformFromFile(string transform_file, double *transform) {
  // Get Transform from file
  string transform_string;
  std::ifstream infile(transform_file.c_str());
  cout << "file open " << endl;
  getline(infile, transform_string);
  istringstream ss(transform_string);
  int i = 0;
  string z;
  while (!ss.eof()) {
    string x;
    getline(ss, x, '\t');
    transform[i] = atof(x.c_str());
    cout << transform[i] << "\t";
    ++i;
  }
  cout << endl;
}

void FindZ(string bag_file, string transform_file,
           vector<ros::Publisher> publishers) {
  // Get Transform from file
  double *transform = new double[6];
  TransformFromFile(transform_file, transform);
  transform[3] = 0;
  transform[4] = 0;
  transform[5] = 0;
  cout << " Finished reading transform " << endl;
  // Get the first pose from the bag file
  rosbag::Bag bag;
  bag.open(bag_file, rosbag::bagmode::Read);
  std::vector<std::string> topics;
  std::vector<std::string> odom_topics;
  odom_topics.push_back(std::string("/camera/depth/points"));
  rosbag::View odom_view(bag, rosbag::TopicQuery(odom_topics));
  rosbag::View::iterator bag_it = odom_view.begin();
  rosbag::View::iterator end = odom_view.end();
  std::deque<pcl::PointCloud<pcl::PointXYZ>> buffer;
  std::deque<double> timestamps;
  pcl::PointCloud<pcl::PointXYZ> cloud, cloud2, cloud3;
  double time;
  // Get Cloud
  bag_it = OneSensorCloudsBrass(bag_it, end, &buffer, &timestamps, &cloud, &time);
  cloud2 = cloud;
  cloud3 = cloud;
  TransformPointCloudInv(&cloud2, transform);
//   TransformPointCloud(&cloud3, transform);
//   while (true) {
//   PublishCloud(cloud3, cloud_pub_1);
//   PublishCloud(cloud2, cloud_pub_2);
//   }
  // Naive Solution find the greatest z value as the height
  pcl::PointXYZ min, max;
  pcl::getMinMax3D(cloud2, min, max);
  cout << "Greatest Z: " << max << endl;
  cout << "Min Z: " << min << endl;
}

int main(int argc, char **argv) {

  char *bag_file = (char *)"pair_upright.bag";
  char *transform_file = (char *)"";
  // Parse arguments.
  static struct poptOption options[] = {
      {"bag-file", 'B', POPT_ARG_STRING, &bag_file, 0, "Process bag file",
       "STR"},
      {"transform-file", 'T', POPT_ARG_STRING, &transform_file, 0,
       "Transform File", "STR"},
      POPT_AUTOHELP{NULL, 0, 0, NULL, 0, NULL, NULL}};

  // parse options
  POpt popt(NULL, argc, (const char **)argv, options, 0);
  int c;
  while ((c = popt.getNextOpt()) >= 0) {
  }

  // Initialize Ros
  ros::init(argc, argv, "delta_calibration",
            ros::init_options::NoSigintHandler);
  ros::NodeHandle n;
  //----------  Setup Publishers ----------
  cloud_pub_1 = n.advertise<sensor_msgs::PointCloud2>("cloud_pub_1", 1);
  cloud_pub_2 = n.advertise<sensor_msgs::PointCloud2>("cloud_pub_2", 1);
  cloud_pub_3 = n.advertise<sensor_msgs::PointCloud2>("cloud_pub_3", 1);
  cloud_pub_4 = n.advertise<sensor_msgs::PointCloud2>("cloud_pub_4", 1);
  vector<ros::Publisher> publishers;
  publishers.push_back(cloud_pub_1);
  publishers.push_back(cloud_pub_2);
  publishers.push_back(cloud_pub_3);
  publishers.push_back(cloud_pub_4);
  marker_pub =
      n.advertise<visualization_msgs::Marker>("visualization_marker", 10);
  markerArray_pub = n.advertise<visualization_msgs::MarkerArray>(
      "visualization_marker_array", 10);

  FindZ(bag_file, transform_file, publishers);

  return 0;
}