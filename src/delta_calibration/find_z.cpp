//----------- INCLUDES
#include <string>
#include <sstream>
#include <iostream>
#include <fstream>
#include <iterator>
#include <algorithm>
#include <cmath>
#include <math.h>
// ROS INCLUDES
#include <ros/ros.h>
#include <rosbag/bag.h>
#include <rosbag/view.h>
#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/PointCloud2.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

// PCL specific includes
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/common/common.h>
#include <nav_msgs/Odometry.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/common/transforms.h>
#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
#include <pcl/io/obj_io.h>
#include <pcl/pcl_exports.h>
#include <pcl/console/parse.h>
#include <pcl/features/normal_3d.h>
#include <pcl/features/normal_3d_omp.h>
// Ceres Includes
#include "ceres/ceres.h"
#include "ceres/rotation.h"
// OTHER
#include "opencv2/core/core.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include <opencv2/highgui/highgui.hpp>
#include "kinect/plane_filtering.h"
#include "shared/util/popt_pp.h"
#include <Eigen/Geometry>
#include <Eigen/Sparse>
#include <Eigen/OrderingMethods>

// OPENMP_PARALLEL_FOR
#include "vector_localization/residual_functors.h"

#include <fstream>

#include "kdtree.h"

#include "delta_calibration/icp.h"

using namespace icp;
ros::Publisher cloud_pub_1;
ros::Publisher cloud_pub_2;
ros::Publisher cloud_pub_3;
ros::Publisher cloud_pub_4;
ros::Publisher marker_pub;
ros::Publisher markerArray_pub;

void TransformFromFile(string transform_file, double* transform) {
  // Get Transform from file
  string transform_string;
  std::ifstream infile(transform_file.c_str());
  cout << "file open " << endl;
  getline(infile, transform_string);
  istringstream ss(transform_string);
  int i = 0;
  string z;
  getline( ss,z, ' ' );
  while(!ss.eof()) {
    string x;
    getline( ss,x, ' ' );
    transform[i] = atof(x.c_str());
    cout << transform[i] << "\t";
    ++i;
  }
  cout << endl;
}

void find_z(string bag_file, string transform_file, vector<ros::Publisher> publishers) { 
  // Get Transform from file
  double* transform = new double[6];
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
  odom_topics.push_back(std::string("/Cobot/Kinect/Depth"));
  rosbag::View odom_view(bag, rosbag::TopicQuery(odom_topics));
  rosbag::View::iterator bag_it = odom_view.begin();
  rosbag::View::iterator end = odom_view.end();
  std::deque<pcl::PointCloud<pcl::PointXYZ> > buffer;
  std::deque<double> timestamps;
  pcl::PointCloud<pcl::PointXYZ> cloud;
  double time;
  // Get Cloud
  bag_it = OneSensorClouds(bag_it, end, &buffer,
                           &timestamps,
                           &cloud, &time);
  
  // Transform the cloud
  int i = 0;
//   while(i < 10) {
//   PublishCloud(cloud, cloud_pub_1);
//   i++;
//   sleep(2);
//   }
//   TransformPointCloudInv(cloud, transform);
//   i = 0;
//   while(i < 10) {
//   PublishCloud(cloud, cloud_pub_2);
//   i++;
//     sleep(2);
//   }
  
  // Naively find the greatest z value as the height
  pcl::PointXYZ min, max;
  pcl::getMinMax3D(cloud, min, max);
  cout << "Greatest Z: " << max << endl;
  // More complicated, find all planes, find the plane with the normal the closest to perpendicular to the z axis, and then find the distance from 0,0,0 along this normal to that plane.
  vector<Eigen::Vector4d> normal_equations;
  vector<Eigen::Vector3d> k1_centroids;
  // Retrieve the planes from the clouds
//   vector<pcl::PointCloud<pcl::PointXYZ> > planes = getPlanes(cloud, &normal_equations, &k1_centroids);
  // Loop over normal equations to find the one most similar to the z-normal (rediscover the format of these equations)
  
  // Given that plane find the z value of it's centroid as the distance.
}

int main(int argc, char **argv) {

  char* bag_file = (char*)"pair_upright.bag";
  char* transform_file = (char*)"";
  // Parse arguments.
  static struct poptOption options[] = {
    { "bag-file" , 'B', POPT_ARG_STRING, &bag_file ,0, "Process bag file" ,
        "STR" },
    { "transform-file" , 'T', POPT_ARG_STRING, &transform_file ,0, "Transform File" ,
       "STR" },
    POPT_AUTOHELP
    { NULL, 0, 0, NULL, 0, NULL, NULL }
  };

  // parse options
  POpt popt(NULL,argc,(const char**)argv,options,0);
  int c;
  while((c = popt.getNextOpt()) >= 0) {
  }

  // Initialize Ros
  ros::init(argc, argv, "delta_calibration",
  ros::init_options::NoSigintHandler);
  ros::NodeHandle n;
  //----------  Setup Publishers ----------
  cloud_pub_1 = n.advertise<sensor_msgs::PointCloud2> ("cloud_pub_1", 1);
  cloud_pub_2 = n.advertise<sensor_msgs::PointCloud2> ("cloud_pub_2", 1);
  cloud_pub_3 = n.advertise<sensor_msgs::PointCloud2> ("cloud_pub_3", 1);
  cloud_pub_4 = n.advertise<sensor_msgs::PointCloud2> ("cloud_pub_4", 1);
  vector<ros::Publisher> publishers;
  publishers.push_back(cloud_pub_1);
  publishers.push_back(cloud_pub_2);
  publishers.push_back(cloud_pub_3);
  publishers.push_back(cloud_pub_4);
  marker_pub =
  n.advertise<visualization_msgs::Marker>("visualization_marker", 10);
  markerArray_pub =
  n.advertise<visualization_msgs::MarkerArray>(
      "visualization_marker_array", 10);

  find_z(bag_file, transform_file, publishers);
  
  return 0;
}