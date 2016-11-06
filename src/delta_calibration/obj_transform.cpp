// TODO

// MOVE IN NECESSARY METHODS
// ADD INITILIZATION OF ZERO ARRAYS
// FINISH WRITING UNWRITTEN METHODS
// TEST ON SIMPLE DATASETS
// WRITE MAIN METHOD
// WRITE FILE IO

// METHODS

// GET_CLOUDS
// checkResidualDist
// checkChange

//----------- INCLUDES
#include <ros/ros.h>
#include <rosbag/bag.h>
#include <rosbag/view.h>
#include <sensor_msgs/image_encodings.h>
#include <signal.h>
#include <cv_bridge/cv_bridge.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <string>
#include <sstream>
#include <iostream>
#include <fstream>
// PCL specific includes
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/common/transforms.h>
#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
#include <pcl/io/obj_io.h>
#include <pcl/pcl_exports.h>
#include <pcl/console/parse.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/features/normal_3d.h>
#include <pcl/features/normal_3d_omp.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/conditional_removal.h>
#include <pcl/filters/statistical_outlier_removal.h>
// Ceres Includes
#include "ceres/ceres.h"
#include "ceres/rotation.h"
#include <cmath>
#include <math.h>
#include "kdtree.h"
#include "opencv2/core/core.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include <opencv2/highgui/highgui.hpp>
#include "fspf/plane_filtering.h"
#include "shared/util/popt_pp.h"
#include <Eigen/Sparse>
#include <Eigen/OrderingMethods>
#include "CImg/CImg.h"
#include <fstream>

#include "delta_calibration/icp.h"


using std::size_t;
using std::vector;
using namespace std;
using namespace icp;
using Eigen::Vector3d;

const float nn_dist = .05;
float neighbor_dist = .5;

//Intialize empty publishers
ros::Publisher cloud_pub;
ros::Publisher cloud_pub_2;
ros::Publisher cloud_pub_3;
ros::Publisher cloud_pub_4;

sensor_msgs::PointCloud2 output;
sensor_msgs::PointCloud2 transformed;
sensor_msgs::PointCloud2 combined;
sensor_msgs::PointCloud2 k_cloud;

ros::Publisher cloud_test_pub;
sensor_msgs::PointCloud2 cloud_test;
ros::Publisher model_cloud_pub;
sensor_msgs::PointCloud2 model_cloud;
ros::Publisher bmodel_cloud_pub;
sensor_msgs::PointCloud2 bmodel_cloud;

ros::Publisher marker_pub;
ros::Publisher markerArray_pub;

// Signal handler for breaks (Ctrl-C)
void HandleStop(int i) {
  printf("\nTerminating.\n");
  exit(0);
}

pcl::PointCloud<pcl::PointXYZ> cloudFromObj(string obj){
  std::ifstream infile(obj.c_str());
  char v;
  double x, y, z;

  vector<Eigen::Vector3f> cloud_vector;
  vector<int> pixelLocs;
  while(infile >> v >> x >> y >> z ){
    Eigen::Vector3f point;
    point(0) = x;
    point(1) = y;
    point(2) = z;
    cloud_vector.push_back(point);
  }
  cout << cloud_vector.size() << endl;
  return CloudFromVector(cloud_vector, pixelLocs);
}

void CheckTransform(const string& obj, const string& outfile, const string& transform_file) {
  // Get Transform from file
  std::ifstream infile(transform_file.c_str());
  string transform_string;
  getline(infile, transform_string);
  double* transform = new double[6];
  istringstream ss(transform_string);
  int i = 0;
  while(!ss.eof()) {
    string x;
    getline( ss, x, ' ' );
    transform[i] = atof(x.c_str());
    cout << transform[i] << "\t";
    ++i;
  }

  string objFile = obj + ".obj";
  cout << objFile << endl;
  string obj2OutFile = outfile;
  // Read in a new cloud from each dataset
  pcl::PointCloud<pcl::PointXYZ> cloud_k1 = cloudFromObj(objFile);
  cout << endl;
  cout << cloud_k1.size() << endl;
  int count = 1;
  TransformPointCloud(cloud_k1, transform);
  WriteToObj(obj2OutFile, "", count, cloud_k1);
}

int main(int argc, char **argv) {
  signal(SIGINT,HandleStop);
  signal(SIGALRM,HandleStop);

  int kMaxClouds = 10;
  int mode = 0;
  char* obj = (char*)"";
  char* obj2 = (char*)"";
  char* transform_file = (char*)"";
  // Parse arguments.
  static struct poptOption options[] = {
    { "kMaxClouds" , 'k', POPT_ARG_INT , &kMaxClouds ,0, "Max Clouds" , "NUM" },
    { "Mode" , 'm', POPT_ARG_INT , &mode ,0, "Run Mode" , "NUM" },
    { "use-ob-file" , 'o', POPT_ARG_STRING, &obj ,0, "Process bag file" ,
        "STR" },
    { "use-obj2-file" , 'O', POPT_ARG_STRING, &obj2 ,0, "Process bag file" ,
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
  // Print option values
  printf("Max Frames: %d \n", kMaxClouds);
  cout << "Using Objs: " << obj << "\t" << obj2 << endl;
  cout << "Mode: " << mode << endl;
  cout << "Using transform file" << transform_file << endl;
  // Initialize Ros
  ros::init(argc, argv, "pose_estimation", ros::init_options::NoSigintHandler);
  ros::NodeHandle n;
  //----------  Setup Visualization ----------
  cloud_pub = n.advertise<sensor_msgs::PointCloud2> ("output", 1);
  cloud_pub_2 = n.advertise<sensor_msgs::PointCloud2> ("transformed", 1);
  cloud_pub_3 = n.advertise<sensor_msgs::PointCloud2> ("combined", 1);
  cloud_pub_4 = n.advertise<sensor_msgs::PointCloud2> ("k_cloud", 1);
  cloud_test_pub = n.advertise<sensor_msgs::PointCloud2> ("cloud_test", 1);
  model_cloud_pub = n.advertise<sensor_msgs::PointCloud2> ("model_cloud", 1);
  bmodel_cloud_pub = n.advertise<sensor_msgs::PointCloud2> ("bmodel_cloud", 1);
  marker_pub =
  n.advertise<visualization_msgs::Marker>("visualization_marker", 10);
  markerArray_pub =
  n.advertise<visualization_msgs::MarkerArray>("visualization_marker_array", 10);

  if (mode == 0) {
    CheckTransform(obj, obj2, transform_file);
  }
  // else if(mode == 1) {
  //   CheckTransform(bagFile, transform_file);
  // }
  return 0;
}
