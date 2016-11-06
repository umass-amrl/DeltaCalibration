//----------- INCLUDES
#include "delta_calibration/icp.h"
#include <string>
#include <sstream>
#include <iostream>
#include <fstream>
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
#include "fspf/plane_filtering.h"
#include "shared/util/popt_pp.h"
#include <Eigen/Geometry>
#include <Eigen/Sparse>
#include <Eigen/OrderingMethods>

// OPENMP_PARALLEL_FOR
#include "vector_localization/residual_functors.h"

#include <fstream>

#include "kdtree.h"



using std::size_t;
using std::vector;
using namespace std;
using namespace icp;
using Eigen::Vector3d;

//Intialize empty publishers
ros::Publisher cloud_pub_1;
ros::Publisher cloud_pub_2;
ros::Publisher cloud_pub_3;
ros::Publisher cloud_pub_4;

ros::Publisher marker_pub;
ros::Publisher markerArray_pub;

// Signal handler for breaks (Ctrl-C)
void HandleStop(int i) {
  printf("\nTerminating.\n");
  exit(0);
}

vector<pcl::PointCloud<pcl::PointXYZ> > GetAllClouds(string bagfile, string topic) {
  fprintf(stdout, "Get All Clouds \n");
  vector<pcl::PointCloud<pcl::PointXYZ> > clouds;
  rosbag::Bag bag;
  bag.open(bagfile, rosbag::bagmode::Read);
  fprintf(stdout, "Bagfile open: %s \n", bagfile.c_str());
  std::vector<std::string> topics;
  topics.push_back(std::string(topic));
  rosbag::View view(bag, rosbag::TopicQuery(topics));
  rosbag::View::iterator it = view.begin();
  rosbag::View::iterator end = view.end();
  fprintf(stdout, "Iterator open");
  while(it != end) {
    const rosbag::MessageInstance &m = *it;
    
    sensor_msgs::PointCloud2Ptr cloudMsg = m.instantiate<sensor_msgs::PointCloud2 >();
    fprintf(stdout, "Instantiated \n");
    pcl::PCLPointCloud2 pcl_pc; 
    pcl::PointCloud<pcl::PointXYZ> cloud;
    pcl_conversions::toPCL(*cloudMsg, pcl_pc);
    fprintf(stdout, "Cloud Convertor \n");
    pcl::fromPCLPointCloud2(pcl_pc, cloud);
    advance(it, 1);
//     pcl::PointCloud<pcl::PointXYZ>* cloud1;
    fprintf(stdout, "Cloud size: %d \n", (int)cloud.size());
    clouds.push_back(cloud);
  }
  fprintf(stdout, "Num Clouds: %d \n", (int)clouds.size());
  return clouds;
}

typedef struct {
  double r,g,b;
} COLOR;

COLOR GetColor(double v,double vmin,double vmax)
{
  COLOR c = {1,1,1}; // white
  double dv;
  
  if (v < vmin)
    v = vmin;
  if (v > vmax)
    v = vmax;
  dv = vmax - vmin;
  
  if (v < (vmin + 0.25 * dv)) {
    c.r = 0;
    c.g = 4 * (v - vmin) / dv;
  } else if (v < (vmin + 0.5 * dv)) {
    c.r = 0;
    c.b = 1 + 4 * (vmin + 0.25 * dv - v) / dv;
  } else if (v < (vmin + 0.75 * dv)) {
    c.r = 4 * (v - vmin - 0.5 * dv) / dv;
    c.b = 0;
  } else {
    c.g = 1 + 4 * (vmin + 0.75 * dv - v) / dv;
    c.b = 0;
  }
  
  return(c);
}

pcl::PointCloud<pcl::PointXYZRGB> Colorize(pcl::PointCloud<pcl::PointXYZ> cloud, COLOR color) {
  pcl::PointCloud<pcl::PointXYZRGB> color_cloud;
  for(size_t i = 0; i < cloud.size(); i++) {
    pcl::PointXYZRGB new_point;
    new_point.x = cloud[i].x;
    new_point.y = cloud[i].y;
    new_point.z = cloud[i].z;
    new_point.r = (int)color.r * 255;
    new_point.g = (int)color.g * 255;
    new_point.b = (int)color.b * 255;
    color_cloud.points.push_back(new_point);
  }
  return color_cloud;
}

pcl::PointCloud<pcl::PointXYZRGB> BuildColorCombo(vector<pcl::PointCloud<pcl::PointXYZ> > clouds, vector<double*>        transforms) {
  pcl::PointCloud<pcl::PointXYZRGB> color_combo;
  vector<int* > colors;
  int yellow[] = { 255,179,0 };
//   std::vector<int> yellow( tmp, tmp+3 ); 
  colors.push_back(yellow);
  int purple[] = { 128,62,117 };
//   std::vector<int> purple( tmp, tmp+3 ); 
  colors.push_back(purple);
  int orange[] = { 255,104,0 };
//   std::vector<int> orange( tmp, tmp+3 ); 
  colors.push_back(orange);
  int blue[] = { 166,189,215 };
//   std::vector<int> blue( tmp, tmp+3 ); 
  colors.push_back(blue);
  int red[] = { 193,0,32 };
//   std::vector<int> red( tmp, tmp+3 ); 
  colors.push_back(red);
  int gray_yellow[] = {206,162,98};
//   std::vector<int> yellow( tmp, tmp+3 ); 
  colors.push_back(gray_yellow);
  int gray[] = {129,112,102};
//   std::vector<int> gray( tmp, tmp+3 ); 
  colors.push_back(gray);
  for(size_t i = 0; i < transforms.size(); i++) {
    color_combo += Colorize(clouds[i], GetColor(i, 0.0, clouds.size()));
  }
  return color_combo;
}

void WritePoseFile(double* pose, const double& timestamp,
                   const int& count, ofstream& file) {
  
  if (file.is_open()) {
    //cout << "Writing to bag_name" << endl;
    for(int j = 0; j < 6; j++) {
      
      file << pose[j] << "\t";
    }
  }
  file << std::setprecision(20) << timestamp << "\t" << count << "\t";
  file << std::flush;
                   }

void KeyframeTest(string bagfile, bool test, vector<ros::Publisher> publishers) {
  vector<pcl::PointCloud<pcl::PointXYZ> > clouds_k1, clouds_k2;
  fprintf(stdout, "About to get clouds 1\n");
  clouds_k1 = GetAllClouds(bagfile, "kinect_1");
  clouds_k2 = GetAllClouds(bagfile, "kinect_2");
  vector<double*> transforms;
  vector<double*> transforms_k2;
  pcl::PointCloud<pcl::PointXYZRGB> combo_k1, combo_k2;
  std::ifstream infile("kinectStick_slow.pose");
  std::string line;
  while (std::getline(infile, line))
  {
    std::istringstream iss(line);
    for(size_t i = 0; i < 2; i++) {
      double* transform = new double[6];
      
      double theta_x, theta_y, theta_z, x, y, z;
      if (!(iss >> theta_x >> theta_y >> 
        theta_z >> x >> y >> z)) { break; } // error
      cout << theta_x << " " << theta_y << " " << theta_z << " " << x << " " << y << " " << z << endl;
      transform[0] = theta_x;
      transform[1] = theta_y;
      transform[2] = theta_z;
      transform[3] = x;
      transform[4] = y;
      transform[5] = z;
      double drop, drop2;
      iss >> drop >> drop2;
        if(i == 0) {
          transforms.push_back(transform);
        } else {
          transforms_k2.push_back(transform);
        }
    }
  }
  fprintf(stdout, "Num Transforms: %d \n", (int)transforms.size());
  

  if(test) {
    // Adjust Deltas to attempt to attempt to improve results
    pcl::PointCloud<pcl::Normal> normal_k1_1 = GetNormals(clouds_k1[0]);
    pcl::PointCloud<pcl::Normal> normal_k1_2;
    pcl::PointCloud<pcl::Normal> normal_k2_1 = GetNormals(clouds_k2[0]);
    pcl::PointCloud<pcl::Normal> normal_k2_2;
  for(size_t k = 0; k < clouds_k2.size() - 1; k++){
    vector<Eigen::Vector2d> empty_coords;
    pcl::PointCloud<pcl::PointXYZ> cloud_k1_1 = clouds_k1[k];
    pcl::PointCloud<pcl::PointXYZ> cloud_k1_2 = clouds_k1[k + 1];
    pcl::PointCloud<pcl::PointXYZ> test_k1_2 = clouds_k1[k + 1];
    pcl::PointCloud<pcl::PointXYZ> cloud_k2_1 = clouds_k2[k];
    pcl::PointCloud<pcl::PointXYZ> cloud_k2_2 = clouds_k2[k + 1];
    normal_k1_2 = GetNormals(cloud_k1_2);
    normal_k2_2 = GetNormals(cloud_k2_2);
    cout << "test case" << endl;
//     TransformPointCloud(test_k1_2, transforms[k]);
    ICP (k,
        .01,
        publishers,
        "",
        "",
        cloud_k1_1,
        cloud_k1_2,
        normal_k1_1,
        normal_k1_2,
        empty_coords,
        empty_coords,
        transforms[k],
        NULL);
    ICP (k,
         .01,
         publishers,
         "",
         "",
         cloud_k2_1,
         cloud_k2_2,
         normal_k2_1,
         normal_k2_2,
         empty_coords,
         empty_coords,
         transforms_k2[k],
         NULL);
    normal_k1_1 = normal_k1_2;
    normal_k2_1 = normal_k2_2;
  }
 }
  
  
  for(size_t i = 1; i < clouds_k2.size(); i++) {
//     PublishCloud(clouds_k1[i], publishers[0]);
//     PublishCloud(clouds_k2[i], publishers[1]);
    sleep(1);
    pcl::PointCloud<pcl::PointXYZ> cloud_k1 = clouds_k1[i];
    pcl::PointCloud<pcl::PointXYZ> cloud_k2 = clouds_k2[i];
    for(int j = i - 1; j >= 0; j--) {
      double* transform_k1 = new double[6];
      double* transform_k2 = new double[6];
      transform_k1 = transforms[j];
      transform_k2 = transforms_k2[j];
      for(size_t k = 0; k < 6; k++) {
        cout << transform_k2[k] << endl;
      }
      cout << endl;
      TransformPointCloud(cloud_k1, transform_k1);
      TransformPointCloud(cloud_k2, transform_k2);
      clouds_k1[i] = cloud_k1;
      clouds_k2[i] = cloud_k2;
    }
//     fprintf(stdout, "Cloud 1 size: %d\n", (int)clouds_k1[i].size());
    
//     combo_k1 += clouds_k1[i];
//     combo_k2 += clouds_k2[i];
//     PublishCloud(clouds_k1[i], publishers[0]);
//     PublishCloud(clouds_k2[i], publishers[1]);
  }
  combo_k1 = BuildColorCombo(clouds_k1, transforms_k2);
  combo_k2 = BuildColorCombo(clouds_k2, transforms_k2);
  fprintf(stdout, "Combo 1 size: %d\n", (int)combo_k1.size());
  string pose_name_1 = "adjusted_transforms.pose";
  ofstream pose_file (pose_name_1.c_str());
  for(size_t i = 0; i < transforms_k2.size(); i++) {
    WritePoseFile(transforms[i], 0, 0, pose_file);
    WritePoseFile(transforms_k2[i], 0, 0, pose_file);
    pose_file << endl;
  }
  pose_file.close();
  while(true) {
  PublishCloud(combo_k1, publishers[2]);
  PublishCloud(combo_k2, publishers[3]);
  }
  
}

int main(int argc, char **argv) {
  signal(SIGINT,HandleStop);
  signal(SIGALRM,HandleStop);
//   int max_clouds = INT_MAX;
//   int max_delta_degrees = 0;
  bool test_mode = false;
  char* bag_file = (char*)"kinectStick_close_keyframes.bag";
  // Parse arguments. (none of these are currently necessary)
  static struct poptOption options[] = {
      { "bag-file" , 'B', POPT_ARG_STRING, &bag_file ,0, "Process bag file" ,
        "STR" },
      { "test-mode", 'T', POPT_ARG_NONE, &test_mode, 0, "Run simulation test",
        "NONE" },
          POPT_AUTOHELP
          { NULL, 0, 0, NULL, 0, NULL, NULL }
  };
  fprintf(stdout, "Keyframe test");
  // parse options
  POpt popt(NULL,argc,(const char**)argv,options,0);
  int c;
  while((c = popt.getNextOpt()) >= 0) {
  }

  fprintf(stdout, "Keyframe test");
  // Initialize Ros
  ros::init(argc, argv, "keyframe_test",
            ros::init_options::NoSigintHandler);
  ros::NodeHandle n;
  //----------  Setup Publishers ----------
  cloud_pub_1 = n.advertise<sensor_msgs::PointCloud2> ("cloud_pub_1x", 1);
  cloud_pub_2 = n.advertise<sensor_msgs::PointCloud2> ("cloud_pub_2x", 1);
  cloud_pub_3 = n.advertise<sensor_msgs::PointCloud2> ("cloud_pub_3x", 1);
  cloud_pub_4 = n.advertise<sensor_msgs::PointCloud2> ("cloud_pub_4x", 1);
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
  fprintf(stdout, "Keyframe test");
  KeyframeTest(bag_file, test_mode, publishers);
  
  return 0;
}