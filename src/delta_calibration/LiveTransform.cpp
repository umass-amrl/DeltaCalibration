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


using std::size_t;
using std::vector;
using namespace std;
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
// Transforms a given eigen point by the given transform (array input)
Eigen::Matrix<double,3,1> TransformPoint(
  const Eigen::Matrix<double,3,1>& point, 
  const double transform[6]){

  //Create the eigen transform from the camera
  Eigen::Matrix<double,3,1> axis(transform[0], transform[1], transform[2]);
  const double angle = axis.norm();
  if(angle != 0){
    axis = axis / angle;
  }
  Eigen::Transform<double, 3, Eigen::Affine> rotation =
  Eigen::Transform<double, 3, Eigen::Affine>(Eigen::AngleAxis<double>(
    angle, axis));


  Eigen::Translation<double, 3> translation = 
  Eigen::Translation<double, 3>(transform[3], transform[4], transform[5]);

  Eigen::Transform<double, 3, Eigen::Affine> affine_transform =
   translation * rotation;

  Eigen::Matrix<double,3,1> transformed_point;
  transformed_point = affine_transform * point;
  return transformed_point;
}

void publish_cloud(
  const pcl::PointCloud<pcl::PointXYZ>& cloud, ros::Publisher publisher) {
  sensor_msgs::PointCloud2 temp_cloud;
  pcl::PCLPointCloud2 pcl_cloud;
  pcl::toPCLPointCloud2(cloud, pcl_cloud);
  pcl_conversions::fromPCL(pcl_cloud, temp_cloud);
  temp_cloud.header.frame_id = "point_cloud";

  publisher.publish(temp_cloud);
}

// Transforms all points in a pcl point cloud (array input)
void TransformPointCloud_PCL(
  pcl::PointCloud<pcl::PointXYZ>& cloud, 
  double transform[6]){

  for(size_t i = 0; i < cloud.size(); ++i){
    pcl::PointXYZ point = cloud[i];
    
    Eigen::Matrix<double,3,1> point_matrix;
    Eigen::Matrix<double,3,1> transformed_point_matrix;
    point_matrix << point.x, point.y, point.z;
    transformed_point_matrix = TransformPoint(point_matrix, transform);
    point.x = transformed_point_matrix[0];
    point.y = transformed_point_matrix[1];
    point.z = transformed_point_matrix[2];
    cloud[i] = point;
  }
}

// Transforms all points in a pcl point cloud (vector input)
void TransformPointCloud_PCL(
  pcl::PointCloud<pcl::PointXYZ>& cloud, 
  const vector<double>& transform){

  double transform_array[6];
  std::copy(transform.begin(), transform.end(), transform_array);
  TransformPointCloud_PCL(cloud, transform_array);
}

pcl::PointCloud<pcl::PointXYZ> CloudFromVector(
  vector<Eigen::Vector3f> pointCloud, vector<int> pixelLocs){
  
  pcl::PointCloud<pcl::PointXYZ> cloud;
  //Process Depth To PCL Cloud
  cloud.resize(pointCloud.size());
  //cloud.height = 480;
  //cloud.width = 640;

  for (uint i = 0; i < cloud.size(); ++i) {

        pcl::PointXYZ point;
        point.x = pointCloud[i](0);
        point.z = pointCloud[i](1);
        point.y=  pointCloud[i](2);
        cloud[i] = point;
    }
    return cloud;
}

// Reads clouds from a given iterator, saves to buffer if they are over
rosbag::View::iterator get_clouds(rosbag::View::iterator it,
 std::deque<pcl::PointCloud<pcl::PointXYZ> >* buffer1, 
 std::deque<pcl::PointCloud<pcl::PointXYZ> >* buffer2,
 std::deque<double>* timestamps_1,
 std::deque<double>* timestamps_2) {
  PlaneFilter filter;
  KinectRawDepthCam camera = KinectRawDepthCam();
  filter.setDepthCamera(&camera);
  string kinect_0 = "kinect_0";
  string kinect_1 = "kinect_1";
  while((buffer1->size() == 0 || buffer2->size() == 0)) {
    const rosbag::MessageInstance &m = *it;
    sensor_msgs::ImagePtr imageMsg = m.instantiate<sensor_msgs::Image>();
    //cout << imageMsg->header.frame_id << endl;
    if (imageMsg != NULL) {
      //file << imageMsg->header.stamp.toSec() << endl;
      vector<Eigen::Vector3f> pointCloud;
      vector<int> pixelLocs;
      filter.GenerateCompletePointCloud((void*)imageMsg->data.data(),
        pointCloud, pixelLocs);
      pcl::PointCloud<pcl::PointXYZ> pcl_cloud = CloudFromVector(pointCloud, pixelLocs);
      if (imageMsg->header.frame_id == kinect_0) {
        buffer1->push_back(pcl_cloud);
        timestamps_1->push_back(imageMsg->header.stamp.toSec());
      }
      else{
        buffer2->push_back(pcl_cloud);
        timestamps_2->push_back(imageMsg->header.stamp.toSec());
      }
    }
    advance(it, 1);
  }
  return it;
}

rosbag::View::iterator time_aligned_clouds(rosbag::View::iterator it,
 rosbag::View::iterator end,
 std::deque<pcl::PointCloud<pcl::PointXYZ> >* buffer1, 
 std::deque<pcl::PointCloud<pcl::PointXYZ> >* buffer2,
 std::deque<double>* timestamps_1,
 std::deque<double>* timestamps_2,
 pcl::PointCloud<pcl::PointXYZ>* cloud1,
 pcl::PointCloud<pcl::PointXYZ>* cloud2,
 double* time1,
 double* time2) {
  // Fill the buffers
  it = get_clouds(it, buffer1, buffer2, timestamps_1, timestamps_2);
  // Get the earliest cloud from the first kinect
  pcl::PointCloud<pcl::PointXYZ> cloud_k1 =  (*buffer1)[0];
  buffer1->pop_front();
  double k1_time = (*timestamps_1)[0];
  timestamps_1->pop_front();
  // Find the closest cloud in buffer_k2
  bool fin = false;
  pcl::PointCloud<pcl::PointXYZ> cloud_k2;
  double k2_time =0;
  double best_deltaT = 100000;
  while(!fin && it != end) {
    // Fill the buffers
    it = get_clouds(it, buffer1, buffer2, timestamps_1, timestamps_2);
    for(size_t i = 0; i < buffer2->size(); i++){
      pcl::PointCloud<pcl::PointXYZ> temp_cloud_k2 =  (*buffer2)[0];
      double temp_k2_time = (*timestamps_2)[0];
      double deltaT = abs(k1_time - temp_k2_time);
      if(deltaT < best_deltaT){
        best_deltaT = deltaT;
        cloud_k2 = temp_cloud_k2;
        k2_time = temp_k2_time;
        buffer2->pop_front();
        timestamps_2->pop_front();
      }
      else {
        fin = true;
        break;
      }
    }
  }
  // return those two as the current clouds to use
  (*cloud1) = cloud_k1;
  *cloud2 = cloud_k2;
  *time1 = k1_time;
  *time2 = k2_time;
  return it;

}

void HSVtoRGB( float *r, float *g, float *b, float h, float s, float v ) {
  int i;
  float f, p, q, t;
  if( s == 0 ) {
    // achromatic (grey)
    *r = *g = *b = v;
    return;
  }
  h /= 60;      // sector 0 to 5
  i = floor( h );
  f = h - i;      // factorial part of h
  p = v * ( 1 - s );
  q = v * ( 1 - s * f );
  t = v * ( 1 - s * ( 1 - f ) );
  switch( i ) {
    case 0:
      *r = v;
      *g = t;
      *b = p;
      break;
    case 1:
      *r = q;
      *g = v;
      *b = p;
      break;
    case 2:
      *r = p;
      *g = v;
      *b = t;
      break;
    case 3:
      *r = p;
      *g = q;
      *b = v;
      break;
    case 4:
      *r = t;
      *g = p;
      *b = v;
      break;
    default:    // case 5:
      *r = v;
      *g = p;
      *b = q;
      break;
  }
}

void writeToObj(const string bagfile, 
  const int num,
  const pcl::PointCloud<pcl::PointXYZ>& cloud) {
  int range = 360 / 600;
  std::string frame_string;
  std::stringstream out;
  string temp = bagfile;
  mkdir(temp.c_str(), 0777);
  out << num;
  string filename = temp + "_" + out.str() + ".obj";
  ofstream file(filename.c_str());
  float r;
  float g;
  float b;
  float h = num * range;
  float s = 0.5;
  float v = 0.5;
  HSVtoRGB(&r, &g, &b, h, s, v);
  for(size_t i = 0; i < cloud.size(); i++){
    pcl::PointXYZ point = cloud[i];
    file << "v " << point.x << " " << point.y << " " << point.z << " " << r << " " << g << " " << b << endl;
  }
  file.close();
}

pcl::PointCloud<pcl::PointXYZ> SaveObjects(
  string bagfile,
  vector<pcl::PointCloud<pcl::PointXYZ> > point_clouds,
  vector<double*> calculated_poses) {

  pcl::PointCloud<pcl::PointXYZ> merged_clouds;
  merged_clouds = point_clouds[0];
  cout << calculated_poses.size() << endl;
  cout << point_clouds.size() << endl;
  // Transform each cloud by the calculated transformation and add it to the
  // base cloud
  for (size_t i = 0; i < calculated_poses.size(); ++i) {
    cout << "writing to object" <<endl;
    TransformPointCloud_PCL(point_clouds[i], calculated_poses[i]);
    writeToObj(bagfile, i, point_clouds[i]);
    //merged_clouds += point_clouds[i];
  }
  return merged_clouds;
}

void printPose(double* pose){
  for(uint i = 0; i < 6; i ++) {
    cout << pose[i] << " ";
  }
  cout << endl;
}

void CheckTransform(const string& bagfile, const string& transform_file) {
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
  cout << endl;

  // Bag file for clouds
  cout << "Opening Bag" << endl;
  rosbag::Bag bag;
  string out_name = bagfile + ".aligned";
  string bag_name = bagfile + ".bag";

  bag.open(bag_name, rosbag::bagmode::Read);
  std::vector<std::string> topics;
  topics.push_back(std::string("/Cobot/Kinect/Depth"));
  rosbag::View view(bag, rosbag::TopicQuery(topics));
  // Iterator over bag file
  rosbag::View::iterator bag_it = view.begin();
  rosbag::View::iterator end = view.end();

  // Buffers of clouds from both kinects to handle reading from a single file
  std::deque<pcl::PointCloud<pcl::PointXYZ> > buffer_k1;
  std::deque<pcl::PointCloud<pcl::PointXYZ> > buffer_k2;
  std::deque<double> times_k1;
  std::deque<double> times_k2;

  // Read in the first cloud from each kinect, set it as the keyframe
  // and cloud k - 1, set the combined transform to the zero transform
  cout << "Getting Initial Clouds" << endl;
  // Keyframes
  cout << "Retrieving Clouds From dequeu" << endl;
  cout << buffer_k1.size() << endl;
  cout << buffer_k2.size() << endl;
  pcl::PointCloud<pcl::PointXYZ> keyframe_k1;
  pcl::PointCloud<pcl::PointXYZ> keyframe_k2;
  double timestamp_1;
  double timestamp_2;

  bag_it = time_aligned_clouds(bag_it, end,  &buffer_k1, &buffer_k2, &times_k1, &times_k2, 
    &keyframe_k1, &keyframe_k2, &timestamp_1, &timestamp_2);

   // While there are still clouds in both datasets
  int count = 0;
  cout << "Starting Loop" << endl; 
  while(((buffer_k1.size() != 0 && buffer_k2.size() != 0 )|| (bag_it != view.end()))) {  // Wrap the multiple cloud work inside this into a loop over the number of kinects
    count += 1;
    cout << count << endl;
    // Read in a new cloud from each dataset
    pcl::PointCloud<pcl::PointXYZ> cloud_k1;
    pcl::PointCloud<pcl::PointXYZ> cloud_k2;

    bag_it = time_aligned_clouds(bag_it, end, &buffer_k1, &buffer_k2, &times_k1, &times_k2, 
    &cloud_k1, &cloud_k2, &timestamp_1, &timestamp_2);

    pcl::PointCloud<pcl::PointXYZ> transformed_cloud = cloud_k2;
    TransformPointCloud_PCL(transformed_cloud, transform);
    publish_cloud(cloud_k1, cloud_pub);
    publish_cloud(transformed_cloud, cloud_pub_2);
    publish_cloud(cloud_k2, cloud_pub_3);
    pcl::PointCloud<pcl::PointXYZ> combo_cloud = cloud_k1;
    combo_cloud += transformed_cloud;
    writeToObj(out_name, count, combo_cloud);
  }
}

void k1Callback(const sensor_msgs::ImagePtr& imageMsg) {
  double  rotation[] = {1.74, 0, 0, 0, 0, 0};
  PlaneFilter filter;
  KinectRawDepthCam camera = KinectRawDepthCam();
  filter.setDepthCamera(&camera);
  vector<Eigen::Vector3f> pointCloud;
  vector<int> pixelLocs;
  filter.GenerateCompletePointCloud((void*)imageMsg->data.data(),
    pointCloud, pixelLocs);
  pcl::PointCloud<pcl::PointXYZ> pcl_cloud = CloudFromVector(pointCloud, pixelLocs);
  TransformPointCloud_PCL(pcl_cloud, rotation);
  publish_cloud(pcl_cloud, cloud_pub);
}

void k0Callback(const sensor_msgs::ImagePtr& imageMsg) {
  cout << "Callback" << endl;
  PlaneFilter filter;
  KinectRawDepthCam camera = KinectRawDepthCam();
  filter.setDepthCamera(&camera);
  double  transform[] = {2.6976, 0.0656, -1.6419, 0.0266, 0.0175, 0.0431}; // Manually inserted transform (calculated by delta-cal)
  double  rotation[] = {1.74, 0, 0, 0, 0, 0};
  vector<Eigen::Vector3f> pointCloud;
  vector<int> pixelLocs;
  filter.GenerateCompletePointCloud((void*)imageMsg->data.data(),
    pointCloud, pixelLocs);
  pcl::PointCloud<pcl::PointXYZ> pcl_cloud = CloudFromVector(pointCloud, pixelLocs);
  publish_cloud(pcl_cloud, cloud_pub_2);
  TransformPointCloud_PCL(pcl_cloud, transform);
  TransformPointCloud_PCL(pcl_cloud, rotation);
  publish_cloud(pcl_cloud, cloud_pub_3);
}

int main(int argc, char **argv) {
  signal(SIGINT,HandleStop);
  signal(SIGALRM,HandleStop);

  int kMaxClouds = 10;
  int mode = 0;
  char* bagFile = (char*)"pair_upright.bag";
  char* transform_file = (char*)"";
  // Parse arguments.
  static struct poptOption options[] = {
    { "kMaxClouds" , 'k', POPT_ARG_INT , &kMaxClouds ,0, "Max Clouds" , "NUM" },
    { "Mode" , 'm', POPT_ARG_INT , &mode ,0, "Run Mode" , "NUM" },
    { "use-bag-file" , 'B', POPT_ARG_STRING, &bagFile ,0, "Process bag file" ,
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
  cout << "Using bagfile: " << bagFile << endl;
  cout << "Mode: " << mode << endl;
  cout << "Using transform file" << transform_file << endl;
  // Initialize Ros
  ros::init(argc, argv, "pose_estimation", ros::init_options::NoSigintHandler);
  ros::NodeHandle n;
  //----------  Setup Visualization ----------
  cloud_pub = n.advertise<sensor_msgs::PointCloud2> ("cloud_pub_1", 1);
  cloud_pub_2 = n.advertise<sensor_msgs::PointCloud2> ("cloud_pub_2", 1);
  cloud_pub_3 = n.advertise<sensor_msgs::PointCloud2> ("cloud_pub_3", 1);
  cloud_pub_4 = n.advertise<sensor_msgs::PointCloud2> ("cloud_pub_4", 1);
  cloud_test_pub = n.advertise<sensor_msgs::PointCloud2> ("cloud_pub_5", 1);
  model_cloud_pub = n.advertise<sensor_msgs::PointCloud2> ("cloud_pub_6", 1);
  bmodel_cloud_pub = n.advertise<sensor_msgs::PointCloud2> ("cloud_pub_7", 1);
  marker_pub =
  n.advertise<visualization_msgs::Marker>("visualization_marker", 10);
  markerArray_pub =
  n.advertise<visualization_msgs::MarkerArray>("visualization_marker_array", 10);

  ros::Subscriber sub_k0 = n.subscribe("Cobot/Kinect/Depth0", 10, k0Callback);
  ros::Subscriber sub_k1 = n.subscribe("Cobot/Kinect/Depth1", 10, k1Callback);
  /**
   * ros::spin() will enter a loop, pumping callbacks.  With this version, all
   * callbacks will be called from within this thread (the main one).  ros::spin()
   * will exit when Ctrl-C is pressed, or the node is shutdown by the master.
   */
  ros::spin();
  // else if(mode == 1) {
  //   CheckTransform(bagFile, transform_file);
  // }
  return 0;
}