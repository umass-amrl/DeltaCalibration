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
#include <pcl/filters/extract_indices.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
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
#include "kinect/plane_filtering.h"
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
  const pcl::PointCloud<pcl::PointXYZRGB>& cloud, ros::Publisher publisher) {
  sensor_msgs::PointCloud2 temp_cloud;
  pcl::PCLPointCloud2 pcl_cloud;
  pcl::toPCLPointCloud2(cloud, pcl_cloud);
  pcl_conversions::fromPCL(pcl_cloud, temp_cloud);
  temp_cloud.header.frame_id = "point_cloud";

  publisher.publish(temp_cloud);
}

// Transforms all points in a pcl point cloud (array input)
void TransformPointCloud_PCL(
  pcl::PointCloud<pcl::PointXYZRGB>& cloud,
  double transform[6]){

  for(size_t i = 0; i < cloud.size(); ++i){
    pcl::PointXYZRGB point = cloud[i];

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
  pcl::PointCloud<pcl::PointXYZRGB>& cloud,
  const vector<double>& transform){

  double transform_array[6];
  std::copy(transform.begin(), transform.end(), transform_array);
  TransformPointCloud_PCL(cloud, transform_array);
}

pcl::PointCloud<pcl::PointXYZRGB> CloudFromVector(
  vector<Eigen::Vector3f> pointCloud, vector<int> pixelLocs){

  pcl::PointCloud<pcl::PointXYZRGB> cloud;
  //Process Depth To PCL Cloud
  cloud.resize(pointCloud.size());
  //cloud.height = 480;
  //cloud.width = 640;

  for (uint i = 0; i < cloud.size(); ++i) {

        pcl::PointXYZRGB point;
        point.x = -pointCloud[i](0);
        point.y = -pointCloud[i](1);
        point.z=  -pointCloud[i](2);
        cloud[i] = point;
    }
    return cloud;
}

pcl::PointCloud<pcl::PointXYZRGB> swapAxis(pcl::PointCloud<pcl::PointXYZRGB> pointCloud){

  pcl::PointCloud<pcl::PointXYZRGB> cloud;
  //Process Depth To PCL Cloud
  cloud.resize(pointCloud.size());
  //cloud.height = 480;
  //cloud.width = 640;

  for (uint i = 0; i < cloud.size(); ++i) {

        pcl::PointXYZRGB point;
        point.y = -pointCloud[i].x;
        point.x = pointCloud[i].z;
        point.z = -pointCloud[i].y;
        point.r = pointCloud[i].r;
        point.g = pointCloud[i].g;
        point.b=  pointCloud[i].b;
        cloud[i] = point;
    }
    return cloud;
}

// Reads clouds from a given iterator, saves to buffer if they are over
rosbag::View::iterator get_clouds(rosbag::View::iterator it,
 std::deque<pcl::PointCloud<pcl::PointXYZRGB> >* buffer1,
 std::deque<pcl::PointCloud<pcl::PointXYZRGB> >* buffer2,
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
      pcl::PointCloud<pcl::PointXYZRGB> pcl_cloud = CloudFromVector(pointCloud, pixelLocs);
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

rosbag::View::iterator get_clouds2(rosbag::View::iterator it,
 std::deque<pcl::PointCloud<pcl::PointXYZRGB> >* buffer1,
 std::deque<double>* timestamps_1) {
  while(buffer1->size() == 0){
    cout << "instance" << endl;

     const rosbag::MessageInstance &m = *it;
     cout << "instantiate" << endl;
    sensor_msgs::PointCloud2 imageMsg= *(m.instantiate<sensor_msgs::PointCloud2>());
    //cout << imageMsg->header.frame_id << endl;
    cout << "msg to pcl" << endl;
    pcl::PCLPointCloud2 pcl_pc2;
    pcl_conversions::toPCL(imageMsg,pcl_pc2);
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr temp_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
    cout << "pcl to pcl" << endl;
    pcl::fromPCLPointCloud2(pcl_pc2,*temp_cloud);
    buffer1->push_back(swapAxis(*temp_cloud));
    //publish_cloud(*temp_cloud, cloud_pub);
    timestamps_1->push_back(imageMsg.header.stamp.toSec());
  advance(it, 1);
  }
  return it;
}

rosbag::View::iterator time_aligned_clouds2(rosbag::View::iterator it,
  rosbag::View::iterator it2,
 rosbag::View::iterator end,
 rosbag::View::iterator end2,
 std::deque<pcl::PointCloud<pcl::PointXYZRGB> >* buffer1,
 std::deque<pcl::PointCloud<pcl::PointXYZRGB> >* buffer2,
 std::deque<double>* timestamps_1,
 std::deque<double>* timestamps_2,
 pcl::PointCloud<pcl::PointXYZRGB>* cloud1,
 pcl::PointCloud<pcl::PointXYZRGB>* cloud2,
 double* time1,
 double* time2) {
  // Fill the buffers
  cout << "fill the buffers" << endl;

it2 = get_clouds2(it2, buffer2, timestamps_2);

  cout << "fill the buffers2" << endl;
     it = get_clouds2(it, buffer1, timestamps_1);
  // Get the earliest cloud from the first kinect
  pcl::PointCloud<pcl::PointXYZRGB> cloud_k1 =  (*buffer1)[0];
  buffer1->pop_front();
  double k1_time = (*timestamps_1)[0];
  timestamps_1->pop_front();
  // Find the closest cloud in buffer_k2
  pcl::PointCloud<pcl::PointXYZRGB> cloud_k2 =  (*buffer2)[0];
  buffer2->pop_front();
  double k2_time = (*timestamps_2)[0];
  timestamps_2->pop_front();
  // while(!fin && it != end & it2 != end) {
  //   // Fill the buffers
  //   cout << "fill the buffers" << endl;
  //   it = get_clouds2(it, buffer1, timestamps_1);
  //   it2 = get_clouds2(it2, buffer2, timestamps_2);
  //   for(size_t i = 0; i < buffer2->size(); i++){
  //     pcl::PointCloud<pcl::PointXYZRGB> temp_cloud_k2 =  (*buffer2)[0];
  //     double temp_k2_time = (*timestamps_2)[0];
  //     double deltaT = abs(k1_time - temp_k2_time);
  //     if(deltaT < best_deltaT){
  //       best_deltaT = deltaT;
  //       cloud_k2 = temp_cloud_k2;
  //       k2_time = temp_k2_time;
  //       buffer2->pop_front();
  //       timestamps_2->pop_front();
  //     }
  //     else {
  //       fin = true;
  //       break;
  //     }
  //   }
  // }
  // return those two as the current clouds to use
  (*cloud1) = cloud_k1;
  *cloud2 = cloud_k2;
  *time1 = k1_time;
  *time2 = k2_time;
  return it;

}

rosbag::View::iterator time_aligned_clouds3(rosbag::View::iterator it,
                                            rosbag::View::iterator it2,
                                            rosbag::View::iterator end,
                                            rosbag::View::iterator end2,
                                            std::deque<pcl::PointCloud<pcl::PointXYZRGB> >* buffer1,
                                            std::deque<pcl::PointCloud<pcl::PointXYZRGB> >* buffer2,
                                            std::deque<double>* timestamps_1,
                                            std::deque<double>* timestamps_2,
                                            pcl::PointCloud<pcl::PointXYZRGB>* cloud1,
                                            pcl::PointCloud<pcl::PointXYZRGB>* cloud2,
                                            double* time1,
                                            double* time2) {
  // Fill the buffers
  cout << "fill the buffers" << endl;
  
  it2 = get_clouds2(it2, buffer2, timestamps_2);
  
  cout << "fill the buffers2" << endl;
//   it = get_clouds2(it, buffer1, timestamps_1);
  // Get the earliest cloud from the first kinect
//   pcl::PointCloud<pcl::PointXYZRGB> cloud_k1 =  (*buffer1)[0];
//   buffer1->pop_front();
//   double k1_time = (*timestamps_1)[0];
//   timestamps_1->pop_front();
  // Find the closest cloud in buffer_k2
  pcl::PointCloud<pcl::PointXYZRGB> cloud_k2 =  (*buffer2)[0];
  buffer2->pop_front();
  double k2_time = (*timestamps_2)[0];
  timestamps_2->pop_front();
  // while(!fin && it != end & it2 != end) {
  //   // Fill the buffers
  //   cout << "fill the buffers" << endl;
  //   it = get_clouds2(it, buffer1, timestamps_1);
  //   it2 = get_clouds2(it2, buffer2, timestamps_2);
  //   for(size_t i = 0; i < buffer2->size(); i++){
  //     pcl::PointCloud<pcl::PointXYZRGB> temp_cloud_k2 =  (*buffer2)[0];
  //     double temp_k2_time = (*timestamps_2)[0];
  //     double deltaT = abs(k1_time - temp_k2_time);
  //     if(deltaT < best_deltaT){
  //       best_deltaT = deltaT;
  //       cloud_k2 = temp_cloud_k2;
  //       k2_time = temp_k2_time;
  //       buffer2->pop_front();
  //       timestamps_2->pop_front();
  //     }
  //     else {
  //       fin = true;
  //       break;
  //     }
  //   }
  // }
  // return those two as the current clouds to use
  *cloud2 = cloud_k2;
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
  const pcl::PointCloud<pcl::PointXYZRGB>& cloud) {
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
    pcl::PointXYZRGB point = cloud[i];
    file << "v " << point.x << " " << point.y << " " << point.z << " " << point.r << " " << point.g << " " << point.b << endl;
  }
  file.close();
}

pcl::PointCloud<pcl::PointXYZRGB> SaveObjects(
  string bagfile,
  vector<pcl::PointCloud<pcl::PointXYZRGB> > point_clouds,
  vector<double*> calculated_poses) {

  pcl::PointCloud<pcl::PointXYZRGB> merged_clouds;
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

pcl::PointCloud<pcl::Normal> getNormals(const pcl::PointCloud<pcl::PointXYZRGB>& cloud) {

  // Get the cloud
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr ptr_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
  ptr_cloud = cloud.makeShared();
  // Create the normal estimation class, and pass the input dataset to it
  pcl::NormalEstimationOMP<pcl::PointXYZRGB, pcl::Normal> ne;
  ne.setInputCloud (ptr_cloud);
  // Create an empty kdtree representation, and pass it to the normal estimation object.
  // Its content will be filled inside the object, based on the given input dataset (as no other search surface is given).
  pcl::search::KdTree<pcl::PointXYZRGB>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZRGB> ());
  ne.setSearchMethod (tree);
  // Output datasets
  pcl::PointCloud<pcl::Normal>::Ptr cloud_normals (new pcl::PointCloud<pcl::Normal>);
  // Use all neighbors in a sphere of radius 3cm
  ne.setRadiusSearch (0.03);

  // Compute the features
  ne.compute (*cloud_normals);

  return *cloud_normals;
}

vector<pcl::PointCloud<pcl::PointXYZRGB> > getPlanes(pcl::PointCloud<pcl::PointXYZRGB> cloud){

  vector<pcl::PointCloud<pcl::PointXYZRGB> > output;
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_filtered (new pcl::PointCloud<pcl::PointXYZRGB>), cloud_p (new pcl::PointCloud<pcl::PointXYZRGB>), cloud_f (new pcl::PointCloud<pcl::PointXYZRGB>);


  // // Create the filtering object: downsample the dataset using a leaf size of 1cm
  // pcl::VoxelGrid<pcl::PCLPointCloud2> sor;
  // sor.setInputCloud (cloud_blob);
  // sor.setLeafSize (0.01f, 0.01f, 0.01f);
  // sor.filter (*cloud_filtered_blob);

  // Convert to the templated PointCloud
  cloud_filtered = cloud.makeShared();

  pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients ());
  pcl::PointIndices::Ptr inliers (new pcl::PointIndices ());
  // Create the segmentation object
  pcl::SACSegmentation<pcl::PointXYZRGB> seg;
  // Optional
  seg.setOptimizeCoefficients (true);
  // Mandatory
  seg.setModelType (pcl::SACMODEL_PLANE);
  seg.setMethodType (pcl::SAC_RANSAC);
  seg.setMaxIterations (1000);
  seg.setDistanceThreshold (0.01);

  // Create the filtering object
  pcl::ExtractIndices<pcl::PointXYZRGB> extract;

  int i = 0;
  // int nr_points = (int) cloud_filtered->points.size ();
  // While 30% of the original cloud is still there
  int num_planes = 0;
  while (num_planes < 2)
  {
    num_planes +=1;
    // Segment the largest planar component from the remaining cloud
    seg.setInputCloud (cloud_filtered);
    seg.segment (*inliers, *coefficients);
    if (inliers->indices.size () == 0)
    {
      std::cerr << "Could not estimate a planar model for the given dataset." << std::endl;
      break;
    }

    // Extract the inliers
    extract.setInputCloud (cloud_filtered);
    extract.setIndices (inliers);
    extract.setNegative (false);
    extract.filter (*cloud_p);
    std::cerr << "PointCloud representing the planar component: " << cloud_p->width * cloud_p->height << " data points." << std::endl;

    // Create the filtering object
    extract.setNegative (true);
    extract.filter (*cloud_f);
    cloud_filtered.swap (cloud_f);
    i++;
    publish_cloud(*cloud_p, cloud_test_pub);
    publish_cloud(*cloud_f, model_cloud_pub);
    publish_cloud(*cloud_filtered, bmodel_cloud_pub);
    output.push_back(*cloud_p);
  }
return output;
}

void ComparePlanes(vector<pcl::PointCloud<pcl::PointXYZRGB> > k1_planes,
  vector<pcl::PointCloud<pcl::PointXYZRGB> > transformed_planes) {

  for(size_t i = 0; i < k1_planes.size(); ++i) {
    pcl::PointCloud<pcl::PointXYZRGB> plane1 = k1_planes[i];
    pcl::PointCloud<pcl::PointXYZRGB> plane2 = transformed_planes[i];
    pcl::PointCloud<pcl::Normal> normals1 = getNormals(plane1);
    pcl::PointCloud<pcl::Normal> normals2 = getNormals(plane2);

    double x1 = 0;
    double y1 = 0;
    double z1 = 0;
    double x2 = 0;
    double y2 = 0;
    double z2 = 0;
    double px1 = 0;
    double py1 = 0;
    double pz1 = 0;
    double px2 = 0;
    double py2 = 0;
    double pz2 = 0;
    pcl::Normal average2;
    for(size_t j = 0; j < normals1.size(); ++j) {
      if(!isnan(normals1[j].normal_x) &&
          !isnan(normals1[j].normal_y) &&
          !isnan(normals1[j].normal_z)){

        x1 += normals1[j].normal_x;
        y1 += normals1[j].normal_y;
        z1 += normals1[j].normal_z;
        px1 += plane1[j].x;
        py1 += plane1[j].y;
        pz1 += plane1[j].z;
      }
    }
    for(size_t j = 0; j < normals2.size(); ++j) {
      if(!isnan(normals2[j].normal_x) &&
          !isnan(normals2[j].normal_y) &&
          !isnan(normals2[j].normal_z)){
        x2 += normals2[j].normal_x;
        y2 += normals2[j].normal_y;
        z2 += normals2[j].normal_z;
        px2 += plane2[j].x;
        py2 += plane2[j].y;
        pz2 += plane2[j].z;
      }
    }
    x1 = x1 / normals1.size();
    y1 = y1 / normals1.size();
    z1 = z1 / normals1.size();
    x2 = x2 / normals2.size();
    y2 = y2 / normals2.size();
    z2 = z2 / normals2.size();
    px1 = px1 / plane1.size();
    py1 = py1 / plane1.size();
    pz1 = pz1 / plane1.size();
    px2 = px2 / plane2.size();
    py2 = py2 / plane2.size();
    pz2 = pz2 / plane2.size();
    Eigen::Matrix<double,3,1> p2_mat;
    Eigen::Matrix<double,3,1> p1_mat;
    Eigen::Matrix<double,3,1> norm_k;
    Eigen::Matrix<double,3,1> norm_k1;
    norm_k[0] = x1 / normals1.size();
    norm_k[1] = y1 / normals1.size();
    norm_k[2] = z1 / normals1.size();
    norm_k1[0] = x2 / normals2.size();
    norm_k1[1] = y2 / normals2.size();
    norm_k1[2] = z2 / normals2.size();
    p1_mat[0] = px1 / plane1.size();
    p1_mat[1] = py1 / plane1.size();
    p1_mat[2] = pz1 / plane1.size();
    p2_mat[0] = px2 / plane2.size();
    p2_mat[1] = py2 / plane2.size();
    p2_mat[2] = pz2 / plane2.size();
    cout << "Offset Residual 1: " << (p1_mat - p2_mat).dot(norm_k) << endl;
    cout << "Offset Residual 2: " << (p2_mat - p1_mat).dot(norm_k1) << endl;

    // Could possibly just do this with eigen, but I don't know if the acos function exists
    double dot = x1*x2 + y1*y2 + z1*z2;
    double lenSq1 = x1*x1 + y1*y1 + z1*z1;
    double lenSq2 = x2*x2 + y2*y2 + z2*z2;
    cout << lenSq1 << endl;
    cout << lenSq2 << endl;
    cout << dot << endl;
    double angle = acos(dot/sqrt(lenSq1 * lenSq2));
    cout << "Angle between plane normals " << i << " : " << angle << endl;
  }
}

void CheckTransform(const string& bagfile, const string& bagfile2, const string& transform_file, const string& transform_file2) {
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
  std::ifstream infile2(transform_file2.c_str());
  getline(infile2, transform_string);
  double* transform2 = new double[6];
  istringstream ss2(transform_string);
  i = 0;
  while(!ss2.eof()) {
    string x;
    getline( ss2, x, ' ' );
    transform2[i] = atof(x.c_str());
    cout << transform2[i] << "\t";
    ++i;
  }
  cout << endl;

  // Bag file for clouds
  cout << "Opening Bag" << endl;
  rosbag::Bag bag;
  rosbag::Bag bag2;
  rosbag::Bag bag3;
  string out_name = bagfile + ".aligned";
  string out_name2 = bagfile + ".aligned2";
  string out_name3 = bagfile + ".aligned3";
  string bag_name = bagfile + ".bag";
  string bag_name2 = bagfile2 + ".bag";

  bag.open(bag_name, rosbag::bagmode::Read);
  std::vector<std::string> topics;
  topics.push_back(std::string("/kinect_1/depth_registered/points"));
  rosbag::View view(bag, rosbag::TopicQuery(topics));
  // Iterator over bag file
  rosbag::View::iterator bag_it = view.begin();
  rosbag::View::iterator end = view.end();

  bag2.open(bag_name2, rosbag::bagmode::Read);
  std::vector<std::string> topics2;
  topics2.push_back(std::string("/kinect_2/depth_registered/points"));
  rosbag::View view2(bag2, rosbag::TopicQuery(topics2));
  // Iterator over bag file
  rosbag::View::iterator bag_it2 = view2.begin();
  rosbag::View::iterator end2 = view2.end();
  

  bag3.open(bag_name2, rosbag::bagmode::Read);
  std::vector<std::string> topics3;
  topics3.push_back(std::string("/kinect_3/depth_registered/points"));
  rosbag::View view3(bag3, rosbag::TopicQuery(topics3));
  
  // Iterator over bag file
  rosbag::View::iterator bag_it3 = view3.begin();
  rosbag::View::iterator end3 = view3.end();
  // Buffers of clouds from both kinects to handle reading from a single file
  std::deque<pcl::PointCloud<pcl::PointXYZRGB> > buffer_k1;
  std::deque<pcl::PointCloud<pcl::PointXYZRGB> > buffer_k2;
  std::deque<pcl::PointCloud<pcl::PointXYZRGB> > buffer_k3;
  std::deque<double> times_k1;
  std::deque<double> times_k2;
  std::deque<double> times_k3;

  // Read in the first cloud from each kinect, set it as the keyframe
  // and cloud k - 1, set the combined transform to the zero transform
  cout << "Getting Initial Clouds" << endl;
  // Keyframes
  cout << "Retrieving Clouds From dequeu" << endl;
  cout << buffer_k1.size() << endl;
  cout << buffer_k2.size() << endl;
  pcl::PointCloud<pcl::PointXYZRGB> keyframe_k1;
  pcl::PointCloud<pcl::PointXYZRGB> keyframe_k2;
  double timestamp_1;
  double timestamp_2;
  double timestamp_3;

   // While there are still clouds in both datasets
  int count = 0;
  cout << "Starting Loop" << endl;
  while(((buffer_k1.size() != 0 && buffer_k2.size() != 0 )|| (bag_it != view.end()))) {  // Wrap the multiple cloud work inside this into a loop over the number of kinects
    count += 1;
    cout << count << endl;
    // Read in a new cloud from each dataset
    pcl::PointCloud<pcl::PointXYZRGB> cloud_k1;
    pcl::PointCloud<pcl::PointXYZRGB> cloud_k2;
    pcl::PointCloud<pcl::PointXYZRGB> cloud_k3;

    bag_it = time_aligned_clouds2(bag_it, bag_it2, end, end2,  &buffer_k1, &buffer_k2, &times_k1, &times_k2,
     &cloud_k1, &cloud_k2, &timestamp_1, &timestamp_2);
    cout << "first clouds retrieved" << endl;
    bag_it3 = time_aligned_clouds3(bag_it, bag_it3, end, end3,  &buffer_k1, &buffer_k3, &times_k1, &times_k3,
                                  &cloud_k1, &cloud_k3, &timestamp_1, &timestamp_3);
    pcl::PointCloud<pcl::PointXYZRGB> transformed_cloud = cloud_k3;
    //writeToObj(out_name2, count, cloud_k1);
    //writeToObj(out_name3, count, cloud_k2);
    cout << "Cloud 1 planes" << endl;
    TransformPointCloud_PCL(transformed_cloud, transform);
    pcl::PointCloud<pcl::PointXYZRGB> combo_cloud = cloud_k2;
    combo_cloud += transformed_cloud;
    transformed_cloud = combo_cloud;
    TransformPointCloud_PCL(transformed_cloud, transform2);
    combo_cloud = cloud_k1;
    combo_cloud += transformed_cloud;
    cout << "transformed planes" << endl;
    while(true) {
//     publish_cloud(cloud_k2, cloud_pub);
//     publish_cloud(transformed_cloud, cloud_pub_4);
//     publish_cloud(cloud_k1, cloud_pub_2);
//     
//     pcl::PointCloud<pcl::PointXYZRGB> combo_cloud = cloud_k1;
//     double  rotation[] = {3.14, 0, 0, 0, 0, 0};
//     combo_cloud += transformed_cloud;
//     TransformPointCloud_PCL(combo_cloud, rotation);
    publish_cloud(combo_cloud, model_cloud_pub);

    writeToObj(out_name, count, combo_cloud);
    }
  }
}

int main(int argc, char **argv) {
  signal(SIGINT,HandleStop);
  signal(SIGALRM,HandleStop);

  int kMaxClouds = 10;
  int mode = 0;
  char* bagFile = (char*)"pair_upright.bag";
  char* bagFile2 = (char*)"pair_upright.bag";
  char* transform_file = (char*)"";
  char* transform_file2 = (char*)"";
  // Parse arguments.
  static struct poptOption options[] = {
    { "kMaxClouds" , 'k', POPT_ARG_INT , &kMaxClouds ,0, "Max Clouds" , "NUM" },
    { "Mode" , 'm', POPT_ARG_INT , &mode ,0, "Run Mode" , "NUM" },
    { "use-bag-file" , 'B', POPT_ARG_STRING, &bagFile ,0, "Process bag file" ,
        "STR" },
    { "use-bag-file" , 'b', POPT_ARG_STRING, &bagFile2 ,0, "Process bag file" ,
    "STR" },
    { "transform-file" , 'T', POPT_ARG_STRING, &transform_file ,0, "Transform File" ,
        "STR" },
        { "transform-file" , 't', POPT_ARG_STRING, &transform_file2 ,0, "Transform File" ,
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
  cout << "Using bagfile: " << bagFile2 << endl;
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
    CheckTransform(bagFile, bagFile2, transform_file, transform_file2);
  }
  // else if(mode == 1) {
  //   CheckTransform(bagFile, transform_file);
  // }
  return 0;
}
