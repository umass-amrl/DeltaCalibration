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
#include <pcl/io/pcd_io.h>
#include "pcl/point_types.h"
#include "pcl/impl/instantiate.hpp"
#include <pcl/filters/voxel_grid.h>
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

//   Eigen::Transform<double, 3, Eigen::Affine> rotationI = rotation.inverse();
  Eigen::Translation<double, 3> translation =
  Eigen::Translation<double, 3>(transform[3], transform[4], transform[5]);

  Eigen::Transform<double, 3, Eigen::Affine> affine_transform =
   translation * rotation;

  Eigen::Matrix<double,3,1> transformed_point;
  transformed_point = point;
//   Eigen::Matrix<double,3, 1> translate = Eigen::Matrix<double,3, 1> (transform[3], transform[4], transform[5]);

  return affine_transform * transformed_point;
}

void GeometryTransform(geometry_msgs::Point* point, const double transform[6]) {
  Eigen::Matrix<double,3,1> point_eig;
  point_eig[0] = point->x;
  point_eig[1] = point->y;
  point_eig[2] = point->z;
  point_eig = TransformPoint(point_eig, transform);
  point->x = point_eig[0];
  point->y = point_eig[1];
  point->z = point_eig[2];
}

// Transforms a given eigen point by the given transform (array input)
template <class T> Eigen::Matrix<T,3,1> TransformPointInv(
    const Eigen::Matrix<T,3,1>& point,
    const T transform[6]) {

  //Create the eigen transform from the camera
  Eigen::Matrix<T,3,1> axis(transform[0], transform[1], transform[2]);
  const T angle = axis.norm();
  if (angle > T(0)) {
    axis = axis / angle;
  }

  const Eigen::Transform<T, 3, Eigen::Affine> rotation =
      Eigen::Transform<T, 3, Eigen::Affine>(
          Eigen::AngleAxis<T>(angle, axis));

  const Eigen::Translation<T, 3> translation =
      Eigen::Translation<T, 3>(transform[3], transform[4], transform[5]);

  const Eigen::Transform<T, 3, Eigen::Affine> affine_transform =
      translation * rotation;

  const Eigen::Matrix<T,3,1> transformed_point = affine_transform.inverse() * point;
  return transformed_point;
}

// Transforms a given eigen vector (not point!) by the given transform (array
// input)
template <class T> Eigen::Matrix<T,3,1> TransformVector(
    const Eigen::Matrix<T,3,1>& vector,
    const T transform[6]) {
  T vector_t[] = {T(vector.x()), T(vector.y()), T(vector.z())};
  T transformed_vector[3];
  ceres::AngleAxisRotatePoint(transform, vector_t, transformed_vector);
  return (Eigen::Matrix<T, 3, 1>(
              transformed_vector[0],
              transformed_vector[1],
              transformed_vector[2]));
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
        point.y = pointCloud[i](1);
        point.z=  pointCloud[i](2);
        cloud[i] = point;
    }
    return cloud;
}

void TransformPointCloudInv(
    pcl::PointCloud<pcl::PointXYZ>& cloud,
    double transform[6]) {
  for(size_t i = 0; i < cloud.size(); ++i) {
    pcl::PointXYZ point = cloud[i];

    Eigen::Matrix<double,3,1> point_matrix;
    Eigen::Matrix<double,3,1> transformed_point_matrix;
    point_matrix << point.x, point.y, point.z;
    transformed_point_matrix = TransformPointInv(point_matrix, transform);
    point.x = transformed_point_matrix[0];
    point.y = transformed_point_matrix[1];
    point.z = transformed_point_matrix[2];
    cloud[i] = point;
  }
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
  //mkdir(temp.c_str(), 0777);
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

pcl::PointCloud<pcl::Normal> getNormals(const pcl::PointCloud<pcl::PointXYZ>& cloud) {

  // Get the cloud
  pcl::PointCloud<pcl::PointXYZ>::Ptr ptr_cloud(new pcl::PointCloud<pcl::PointXYZ>);
  ptr_cloud = cloud.makeShared();
  // Create the normal estimation class, and pass the input dataset to it
  pcl::NormalEstimationOMP<pcl::PointXYZ, pcl::Normal> ne;
  ne.setInputCloud (ptr_cloud);
  // Create an empty kdtree representation, and pass it to the normal estimation object.
  // Its content will be filled inside the object, based on the given input dataset (as no other search surface is given).
  pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ> ());
  ne.setSearchMethod (tree);
  // Output datasets
  pcl::PointCloud<pcl::Normal>::Ptr cloud_normals (new pcl::PointCloud<pcl::Normal>);
  // Use all neighbors in a sphere of radius 3cm
  ne.setRadiusSearch (0.03);

  // Compute the features
  ne.compute (*cloud_normals);

  return *cloud_normals;
}

vector<pcl::PointCloud<pcl::PointXYZ> > getPlanes(
    pcl::PointCloud<pcl::PointXYZ> cloud,
    vector<Eigen::Vector4d>* normal_equations,
    vector<Eigen::Vector3d>* centroids
                                                 ) {

  vector<pcl::PointCloud<pcl::PointXYZ> > output;
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered (new pcl::PointCloud<pcl::PointXYZ>),
  cloud_p (new pcl::PointCloud<pcl::PointXYZ>),
  cloud_f (new pcl::PointCloud<pcl::PointXYZ>);


  // // Create the filtering object: downsample the dataset using a leaf size of 1cm
  // pcl::VoxelGrid<pcl::PCLPointCloud2> sor;
  // sor.setInputCloud (cloud_blob);
  // sor.setLeafSize (0.01f, 0.01f, 0.01f);
  // sor.filter (*cloud_filtered_blob);

  // Convert to the templated PointCloud
  cloud_filtered = cloud.makeShared();
  vector<Eigen::Vector4d> equations;
  pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients ());
  pcl::PointIndices::Ptr inliers (new pcl::PointIndices ());
  // Create the segmentation object
  pcl::SACSegmentation<pcl::PointXYZ> seg;
  // Optional
  seg.setOptimizeCoefficients (true);
  // Mandatory
  seg.setModelType (pcl::SACMODEL_PLANE);
  seg.setMethodType (pcl::SAC_RANSAC);
  seg.setMaxIterations (1000);
  seg.setDistanceThreshold (0.01);

  // Create the filtering object
  pcl::ExtractIndices<pcl::PointXYZ> extract;

  int i = 0;
  //nr_points = (int) cloud_filtered->points.size ();
  // While 30% of the original cloud is still there
  int num_planes = 0;
  while (num_planes < 3)
  {
    Eigen::Vector4d equation;
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
    equation[0] = coefficients->values[0];
    equation[1] = coefficients->values[1];
    equation[2] = coefficients->values[2];
    equation[3] = coefficients->values[3];
    equations.push_back(equation);
    output.push_back(*cloud_p);
  }
  for(size_t i = 0; i < output.size(); i++) {
      Eigen::Vector3d centroid;
      pcl::PointCloud<pcl::PointXYZ> current_cloud = output[i];
      for(size_t j = 0; j < output[i].size(); j++) {
          pcl::PointXYZ point = current_cloud[j];
          centroid[0] += point.x;
          centroid[1] += point.y;
          centroid[2] += point.z;
      }
      centroid = centroid / current_cloud.size();
      centroids->push_back(centroid);
  }
*normal_equations = equations;
return output;
}
// For checking if the mean has not changed
bool DoubleEquals(double x, double y) {
  return fabs(x-y) < .000005;
}


double* CombineTransform(double* pose1, double* pose2) {

  Eigen::Transform<double, 3, Eigen::Affine> combine_transform;

  // For the two poses create a transform
  double* posek = new double[6];
  double* transform = pose1;

  //Create the eigen transform from the first pose
  Eigen::Matrix<double,3,1> axis(transform[0], transform[1], transform[2]);
  const double angle = axis.norm();
  if(angle != 0) {
    axis = axis / angle;
  }
  Eigen::Transform<double, 3, Eigen::Affine> rotation =
  Eigen::Transform<double, 3, Eigen::Affine>(Eigen::AngleAxis<double>(
    angle, axis));

  Eigen::Translation<double, 3> translation =
  Eigen::Translation<double, 3>(transform[3], transform[4], transform[5]);

  Eigen::Transform<double, 3, Eigen::Affine> affine_transform =
      translation * rotation;

  // For the second pose
  //double* posek = new double[6];
  double* transform2 = pose2;

  //Create the eigen transform from the pose
  Eigen::Matrix<double,3,1> axis_2(transform2[0], transform2[1], transform2[2]);
  const double angle_2 = axis.norm();
  if(angle_2 != 0) {
    axis_2 = axis_2 / angle_2;
  }
  Eigen::Transform<double, 3, Eigen::Affine> rotation_2 =
  Eigen::Transform<double, 3, Eigen::Affine>(Eigen::AngleAxis<double>(
    angle_2, axis_2));

  Eigen::Translation<double, 3> translation_2 =
  Eigen::Translation<double, 3>(transform2[3], transform2[4], transform2[5]);

  Eigen::Transform<double, 3, Eigen::Affine> affine_transform_2 =
      translation_2 * rotation_2;

  // Combine the two transforms
  combine_transform =  affine_transform * affine_transform_2;

  // Find the rotation component
  // Find the angle axis format
  Eigen::AngleAxis<double> angle_axis(combine_transform.rotation());

  // Get the axis
  Eigen::Vector3d normal_axis = angle_axis.axis();

  // Recompute the rotation angle
  double combined_angle = angle_axis.angle();
  Eigen::Vector3d combined_axis = normal_axis * combined_angle;

  // Recompute the rotation matrix
  Eigen::Transform<double, 3, Eigen::Affine> combined_rotation =
      Eigen::Transform<double, 3, Eigen::Affine>(
          Eigen::AngleAxis<double>(combined_angle, normal_axis));

  // Compute Translation
  Eigen::Translation<double, 3> combined_translation(
      combine_transform.translation());

  // Assign values to pose
  posek[3] = (combined_rotation.inverse() *
      combined_translation).translation().x();
  posek[4] = (combined_rotation.inverse() *
      combined_translation).translation().y();
  posek[5] = (combined_rotation.inverse() *
      combined_translation).translation().z();
  posek[0] = combined_axis(0);
  posek[1] = combined_axis(1);
  posek[2] = combined_axis(2);
  if (DoubleEquals( posek[0], 0)
      && DoubleEquals( posek[1], 0)  && DoubleEquals( posek[2], 0) ) {
    posek[0] = pose2[0];
    posek[1] = pose2[1];
    posek[2] = pose2[2];
  }
  return posek;
}

void ComparePlanes(vector<pcl::PointCloud<pcl::PointXYZ> > k1_planes,
  vector<pcl::PointCloud<pcl::PointXYZ> > transformed_planes,
  vector<Eigen::Vector4d> normal_equations,
  vector<Eigen::Vector4d> normal_equations_trans,
  Eigen::Vector3d& rotation_correction,
  Eigen::Vector3d& translation_correction) {


  for(size_t i = 0; i < k1_planes.size(); ++i) {
    cout << i << endl;
    pcl::PointCloud<pcl::PointXYZ> plane1 = k1_planes[i];
    pcl::PointCloud<pcl::PointXYZ> plane2 = transformed_planes[i];
    publish_cloud(plane1, cloud_pub);
    publish_cloud(plane2, cloud_pub_2);
    sleep(3);

//     double x1 = 0;
//     double y1 = 0;
//     double z1 = 0;
//     double x2 = 0;
//     double y2 = 0;
//     double z2 = 0;
    double px1 = 0;
    double py1 = 0;
    double pz1 = 0;
    double px2 = 0;
    double py2 = 0;
    double pz2 = 0;
    pcl::Normal average2;
    for(size_t j = 0; j < k1_planes[i].size(); ++j) {
        px1 += plane1[j].x;
        py1 += plane1[j].y;
        pz1 += plane1[j].z;
    }
    for(size_t j = 0; j < transformed_planes[i].size(); ++j) {
        px2 += plane2[j].x;
        py2 += plane2[j].y;
        pz2 += plane2[j].z;
    }
//     x1 = x1 / normals1.size();
//     y1 = y1 / normals1.size();
//     z1 = z1 / normals1.size();
//     x2 = x2 / normals2.size();
//     y2 = y2 / normals2.size();
//     z2 = z2 / normals2.size();
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
    cout << "num normals: " << normal_equations.size() << endl;
    Eigen::Vector4d plane_equation_1 = normal_equations[i];
    Eigen::Vector4d plane_equation_2 = normal_equations_trans[i];
    norm_k[0] = plane_equation_1[0];
    norm_k[1] = plane_equation_1[1];
    norm_k[2] = plane_equation_1[2];
    norm_k1[0] = plane_equation_2[0];
    norm_k1[1] = plane_equation_2[1];
    norm_k1[2] = plane_equation_2[2];
    p1_mat[0] = px1;
    p1_mat[1] = py1;
    p1_mat[2] = pz1;
    p2_mat[0] = px2;
    p2_mat[1] = py2;
    p2_mat[2] = pz2;
    cout << "Offset Residual 1: " << (p1_mat - p2_mat).dot(norm_k) << endl;
    cout << "Offset Residual 2: " << (p2_mat - p1_mat).dot(norm_k1) << endl;

    double dot = plane_equation_1[0]*plane_equation_2[0] +\
        plane_equation_1[1]*plane_equation_2[1] +\
        plane_equation_1[2]*plane_equation_2[2];
        double lenSq1 = plane_equation_1[0]*plane_equation_1[0] + plane_equation_1[1]*plane_equation_1[1] + plane_equation_1[2]*plane_equation_1[2];
        double lenSq2 =plane_equation_2[0]*plane_equation_2[0] + plane_equation_2[1]*plane_equation_2[1] + plane_equation_2[2]*plane_equation_2[2       ];
    double angle = acos(dot/sqrt(lenSq1 * lenSq2));
    Eigen::Vector3d axis = norm_k.cross(norm_k1);
    rotation_correction = axis * angle;
    cout << rotation_correction[0] << endl;
    cout << rotation_correction[1] << endl;
    cout << rotation_correction[2] << endl;
    cout << "Angle between plane normals " << i << " : " << angle << endl;
    double d_diff = abs(plane_equation_1[3] - plane_equation_2[3]);
    cout << "D Difference between planes " << i << " : " << d_diff << endl;
  }
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
  string out_name2 = bagfile + ".aligned2";
  string out_name3 = bagfile + ".aligned3";
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

  visualization_msgs::Marker line_list;
  std_msgs::Header header;
  header.frame_id = "point_clouds";
  line_list.action = visualization_msgs::Marker::ADD;
  //line_list.pose.orientation.w = 1.0;
  line_list.header.frame_id = "point_cloud";

  line_list.id = 0;
  line_list.type = visualization_msgs::Marker::LINE_LIST;
  line_list.scale.x = 0.01;

  // Line list is red
  line_list.color.r = 1.0;
  line_list.color.a = 1.0;
  geometry_msgs::Point center, x, y, z;
  center.x = center.y = center.z = x.y = x.z = y.x = y.z = z.x = z.y = 0;
  z.z = .5;
  x.x = .5;
  y.y = .5;
  bag_it = time_aligned_clouds(bag_it, end,  &buffer_k1, &buffer_k2, &times_k1, &times_k2,
    &keyframe_k1, &keyframe_k2, &timestamp_1, &timestamp_2);

   // While there are still clouds in both datasets
  int count = 0;
  cout << "Starting Loop" << endl;
  while(((buffer_k1.size() != 0 && buffer_k2.size() != 0 )|| (bag_it != view.end())) && count < 1) {  // Wrap the multiple cloud work inside this into a loop over the number of kinects
    count += 1;
    cout << count << endl;
    // Read in a new cloud from each dataset
    pcl::PointCloud<pcl::PointXYZ> cloud_k1;
    pcl::PointCloud<pcl::PointXYZ> cloud_k2;

    bag_it = time_aligned_clouds(bag_it, end, &buffer_k1, &buffer_k2, &times_k1, &times_k2,
    &cloud_k1, &cloud_k2, &timestamp_1, &timestamp_2);
    pcl::PointCloud<pcl::PointXYZ> transformed_cloud = cloud_k2;
    writeToObj(out_name2, count, cloud_k1);
    writeToObj(out_name3, count, cloud_k2);
    cout << "Cloud 1 planes" << endl;
    vector<Eigen::Vector4d> normal_equations, k2_normal_equations;
    vector<Eigen::Vector4d> normal_equations_trans;
    vector<Eigen::Vector3d> k1_centroids, k2_centroids;
    // Retrieve the planes from the clouds
    vector<pcl::PointCloud<pcl::PointXYZ> > k1_planes = getPlanes(cloud_k1, &normal_equations, &k1_centroids);
    vector<pcl::PointCloud<pcl::PointXYZ> > k2_planes = getPlanes(cloud_k2, &k2_normal_equations, &k2_centroids);

    double transform_division = 100;
    // calculate partial transform
    transform[3] = transform[3] / transform_division;
    transform[4] = transform[4] / transform_division;
    transform[5] = transform[5] / transform_division;
    Eigen::Matrix<double,3,1> axis(transform[0], transform[1], transform[2]);
    const double angle = axis.norm();
    if(angle != 0) {
      axis = axis / angle;
    }
    transform[0] = axis[0] * (angle / transform_division);
    transform[1] = axis[1] * (angle / transform_division);
    transform[2] = axis[2] * (angle / transform_division);
    // double  rotation[] = {1.74, 0, 0, 0, 0, 0};
//     TransformPointCloud_PCL(cloud_k1, rotation);
//     TransformPointCloud_PCL(cloud_k2, rotation);
    // in a loop apply partial transform n times
    for(int i = 0; i < transform_division; i++) {
      TransformPointCloud_PCL(transformed_cloud, transform);
//       TransformPointCloud_PCL(transformed_cloud, rotation);

      GeometryTransform(&x, transform);
      GeometryTransform(&y, transform);
      GeometryTransform(&z, transform);
      line_list.points.push_back(center);
      line_list.points.push_back(x);
      line_list.points.push_back(center);
      line_list.points.push_back(y);
      line_list.points.push_back(center);
      line_list.points.push_back(z);
      publish_cloud(cloud_k1, cloud_pub);
      publish_cloud(cloud_k2, cloud_pub_2);
      publish_cloud(transformed_cloud, cloud_pub_3);
//       TransformPointCloudInv(transformed_cloud, rotation);

      marker_pub.publish(line_list);
      line_list.points.clear();
    }
    pcl::PointCloud<pcl::PointXYZ> combo_cloud = cloud_k1;
//     Eigen::Vector3d rotation_correction, translation_correction;
//     vector<pcl::PointCloud<pcl::PointXYZ> > transformed_planes = getPlanes(transformed_cloud, &normal_equations_trans);
//     ComparePlanes(k1_planes, transformed_planes, normal_equations, normal_equations_trans, rotation_correction, translation_correction);
//     transform[0] = rotation_correction[0];
//     transform[1] = rotation_correction[1];
//     transform[2] = rotation_correction[2];
//     transform[3] = 0;
//     transform[4] = 0;
//     transform[5] = 0;

//     TransformPointCloud_PCL(transformed_cloud, rotation);


//     publish_cloud(transformed_cloud, cloud_pub_3);
//     publish_cloud(cloud_k1, cloud_pub);
// //     TransformPointCloudInv(transformed_cloud, rotation);
//     TransformPointCloud_PCL(transformed_cloud, transform);
// //     TransformPointCloud_PCL(transformed_cloud, rotation);
//     sleep(3);
//     publish_cloud(transformed_cloud, cloud_pub_3);
//     publish_cloud(cloud_k1, cloud_pub);
    //writeToObj(out_name, count, combo_cloud);
  }
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

  if (mode == 0) {
    CheckTransform(bagFile, transform_file);
  }
  // else if(mode == 1) {
  //   CheckTransform(bagFile, transform_file);
  // }
  return 0;
}
