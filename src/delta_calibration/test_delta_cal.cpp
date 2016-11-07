//----------- INCLUDES
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
ros::Publisher depth_pub;
ros::Publisher markerArray_pub;

int small_error, medium_error, large_error = 0;

// Signal handler for breaks (Ctrl-C)
void HandleStop(int i) {
  printf("\nTerminating.\n");
  exit(0);
}

void PrintPose(double* pose){
  for(uint i = 0; i < 6; i ++) {
    cout << pose[i] << " ";
  }
  cout << endl;
}


void SavePoses(const vector<double*>& poses, string& filename) {
  filename += ".poses";
  ofstream myfile (filename.c_str());
  if (myfile.is_open()) {
    for(size_t i = 0; i < poses.size(); i++) {
        double* pose = poses[i];
         for(int j = 0; j < 6; j++) {
          myfile << pose[j] << "\t";
    }
    myfile << "\n";
  }
    myfile.close();
  }
  else {
    cout << "Unable to open file";
  }
}

void WritePose(double* pose, ofstream file) {
  if (file.is_open()) {
    for(int j = 0; j < 6; j++) {
      file << pose[j] << "\t";
    }
  file << "\n";
  }
}

void WriteToObj(const string folder, const string bagfile,
    const int num,
    const pcl::PointCloud<pcl::PointXYZ>& cloud) {

  std::string frame_string;
  std::stringstream out;
  string temp = bagfile;
  mkdir(folder.c_str(), 0777);
  out << num;
  string filename = folder + "/" + bagfile + "_" + out.str() + ".obj";
  ofstream file(filename.c_str());
  for(size_t i = 0; i < cloud.size(); i++) {
    pcl::PointXYZ point = cloud[i];
    file << "v " << point.x << " " << point.y << " " << point.z << endl;
  }
  file.close();
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

pcl::PointCloud<pcl::PointXYZ> PlaneToPoints(
    Eigen::Vector3d v1, 
    Eigen::Vector3d v2, 
    Eigen::Vector3d v3, 
    Eigen::Vector3d v4) {

  double step_size = .001;
  double dist = PointDistance(v1, v4);
  Eigen::Vector3d dir = v4 - v1;
  Eigen::Vector3d temp = v1;
  Eigen::Vector3d temp3 = v2;
  Eigen::Vector3d dir3 = v3 - v2;
  double dist3 = PointDistance(v2, v3);
  pcl::PointCloud<pcl::PointXYZ> point_cloud;
  // Iterate over one side
  for(double i = 0; i < dist; i += step_size){
    double dist2 = PointDistance(v1, v2);
    Eigen::Vector3d temp2 = temp;
    Eigen::Vector3d dir2 = temp3 - temp2;
    // Iterate over the other
    for(double j = 0; j < dist2; j += step_size){
      pcl::PointXYZ point;
      temp2 = temp2 + (step_size * (dir2 / dist2));
      point.x = temp2[0];
      point.y = temp2[1];
      point.z = temp2[2];
      point_cloud.points.push_back(point);
    }
    temp = temp + (step_size * (dir / dist));
    temp3 = temp3 + (step_size * (dir3 / dist3));
  }
  PublishCloud(point_cloud, cloud_pub_1);
  return point_cloud;
}

pcl::PointCloud<pcl::PointXYZ> CreateView(
    pcl::PointCloud<pcl::PointXYZ> cloud,
    int quant,
    std::vector<int>& index,
    std::vector<Eigen::Vector2d>& image_coords) {

  // Camera Parameters
  double fovH = RAD(57);
  double fovV = RAD(43);
  double tanHFovH = tan(fovH*0.5); 
  double tanHFovV = tan(fovV*0.5);
  double a = 3.008;
  double b = -.002745;
  int width = 640;
  int height = 480;
  
  pcl::PointCloud<pcl::PointXYZ> view;

  Eigen::MatrixXd image = Eigen::MatrixXd::Constant(480, 640, -1);
  // Find pixel location
  cout << "Loop" << endl;
  for(size_t i = 0; i < cloud.size(); i++) {
    double x = 0;
    if(quant) {
      x = (1.0 - a * cloud.points[i].x) / (b * cloud.points[i].x);
      x = 1.0/(a+b*float(x));
    } else { 
      x = cloud.points[i].x;
    }
    double y = cloud.points[i].y;
    double z = cloud.points[i].z;
    double col = y / x;
    double row = z / x;
    col = (.5 * (double(width) - 1.0) * (tanHFovH - col))/ tanHFovH;
    row = (.5 * (double(height) - 1.0) * (tanHFovV - row))/ tanHFovV;
    // double col = (-y/tanHFovH + 1) * (width -1 *.5);
    // double row = (-z/tanHFovV + 1) * (height -1 *.5);
    
    if(col < 640 && row < 480 && col > 0 && row > 0){
      if (( image(row,col) == -1 || x < cloud.points[image(row, col)].x)&& x > 0) {
        image(row, col) = i;
      }
    }
  }
  int trim_value = 0;
  for(int i = trim_value; i < image.rows() - trim_value; i++) {
    for(int j = trim_value; j < image.cols() - trim_value; j++) {
      if(cloud.points[image(i, j)].x > 0) {
        view.points.push_back(cloud.points[image(i, j)]);
        index.push_back(image(i, j));
        Eigen::Vector2d coord;
        coord[0] = i;
        coord[1] = j;
        image_coords.push_back(coord);
      }
    }
  }
  PublishCloud(view, cloud_pub_1);
  return view;
}

pcl::PointCloud<pcl::PointXYZ> CreateBox(double f, double w, double t, double b, double l, double l_2) {
  pcl::PointCloud<pcl::PointXYZ> box;
  Eigen::Vector3d v1, v2, v3, v4;
  v1 << l_2,f,b;
  v2 << l,f,b;
  v3 << l,f,t;
  v4 << l_2,f,t;
  box = PlaneToPoints(v1, v2, v3, v4);
  Eigen::Vector3d v5, v6, v7, v8;
  v5 << l_2,w,b;
  v6 << l,w,b;
  v7 << l,w,t;
  v8 << l_2,w,t;
  box += PlaneToPoints(v5, v6, v7, v8);
  Eigen::Vector3d v9, v10, v11, v12;
  v9 << l_2,f,t;
  v10 << l,f,t;
  v11 << l,w,t;
  v12 << l_2,w,t;
  box += PlaneToPoints(v9, v10, v11, v12);
  Eigen::Vector3d v13, v14, v15, v16;
  v13 << l_2,f,b;
  v14 << l,f,b;
  v15 << l,w,b;
  v16 << l_2,w,b;
  box += PlaneToPoints(v13, v14, v15, v16);
  Eigen::Vector3d v17, v18, v19, v20;
  v17 << l_2,f,t;
  v18 << l_2,w,t;
  v19 << l_2,w,b;
  v20 << l_2,f,b;
  box += PlaneToPoints(v17, v18, v19, v20);
  Eigen::Vector3d v21, v22, v23, v24;
  v21 << l,f,t;
  v22 << l,w,t;
  v23 << l,w,b;
  v24 << l,f,b;
  box += PlaneToPoints(v21, v22, v23, v24);
  return box;
}

pcl::PointCloud<pcl::PointXYZ> CreateRoom() {

  pcl::PointCloud<pcl::PointXYZ> room;

  room = CreateBox(-1, 1, 1, -1, 1, -1);

  // for(double i = -.25; i < .25; i += .2){
  //   room += CreateBox(1, .75, .04, .14, i, i + .1);
  //   room += CreateBox(-1, -.75, .04, .14, i, i + .1);
  // }

  for(double i = -.9; i < 1; i += .2){
    room += CreateBox(1, .75, .04, .14, i, i + .1);
    room += CreateBox(-1, -.75, .04, .14, i, i + .1);
  }

  for(double i = -.9; i < 1; i += .2){
    room += CreateBox(.75, 1, .4, .5, i, i + .1);
    room += CreateBox(-.75, -1, .4, .5, i, i + .1);
  }

  for(double i = -.9; i < 1; i += .2){
    room += CreateBox(.75, 1, -.4, -.5, i, i + .1);
    room += CreateBox(-.75, -1, -.4, -.5, i, i + .1);
  }
  for(double i = -.9; i < 1; i += .2){
    room += CreateBox(.75, 1, -.14, -.24, i, i + .1);
    room += CreateBox(-.75, -1, -.14, -.24, i, i + .1);
  }
  // for(double i = -.9; i < 1; i += .2){
  //   room += CreateBox(.75, 1, -.24, -.34, i, i + .1);
  //   room += CreateBox(-.75, -1, -.24, -.34, i, i + .1);
  // }

  for(double i = -.9; i < 1; i += .2){
    room += CreateBox(i, i + .1, .04, .14, .75, 1);
    room += CreateBox(i, i + .1, .04, .14, -.75, -1);
  }

  for(double i = -.9; i < 1; i += .2){
    room += CreateBox(i, i + .1, -.14, -.24, .75, 1);
    room += CreateBox(i, i + .1, -.14, -.24, -.75, -1);
  }

  for(double i = -.9; i < 1; i += .2){
    room += CreateBox(i, i + .1, .4, .24, .75, 1);
    room += CreateBox(i, i + .1, .4, .24, -.75, -1);
  }
  for(double i = -.9; i < 1; i += .2){
    room += CreateBox(i, i + .1, -.4, -.5, .75, 1);
    room += CreateBox(i, i + .1, -.4, -.5, -.75, -1);
  }

  return room;
}

vector<int> ReverseLookup(vector<int> index, const pcl::PointCloud<pcl::PointXYZ>& cloud) {
  vector<int> reverse(40729230);
  for(size_t i = 0; i < cloud.size(); i++) {
    reverse[index[i]] = i;
  }
  return reverse;
}

void VisualizeNormal(
    const pcl::PointCloud<pcl::PointXYZ>& key_cloud,
    const pcl::PointCloud<pcl::PointXYZ>& k1_cloud,
    const pcl::PointCloud<pcl::Normal>& normal_key,
    const pcl::PointCloud<pcl::Normal>& normal_k1) {

  visualization_msgs::Marker line_list;
  std_msgs::Header header;
  header.frame_id = "point_clouds";
  line_list.action = visualization_msgs::Marker::ADD;
  //line_list.pose.orientation.w = 1.0;
  line_list.header.frame_id = "point_cloud";

  line_list.id = 0;
  line_list.type = visualization_msgs::Marker::LINE_LIST;
  line_list.scale.x = 0.0001;

  // Line list is blue
  line_list.color.b = 1.0;
  line_list.color.a = 1.0;

  for(size_t i = 0; i < k1_cloud.size(); ++i) {
    pcl::PointXYZ k1_point = k1_cloud[i];
    pcl::Normal k1_normal = normal_k1[i];
    pcl::PointXYZ k2_point = key_cloud[i];
    pcl::Normal key_normal = normal_key[i];
    if (!isnan(k1_point.x) &&
        !isnan(k1_point.y) &&
        !isnan(k1_point.z) &&
        !isnan(k2_point.x) &&
        !isnan(k2_point.y) &&
        !isnan(k2_point.z)) {
      geometry_msgs::Point p, p2, p3, p4;
      p.x = k1_point.x;
      p.y = k1_point.y;
      p.z = k1_point.z;
      p2.x = k1_point.x + (k1_normal.normal_x / 100);
      p2.y = k1_point.y + (k1_normal.normal_y / 100);
      p2.z = k1_point.z + (k1_normal.normal_z / 100);
      p3.x = k2_point.x;
      p3.y = k2_point.y;
      p3.z = k2_point.z;
      p4.x = k2_point.x + (key_normal.normal_x /100);
      p4.y = k2_point.y + (key_normal.normal_y/ 100);
      p4.z = k2_point.z + (key_normal.normal_z /100);
//       line_list.points.push_back(p);
//       line_list.points.push_back(p2);
      line_list.points.push_back(p3);
      line_list.points.push_back(p4);
    }
  }

  marker_pub.publish(line_list);
}

void VisualizeError(const ros::Publisher& marker_pub,
                    const pcl::PointCloud<pcl::PointXYZ>& cloud_1, 
                    const pcl::PointCloud<pcl::PointXYZ>& cloud_key,
                    const pcl::PointCloud<pcl::Normal>& normal_k1,
                    const pcl::PointCloud<pcl::Normal>& normal_key,
                    vector<int> index_k1,
                    vector<int> index_key,
                    double* transform) {
  
  bool found_small = false;
  bool found_medium = false;
  bool found_large = false;
  // For visualizing normal error
  visualization_msgs::Marker line_list;
  std_msgs::Header header;
  header.frame_id = "point_clouds";
  line_list.action = visualization_msgs::Marker::ADD;
  //line_list.pose.orientation.w = 1.0;
  line_list.header.frame_id = "point_cloud";

  line_list.id = 0;
  line_list.type = visualization_msgs::Marker::LINE_LIST;
  line_list.scale.x = 0.0001;

  // Line list is blue
  line_list.color.b = 1.0;
  line_list.color.a = 1.0;
  
  pcl::PointCloud<pcl::PointXYZRGB> color_1;
  pcl::PointCloud<pcl::PointXYZRGB> color_2;
  pcl::PointCloud<pcl::PointXYZ> cloud_k1 = cloud_1;
  TransformPointCloud(cloud_k1, transform);
  copyPointCloud(cloud_k1, color_1);
  copyPointCloud(cloud_key, color_2);
  vector<int> reverse = ReverseLookup(index_key, cloud_key);
  vector<int> start_points;
  vector<int> neighbor_points;
  for(size_t i = 0; i < cloud_k1.size(); i++){
    pcl::PointXYZ k1_point, base_point;
    // Use the original cloud, not the transformed cloud
    k1_point = cloud_1[i];
    // Base cloud is unchanged
    
    pcl::Normal normal_1;
    pcl::Normal key_normal;
    int num;
    bool corresponds = GetKnownCor(cloud_key, normal_key, index_k1[i], index_key,  &normal_1, &base_point, num);
    // int index_k1_p = index_k1[i];
    pcl::PointXYZ point_k1 = cloud_k1[i];
    pcl::Normal k1_normal = normal_k1[i];
    if(corresponds) {
      start_points.push_back(i);
      neighbor_points.push_back(num);
      
      pcl::PointXYZ point_key = cloud_key[num];
//       pcl::Normal key_normal = normal_key[num];
      double error = PointDistance(point_k1,point_key);
      pcl::PointXYZRGB cpoint_k1;
      pcl::PointXYZRGB cpoint_key;
      int r = 0;
      int g = 0;
      int b = 0;
      if(error < .0001) {
        r = 255;
        g = 255;
        b = 255;
      } else if(error < .001) {
        found_small = true;
        r = 255;
        g = 255;
        b = 102;
      }
      else if(error < .01 and error > .001) {
        found_medium = true;
        r = 255;
        g = 128;
        b = 0;
      }
      else if(error >= .01) {
        found_large = true;
        r = 255;
        g = 0;
        b = 0;
      }
      cpoint_k1.x = point_k1.x;
      cpoint_k1.y = point_k1.y;
      cpoint_k1.z = point_k1.z;
      cpoint_k1.r = r;
      cpoint_k1.g = g;
      cpoint_k1.b = b;
      cpoint_key.x = point_key.x;
      cpoint_key.y = point_key.y;
      cpoint_key.z = point_key.z;
      cpoint_key.r = r;
      cpoint_key.g = g;
      cpoint_key.b = b;
      color_1[i] =cpoint_k1;
      color_2[num] = cpoint_key;
      
      geometry_msgs::Point p, p2, p3, p4;
      p.x = point_k1.x;
      p.y = point_k1.y;
      p.z = point_k1.z;
      p2.x = point_k1.x + (k1_normal.normal_x * error);
      p2.y = point_k1.y + (k1_normal.normal_y * error);
      p2.z = point_k1.z + (k1_normal.normal_z * error);
//       p3.x = point_key.x;
//       p3.y = point_key.y;
//       p3.z = point_key.z;
//       p4.x = point_key.x + (key_normal.normal_x * error);
//       p4.y = point_key.y + (key_normal.normal_y * error);
//       p4.z = point_key.z + (key_normal.normal_z * error);
      line_list.points.push_back(p);
      line_list.points.push_back(p2);
      
    } 
  }
  if(found_medium){
    fprintf(stdout,"Found Medium");
    medium_error++;
  }
  if(found_small){
    fprintf(stdout, "found small");
    small_error++;
  }
  if(found_large){
    large_error++;
  }
  PublishCloud(color_1,cloud_pub_3);
  PublishCloud(color_2,cloud_pub_4);
  pcl::PointCloud<pcl::PointXYZ> transformed_cloud = cloud_key;
  
//   // Shifts cloud by calculated transform
//   TransformPointCloud(transformed_cloud, transform);
//   PublishCloud(cloud_1, cloud_pub_1);
//   PublishCloud(transformed_cloud, cloud_pub_2);
//   vector<int > nearest_neigbors2;
//   vector<int > start_points2;
//   KdTreeNN(cloud_1, transformed_cloud,
//            nearest_neigbors2, start_points2);
//   VisualizeNN(cloud_1, transformed_cloud, nearest_neigbors2, start_points2, marker_pub);
  //marker_pub.publish(line_list);
  //VisualizeNormal(cloud_key, cloud_k1, normal_key, normal_k1);
  //VisualizeNN(cloud_key, cloud_k1, neighbor_points, start_points, marker_pub);
}

void TestDeltaCalc(int quant, vector<ros::Publisher> publishers, const ros::Publisher& marker_pub, const double nn_dist) {

  vector<pcl::PointCloud<pcl::Normal> > all_normals;
  // Opening pose and trajectory files
  string pose_name_1 = "delta_cal_test.pose";
  ofstream pose_file (pose_name_1.c_str());
  rosbag::Bag keyframe_bag;
  keyframe_bag.open("simulator_keyframes.bag", rosbag::bagmode::Write);
  // Rotate two initial setups
  pcl::PointCloud<pcl::PointXYZ> room1 = CreateRoom();
  pcl::PointCloud<pcl::PointXYZ> room2 = CreateRoom();
  vector<int> index_pred_k1;
  vector<int> index_pred_k2;
  vector<int> index_key_k1;
  vector<int> index_key_k2;
  double* pose1 = new double[6];
  pose1[0] = 0;
  pose1[1] = 0;
  pose1[2] = 0;
  pose1[3] = 0;
  pose1[4] = 0;
  pose1[5] = 0;
  vector<Eigen::Vector2d> image_coords_pred_k1;
  TransformPointCloudInv(room1, pose1);
  pcl::PointCloud<pcl::PointXYZ> pred_k1 =
  CreateView(room1, quant, index_pred_k1, image_coords_pred_k1);
  double* pose2 = new double[6];
  pose2[0] = 0;
  pose2[1] = 0;
  pose2[2] = 0.785398;
  pose2[3] = 0;
  pose2[4] = 0;
  pose2[5] = 0;
  TransformPointCloudInv(room2, pose2);
  vector<Eigen::Vector2d> image_coords_pred_k2;
  pcl::PointCloud<pcl::PointXYZ> pred_k2 = CreateView(room2, quant, index_pred_k2, image_coords_pred_k2);
  index_key_k1 = index_pred_k1;
  index_key_k2 = index_pred_k2;
  pcl::PointCloud<pcl::PointXYZ> keyframe_k1 = pred_k1;
  pcl::PointCloud<pcl::PointXYZ> keyframe_k2 = pred_k2;

  cout << "Views Created " << endl;
  pcl::PointCloud<pcl::Normal> pred_normal_k1 = GetNormals(pred_k1);
  pcl::PointCloud<pcl::Normal> pred_normal_k2 = GetNormals(pred_k2);
  all_normals.push_back(pred_normal_k1);
  all_normals.push_back(pred_normal_k2);
  all_normals.push_back(pred_normal_k1);
  all_normals.push_back(pred_normal_k2);
  WriteToBag("kinect_1", &keyframe_bag, keyframe_k1);
  WriteToBag("kinect_2", &keyframe_bag, keyframe_k2);
  pcl::PointCloud<pcl::Normal> key_normal_k1 = pred_normal_k1;
  pcl::PointCloud<pcl::Normal> key_normal_k2 = pred_normal_k2;
  cout << "Normals Created " << endl;
  // Get Transforms from file
  // From this point on, calculating the icp between clouds
  double* final_transform_k1 = new double[6];
  double* final_transform_k2 = new double[6];
  // Initialize transform ARRAYS
  vector<double> pose0(6, 0.0);
  std::copy(pose0.begin(), pose0.end(), final_transform_k1);
  std::copy(pose0.begin(), pose0.end(), final_transform_k2);
  string object_file = "test_delta_cal_objects";
  string base_name_1 =  "test_delta_cal.base";
  string base_name_2 = "test_delta_cal.base";
  std::ifstream infile("withTranslation.pose");
  double rx1, ry1, rz1, tx1, ty1, tz1, a, b, rx2, ry2, rz2, tx2, ty2, tz2,c,d;
  int count = 0;
  cout << "reading transforms" << endl;
  while(infile >> rx1>> ry1>> rz1>> tx1>> ty1>> tz1>> a>> b>> rx2>> ry2>> rz2>> tx2>> ty2>> tz2>>c>>d) {
    count++;
    cout << count << endl;
    pose1[0] = rx1;
    pose1[1] = ry1;
    pose1[2] = rz1;
    pose1[3] = tx1;
    pose1[4] = ty1;
    pose1[5] = tz1;
    TransformPointCloudInv(room1, pose1);
    vector<int> index_cloud_k1;
    vector<int> index_cloud_k2;
    vector<Eigen::Vector2d> image_coords_cloud_k1;
    pcl::PointCloud<pcl::PointXYZ> cloud_k1 = CreateView(room1, quant, index_cloud_k1, image_coords_cloud_k1);
    pose2[0] = rx2;
    pose2[1] = ry2;
    pose2[2] = rz2;
    pose2[3] = tx2;
    pose2[4] = ty2;
    pose2[5] = tz2;
    TransformPointCloudInv(room2, pose2);
    vector<Eigen::Vector2d> image_coords_cloud_k2;
    pcl::PointCloud<pcl::PointXYZ> cloud_k2 = CreateView(room2, quant, index_cloud_k2, image_coords_cloud_k2);

    // From this point on, calculating the icp between clouds
    double* calculated_delta_k1 = new double[6];
    double* calculated_delta_k2 = new double[6];
    // Initialize transform ARRAYS
    vector<double> pose0(6, 0.0);
    std::copy(pose0.begin(), pose0.end(), calculated_delta_k1);
    std::copy(pose0.begin(), pose0.end(), calculated_delta_k2);
    
    pcl::PointCloud<pcl::Normal> normal_k1 = GetNormals(cloud_k1);
    pcl::PointCloud<pcl::Normal> normal_k2 = GetNormals(cloud_k2);
    all_normals.push_back(normal_k1);
    all_normals.push_back(normal_k2);
    double rmse1, rmse2;
    ICP(count,
        nn_dist,
        publishers,
        "",
        "",
        pred_k1,
        cloud_k1,
        pred_normal_k1,
        normal_k1,
        image_coords_pred_k1,
        image_coords_cloud_k1,
        calculated_delta_k1,
        &rmse1);

    ICP(count,
        nn_dist,
        publishers,
        "",
        "",
        pred_k2,
        cloud_k2,
        pred_normal_k2,
        normal_k2,
        image_coords_pred_k2,
        image_coords_cloud_k2,
        calculated_delta_k2,
        &rmse1);

    final_transform_k1 = CombineTransform(final_transform_k2,
                                               calculated_delta_k1);

    final_transform_k2 = CombineTransform(final_transform_k2,
                                               calculated_delta_k2);
    ICP(count,
        nn_dist,
        publishers,
        "",
        "",
        keyframe_k2,
        cloud_k2,
        key_normal_k2,
        normal_k2,
        image_coords_pred_k2,
        image_coords_cloud_k2,
        final_transform_k2,
        &rmse1);
    ICP(count,
        nn_dist,
        publishers,
        "",
        "",
        keyframe_k1,
        cloud_k1,
        key_normal_k1,
        normal_k1,
        image_coords_pred_k1,
        image_coords_cloud_k1,
        final_transform_k1,
        &rmse2);
    
//     if(rmse1 > .0006 || rmse2 > .0006){
//       ICP(count,
// 	.01,
//         publishers,
//         "",
//         "",
//         keyframe_k2,
//         cloud_k2,
//         key_normal_k2,
//         normal_k2,
//         final_transform_k2,
//         &rmse1);
//     ICP(count,
// 	.01,
//         publishers,
//         "",
//         "",
//         keyframe_k1,
//         cloud_k1,
//         key_normal_k1,
//         normal_k1,
//         final_transform_k1,
//         &rmse2);
//       
//     }
    fprintf(stdout, "RMSE1: %f", rmse1);
    
    fprintf(stdout, "RMSE2: %f", rmse2);
    const vector<double> velocity_list1;
    const vector<double> velocity_list2;
    VisualizeError( marker_pub, cloud_k1, keyframe_k1, normal_k1, key_normal_k1, index_cloud_k1, index_key_k1, final_transform_k1);
    if(CheckChangeVel(final_transform_k1, 5, velocity_list1)
        && CheckChangeVel(final_transform_k2, 5, velocity_list2)) {
      // push back the keyframe and the combined transform to the final results
      // Set keyframe = k, set combined transform = to the zero transform
      // set Current value to keyframe
      keyframe_k1 = cloud_k1;
      keyframe_k2 = cloud_k2;
      key_normal_k2 = normal_k2;
      key_normal_k1 = normal_k1;
      index_key_k1 = index_cloud_k1;
      index_key_k2 = index_cloud_k2;
      // Writing deltas to file
      WritePoseFile(final_transform_k1, 0, count, pose_file);
      WritePoseFile(final_transform_k2, 0, count, pose_file);
      pose_file << endl;

      //Transform the clouds and then write them to the object bag_name
      pcl::PointCloud<pcl::PointXYZ> temp_cloud1 = cloud_k1;
      pcl::PointCloud<pcl::PointXYZ> temp_cloud2 = cloud_k2;
      WriteToBag("kinect_1", &keyframe_bag, cloud_k1);
      WriteToBag("kinect_2", &keyframe_bag, cloud_k2);
      TransformPointCloud(temp_cloud1, final_transform_k1);
      TransformPointCloud(temp_cloud2, final_transform_k2);
      // WriteToObj(object_file, output_name_1, count, temp_cloud1);
      // WriteToObj(object_file, output_name_2, count, temp_cloud2);
      // WriteToObj(object_file, base_name_1, count, cloud_k1);
      // WriteToObj(object_file, base_name_2, count, cloud_k2);
      // Set combined transforms to zero
      std::copy(pose0.begin(), pose0.end(), final_transform_k1);
      std::copy(pose0.begin(), pose0.end()  , final_transform_k2);
    }
    pred_k1 = cloud_k1;
    pred_normal_k1 = normal_k1;
    pred_k2 = cloud_k2;
    pred_normal_k2 = normal_k2;
    index_pred_k1 = index_cloud_k1;
    index_pred_k2 = index_cloud_k2;
  }
  fprintf(stdout, "S-Er: %d \t M-Er: %d \t L-Er: %d", small_error, medium_error, large_error);
  ofstream fout("simulator_normals.dat", ios::out | ios::binary);
  size_t size = all_normals[0].size();
  fout.write((char*)&size, sizeof(size));
  fout.write((char*)&all_normals[0], all_normals.size() * size);
  fout.close();
  keyframe_bag.close();
}

int main(int argc, char **argv) {
  signal(SIGINT,HandleStop);
  signal(SIGALRM,HandleStop);

  int max_clouds = INT_MAX;
  double nn_dist = 0.05;
  char* bag_file = (char*)"pair_upright.bag";
  bool test_mode = false;
  // Parse arguments.
  static struct poptOption options[] = {
    { "max-clouds" , 'k', POPT_ARG_INT , &max_clouds ,0, "Max Clouds" , "NUM" },
    { "delta" , 'd', POPT_ARG_DOUBLE, &nn_dist ,0, "Angular Change" ,
        "NUM" },
    { "bag-file" , 'B', POPT_ARG_STRING, &bag_file ,0, "Process bag file" ,
        "STR" },
    { "test-mode", 'T', POPT_ARG_NONE, &test_mode, 0, "Run simulation test",
        "NONE" },
    POPT_AUTOHELP
    { NULL, 0, 0, NULL, 0, NULL, NULL }
  };

  // parse options
  POpt popt(NULL,argc,(const char**)argv,options,0);
  int c;
  while((c = popt.getNextOpt()) >= 0) {
  }
  // Print option values
  printf("Max Frames: %d\nBagfile: %s\n Delta Size: %f\n",
         max_clouds,
         bag_file,
         nn_dist);

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
  publishers.push_back(marker_pub);
  depth_pub = n.advertise<sensor_msgs::Image> ("Cobot/Kinect/Depth", 1);
  publishers.push_back(depth_pub);
  markerArray_pub =
  n.advertise<visualization_msgs::MarkerArray>(
      "visualization_marker_array", 10);

  TestDeltaCalc(1, publishers, marker_pub, nn_dist);
  return 0;
}