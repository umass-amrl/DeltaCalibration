#include "ros/ros.h"
#include "std_msgs/String.h"
#include "std_msgs/Float32.h"
#include "std_msgs/Float32MultiArray.h"
#include "std_msgs/Int32MultiArray.h"
#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/Image.h>
#include <geometry_msgs/Twist.h>
#include <actionlib_msgs/GoalID.h>
#include <actionlib_msgs/GoalStatusArray.h>
#include <nav_msgs/Odometry.h>
#include "delta_calibration/icp.h"
#include <pcl/common/common.h>
#include <frontier_exploration/ExploreTaskAction.h>
#include <actionlib/client/simple_action_client.h>
#include <rosbag/bag.h>
#include <pthread.h>
#include<delta_calc.h>
#include<partial_calibrate.h>
#include <string> 

ros::Publisher ground_error_pub;
ros::Publisher calibration_error_pub;
ros::Publisher recalibrate_pub;
ros::Publisher cloudx_pub_1;
ros::Publisher cloudx_pub_2;
ros::Publisher cloudx_pub_3;
ros::Publisher image_pub;
ros::Publisher cancel_pub;
ros::Publisher move_pub;
ros::Publisher velocity_pub;
rosbag::Bag rot_bag, trans_bag, find_bag;
bool flag_stopped = true;
bool x_orth, y_orth, z_orth = false;
bool x_par, y_par, z_par = false;
using namespace icp;
using namespace std;
string pertubation = "0_1_0_0_0_2_";
int num_passes = 0;
double extrinsics[7] = {0, 0, 0, 1, -.087, -.0125, .2870};
double current_pose[7];
double last_pose[7];
double odom_time;
double cloud_time;
double odom_time_last;
double cloud_time_last;
pcl::PointCloud<pcl::PointXYZ> global_cloud;
pcl::PointCloud<pcl::PointXYZ> last_cloud;
pcl::PointCloud<pcl::PointXYZ> ground_cloud;
double global_min;
typedef actionlib::SimpleActionClient<frontier_exploration::ExploreTaskAction> Client;
enum Mode{monitor, record, record_rot, record_trans, find_scene, calibrate};

Mode mode = monitor;
vector<vector<double> > moves;
void HandleStop(int i) {
  printf("\nTerminating.\n");
  exit(0);
}

template<typename T>
void PopFront(std::vector<T>* vec)
{
  assert(!vec.empty());
  vec.erase(vec.begin());
}


void DeltasFromFile(vector<vector<double> >* sensor_deltas,
                    vector<vector<double> >* odom_deltas) {
  string delta_file = "brass.pose";
}

Eigen::Vector4d RotateQuaternion(Eigen::Vector4d q1, double* q2) {
    double w = q1[3] * q2[3] - q1[0] * q2[0] - q1[1] * q2[1] - q1[2] * q2[2];
    double x = q1[3] * q2[0] + q1[0] * q2[3] + q1[1] * q2[2] - q1[2] * q2[1];
    double y = q1[3] * q2[1] + q1[1] * q2[3] + q1[2] * q2[0] - q1[0] * q2[2];
    double z = q1[3] * q2[2] + q1[2] * q2[3] + q1[0] * q2[1] - q1[1] * q2[0];
    Eigen::Vector4d out = {x, y, z, w};
    return out;
}

double QuaternionDifference(Eigen::Vector4d q1, Eigen::Vector4d q2) {
  double q3[4] = {-q2[0], -q2[1], -q2[2], q2[3]};
  Eigen::Vector4d diff = RotateQuaternion(q1, q3);
  return 2 * atan2(diff.norm(), diff[3]);
}

bool DoubleEquals(double x, double y) {
  return fabs(x-y) < .3;
}

bool DoubleEquals2(double x, double y) {
//   cout << fabs(x-y) << endl;
  return fabs(x-y) < .05;
}

double* Quat2AA(double* quat) {
  double qw = quat[3];
  double qx = quat[0];
  double qy = quat[1];
  double qz = quat[2];
  double angle = 2 * acos(qw);
  double x;
  double y;
  double z;
  if (qw != 1) {
    x = qx / sqrt(1-qw*qw);
    y = qy / sqrt(1-qw*qw);
    z = qz / sqrt(1-qw*qw);
  }
  else {
      x = qx;
      y = qy;
      z = qz;
  }
  double* aa = new double[7];
  aa[0] = angle * x;
  aa[1] = angle * y;
  aa[2] = angle * z;
  aa[3] = quat[4];
  aa[4] = quat[5];
  aa[5] = quat[6];
  return aa;
}

double* AA2Quat(double* AA) {
  Eigen::Vector3d aa= {AA[0], AA[1], AA[2]};
  double* q = new double[6];
  double angle = aa.norm();
  if( angle) {
    q[0] = 0;
    q[1] = 0;
    q[2] = 0;
    q[3] = 1;
  } else {
    Eigen::Vector3d  axis = aa / angle;
    q[0] = sin(angle/2) * axis[0];
    q[1] = sin(angle/2) * axis[0];
    q[2] = sin(angle/2) * axis[0];
    q[3] = cos(angle/2);
  }
  q[4] = AA[3];
  q[5] = AA[4];
  q[6] = AA[5];
  return q;
}

void PrintPose(double* pose){
  for(uint i = 0; i < 6; i ++) {
    cout << pose[i] << " ";
  }
  cout << endl;
}

double* DeltaFromOdom(double* current, 
                      double* previous) {
  Eigen::Transform<double, 3, Eigen::Affine> current_transform;
  Eigen::Transform<double, 3, Eigen::Affine> previous_transform;
  // For the two poses create a transform
  double* posek = new double[6];
  
  Eigen::Vector3d cur_translation;
  cur_translation[0] = current[4];
  cur_translation[1] = current[5];
  cur_translation[2] = current[6];
  
    // Build the affine transforms for the previous pose
  Eigen::Quaternion<double> previous_quat(previous[3], previous[0], previous[1], previous[2]);
  Eigen::Translation<double, 3> previous_translation =
      Eigen::Translation<double, 3>(previous[4], previous[5], previous[6]);
  
  // Build the affine transforms based on the current pose
  Eigen::Quaternion<double> current_quat(current[3], current[0], current[1], current[2]);
  
  // Calculate delta rotation
  Eigen::Quaternion<double> rotation = previous_quat.inverse() * current_quat;
  
  Eigen::Quaternion<double> extrinsic_quat(extrinsics[3], extrinsics[0], extrinsics[1], extrinsics[2]);
  
  Eigen::Translation<double, 3> translation =
  Eigen::Translation<double, 3>(-cur_translation[0] + previous_translation.x(), -cur_translation[1] + previous_translation.y(), -cur_translation[2] + previous_translation.z());
  Eigen::Transform<double, 3, Eigen::Affine> transform =
      translation * rotation;
      
  transform = transform.inverse();
  Eigen::AngleAxis<double> test(transform.rotation());
  // Get the axis
  Eigen::Vector3d test_axis = test.axis();
  // Recompute the rotation angle
  double test_angle = test.angle();
  Eigen::Vector3d test_trans = test_axis * test_angle;
  cout << test_trans[0] << " " << test_trans[1] << " " << test_trans[2] << endl;
  transform = transform;
      
  // Find the rotation component
  // Find the angle axis format
      Eigen::AngleAxis<double> angle_axis(transform.rotation());
  Eigen::Quaternion<double> final_quat(transform.rotation());
  // Get the axis
  Eigen::Vector3d normal_axis = angle_axis.axis();
  
  // Recompute the rotation angle
  double combined_angle = angle_axis.angle();
  Eigen::Vector3d combined_axis = normal_axis * combined_angle;
  
  // Compute Translation
  Eigen::Translation<double, 3> combined_translation(
    transform.translation());
  Eigen::Vector3d combined_trans = {combined_translation.x(), combined_translation.y(), combined_translation.z()};
  combined_trans = current_quat.inverse() * combined_trans;
  posek[3] = -combined_trans[0];
  posek[4] = -combined_trans[1];
  posek[5] = -combined_trans[2];

  // Recompute the rotation angle
  posek[0] = combined_axis[0];
  posek[1] = combined_axis[1];
  posek[2] = combined_axis[2];

  return posek;
}

void CheckCalibration() {
  vector<vector<double> > sensor_deltas, odom_deltas;
  DeltasFromFile(&sensor_deltas, &odom_deltas);
  double trans_average = 0;
  double rot_average = 0;
  for(size_t i = 0; i < sensor_deltas.size(); i++) {
    vector<double> current_sensor = sensor_deltas[i];
    vector<double> current_odom = odom_deltas[i];
    Eigen::Vector3d odom_trans = {current_odom[4], current_odom[5], current_odom[6]};
    Eigen::Vector4d odom_rot = {current_odom[0], current_odom[1], current_odom[2], current_odom[3]};
    // Transform the sensor delta 
    Eigen::Vector3d sensor_trans = {current_sensor[4], current_sensor[5], current_sensor[6]};
    Eigen::Vector4d sensor_rot = {current_sensor[0], current_sensor[1], current_sensor[2], current_sensor[3]};
    sensor_trans = TransformPointQuaternion(sensor_trans, extrinsics);
    sensor_rot = RotateQuaternion(sensor_rot, extrinsics);
    // Find the difference between it and the odom delta
    Eigen::Vector3d trans_diff = sensor_trans - odom_trans;
    
    // Add to the average
    trans_average += abs(trans_diff.norm());
    rot_average += abs(QuaternionDifference(sensor_rot, odom_rot));
  }
  trans_average = trans_average / sensor_deltas.size();
  rot_average = rot_average / sensor_deltas.size();
  
  std_msgs::Float32MultiArray error_msg;
  vector<float> message = {(float)trans_average, (float)rot_average};
  error_msg.data = message;
  calibration_error_pub.publish(error_msg);
}

//This name is terrible, fix it Jarrett
double* TransformTransform(double* base_transform, double* transform) {
  //Create the eigen transform from the first pose
  Eigen::Matrix<double,3,1> axis(transform[0], transform[1], transform[2]);
  const double angle = axis.norm();
  if(angle != 0) {
    axis = axis / angle;
  }
  Eigen::Transform<double, 3, Eigen::Affine> rotation =
  Eigen::Transform<double, 3, Eigen::Affine>(Eigen::AngleAxis<double>(
    angle, axis));
  
  Eigen::Matrix<double,3,1> axis2(base_transform[0], base_transform[1], base_transform[2]);
  const double angle2 = axis2.norm();
  if(angle2 != 0) {
    axis2 = axis2 / angle2;
  }
  Eigen::Transform<double, 3, Eigen::Affine> rotation2 =
  Eigen::Transform<double, 3, Eigen::Affine>(Eigen::AngleAxis<double>(
    angle2, axis2));
  
  // T2 = R^-1 (R_1T + T_1 - T)
  Vector3d ext_tran = {transform[3], transform[4], transform[5]};
  Vector3d rot_vec = {base_transform[0], base_transform[1], base_transform[2]};
  Vector3d trans_vec = {base_transform[3], base_transform[4], base_transform[5]};
  rot_vec = rotation.inverse() * rot_vec;
  trans_vec = trans_vec + (rotation2 * ext_tran) - ext_tran;
  trans_vec = rotation.inverse() * trans_vec;
  double* ret_val = new double[6];
  ret_val[0] = rot_vec[0];
  ret_val[1] = rot_vec[1];
  ret_val[2] = rot_vec[2];
  ret_val[3] = trans_vec[0];
  ret_val[4] = trans_vec[1];
  ret_val[5] = trans_vec[2];
  return ret_val;
}

void DeltaErr() {
  
  if(DoubleEquals2(cloud_time, odom_time) && DoubleEquals2(cloud_time_last, odom_time_last)) {  
    
    double* transform = DeltaFromOdom(last_pose, current_pose);
    pcl::PointCloud<pcl::PointXYZ> cloud = last_cloud;
    pcl::PointCloud<pcl::Normal> normals_1 = GetNormals(cloud);
    pcl::PointCloud<pcl::Normal> normals_2 = GetNormals(global_cloud);
    double* ext = Quat2AA(extrinsics);
    double* combined = TransformTransform(transform, ext);
    TransformPointCloud(&cloud, combined);
    vector<int > nearest_neigbors;
    vector<int > start_points;
    vector<Eigen::Vector2d> image_coords_1;
    vector<Eigen::Vector2d> image_coords_2;
    double* calculated_delta = new double[6];
    // Initialize transform ARRAYS
    vector<double> pose0(6, 0.0);
    std::copy(pose0.begin(), pose0.end(), calculated_delta);
    vector<Eigen::Vector2d> empty_coords;
    vector<ros::Publisher> publishers;
    publishers.push_back(cloudx_pub_1);
    publishers.push_back(cloudx_pub_2);
    publishers.push_back(cloudx_pub_3);
    ICP (10,
       .05,
       publishers,
       "",
       "",
       last_cloud,
       global_cloud,
       normals_1,
       normals_2,
       empty_coords,
       empty_coords,
       calculated_delta,
       NULL);
    vector<Eigen::Vector4d> plane_normals;
    vector<Eigen::Vector3d> plane_centroids;
    delta_calc::ExtractPlanes(cloud, &plane_normals, &plane_centroids);
    
    Eigen::Vector3d uncertainty_t;
    Eigen::Vector3d uncertainty_r;
    delta_calc::ExtractUncertainty(plane_normals, &uncertainty_t, &uncertainty_r);
    delta_calc::StripUncertainty(uncertainty_t, uncertainty_r, calculated_delta);
    delta_calc::StripUncertainty(uncertainty_t, uncertainty_r, combined);

    Eigen::Matrix<double, 4, 1> error = TransformDifference(combined, calculated_delta);
    const unsigned int data_sz = 4;
    std_msgs::Float32MultiArray m;
    
    m.layout.dim.push_back(std_msgs::MultiArrayDimension());
    m.layout.dim[0].size = data_sz;
    
    // only needed if you don't want to use push_back
    m.data.resize(data_sz);
    m.data[0] = error[0];
    m.data[1] = error[1];
    m.data[2] = error[2];
    m.data[3] = error[3];
    calibration_error_pub.publish(m);
  }
}

double VectorAngle(Eigen::Vector3d vec1, Eigen::Vector3d vec2) {
  double dot =  vec1[0]*vec2[0] + vec1[2]*vec2[2] + vec1[3]*vec2[3];
  double lenSq1 = vec1[0]*vec1[0] + vec1[1]*vec1[1] + vec1[2]*vec1[2];
  double lenSq2 = vec2[0]*vec2[0] + vec2[1]*vec2[1] + vec2[2]*vec2[2];
  double angle = acos(dot/sqrt(lenSq1 * lenSq2));
  if(angle > 1.57) {
    angle = 3.14 - abs(angle);
  }
  return angle;
}

bool IsLow(int index) {
  pcl::PointXYZ point = global_cloud[index];
  if(point.z > global_min) {
    return true;
  } 
  return false;
}

vector<int> GetNeighborsPCL(pcl::PointCloud<pcl::PointXYZ> cloud, 
                            pcl::PointXYZ point,
                            int K) {
  
  pcl::KdTreeFLANN<pcl::PointXYZ> kdtree;
  pcl::PointCloud<pcl::PointXYZ>::Ptr ptr_cloud (new pcl::PointCloud<pcl::PointXYZ>);
  ptr_cloud = cloud.makeShared();
  kdtree.setInputCloud (ptr_cloud);
  
  // K nearest neighbor search
  std::vector<int> pointIdxNKNSearch(K);
  std::vector<float> pointNKNSquaredDistance(K);
  
  kdtree.nearestKSearch (point, K, pointIdxNKNSearch, pointNKNSquaredDistance);
//   global_cloud = cloud;
  global_min = point.z;
  std::remove_if (pointIdxNKNSearch.begin(), pointIdxNKNSearch.end(), IsLow); 
  return pointIdxNKNSearch;
}

void OrientCloud(pcl::PointCloud<pcl::PointXYZ>* cloud) {
  for(size_t i = 0; i < cloud->size(); ++i) {
    pcl::PointXYZ point = (*cloud)[i];
    pcl::PointXYZ point2;
    point2.x = point.z;
    point2.y = -point.x;
    point2.z = -point.y;
    (*cloud)[i] = point2;
  }
}

void CheckGroundPlane(const pcl::PointCloud<pcl::PointXYZ>& pcl_cloud) {
  
  // Extract Planes
  vector<Eigen::Vector4d> plane_normals;
  vector<Eigen::Vector3d> plane_centroids;
  delta_calc::ExtractPlanes(pcl_cloud, &plane_normals, &plane_centroids);
  
  // Get Lowest Plane
  double lowest_value = 500;
  Eigen::Vector4d ground_normal;
  for(size_t i = 0; i < plane_centroids.size(); ++i) {
      if(plane_centroids[i][2] < lowest_value) {
        lowest_value = plane_centroids[i][2];
        ground_normal = plane_normals[i];
      }
  }
  
  // Compare that normal to the z normal
  Eigen::Vector3d z_axis = {0, 0, 1};
  Eigen::Vector3d normal_axis = {ground_normal[0], ground_normal[1], ground_normal[2]};
  double angle = VectorAngle(z_axis, normal_axis);
  double offset = 0 - lowest_value;
  // Take the angle between the two and publish it as the error
  const unsigned int data_sz = 2;
  std_msgs::Float32MultiArray m;
  
  m.layout.dim.push_back(std_msgs::MultiArrayDimension());
  m.layout.dim[0].size = data_sz;
  
  // only needed if you don't want to use push_back
  m.data.resize(data_sz);
  m.data[0] = angle;
  m.data[1] = offset;
  ground_cloud = pcl_cloud;
  ground_error_pub.publish(m);
}


void GroundRecalibrate() {
  // Extract Planes
  vector<Eigen::Vector4d> plane_normals;
  vector<Eigen::Vector3d> plane_centroids;
  delta_calc::ExtractPlanes(ground_cloud, &plane_normals, &plane_centroids);
  
  // Get Lowest Plane
  double lowest_value = 500;
  Eigen::Vector4d ground_normal;
  for(size_t i = 0; i < plane_centroids.size(); ++i) {
    if(plane_centroids[i][2] < lowest_value) {
      lowest_value = plane_centroids[i][2];
      ground_normal = plane_normals[i];
    }
  }
  
  // Compare that normal to the z normal
  Eigen::Vector3d z_axis = {0, 0, 1};
  Eigen::Vector3d normal_axis = {ground_normal[0], ground_normal[1], ground_normal[2]};
  Vector3d rotation_axis = normal_axis.cross(z_axis);
  double angle = VectorAngle(normal_axis, z_axis);
  Vector3d aa = angle * rotation_axis;
  
  double offset = 0 - lowest_value;
  double new_transf[6] = {aa[0], aa[1], aa[2], 0, 0, offset};
  double* aa_extr = Quat2AA(extrinsics);
  double* new_cal = CombineTransform(aa_extr, new_transf);
  
  // Save new extrinsics
  double* new_quat = AA2Quat(new_cal);
  extrinsics[0] = new_quat[0];
  extrinsics[1] = new_quat[1];
  extrinsics[2] = new_quat[2];
  extrinsics[3] = new_quat[3];
  extrinsics[4] = new_quat[4];
  extrinsics[5] = new_quat[5];
  extrinsics[6] = new_quat[6];
  geometry_msgs::PoseStamped message;
  message.pose.position.x = extrinsics[4];
  message.pose.position.y = extrinsics[5];
  message.pose.position.z = current_pose[6];
  message.pose.orientation.x = extrinsics[0];
  message.pose.orientation.y = extrinsics[1];
  message.pose.orientation.z = extrinsics[2];
  message.pose.orientation.w = extrinsics[3];
  message.header.stamp = ros::Time::now();
  message.header.frame_id = "map";
  recalibrate_pub.publish(message);
}

void MakeMove(vector<double> move) {
  geometry_msgs::PoseStamped message;
  message.pose.position.x = current_pose[4] + move[4];
  message.pose.position.y = current_pose[5] + move[5];
  message.pose.position.z = current_pose[6] + move[6];
  message.pose.orientation.x = current_pose[0] + move[0];
  message.pose.orientation.y = current_pose[1] + move[1];
  message.pose.orientation.z = current_pose[2] + move[2];
  message.pose.orientation.w = current_pose[3] + move[3];
  message.header.stamp = ros::Time::now();
  message.header.frame_id = "map";
  move_pub.publish(message);
}

void* full_turtlebot_record(void *arg) {
  geometry_msgs::Twist message;
  message.linear.x = 0;
  message.linear.y = 0;
  message.linear.z = 0;
  message.angular.x = 0;
  message.angular.y = 0;
  message.angular.z = 0;
  
  message.linear.x = .2;
  for(int i = 0; i < num_passes; i++) {
    for(int i = 0; i < 1; i++) {
      velocity_pub.publish(message);
      sleep(3);
    }
    sleep(2);
    message.linear.x = -message.linear.x ;
  }
  message.angular.z = .5;
  message.linear.x = 0;
  mode = record_rot;
  for(int i = 0; i < num_passes; i++) {
    for(int i = 0; i < 5; i++) {
      cout << "Velocity published" << endl;
      velocity_pub.publish(message);
    }
    sleep(2);
    message.angular.z = -message.angular.z;
  }
//   rot_bag.close();
//   trans_bag.close();
  mode = find_scene;
  pthread_exit(NULL);
}

void* part_turtlebot_record(void *arg) {
  geometry_msgs::Twist message;
  message.linear.x = 0;
  message.linear.y = 0;
  message.linear.z = 0;
  message.angular.x = 0;
  message.angular.y = 0;
  message.angular.z = 0;
  
  message.angular.z = .5;
  message.linear.x = 0;
  mode = record_rot;
  for(int i = 0; i < num_passes; i++) {
    for(int i = 0; i < 5; i++) {
      cout << "Velocity published" << endl;
      velocity_pub.publish(message);
    }
    sleep(2);
    message.angular.z = -message.angular.z;
  }
  mode = record_trans;
  message.angular.z = 0;
  message.linear.x = .2;
  for(int i = 0; i < num_passes; i++) {
    for(int i = 0; i < 1; i++) {
      velocity_pub.publish(message);
      sleep(3);
    }
    sleep(2);
    message.linear.x = -message.linear.x ;
  }
  
  
//   rot_bag.close();
//   trans_bag.close();
  mode = find_scene;
  pthread_exit(NULL);
}

void CheckNormals(pcl::PointCloud<pcl::Normal> normals) {
  bool new_info = false;
  Eigen::Vector3d x = {1, 0, 0};
  Eigen::Vector3d y = {0, 1, 0};
  Eigen::Vector3d z = {0, 0, 1};
  int x_p_count = 0;
  int y_p_count = 0;
  int z_p_count = 0;
  int x_o_count = 0;
  int y_o_count = 0;
  int z_o_count = 0;
  for(size_t i = 0; i < normals.size(); i++){
    Eigen::Vector3d normal = {normals[i].normal_x, normals[i].normal_y, normals[i].normal_z};
    normal.normalize();
    double x_dot = abs(normal.dot(x));
    double y_dot = abs(normal.dot(y));
    double z_dot = abs(normal.dot(z));
    if(!DoubleEquals(x_dot, 0)) {
      x_o_count += 1;
    }
    if(!DoubleEquals(x_dot, 1)) {
      x_p_count += 1;
    }
    if(!DoubleEquals(y_dot, 0)) {
      y_o_count += 1;
    }
    if(!DoubleEquals(y_dot, 1)) {
      y_p_count += 1;
    }
    if(!DoubleEquals(z_dot, 0)) {
      z_o_count += 1;
    }
    if(!DoubleEquals(z_dot, 1)) {
      z_p_count += 1;
    }
  }
  if(!x_orth && x_o_count > 10000) {
    new_info = true;
    x_orth = true;
  }
  if(!y_orth && y_o_count > 10000) {
    new_info = true;
    y_orth = true;
  }
  if(!z_orth && z_o_count > 10000) {
    new_info = true;
    z_orth = true;
  }
  if(!x_par && x_p_count > 10000) {
    new_info = true;
    x_par = true;
  }
  if(!y_par && y_p_count > 10000) {
    new_info = true;
    y_par = true;
  }
  if(!z_par && z_p_count > 10000) {
    new_info = true;
    z_par = true;
  }
  cout << "x_o_count: " << x_o_count << endl;
  cout << "y_o_count: " << y_o_count << endl;
  cout << "z_o_count: " << z_o_count << endl;
  cout << "x_p_count: " << x_p_count << endl;
  cout << "y_p_count: " << y_p_count << endl;
  cout << "z_p_count: " << z_p_count << endl;
  if(new_info) {
    flag_stopped= true;
    actionlib_msgs::GoalID message;
    message.id = "";
    cancel_pub.publish(message);
    mode = record_trans;
    pthread_t drive_thread;
    num_passes = 2;
    int ret =  pthread_create(&drive_thread, NULL, &part_turtlebot_record, NULL);
    if(ret != 0) {
            printf("Error: pthread_create() failed\n");
            exit(EXIT_FAILURE);
    }
  }
}

bool AllInfo() {
  return x_orth && y_orth && z_orth && x_par && y_par && z_par;
}

void UnsetInfo() {
  rot_bag.open("recorded_data_rot.bag", rosbag::bagmode::Write);
  trans_bag.open("recorded_data_trans.bag", rosbag::bagmode::Write);
  rot_bag.close();
  trans_bag.close();
  x_orth = y_orth = z_orth = x_par = y_par = z_par = false;
}

void SetAll() {
  x_orth = y_orth = z_orth = x_par = y_par = z_par = true;
}

void CheckSceneInformation(const pcl::PointCloud<pcl::PointXYZ>& pcl_cloud) {
  pcl::PointCloud<pcl::Normal> normals = GetNormals(pcl_cloud);
  Eigen::Matrix3d scatter = CalcScatterMatrix(normals);
  Eigen::Matrix3d m;
  m << 1, 0, 0,
       0, 1, 0,
       0, 0, 1;
  double condition = CalcConditionNumber(scatter);
  cout << condition << endl;
  if(condition < 15) {
    cout << "Full Record" << endl;
    SetAll();
    actionlib_msgs::GoalID message;
    message.id = "";
    cancel_pub.publish(message);
    mode = record_trans;
    rot_bag.open("recorded_data_rot.bag", rosbag::bagmode::Write);
    trans_bag.open("recorded_data_trans.bag", rosbag::bagmode::Write);
    pthread_t drive_thread;
    num_passes = 5;
    int ret =  pthread_create(&drive_thread, NULL, &full_turtlebot_record, NULL);
    if(ret != 0) {
            printf("Error: pthread_create() failed\n");
            exit(EXIT_FAILURE);
    }
    cout << "pthread created" << endl;
  } else {
    cout << "Partial Record" << endl;
    rot_bag.open("recorded_data_rot.bag", rosbag::bagmode::Append);
    trans_bag.open("recorded_data_trans.bag", rosbag::bagmode::Append);
    CheckNormals(normals);
  }
}

void RecordDepth(sensor_msgs::PointCloud2 msg) {
  if(mode == record_rot) {
    rot_bag.write("/camera/depth/points", ros::Time::now(), msg);
  } else if(mode == record_trans) {
    trans_bag.write("/camera/depth/points", ros::Time::now(), msg);
  }
}

void RecordOdom(const nav_msgs::Odometry& msg) {
  if(mode == record_rot) {
      rot_bag.write("/odom", ros::Time::now(), msg);
    } else if(mode == record_trans) {
      trans_bag.write("/odom", ros::Time::now(), msg);
    }
}

pcl::PointCloud<pcl::PointXYZ> CutCloud(pcl::PointCloud<pcl::PointXYZ> cloud) {
  pcl::PointCloud<pcl::PointXYZ> return_cloud;
  for(size_t i = 0; i < cloud.size(); ++i) {
    if(cloud[i].x < 2) {
      return_cloud.points.push_back(cloud[i]);
    } else {
    }
  }
  return return_cloud;
}

void Recalibrate() {
  cout << pertubation << endl;
  double* new_cal = partial_calibrate::turtlebot_calibrate(pertubation);
  double* new_quat = AA2Quat(new_cal);
  PrintPose(new_cal);
  extrinsics[0] = new_quat[0];
  extrinsics[1] = new_quat[1];
  extrinsics[2] = new_quat[2];
  extrinsics[3] = new_quat[3];
  extrinsics[4] = new_quat[4] + extrinsics[4];
  extrinsics[5] = new_quat[5] + extrinsics[5];
  extrinsics[6] = extrinsics[6];
  geometry_msgs::PoseStamped message;
  message.pose.position.x = extrinsics[4];
  message.pose.position.y = extrinsics[5];
  message.pose.position.z = extrinsics[6];
  message.pose.orientation.x = extrinsics[0];
  message.pose.orientation.y = extrinsics[1];
  message.pose.orientation.z = extrinsics[2];
  message.pose.orientation.w = extrinsics[3];
  message.header.stamp = ros::Time::now();
  message.header.frame_id = "map";
  recalibrate_pub.publish(message);
}

void DepthCb(sensor_msgs::PointCloud2 msg) {
  cloud_time_last = cloud_time;
  cloud_time = msg.header.stamp.toSec();
  
  pcl::PointCloud<pcl::PointXYZ> pcl_cloud;
  pcl::PCLPointCloud2 pcl_pc2;
  pcl_conversions::toPCL(msg,pcl_pc2);
  pcl::fromPCLPointCloud2(pcl_pc2,pcl_cloud);
  pcl_cloud = VoxelFilter(pcl_cloud);
  OrientCloud(&pcl_cloud);
  pcl_cloud = CutCloud(pcl_cloud);
  pcl::copyPointCloud(global_cloud, last_cloud);
  pcl::copyPointCloud(pcl_cloud, global_cloud);
  
  
  // Pose will need to be adjusted based on robot position
  // I believe, so we'll need to save the most recent odometry message
  // we've received I guess.
  TransformPointCloudQuat(pcl_cloud, extrinsics);
  TransformPointCloudQuat(pcl_cloud, current_pose);
  
//   PublishCloud(pcl_cloud, cloudx_pub_1);
  
  
  if(mode == monitor) {
    CheckGroundPlane(pcl_cloud);
    DeltaErr();
  } else if(mode == find_scene) {
    if(!AllInfo()) {
      if(flag_stopped == true) {
        flag_stopped = false;
	cout << "starting explore" << endl;
        Client client("explore_server", true);
        client.waitForServer();
        frontier_exploration::ExploreTaskGoal goal;
        goal.explore_center.point.x = 0;
        goal.explore_center.point.y = 0;
        goal.explore_center.point.z = 0;
        goal.explore_center.header.frame_id = "/odom";
        goal.explore_boundary.header.frame_id = "/odom";
        client.sendGoal(goal);
      }
      if(rot_bag.getMode()) {
        rot_bag.close();
        trans_bag.close();
      }
      CheckSceneInformation(pcl_cloud);
    } else {
      rot_bag.close();
      trans_bag.close();
      vector<ros::Publisher> publishers;
      publishers.push_back(cloudx_pub_1);
      publishers.push_back(cloudx_pub_2);
      publishers.push_back(cloudx_pub_3);
      publishers.push_back(cloudx_pub_3);
      publishers.push_back(cloudx_pub_3);
      publishers.push_back(cloudx_pub_3);
      publishers.push_back(cloudx_pub_3);
      cout << "Calculating Deltas" << endl;
      delta_calc::DeltaCalculationBrass("recorded_data_trans",
                              publishers,
                              0,
                              INT_MAX);
      delta_calc::DeltaCalculationBrass("recorded_data_rot",
                              publishers,
                              1,
                              INT_MAX);
      partial_calibrate::turtlebot_calibrate("recorded_data");
      
    }
  } else if (mode == record_rot || mode == record_trans) {
    RecordDepth(msg);
  }
}

void OdomCb(const nav_msgs::Odometry& msg) {
  odom_time_last = odom_time;
  odom_time = msg.header.stamp.toSec();
  std::copy(std::begin(current_pose), std::end(current_pose), std::begin(last_pose));
  current_pose[0] = msg.pose.pose.orientation.x;
  current_pose[1] = msg.pose.pose.orientation.y;
  current_pose[2] = msg.pose.pose.orientation.z;
  current_pose[3] = msg.pose.pose.orientation.w;
  current_pose[4] = msg.pose.pose.position.x;
  current_pose[5] = msg.pose.pose.position.y;
  current_pose[6] = msg.pose.pose.position.z;
  if(mode == record_rot || mode == record_trans) {
    RecordOdom(msg);
  }
}

void PerturbCb(const std_msgs::Int32MultiArray& msg) {
  string s = "";
  for(size_t i = 0; i < 6; ++i) {
    s = s + std::to_string(msg.data[i]) + "_";
  }
  cout << s << endl;
  pertubation = s;
}

void CommandCb(const std_msgs::String::ConstPtr& msg)
{
  ROS_INFO("I heard: [%s]", msg->data.c_str());
  string recalibrate = "recalibrate";
  string ground_recalibrate = "ground_recalibrate";
  if(recalibrate.compare(msg->data.c_str()) == 0) {
    Recalibrate();
  }
  if(ground_recalibrate.compare(msg->data.c_str()) == 0) {
    GroundRecalibrate();
  }
  
}

void StatusCb(const actionlib_msgs::GoalStatusArray::ConstPtr& msg) {
  if(mode == record) {
    if(msg->status_list[msg->status_list.size() -1].status == 3) {
      cout << "Activating Move" << endl;
      sleep(10);
      geometry_msgs::PoseStamped message;
      vector<double> move = moves[0];
      moves.erase(moves.begin());
      message.pose.position.x = current_pose[4] + move[4];
      message.pose.position.y = current_pose[5] + move[5];
      message.pose.position.z = current_pose[6] + move[6];
      message.pose.orientation.x = current_pose[0] + move[0];
      message.pose.orientation.y = current_pose[1] + move[1];
      message.pose.orientation.z = current_pose[2] + move[2];
      message.pose.orientation.w = current_pose[3] + move[3];
      message.header.stamp = ros::Time::now();
      message.header.frame_id = "map";
      move_pub.publish(message);
    }
    
  }
}

int main(int argc, char **argv) {
  signal(SIGINT,HandleStop);
  signal(SIGALRM,HandleStop);
  ros::init(argc, argv, "calibration_watcher");

  ros::NodeHandle n;
  rot_bag.open("recorded_data_rot.bag", rosbag::bagmode::Write);
  trans_bag.open("recorded_data_trans.bag", rosbag::bagmode::Write);
  rot_bag.close();
  trans_bag.close();
  ground_error_pub = n.advertise<std_msgs::Float32MultiArray>("/calibration/ground_plane_error", 1000);
  calibration_error_pub = n.advertise<std_msgs::Float32MultiArray>("/calibration/calibration_error", 1000);
  cloudx_pub_1 = n.advertise<sensor_msgs::PointCloud2> ("cloud_pub_1", 1);
  cloudx_pub_2 = n.advertise<sensor_msgs::PointCloud2> ("cloud_pub_2", 1);
  cloudx_pub_3 = n.advertise<sensor_msgs::PointCloud2> ("cloud_pub_3", 1);
  image_pub = n.advertise<sensor_msgs::Image> ("image_pub", 1);
  cancel_pub = n.advertise<actionlib_msgs::GoalID> ("/explore_server/cancel", 1);
  move_pub = n.advertise<geometry_msgs::PoseStamped> ("/move_base_simple/goal", 1);
  recalibrate_pub = n.advertise<geometry_msgs::PoseStamped> ("/calibration/extrinsic_update", 1);
  velocity_pub = n.advertise<geometry_msgs::Twist> ("/mobile_base/commands/velocity", 1);
  ros::Subscriber depth_sub = n.subscribe("/camera/depth/points", 1, DepthCb);
  ros::Subscriber odom_sub = n.subscribe("/odom", 1, OdomCb);
  ros::Subscriber command_sub = n.subscribe("/calibration/commands", 1, CommandCb);
  ros::Subscriber pertubation_sub = n.subscribe("/calibration/perturb", 1, PerturbCb);
  ros::Subscriber status_sub = n.subscribe("/move_base/status", 1, StatusCb);
  current_pose[0] = 0;
  current_pose[1] = 0;
  current_pose[2] = 0;
  current_pose[3] = 0;
  current_pose[4] = 0;
  current_pose[5] = 0;
  current_pose[6] = 0;
  last_pose[0] = 0;
  last_pose[1] = 0;
  last_pose[2] = 0;
  last_pose[3] = 0;
  last_pose[4] = 0;
  last_pose[5] = 0;
  last_pose[6] = 0;
  ros::spin();

  return 0;
}
