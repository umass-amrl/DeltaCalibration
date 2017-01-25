#include "ros/ros.h"
#include "std_msgs/String.h"
#include "std_msgs/Float32.h"
#include "std_msgs/Float32MultiArray.h"
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

ros::Publisher ground_error_pub;
ros::Publisher calibration_error_pub;
ros::Publisher cloud_pub_1;
ros::Publisher image_pub;
ros::Publisher cancel_pub;
ros::Publisher move_pub;
ros::Publisher velocity_pub;
rosbag::Bag bag;
using namespace icp;
using namespace std;
double extrinsics[7] = {0, 0, 0, 1, -.087, -.0125, .2870};
double current_pose[7];
pcl::PointCloud<pcl::PointXYZ> global_cloud;
double global_min;
typedef actionlib::SimpleActionClient<frontier_exploration::ExploreTaskAction> Client;
enum Mode{monitor, record, find_scene};

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
  global_cloud = cloud;
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
  pcl::PointXYZ min, max;
  pcl::getMinMax3D(pcl_cloud, min, max);
  
  min.x = 0;
  min.y = 0;
  // Get neighbors of lowest point
  int K = 50;
  std::vector<int> neighbor_indices = GetNeighborsPCL(pcl_cloud, min, K);
  
  // Calculate the normal for the lowest point
  Eigen::Vector4f plane_parameters;
  float curvature;
  pcl::computePointNormal(pcl_cloud, neighbor_indices, plane_parameters, curvature);
  
  
  // Compare that normal to the z normal
  Eigen::Vector3d z_axis = {0, 0, 1};
  Eigen::Vector3d normal_axis = {plane_parameters[0], plane_parameters[1], plane_parameters[2]};
  double angle = VectorAngle(z_axis, normal_axis);
  
  // Take the angle between the two and publish it as the error
  std_msgs::Float32 error_msg;
  error_msg.data = angle;
  ground_error_pub.publish(error_msg);
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

void* worker_thread(void *arg) {
  geometry_msgs::Twist message;
  message.linear.x = 0;
  message.linear.y = 0;
  message.linear.z = 0;
  message.angular.x = 0;
  message.angular.y = 0;
  message.angular.z = .2;
  
  for(int i = 0; i < 6; i++) {
    for(int i = 0; i < 3; i++) {
      cout << "Velocity published" << endl;
      velocity_pub.publish(message);
      sleep(3);
    }
    sleep(3);
    message.angular.z = -message.angular.z;
  }
  message.angular.z = 0;
  message.linear.x = .2;
  for(int i = 0; i < 6; i++) {
    for(int i = 0; i < 1; i++) {
      velocity_pub.publish(message);
      sleep(3);
    }
    sleep(3);
    message.linear.x = -message.linear.x ;
  }
  mode = monitor;
  bag.close();
  pthread_exit(NULL);
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
  if(condition < 5) {
    actionlib_msgs::GoalID message;
    message.id = "";
    cancel_pub.publish(message);
    mode = record;
    vector<double> move1 = {0, 0, 0, 1, 0, 0, 0};
    vector<double> move2 = {0, 0, 0, 0, 1, 0, 0};
    moves.push_back(move2);
    moves.push_back(move1);
    bag.open("recorded_data.bag", rosbag::bagmode::Write);
    pthread_t drive_thread;
    int ret =  pthread_create(&drive_thread, NULL, &worker_thread, NULL);
    if(ret != 0) {
            printf("Error: pthread_create() failed\n");
            exit(EXIT_FAILURE);
    }
    cout << "pthread created" << endl;
  }
}

void RecordDepth(const pcl::PointCloud<pcl::PointXYZ>& pcl_cloud) {
  cout << "record" << endl;
  sensor_msgs::PointCloud2 msg;
  pcl::PCLPointCloud2 pcl_pc2;
  pcl::toROSMsg(pcl_cloud, msg);
  bag.write("camera/depth/points", ros::Time::now(), msg);
}

void DepthCb(sensor_msgs::PointCloud2 msg) {
  pcl::PointCloud<pcl::PointXYZ> pcl_cloud;
  pcl::PCLPointCloud2 pcl_pc2;
  pcl_conversions::toPCL(msg,pcl_pc2);
  pcl::fromPCLPointCloud2(pcl_pc2,pcl_cloud);
  
  OrientCloud(&pcl_cloud);
  // Pose will need to be adjusted based on robot position
  // I believe, so we'll need to save the most recent odometry message
  // we've received I guess.
  TransformPointCloudQuat(pcl_cloud, current_pose);
  TransformPointCloudQuat(pcl_cloud, extrinsics);
  pcl_cloud = VoxelFilter(pcl_cloud);
  PublishCloud(pcl_cloud, cloud_pub_1);
  
  if(mode == monitor) {
    CheckGroundPlane(pcl_cloud);
  } else if(mode == find_scene) {
    CheckSceneInformation(pcl_cloud);
    
  } else if (mode == record) {
    
    RecordDepth(pcl_cloud);
  }
}

void OdomCb(const nav_msgs::Odometry& msg) {
  current_pose[0] = msg.pose.pose.orientation.x;
  current_pose[1] = msg.pose.pose.orientation.y;
  current_pose[2] = msg.pose.pose.orientation.z;
  current_pose[3] = msg.pose.pose.orientation.w;
  current_pose[4] = msg.pose.pose.position.x;
  current_pose[5] = msg.pose.pose.position.y;
  current_pose[6] = msg.pose.pose.position.z;
}

void CommandCb(const std_msgs::String::ConstPtr& msg)
{
  ROS_INFO("I heard: [%s]", msg->data.c_str());
  string scene_find = "find_scene";
  if(scene_find.compare(msg->data.c_str()) == 0) {
    mode = find_scene;
    Client client("explore_server", true);
    client.waitForServer();
    frontier_exploration::ExploreTaskGoal goal;
    goal.explore_center.point.x = 0;
    goal.explore_center.point.y = 0;
    goal.explore_center.point.z = 0;
    goal.explore_center.header.frame_id = "map";
    goal.explore_boundary.header.frame_id = "map";
    client.sendGoal(goal);
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
  ground_error_pub = n.advertise<std_msgs::Float32>("/calibration/ground_plane_error", 1000);
  calibration_error_pub = n.advertise<std_msgs::Float32MultiArray>("/calibration/calibration_error", 1000);
  cloud_pub_1 = n.advertise<sensor_msgs::PointCloud2> ("cloud_pub_1", 1);
  image_pub = n.advertise<sensor_msgs::Image> ("image_pub", 1);
  cancel_pub = n.advertise<actionlib_msgs::GoalID> ("/explore_server/cancel", 1);
  move_pub = n.advertise<geometry_msgs::PoseStamped> ("/move_base_simple/goal", 1);
  velocity_pub = n.advertise<geometry_msgs::Twist> ("/mobile_base/commands/velocity", 1);
  ros::Subscriber depth_sub = n.subscribe("/camera/depth/points", 1, DepthCb);
  ros::Subscriber odom_sub = n.subscribe("/odom", 1, OdomCb);
  ros::Subscriber command_sub = n.subscribe("/calibration_commands", 1, CommandCb);
  ros::Subscriber status_sub = n.subscribe("/move_base/status", 1, StatusCb);
  current_pose[0] = 0;
  current_pose[1] = 0;
  current_pose[2] = 0;
  current_pose[3] = 0;
  current_pose[4] = 0;
  current_pose[5] = 0;
  current_pose[6] = 0;
  ros::spin();

  return 0;
}