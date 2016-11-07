//----------- INCLUDES
// ROS INCLUDES
#include <nav_msgs/Odometry.h>
#include "delta_calibration/icp.h"

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

rosbag::View::iterator OdomFromBag(rosbag::View::iterator it, 
                                   rosbag::View::iterator end,
                                   vector<double>* keyframe_odom) {
   
    
    nav_msgs::OdometryPtr best_odom;
    const rosbag::MessageInstance &m = *it; 
    nav_msgs::OdometryPtr odomMsg = m.instantiate<nav_msgs::Odometry>();
    double time = odomMsg->header.stamp.toSec();
    best_odom = odomMsg;
    advance(it, 1);

    keyframe_odom->clear();
    keyframe_odom->push_back(time);
    keyframe_odom->push_back(best_odom->pose.pose.position.x);
    keyframe_odom->push_back(best_odom->pose.pose.position.y);
    keyframe_odom->push_back(best_odom->pose.pose.position.z);
    keyframe_odom->push_back(best_odom->pose.pose.orientation.x);
    keyframe_odom->push_back(best_odom->pose.pose.orientation.y);
    keyframe_odom->push_back(best_odom->pose.pose.orientation.z);
    keyframe_odom->push_back(best_odom->pose.pose.orientation.w);
    return it;
}


vector<double> TransformOdom(
    const vector<double>& odom,
    double* transform) {
  
  // Create the affine transform from the transform
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
  // Create the quaternion rotation and the translation from the odom
  Eigen::Quaternion<double> previous_quat(odom[7], odom[4], odom[5], odom[6]);
  Eigen::Translation<double, 3> previous_translation =
    Eigen::Translation<double, 3>(odom[1], odom[2], odom[3]);
  // Transform the components of the odometry by the transform and create a new output
  Eigen::Transform<double, 3, Eigen::Affine> rot = affine_transform * previous_quat;
  Eigen::Transform<double, 3, Eigen::Affine> trans = affine_transform * previous_translation;
  
  Eigen::Quaternion<double>* final_quat = new Eigen::Quaternion<double>(rot.rotation());
  
  Eigen::Translation<double, 3> final_translation(
    trans.translation());
  
  vector<double> output;
  output.push_back(final_translation.x());
  output.push_back(final_translation.y());
  output.push_back(final_translation.z());
  output.push_back(final_quat->x());
  output.push_back(final_quat->y());
  output.push_back(final_quat->z());
  output.push_back(final_quat->w());

  return output;
}

void visualize_poses(string bag_file, string transform_file, vector<ros::Publisher> publishers) { 
  // Get Transform from file
  double* transform = new double[6];
  double* start_kinect = new double[6];
  start_kinect[0] = 0;
  start_kinect[1] = 0;
  start_kinect[2] = 1.571;
  start_kinect[3] = 0;
  start_kinect[4] = 0;
  start_kinect[5] = 0;
  TransformFromFile(transform_file, transform);
  cout << " Finished reading transform " << endl;
  // Get the first pose from the bag file
  rosbag::Bag bag;
  bag.open(bag_file, rosbag::bagmode::Read);
  std::vector<std::string> topics;
  std::vector<std::string> odom_topics;
  odom_topics.push_back(std::string("/odom"));
  rosbag::View odom_view(bag, rosbag::TopicQuery(odom_topics));
  rosbag::View::iterator odom_it = odom_view.begin();
  rosbag::View::iterator odom_end = odom_view.end();
  
  vector<double> odom;
  odom_it = OdomFromBag(odom_it, odom_end, &odom);
  cout << "finished reading from bag" << endl;
  // Transform the pose based on the calculated extrinsic transform
  vector<double> sensor_pose = odom;
  
  sensor_pose = TransformOdom(sensor_pose, transform);
  
  cout << "finished transform" << endl;
  
//   //Visualize the original pose as the turtlebot, and the transformed pose as the kinect.
//   visualization_msgs::Marker base;
//   base.header.stamp = ros::Time();
//   base.header.frame_id = "point_cloud";
//   base.id = 0;
//   base.pose.position.x = odom[1];
//   base.pose.position.y = odom[2];
//   base.pose.position.z = odom[3];
//   base.type = visualization_msgs::Marker::ARROW;
//   base.action = visualization_msgs::Marker::ADD;
//   base.pose.orientation.x = odom[4];
//   base.pose.orientation.y = odom[5];
//   base.pose.orientation.z = odom[6];
//   base.pose.orientation.w = odom[7];
//   base.scale.x = 1;
//   base.scale.y = 0.1;
//   base.scale.z = 0.1;
//   base.color.a = 1.0; // Don't forget to set the alpha!
//   base.color.r = 0.0;
//   base.color.g = 1.0;
//   base.color.b = 0.0;
//   
  //Visualize the original pose as the turtlebot, and the transformed pose as the kinect.
  visualization_msgs::Marker base;
  base.header.stamp = ros::Time();
  base.header.frame_id = "point_cloud";
  base.id = 0;
  base.pose.position.x = odom[1];
  base.pose.position.y = odom[2];
  base.pose.position.z = odom[3];
  base.pose.orientation.x = odom[4];
  base.pose.orientation.y = odom[5];
  base.pose.orientation.z = odom[6];
  base.pose.orientation.w = odom[7];
  base.type = visualization_msgs::Marker::MESH_RESOURCE;
  base.action = visualization_msgs::Marker::ADD;
  base.mesh_resource = "package://kobuki_description/meshes/main_body.dae" ;
  base.color.a = 0.5; // Don't forget to set the alpha!
  base.scale.x = 1;
  base.scale.y = 1;
  base.scale.z = 1;
  visualization_msgs::Marker sensor;
  sensor.header.stamp = ros::Time();
  sensor.header.frame_id = "point_cloud";
  sensor.id = 1;
  sensor.pose.position.x = sensor_pose[0];
  sensor.pose.position.y = sensor_pose[1];
  sensor.pose.position.z = sensor_pose[2];
  sensor.pose.position.x = 0;
  sensor.pose.position.y = 0;
  sensor.pose.position.z = 0;
  sensor.type = visualization_msgs::Marker::MESH_RESOURCE;
  sensor.action = visualization_msgs::Marker::ADD;
  sensor.pose.orientation.x = sensor_pose[3];
  sensor.pose.orientation.y = sensor_pose[4];
  sensor.pose.orientation.z = sensor_pose[5];
  sensor.pose.orientation.w = sensor_pose[6];
  sensor.mesh_resource = "package://turtlebot_description/meshes/sensors/kinect.dae" ;
  sensor.scale.x = 1;
  sensor.scale.y = 1;
  sensor.scale.z = 1;
  sensor.color.a = 1.0; // Don't forget to set the alpha!
  visualization_msgs::Marker sensor_a;
  sensor_a.header.stamp = ros::Time();
  sensor_a.header.frame_id = "point_cloud";
  sensor_a.id = 2;
  sensor_a.pose.position.x = sensor_pose[0];
  sensor_a.pose.position.y = sensor_pose[1];
  sensor_a.pose.position.z = sensor_pose[2];
  sensor_a.type = visualization_msgs::Marker::ARROW;
  sensor_a.action = visualization_msgs::Marker::ADD;
  sensor_a.pose.orientation.x = sensor_pose[3];
  sensor_a.pose.orientation.y = sensor_pose[4];
  sensor_a.pose.orientation.z = sensor_pose[5];
  sensor_a.pose.orientation.w = sensor_pose[6];
  sensor_a.scale.x = .3;
  sensor_a.scale.y = 0.03;
  sensor_a.scale.z = 0.03;
  sensor_a.color.a = 1.0; // Don't forget to set the alpha!
  sensor_a.color.r = 0.0;
  sensor_a.color.g = 0.0;
  sensor_a.color.b = 1.0;
  visualization_msgs::MarkerArray marker_array = visualization_msgs::MarkerArray();
  marker_array.markers.push_back(base);
  marker_array.markers.push_back(sensor_a);
  while(true) {
    cout << "publish" << endl;
    markerArray_pub.publish(marker_array);
    sleep(3);
  }
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

  visualize_poses(bag_file, transform_file, publishers);
  
  return 0;
}