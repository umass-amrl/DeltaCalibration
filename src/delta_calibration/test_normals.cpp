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

void testNormals(string bagfile) {
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
  
  bag_it = TimeAlignedClouds(bag_it, end,  &buffer_k1, &buffer_k2, &times_k1, 
          &times_k2,
          &keyframe_k1, &keyframe_k2, &timestamp_1, &timestamp_2);

  pcl::PointCloud<pcl::Normal> normals = GetNormals(keyframe_k1);
  visualization_msgs::Marker marker;
  marker.header.frame_id = "point_cloud";
  marker.id = 0;
  marker.header.stamp = ros::Time();
  marker.type = visualization_msgs::Marker::SPHERE_LIST;
  marker.action = visualization_msgs::Marker::ADD;
  marker.scale.x = .02;
  marker.scale.y = .02;
  marker.scale.z = .02;
  marker.color.r = 0;
  marker.color.a = 1;
  marker.color.b = 0;
  marker.color.g = 1;
  for(size_t i = 0; i < normals.size(); i++) {
    pcl::Normal normal = normals[i];
    double norm = 
        sqrt(pow(normal.normal_x,2) 
        + pow(normal.normal_y,2) + pow(normal.normal_z,2));
    double x = normal.normal_x / norm;
    double y = normal.normal_y / norm;
    double z = normal.normal_z / norm;
//     visualization_msgs::Marker marker;
//     marker.header.frame_id = "point_cloud";
//     marker.id = i;
//     marker.header.stamp = ros::Time();
//     marker.type = visualization_msgs::Marker::SPHERE;
//     marker.action = visualization_msgs::Marker::ADD;
//     marker.scale.x = .02;
//     marker.scale.y = .02;
//     marker.scale.z = .02;
//     marker.color.r = 0;
//     marker.color.a = 1;
//     marker.color.b = 0;
//     marker.color.g = 1;
//     marker.pose.position.x = x;
//     marker.pose.position.y = y;
//     marker.pose.position.z = z;
    geometry_msgs::Point point;
    point.x = x;
    point.y = y;
    point.z = z;
    marker.points.push_back(point);
  }
  int i = 0;
  while(true) {
    cout << "publishing" << endl;
    PublishCloud(keyframe_k1, cloud_pub);
    marker_pub.publish(marker);
    i++;
  }
}

int main(int argc, char **argv) {

  char* bagFile = (char*)"pair_upright.bag";
  // Parse arguments.
  static struct poptOption options[] = {
    { "use-bag-file" , 'B', POPT_ARG_STRING, &bagFile ,0, "Process bag file" ,
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
  ros::init(argc, argv, "normal_testing", ros::init_options::NoSigintHandler);
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
  n.advertise<visualization_msgs::MarkerArray>("visualization_marker_array", 
10);
  testNormals(bagFile);
  return 0;
}