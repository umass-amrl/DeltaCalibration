#include "delta_calibration/icp.h"
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/extract_indices.h>
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

using namespace icp; 
// Signal handler for breaks (Ctrl-C)
void HandleStop(int i) {
  printf("\nTerminating.\n");
  exit(0);
}

void GeometryTransform(geometry_msgs::Point* point, double* transform) {
  Eigen::Matrix<double,3,1> point_eig;
  point_eig[0] = point->x;
  point_eig[1] = point->y;
  point_eig[2] = point->z;
  point_eig = TransformPoint(point_eig, transform);
  point->x = point_eig[0];
  point->y = point_eig[1];
  point->z = point_eig[2];
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
    PublishCloud(*cloud_p, cloud_test_pub);
    PublishCloud(*cloud_f, model_cloud_pub);
    PublishCloud(*cloud_filtered, bmodel_cloud_pub);
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
    PublishCloud(plane1, cloud_pub);
    PublishCloud(plane2, cloud_pub_2);
    sleep(3);

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
  bag_it = TimeAlignedClouds(bag_it, end,  &buffer_k1, &buffer_k2, &times_k1, 
      &times_k2,
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

    bag_it = TimeAlignedClouds(bag_it, end, &buffer_k1, &buffer_k2, &times_k1, 
        &times_k2,
        &cloud_k1, &cloud_k2, &timestamp_1, &timestamp_2);
    pcl::PointCloud<pcl::PointXYZ> transformed_cloud = cloud_k2;
    WriteToObj("", out_name2, count, cloud_k1);
    WriteToObj("", out_name3, count, cloud_k2);
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
      PublishCloud(cloud_k1, cloud_pub);
      PublishCloud(cloud_k2, cloud_pub_2);
      PublishCloud(transformed_cloud, cloud_pub_3);
      //       TransformPointCloudInv(transformed_cloud, rotation);

      marker_pub.publish(line_list);
      line_list.points.clear();
    }
    pcl::PointCloud<pcl::PointXYZ> combo_cloud = cloud_k1;
  }
}

int main(int argc, char **argv) {
  signal(SIGINT,HandleStop);
  signal(SIGALRM,HandleStop);

  char* bagFile = (char*)"pair_upright.bag";
  char* transform_file = (char*)"";
  // Parse arguments.
  static struct poptOption options[] = {
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
  cout << "Using bagfile: " << bagFile << endl;
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

  CheckTransform(bagFile, transform_file);

  return 0;
}
