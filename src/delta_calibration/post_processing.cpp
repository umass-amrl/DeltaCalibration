//----------- INCLUDES
#include <string>
#include <sstream>
#include <iostream>
#include <fstream>
#include <iterator>
#include <algorithm>
#include <cmath>
#include <math.h>
#include <QApplication>
#include <QWidget>
#include <QHBoxLayout>
#include <QSlider>
#include <QSpinBox>
#include <QLabel>
#include <QCloseEvent>
// #include "slider.h"

// ROS INCLUDES
#include <ros/ros.h>
#include <rosbag/bag.h>
#include <rosbag/view.h>
#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/PointCloud2.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

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

const float nn_dist = .05;
float neighbor_dist = .5;

double* transform_global;
double* final_transform_global;
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

Eigen::Vector3d free_vector;
pcl::PointCloud<pcl::PointXYZ> transformed_global, base_global;

// Signal handler for breaks (Ctrl-C)
void HandleStop(int i) {
  printf("\nTerminating.\n");
  exit(0);
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
  while (num_planes < 4)
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

void ComparePlanes(vector<pcl::PointCloud<pcl::PointXYZ> > k1_planes,
                   vector<pcl::PointCloud<pcl::PointXYZ> > transformed_planes,
                   vector<Eigen::Vector4d> normal_equations,
                   vector<Eigen::Vector4d> normal_equations_trans) {
  
  
  Eigen::Vector4d plane_equation_1;
  Eigen::Vector4d plane_equation_2;
  for(size_t i = 0; i < k1_planes.size(); ++i) {
    cout << "I: " << i << endl;
    pcl::PointCloud<pcl::PointXYZ> plane1 = k1_planes[i];
    plane_equation_1 = normal_equations[i];
    for(size_t k = 0; k < k1_planes.size(); ++k) {
      cout << "K: " << k  << endl;
      pcl::PointCloud<pcl::PointXYZ> plane2 = transformed_planes[k];
      plane_equation_2 = normal_equations_trans[k];
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
      for(size_t j = 0; j < plane1.size(); ++j) {
        px1 += plane1[j].x;
        py1 += plane1[j].y;
        pz1 += plane1[j].z;
      }
      for(size_t j = 0; j < plane2.size(); ++j) {
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
      cout << "Offset residuals: " << endl;
      cout <<  (p1_mat - p2_mat).dot(norm_k) << endl;
      cout << (p2_mat - p1_mat).dot(norm_k1) << endl;
      cout <<  (p1_mat - p2_mat).dot(norm_k) / (1 + ((p1_mat[0] + p2_mat[0]) / 2)) << endl;
      cout << (p2_mat - p1_mat).dot(norm_k1) / (1 +((p1_mat[0] + p2_mat[0]) / 2)) << endl;
      double dot = plane_equation_1[0]*plane_equation_2[0] +\
      plane_equation_1[1]*plane_equation_2[1] +\
      plane_equation_1[2]*plane_equation_2[2];
      double lenSq1 = plane_equation_1[0]*plane_equation_1[0] + plane_equation_1[1]*plane_equation_1[1] + plane_equation_1[2]*plane_equation_1[2];
      double lenSq2 =plane_equation_2[0]*plane_equation_2[0] + plane_equation_2[1]*plane_equation_2[1] + plane_equation_2[2]*plane_equation_2[2       ];
      double angle = acos(dot/sqrt(lenSq1 * lenSq2));
      cout << "Angle between plane normals " << i << " : " << angle << endl; 
      double d_diff = abs(plane_equation_1[3] - plane_equation_2[3]);
      cout << "D Difference between planes " << i << " : " << d_diff << endl;
    }
  }
}

// Used to adjust the unknown degree of freedom.
void AdjustTranslation(pcl::PointCloud<pcl::PointXYZ> cloud_1,
                       pcl::PointCloud<pcl::PointXYZ> cloud_2,
                       vector<Eigen::Vector4d> normal_equations,
                       vector<Eigen::Vector4d> normal_equations_trans,
                       const QApplication& app) {
    fprintf(stdout, "Adjust translation\n");
    // Calculate the translation vector
    fprintf(stdout, "Plane 1\n");
    Eigen::Vector3d plane_1 = Eigen::Matrix<double, 3, 1>(
        normal_equations_trans[0][0],
        normal_equations_trans[0][1],
        normal_equations_trans[0][2]);
    fprintf(stdout, "Plane 2\n");
    Eigen::Vector3d plane_2 = Eigen::Matrix<double, 3, 1>(
        normal_equations_trans[1][0],
        normal_equations_trans[1][1],
        normal_equations_trans[1][2]);
    fprintf(stdout, "Calculate Free vector\n");
    free_vector = plane_2.cross(plane_1);
    
    // Perform the actual translation
    transformed_global = cloud_2;
    pcl::PointCloud<pcl::PointXYZ> transformed_cloud = transformed_global;
    fprintf(stdout, "Vector Translate Point Cloud\n");
//     VectorTranslatePointCloud(transformed_cloud, 1, free_vector);
    PublishCloud(transformed_cloud, cloud_pub_2);
    PublishCloud(cloud_1, cloud_pub);
    app.exec();
}
  
  void WriteTransform(double* pose, string filename) {
    ofstream file (filename.c_str());
    if (file.is_open()) {
      //cout << "Writing to bag_name" << endl;
      for(int j = 0; j < 6; j++) {
        
        file << pose[j] << " ";
      }
    }
    file << std::flush;
}

bool SameDirection(Eigen::Vector4d x, Eigen::Vector4d y) {
  for(size_t i = 0; i < 3; i++) {
    if(!(x[i] >= 0) ^ (y[i] < 0)) {
      return false;
    }
    }
    return true;
}

// Transforms a given eigen point by the given transform (array input)
template <class T> Eigen::Matrix<T,3,1> TransformPointQuat(
  const Eigen::Matrix<T,3,1>& point,
  const T* transform) {
  
  T point_t[] = {T(point.x()), T(point.y()), T(point.z())};
  T transformed_point[3];
  //   const T* transform_rotated[4];
  //   transform_rotated[0] = transform[3];
  //   transform_rotated[1] = transform[0];
  //   transform_rotated[2] = transform[1];
  //   transform_rotated[3] = transform[2];
  ceres::QuaternionRotatePoint(transform, point_t, transformed_point);
  for (int i = 0; i < 3; ++i) {
    transformed_point[i] += transform[4 + i];
  }
  return (Eigen::Matrix<T, 3, 1>(
    transformed_point[0],
    transformed_point[1],
    transformed_point[2]));
  }
  
  // Transforms all points in a pcl point cloud (array input)
  void TransformPointCloudQuaternion(
    pcl::PointCloud<pcl::PointXYZ>& cloud,
    double transform[7]) {
    OMP_PARALLEL_FOR
    for(size_t i = 0; i < cloud.size(); ++i) {
      pcl::PointXYZ point = cloud[i];
      
      Eigen::Matrix<double,3,1> point_matrix;
      Eigen::Matrix<double,3,1> transformed_point_matrix;
      point_matrix << point.x, point.y, point.z;
      transformed_point_matrix = TransformPointQuat(point_matrix, transform);
      point.x = transformed_point_matrix[0];
      point.y = transformed_point_matrix[1];
      point.z = transformed_point_matrix[2];
      cloud[i] = point;
    }
    }

void PostProcess(const string& bagfile, const string& transform_file, vector<ros::Publisher> publishers, const bool& openi_clouds, const QApplication& app) {
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
  // Read in the first cloud from each kinect, set it as the keyframe
  cout << "Getting Initial Clouds" << endl;
  string out_name = bagfile + ".post_processed";
  string out_name2 = bagfile + ".aligned2";
  string out_name3 = bagfile + ".aligned3";
  string bag_name = bagfile + ".bag";
  
  // Keyframes
  pcl::PointCloud<pcl::PointXYZ> keyframe_k1;
  pcl::PointCloud<pcl::PointXYZ> keyframe_k2;
  if(!openi_clouds){
  cout << "Opening Bag" << endl;
  rosbag::Bag bag;
  
  bag.open(bag_name, rosbag::bagmode::Read);
  std::vector<std::string> topics;
  topics.push_back(std::string("/Cobot/Kinect/Depth"));
  rosbag::View view(bag, rosbag::TopicQuery(topics));
  // Iterator over bag file
  rosbag::View::iterator bag_it = view.begin();
  rosbag::View::iterator end = view.end();
//   
//   // Buffers of clouds from both kinects to handle reading from a single file
  std::deque<pcl::PointCloud<pcl::PointXYZ> > buffer_k1;
  std::deque<pcl::PointCloud<pcl::PointXYZ> > buffer_k2;
  std::deque<double> times_k1;
  std::deque<double> times_k2;
  
//   std::vector<std::string> topics;
//   topics.push_back(std::string("/kinect_1/depth_registered/image_raw"));
//   std::vector<std::string> topics2;
//   topics2.push_back(std::string("/kinect_2/depth_registered/image_raw"));
//   rosbag::View view(bag, rosbag::TopicQuery(topics));
//   rosbag::View view2(bag, rosbag::TopicQuery(topics2));

//   rosbag::View::iterator bag_it = view.begin();
//   rosbag::View::iterator end = view.end();
//   rosbag::View::iterator bag_it2 = view2.begin();
//   rosbag::View::iterator end2 = view2.end();
  
  
  
  double timestamp_1;
  double timestamp_2;
//   bag_it = TimeAlignedCloudsSlamBag(bag_it, end, &bag_it2, end2, &buffer_k1,
//                                      &buffer_k2, &times_k1, &times_k2, 
//                                     &keyframe_k1, &keyframe_k2, &timestamp_1, &timestamp_2);
  bag_it = TimeAlignedClouds(bag_it, end,  &buffer_k1, &buffer_k2, &times_k1, &times_k2, 
                               &keyframe_k1, &keyframe_k2, &timestamp_1, &timestamp_2);
  } else{
    keyframe_k1 = CloudFromObj(bagfile + "_1.obj");
    keyframe_k2 = CloudFromObj(bagfile + "_2.obj");
    cout << keyframe_k1.size() << endl;
    cout << keyframe_k2.size() << endl;
  }
  pcl::PointCloud<pcl::PointXYZ> transformed_cloud = keyframe_k2;
  vector<Eigen::Vector4d> normal_equations, k2_normal_equations, transformed_normals_equations;
  vector<Eigen::Vector4d> normal_equations_trans;
  vector<Eigen::Vector3d> k1_centroids, k2_centroids, transformed_centroids;
  // Retrieve the planes from the clouds
  vector<pcl::PointCloud<pcl::PointXYZ> > k1_planes = getPlanes(keyframe_k1, &normal_equations, &k1_centroids);
  vector<pcl::PointCloud<pcl::PointXYZ> > k2_planes = getPlanes(keyframe_k2, &k2_normal_equations, &k2_centroids);
  int ticker = 0;
  while(ticker < 5) {
  PublishCloud(keyframe_k1, cloud_pub);
  PublishCloud(keyframe_k2, cloud_pub_2);
  transformed_cloud = keyframe_k2;
  TransformPointCloud(transformed_cloud, transform);
  PublishCloud(transformed_cloud, cloud_pub_4);
  sleep(3);
  ticker++;
  }
  vector<pcl::PointCloud<pcl::PointXYZ> > transformed_planes = getPlanes(transformed_cloud, &transformed_normals_equations, &transformed_centroids);
  vector<double> pose0(6, 0.0);
  double zero_pose[6];
  std::copy(pose0.begin(), pose0.end(), zero_pose);
  ComparePlanes(k1_planes, transformed_planes, normal_equations, transformed_normals_equations);
//   PlaneCorrections(keyframe_k1,transformed_cloud, k1_centroids, transformed_centroids, k2_normal_equations, transformed_normals_equations, publishers, zero_pose);
//   transform = CombineTransform(transform, zero_pose );
//   transform_global = new double[6];
//   final_transform_global = new double[6];
//   memcpy(transform_global, transform, sizeof(double) * 6);
//   memcpy(final_transform_global, transform, sizeof(double) * 6);
// //   transformed_cloud = keyframe_k2;
// //   TransformPointCloud(transformed_cloud, transform);
// //   fprintf(stdout, "Transformed by combo transform\n");
// //   PublishCloud(transformed_cloud, cloud_pub_3);
//   sleep(3);
//   PublishCloud(transformed_cloud, cloud_pub_3);
//   TransformPointCloud(transformed_cloud, zero_pose);
//   transformed_planes = getPlanes(transformed_cloud, &transformed_normals_equations, &transformed_centroids);
// //   for(size_t i = 0; i < 2; i++) {
// //     Eigen::Vector4d trans_equation = transformed_normals_equations[i];
// //     Eigen::Vector4d normal_equation = normal_equations[i];
// //     if(!SameDirection(trans_equation, normal_equation)) {
// //       trans_equation[0] = -trans_equation[0];
// //       trans_equation[1] = -trans_equation[1];
// //       trans_equation[2] = -trans_equation[2];
// //       transformed_normals_equations[i] = trans_equation;
// //     }
// //   }
//   ComparePlanes(k1_planes, transformed_planes, normal_equations, transformed_normals_equations);
// //   AdjustTranslation(keyframe_k1, transformed_cloud, normal_equations, transformed_normals_equations, app); 
//   pcl::PointCloud<pcl::PointXYZ> combo_cloud;
//   combo_cloud = transformed_cloud;
//   combo_cloud += keyframe_k1;
//   WriteToObj("", out_name, 1, combo_cloud);
}

class Slider : public QWidget {
  Q_OBJECT
public:
  Slider(QWidget *parent = 0);
  virtual ~Slider() {};
  
  
public slots:
  void Translator(int num);
private:
  void closeEvent(QCloseEvent *bar);
  QSlider *slider; 
  QLabel *label;
};


Slider::Slider(QWidget *parent)
: QWidget(parent) {
  
  QHBoxLayout *hbox = new QHBoxLayout(this);
  
  slider = new QSlider(Qt::Horizontal , this);
  hbox->addWidget(slider);
  
  label = new QLabel("0", this);
  hbox->addWidget(label);
  slider->setRange(-100, 100);
  QObject::connect(slider, SIGNAL(valueChanged(int)), label, SLOT(setNum(int)));
  QObject::connect(slider, SIGNAL(valueChanged(int)), this, SLOT(Translator(int)));
}

void Slider::Translator ( int num ) {
  cout << (double)num / 100.0 << endl;
  memcpy(final_transform_global, transform_global, sizeof(double) * 6);
  pcl::PointCloud<pcl::PointXYZ> translated = transformed_global;
  VectorTranslatePointCloud(translated, (double)num / 100.0, free_vector);
  final_transform_global[3] = final_transform_global[3] + (free_vector[0] * (double)num / 100.0);
  final_transform_global[4] = final_transform_global[4] + (free_vector[1] * (double)num / 100.0);
  final_transform_global[5] = final_transform_global[5] + (free_vector[2] * (double)num / 100.0);
  PublishCloud(translated, cloud_pub_2);
  PublishCloud(base_global, cloud_pub);
}

void Slider::closeEvent (QCloseEvent *event)
{
  cout << "closed " << endl;
  WriteTransform(final_transform_global, "post_processed_transform.txt");
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
  ros::init(argc, argv, "post_processing", ros::init_options::NoSigintHandler);
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
  vector<ros::Publisher> publishers;
  publishers.push_back(cloud_pub);
  publishers.push_back(cloud_pub_2);
  publishers.push_back(cloud_pub_3);
  publishers.push_back(cloud_pub_4);
  publishers.push_back(marker_pub);
 
  
  QApplication app(argc, argv);  
  Slider window;
  
  window.setWindowTitle("QSlider");
  window.show();
  
//   return app.exec();
  
 
  PostProcess(bagFile, transform_file, publishers, true, app);

  return 0;
}
#include "post_processing.moc"