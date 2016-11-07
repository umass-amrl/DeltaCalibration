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

bool doubleEquals(double x, double y){
  return fabs(x-y) < .00005;
}

void visualize_nn(
  const pcl::PointCloud<pcl::PointXYZ>& base_cloud,
  const pcl::PointCloud<pcl::PointXYZ>& k1_cloud,
  const vector<int>& nearest_neighbors,
  const vector<int>& start_points) {

  visualization_msgs::Marker line_list;
  std_msgs::Header header;
  header.frame_id = "point_clouds";
  line_list.action = visualization_msgs::Marker::ADD;
  //line_list.pose.orientation.w = 1.0;
  line_list.header.frame_id = "point_cloud";

  line_list.id = 0;
  line_list.type = visualization_msgs::Marker::LINE_LIST;
  line_list.scale.x = 0.0001;

  // Line list is red
  line_list.color.r = 1.0;
  line_list.color.a = 1.0;

  for(size_t i = 0; i < start_points.size(); ++i){
    pcl::PointXYZ temp_point = k1_cloud[start_points[i]];
    pcl::PointXYZ temp_point2 = base_cloud[nearest_neighbors[i]];
    if (!isnan(temp_point.x) &&
        !isnan(temp_point.y) &&
        !isnan(temp_point.z) &&
        !isnan(temp_point2.x) &&
        !isnan(temp_point2.y) &&
        !isnan(temp_point2.z)){
      geometry_msgs::Point p, p2;
      p.x = temp_point.x;
      p.y = temp_point.y;
      p.z = temp_point.z;
      p2.x = temp_point2.x;
      p2.y = temp_point2.y;
      p2.z = temp_point2.z;
      line_list.points.push_back(p);
      line_list.points.push_back(p2);
    }
  }

  marker_pub.publish(line_list);
}


struct PointToPointErrorNumeric {
  PointToPointErrorNumeric(double base_pointX,
                          double base_pointY,
                          double base_pointZ,
                          double transformed_pointX,
                          double transformed_pointY,
                          double transformed_pointZ,
                          pcl::Normal normal_k,
                          pcl::Normal normal_k1) :
      transformed_pointX(transformed_pointX),
      transformed_pointY(transformed_pointY),
      transformed_pointZ(transformed_pointZ),
      base_pointX(base_pointX),
      base_pointY(base_pointY),
      base_pointZ(base_pointZ),
      normal_k(normal_k),
      normal_k1(normal_k1) {}

  bool operator()(const double* const camera,
    double* residuals) const {

    // Base Point Converted to an array for rotation
    Eigen::Matrix<double,3,1> base_point_mat;
    base_point_mat[0] = base_pointX;
    base_point_mat[1] = base_pointY;
    base_point_mat[2] = base_pointZ;
    // Transformed Point Converted to an array
    Eigen::Matrix<double,3,1> transformed_point_mat;
    transformed_point_mat[0] = transformed_pointX;
    transformed_point_mat[1] = transformed_pointY;
    transformed_point_mat[2] = transformed_pointZ;

    Eigen::Matrix<double,3,1> norm_k;
    Eigen::Matrix<double,3,1> norm_k1;
    norm_k[0] = normal_k.normal_x;
    norm_k[1] = normal_k.normal_y;
    norm_k[2] = normal_k.normal_z;

    norm_k1[0] = normal_k1.normal_x;
    norm_k1[1] = normal_k1.normal_y;
    norm_k1[2] = normal_k1.normal_z;

    //Create the eigen transform from the camera
    transformed_point_mat = TransformPoint(transformed_point_mat, camera);

    // The error is the difference between the predicted and observed position.
    residuals[0] = (transformed_point_mat - base_point_mat).dot(norm_k);
    residuals[1] = (base_point_mat - transformed_point_mat).dot(norm_k1);
    //residuals[2] = transformed_point_mat[2] - base_point[2];
    return true;
  }

   // Factory to hide the construction of the CostFunction object from
   // the client code.
  static ceres::CostFunction* Create(const double base_pointX,
    const double base_pointY, const double base_pointZ,
    const double transformed_pointX, const double transformed_pointY,
    const double transformed_pointZ, const pcl::Normal normal_k,
    const pcl::Normal normal_k1) {
    // 3 is number of residuals, 6 number of parameters (camera transform)
   return (new ceres::NumericDiffCostFunction<PointToPointErrorNumeric,
    ceres::CENTRAL, 2, 6>(
     new PointToPointErrorNumeric(base_pointX, base_pointY, base_pointZ,
      transformed_pointX, transformed_pointY, transformed_pointZ, normal_k,
      normal_k1)));
 }

 double transformed_pointX;
 double transformed_pointY;
 double transformed_pointZ;
 double base_pointX;
 double base_pointY;
 double base_pointZ;
 pcl::Normal normal_k;
 pcl::Normal normal_k1;
};

Eigen::MatrixXd CalculateJTJ(const ceres::CRSMatrix& jacobian){

  int num_rows = jacobian.num_rows;
  // Convert the sparse matrix to a dense matrix

  // Convert the CRS Matrix to an eigen matrix
  Eigen::MatrixXd denseJacobian = Eigen::MatrixXd::Zero(num_rows, jacobian.num_cols);
  for(size_t k = 0; k < jacobian.rows.size() - 1; ++k){
    size_t row = jacobian.rows[k];
    size_t nextRow = jacobian.rows[k + 1];
    for(size_t l = row; l < nextRow; ++l){
      int column = jacobian.cols[l];
      double value = jacobian.values[l];
        //std::cout << row << " " << column << " " << value << std::endl;
        denseJacobian(k,column) = value;
    }
  }

  //denseJacobian = Eigen::MatrixXd(jacobianMatrix);
  // Calculate j.transpose j
  Eigen::MatrixXd jTj = denseJacobian.transpose() * denseJacobian;

  return jTj;
}

Eigen::MatrixXd CalculateCovariance(Eigen::MatrixXd mat){
  Eigen::MatrixXd centered = mat.rowwise() - mat.colwise().mean();
  Eigen::MatrixXd cov = (centered.adjoint() * centered) / double(mat.rows() - 1);
  return cov;
}

double pointDistance(pcl::PointXYZ point1, pcl::PointXYZ point2) {
  double distance = 0;
  double x1 = point1.x;
  double y1 = point1.y;
  double z1 = point1.z;

  double x2 = point2.x;
  double y2 = point2.y;
  double z2 = point2.z;

  distance = sqrt(pow(x2 - x1, 2) + pow(y2 - y1, 2) + pow(z2 - z1, 2));

  return distance;
}

double kdTreeNN(
  const pcl::PointCloud<pcl::PointXYZ>& pointcloud,
  const pcl::PointCloud<pcl::PointXYZ>& transformed_cloud,
  vector<int>& nearest_neigbors, vector<int>& start_points){
  // Initialize KDTree_t
  // Initialization of tree and base cloud (which doesn't get new transforms at
  // each iteration) could be moved to a new loop that happened with every
  // change of k instead of every change nearest neighbor calculation.
  typedef float KDTree_t;
  static const KDTree_t kThreshold = nn_dist;
  vector<KDNodeValue<KDTree_t, 3> > points;
  vector<KDNodeValue<KDTree_t, 3> > transform_points;

  // Initialize the points for the kd_tree (pulled from the pcl clouds)
  for(size_t i = 0; i < pointcloud.size(); i++){
    KDNodeValue<KDTree_t, 3> temp1;
    if(!isnan(pointcloud.points[i].x) && !isnan(pointcloud.points[i].y) &&
      !isnan(pointcloud.points[i].z)){
      temp1.point(0) = pointcloud.points[i].x;
      temp1.point(1) = pointcloud.points[i].y;
      temp1.point(2) = pointcloud.points[i].z;
      temp1.index = i;
      if (temp1.point(0) != 0 && temp1.point(1) != 0 && temp1.point(2) != 0){
        points.push_back(temp1);
      }
    }
  }
  for(size_t i = 0; i < transformed_cloud.size(); i++){
    KDNodeValue<KDTree_t, 3> temp2;
    if (!isnan(transformed_cloud.points[i].x) &&
            !isnan(transformed_cloud.points[i].y) &&
      !isnan(transformed_cloud.points[i].z)){
      temp2.point(0) = transformed_cloud.points[i].x;
      temp2.point(1) = transformed_cloud.points[i].y;
      temp2.point(2) = transformed_cloud.points[i].z;

      temp2.index = i;
      if (temp2.point(0) != 0 && temp2.point(1) != 0 && temp2.point(2) != 0){
        transform_points.push_back(temp2);
      }
    }
  }
  // Run the actual nearest neighbor calculation
  KDTree<KDTree_t, 3> tree(points);
  double error = 0.0;
  double mean = 0;
  // double maxError = 0;
  for (unsigned int i = 0; i <  transform_points.size(); i++) {
      pcl::PointXYZ neighborPoint;
      pcl::PointXYZ startPoint;
      KDNodeValue<KDTree_t, 3> neighbor;

      error = tree.FindNearestPoint(
          transform_points[i].point,
          kThreshold,
          &neighbor);

      double dist = (transform_points[i].point - neighbor.point).norm();
      if (abs(dist) < nn_dist) {
        nearest_neigbors.push_back(neighbor.index);
        start_points.push_back(transform_points[i].index);
        mean += error;
      }
    }
    return mean / transform_points.size();
}

void WritePose(double* pose, ofstream file) {
  if (file.is_open()) {
    for(int j = 0; j < 6; j++){
      file << pose[j] << "\t";
    }
  file << "\n";
  }
}


void BuildProblem_noDense(
  const pcl::PointCloud<pcl::PointXYZ>& k_cloud,
  const pcl::PointCloud<pcl::PointXYZ>& l_cloud,
  const vector<int>& nearest_neighbors,
  const vector<int>& start_points,
  double l_pose[6],
  const pcl::PointCloud<pcl::Normal>& normals,
  const pcl::PointCloud<pcl::Normal>& normals_l,
  ceres::Problem* problem){
  // Ceres-Solver Problem setup
  for (size_t i = 0; i < start_points.size(); i++) {
    pcl::PointXYZ l_point, k_point;
    l_point = l_cloud[start_points[i]];
    k_point = k_cloud[nearest_neighbors[i]];
    pcl::Normal normal_k = normals[nearest_neighbors[i]];
    pcl::Normal normal_k1 = normals_l[i];

    // Calculate distance to avoid using points too far apart
    double distance = pointDistance(k_point, l_point);

    // Add non nan values to the problem
    if (distance < nn_dist) {
      if (!isnan(l_point.x) &&
          !isnan(l_point.y) &&
          !isnan(l_point.z) &&
          !isnan(normal_k.normal_x)&&
          !isnan(normal_k1.normal_x)){
        ceres::CostFunction* cost_function =
                PointToPointErrorNumeric::Create(k_point.x,
                                                k_point.y,
                                                k_point.z,
                                                l_point.x,
                                                l_point.y,
                                                l_point.z,
                                                normal_k,
                                                normal_k1);
            problem->AddResidualBlock(cost_function,
                                    NULL, // squared loss
                                    l_pose);
      }
    }
  }
}

void ConstructICP_problem(const pcl::PointCloud<pcl::PointXYZ>& cloud_1,
  const pcl::PointCloud<pcl::PointXYZ>& cloud_2,
  const pcl::PointCloud<pcl::Normal>& normal_1,
  const pcl::PointCloud<pcl::Normal>& normal_2,
  double* transform,
  ceres::Problem* problem) {

  //----------  Transform based on calculated transformation  ----------
  pcl::PointCloud<pcl::PointXYZ> transformed_cloud = cloud_2;

  // Shifts cloud by calculated transform
  TransformPointCloud_PCL(transformed_cloud, transform);
  publish_cloud(cloud_1, cloud_pub);
  publish_cloud(transformed_cloud, cloud_pub_2);
  publish_cloud(cloud_2, cloud_pub_3);
  //----------  Visualize it ----------

  //----------  Find Nearest Neighbors  ----------
  //vector<int> nearest_neighbors(transformed_cloud.points.size());
  //vector<float> squared_distances(transformed_cloud.points.size(),100);
  vector<int> nearest_neigbors;
  vector<int> start_points;
  // mean = kdTreeNN_normal(base_cloud, transformed_cloud, normals[k], normals[k+1],
  // nearest_neigbors,  start_points);
  cout << "Calculating nearest neighbor" << endl;
  kdTreeNN(cloud_1, transformed_cloud,
  nearest_neigbors,  start_points);


  //ROS_INFO("Mean = %f", mean);

  //----------  Visualize NN ----------
  visualize_nn(cloud_1,
               transformed_cloud,
               nearest_neigbors,
               start_points);
  //sleep(1);

  //----------  Compute Pose with ceres-solver  ----------
  // Add for each pair of nearest neighbors
  int pair_count = 0;
  cout << "Building Problem" << endl;
  for (size_t i = 0; i < start_points.size(); i++) {
    pcl::PointXYZ k1_point, base_point;
    // Use the original cloud, not the transformed cloud
    k1_point = cloud_2[start_points[i]];
    // Base cloud is unchanged
    base_point = cloud_1[nearest_neigbors[i]];
    pcl::Normal normal_k = normal_1[nearest_neigbors[i]];
    pcl::Normal normal_k1 = normal_2[i];
    pcl::PointXYZ transformed_k1 = transformed_cloud[start_points[i]];
    double distance = pointDistance(base_point, transformed_k1);

    if (distance < nn_dist) {
      //pcl::Normal zeroNormal = pcl::Normal(0,0,0);
      if (!isnan(k1_point.x) &&
          !isnan(k1_point.y) &&
          !isnan(k1_point.z) &&
          !isnan(base_point.x) &&
          !isnan(base_point.y) &&
          !isnan(base_point.z) &&
          !isnan(normal_k.normal_x) &&
          !isnan(normal_k1.normal_x)) {
        ceres::CostFunction* cost_function =
            PointToPointErrorNumeric::Create(base_point.x,
                                            base_point.y,
                                            base_point.z,
                                            k1_point.x,
                                            k1_point.y,
                                            k1_point.z,
                                            normal_k,
                                            normal_k1);
        problem->AddResidualBlock(cost_function,
                                NULL, // squared loss
                                transform);
        pair_count += 1;
      }
    }
  }
}

void VisualizeCovariance(
  const int num,
  const string covarianceFolder,
  const string bagfile,
  const ceres::CRSMatrix& jacobian) {
  cout << "vizualizing covariance" << endl;
  std::stringstream out;

  mkdir(covarianceFolder.c_str(), 0777);
  out << num;
  string filename = covarianceFolder + "/" + bagfile + "_" + out.str() + ".covariance";

  ofstream file(filename.c_str());
  // Calculate the jtj and the covariance matrix
  Eigen::MatrixXd jtj = CalculateJTJ(jacobian);
  //cout << "Information Size:: " << jtj.size() << endl;
  // Translation Portion
  Eigen::MatrixXd translation = jtj.inverse().block(3,3, 3, 3);
  Eigen::MatrixXd covarianceMat = translation;
  //cout << jtj << endl;
  file << covarianceMat << endl;
  // Solve to retrieve eigenvectors/values
  Eigen::SelfAdjointEigenSolver<Eigen::MatrixXd> eigenSolver(covarianceMat);
  Eigen::MatrixXd eigenvalues = eigenSolver.eigenvalues();
  Eigen::MatrixXd eigenvectors = eigenSolver.eigenvectors();
  //cout << "Eigen values: \n" << eigenvalues << endl;
  //Calculate Rotation Matrix and convert to quaternion
  Eigen::Matrix3d rotationMat = eigenvectors;

  Eigen::Quaterniond quat;
  quat = rotationMat;
  //Eigen::Quaterniond quat = aa;

  visualization_msgs::Marker marker;
  marker.action = visualization_msgs::Marker::ADD;
  marker.header.frame_id = "point_cloud";
  marker.id= 0;
  marker.type = visualization_msgs::Marker::SPHERE;
  marker.pose.position.x = 0;
  marker.pose.position.y = 0;
  marker.pose.position.z = 0;
  marker.pose.orientation.x = quat.x();
  marker.pose.orientation.y = quat.y();
  marker.pose.orientation.z = quat.z();
  marker.pose.orientation.w = quat.w();
  marker.color.a = 0.5;
  marker.color.r = 0.0;
  marker.color.g = 1.0;
  marker.color.b = 0.0;
  marker.scale.x = eigenvalues(0) * 1000;
  marker.scale.y = eigenvalues(1) * 1000;
  marker.scale.z =eigenvalues(2) * 1000;

  marker_pub.publish(marker);
  file.close();
}

double residual_dist(
  const pcl::PointCloud<pcl::PointXYZ>& k_cloud,
  const pcl::PointCloud<pcl::PointXYZ>& l_cloud,
  const pcl::PointCloud<pcl::Normal>& k_normal,
  const pcl::PointCloud<pcl::Normal>& l_normal,
  double* pose) {

  // Create a new problem
  ceres::Problem problem;

  //----------  Find Nearest Neighbors  ----------
  vector<int > nearest_neigbors;
  vector<int > start_points;
  // mean += kdTreeNN_normal(base_cloud, transformed_cloud, normals[k], normals[l],
  // nearest_neigbors, start_points);
  // Reverse order nearest neighbor to make sure we're transforming in the right direction
  cout << "Finding Neighbors" << endl;
  kdTreeNN(k_cloud, l_cloud,
  nearest_neigbors, start_points);

  //----------  Add to ceres problem  ----------
  // We give this pose k instead of l, and switch the order of the point clouds
  // and normals passed in to make sure that it's a transformation from k to l
  // being calculated
  BuildProblem_noDense(
               k_cloud,
               l_cloud,
               nearest_neigbors,
               start_points,
               pose,
               k_normal,
               l_normal,
               &problem);

          //----------  Ceres Solve  ----------
  ceres::Solver::Options options;
  options.num_threads = 12;
  options.num_linear_solver_threads = 12;
  options.linear_solver_type = ceres::ITERATIVE_SCHUR;
  //options.minimizer_progress_to_stdout = true;
  ceres::Solver::Summary summary;
  ceres::Solve(options, &problem, &summary);
  //std::cout << summary.FullReport() << "\n";
  vector<double> residuals;
  ceres::Problem::EvaluateOptions evalOptions = ceres::Problem::EvaluateOptions();
  //std::vector<double*> parameter_blocks;
  //parameter_blocks.push_back(arrays[k+1]);
  //evalOptions.parameter_blocks = parameter_blocks;
  ceres::CRSMatrix jacobian;
  problem.Evaluate(evalOptions, NULL, &residuals, NULL, &jacobian);
  double mean = std::accumulate(residuals.begin(), residuals.end(), 0.0);
  mean = mean / residuals.size();
  cout << "Mean: " << mean << endl;
  return mean;
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

vector<pcl::PointCloud<pcl::PointXYZ> > getPlanes(pcl::PointCloud<pcl::PointXYZ> cloud){

  vector<pcl::PointCloud<pcl::PointXYZ> > output;
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered (new pcl::PointCloud<pcl::PointXYZ>), cloud_p (new pcl::PointCloud<pcl::PointXYZ>), cloud_f (new pcl::PointCloud<pcl::PointXYZ>);


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

void ComparePlanes(vector<pcl::PointCloud<pcl::PointXYZ> > k1_planes,
  vector<pcl::PointCloud<pcl::PointXYZ> > transformed_planes) {

  for(size_t i = 0; i < k1_planes.size(); ++i) {
    pcl::PointCloud<pcl::PointXYZ> plane1 = k1_planes[i];
    pcl::PointCloud<pcl::PointXYZ> plane2 = transformed_planes[i];
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



double* ICP(int k,
  string covarianceFolder,
  string bagFile,
  const pcl::PointCloud<pcl::PointXYZ>& cloud_1,
  const pcl::PointCloud<pcl::PointXYZ>& cloud_2,
  const pcl::PointCloud<pcl::Normal>& normal_1,
  const pcl::PointCloud<pcl::Normal>& normal_2,
  double* transform,
  double* final_mean) {

  double mean = 1000000;
  double last_mean = 1000010;
  int same_counter = 0;
  vector<double> residuals;
  double counter = 0;
  while (abs(mean) > 0.00001 && same_counter < 5 && counter < 30) {
    if (doubleEquals(mean, last_mean)) {
      same_counter++;
    }
    residuals.clear();
    last_mean = mean;
    counter += 1;
    // Construct ICP problem
    ceres::Problem problem;
    ConstructICP_problem(
      cloud_1,
      cloud_2,
      normal_1,
      normal_2,
      transform,
      &problem);

    // Run Ceres problem
    ceres::Solver::Options options;
    options.num_threads = 12;
    options.num_linear_solver_threads = 12;
    //options.use_explicit_schur_complement = true;
    options.linear_solver_type = ceres::ITERATIVE_SCHUR;
    //options.minimizer_progress_to_stdout = true;
    ceres::Solver::Summary summary;
    ceres::Solve(options, &problem, &summary);

    cout << "solving problem" << endl;
    //ceres::Problem::EvaluateOptions evalOptions = ceres::Problem::EvaluateOptions();
    //std::vector<double*> parameter_blocks;
    //parameter_blocks.push_back(arrays[k+1]);
    //evalOptions.parameter_blocks = parameter_blocks;
    ceres::Problem::EvaluateOptions evalOptions = ceres::Problem::EvaluateOptions();
    ceres::CRSMatrix jacobian;
    problem.Evaluate(evalOptions, NULL, &residuals, NULL, &jacobian);
    mean = std::accumulate(residuals.begin(), residuals.end(), 0.0);
    mean = mean / residuals.size();
    final_mean = &mean;
    ROS_INFO("Mean = %f", mean);
    //std::cout << summary.FullReport() << "\n";
    VisualizeCovariance(k, covarianceFolder, bagFile, jacobian);
    final_mean[0] = mean;
  }

  return transform;
}

void residual_comparison(pcl::PointCloud<pcl::PointXYZ> cloud1,
  pcl::PointCloud<pcl::PointXYZ> cloud2,
  pcl::PointCloud<pcl::PointXYZ> transformed_cloud,
  double* transform){

  pcl::PointCloud<pcl::Normal> normal1;
  pcl::PointCloud<pcl::Normal> normal2;
  pcl::PointCloud<pcl::Normal> normalT;
  normal1 = getNormals(cloud1);
  normal2 = getNormals(cloud2);
  normalT = getNormals(transformed_cloud);

  // Initial residual distance
  double start_dist = residual_dist(cloud1, transformed_cloud, normal1, normalT, transform);

  double mean1;
  double count = 0;
  string covarianceFile = "";
  string covariance_1 = "";
  // Final transform after ICP
  transform = ICP(count, covarianceFile, covariance_1, cloud1, cloud2, normal1,
        normal2, transform, &mean1);

  pcl::PointCloud<pcl::PointXYZ> final = cloud2;

  TransformPointCloud_PCL(final, transform);
  pcl::PointCloud<pcl::Normal> normalF;
  normalF = getNormals(final);

  // Initial residual distance
  double f_dist = residual_dist(cloud1, final, normal1, normalF, transform);

  cout << "ICP_error" << abs(start_dist) - abs(f_dist) << endl;
  cout << start_dist << " \t" << f_dist << endl;
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
    writeToObj(out_name2, count, cloud_k1);
    writeToObj(out_name3, count, cloud_k2);
    cout << "Cloud 1 planes" << endl;
    TransformPointCloud_PCL(transformed_cloud, transform);
    cout << "transformed planes" << endl;
    residual_comparison(cloud_k1, cloud_k2, transformed_cloud, transform);
    publish_cloud(cloud_k1, cloud_pub);
    publish_cloud(transformed_cloud, cloud_pub_2);
    publish_cloud(cloud_k2, cloud_pub_3);
    pcl::PointCloud<pcl::PointXYZ> combo_cloud = cloud_k1;
    combo_cloud += transformed_cloud;
    writeToObj(out_name, count, combo_cloud);
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
    CheckTransform(bagFile, transform_file);
  }
  // else if(mode == 1) {
  //   CheckTransform(bagFile, transform_file);
  // }
  return 0;
}
