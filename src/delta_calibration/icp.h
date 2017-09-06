//========================================================================
//  This software is free: you can redistribute it and/or modify
//  it under the terms of the GNU Lesser General Public License Version 3,
//  as published by the Free Software Foundation.
//
//  This software is distributed in the hope that it will be useful,
//  but WITHOUT ANY WARRANTY; without even the implied warranty of
//  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
//  GNU Lesser General Public License for more details.
//
//  You should have received a copy of the GNU Lesser General Public License
//  Version 3 in the file COPYING that came with this distribution.
//  If not, see <http://www.gnu.org/licenses/>.
//========================================================================
//========================================================================

#ifndef ICP_H
#define ICP_H
//----------- INCLUDES
#include <cmath>
#include <cstdio>
#include <fstream>
#include <iostream>
#include <math.h>
#include <sstream>
#include <string>
// ROS INCLUDES
#include <ros/ros.h>
#include <rosbag/bag.h>
#include <rosbag/view.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/image_encodings.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

// PCL specific includes
#include <pcl/common/transforms.h>
#include <pcl/console/parse.h>
#include <pcl/features/normal_3d.h>
#include <pcl/features/normal_3d_omp.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/io/obj_io.h>
#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/pcl_exports.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/search/kdtree.h>
#include <pcl/search/search.h>
#include <pcl_conversions/pcl_conversions.h>
// Ceres Includes
#include "ceres/ceres.h"
#include "ceres/rotation.h"
// OTHER
//#include "CImg/CImg.h"
#include "fspf/plane_filtering.h"
#include "opencv2/core/core.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include "shared/util/popt_pp.h"
#include <Eigen/Geometry>
#include <Eigen/OrderingMethods>
#include <Eigen/Sparse>
#include <opencv2/highgui/highgui.hpp>

// OPENMP_PARALLEL_FOR
#include "shared/util/openmp_utils.h"
#include <fstream>

#include "perception_tools/kdtree.h"

// #include "kinect/lib/freenect_internal.h"
// #include "kinect/include/libfreenect.h"
// #include <stdio.h>
// #include <stdlib.h>
// #include <string.h>
// #include <unistd.h>
// #include "vector_map.h"
// #include <X11/Xlib.h>
// #include <X11/Xutil.h>
// #include <fstream>
// #include <ros/package.h>

// #include <fstream>

using namespace std;
using Eigen::Vector3d;

namespace icp {

template <class T>
Eigen::Matrix<T, 3, 1> TransformPoint(const Eigen::Matrix<T, 3, 1> &point,
                                      const T *transform);

Eigen::Matrix<double, 3, 1>
TransformPointQuaternion(const Eigen::Matrix<double, 3, 1> &point,
                         double *pose);

template <class T>
Eigen::Matrix<T, 3, 1> TransformPointInv(const Eigen::Matrix<T, 3, 1> &point,
                                         const T transform[6]);

double *InvertTransform(double *transform);

void PublishCloud(const pcl::PointCloud<pcl::PointXYZ> &cloud,
                  ros::Publisher publisher);

void PublishCloud(const pcl::PointCloud<pcl::PointXYZRGB> &cloud,
                  ros::Publisher publisher);

void WriteToBag(const string topic, rosbag::Bag *bag,
                const pcl::PointCloud<pcl::PointXYZ> cloud);

void WriteToObj(const string folder, const string bagfile, const int num,
                const pcl::PointCloud<pcl::PointXYZ> &cloud);

template <class T>
Eigen::Matrix<T, 3, 1> TransformVector(const Eigen::Matrix<T, 3, 1> &vector,
                                       const T transform[6]);

double KdTreeNN(const double nn_dist,
                const pcl::PointCloud<pcl::PointXYZ> &pointcloud,
                const pcl::PointCloud<pcl::PointXYZ> &transformed_cloud,
                const pcl::PointCloud<pcl::Normal> &normal_1,
                const pcl::PointCloud<pcl::Normal> &normal_2,
                const vector<Eigen::Vector2d> &image_coords_1,
                const vector<Eigen::Vector2d> &image_coords_2,
                vector<int> &nearest_neigbors, vector<int> &start_points);

void TransformPointCloudInv(pcl::PointCloud<pcl::PointXYZ> *cloud,
                            double transform[6]);

void TransformPointCloud(pcl::PointCloud<pcl::PointXYZ> *cloud,
                         double transform[6]);

void TransformPointCloudQuat(pcl::PointCloud<pcl::PointXYZ> &cloud,
                             double transform[7]);

void SanitizeTransform(const pcl::PointCloud<pcl::PointXYZ> &cloud,
                       const pcl::PointCloud<pcl::Normal> &normal,
                       double *pose);

void TransformPointCloud(pcl::PointCloud<pcl::PointXYZ> *cloud,
                         const vector<double> &transform);

pcl::PointCloud<pcl::PointXYZ>
CloudFromVector(const vector<Eigen::Vector3f> &pointCloud,
                const vector<int> &pixelLocs);

void VectorTranslatePointCloud(pcl::PointCloud<pcl::PointXYZ> *cloud,
                               double magnitude,
                               Eigen::Matrix<double, 3, 1> vector);

double PointDistance(Eigen::Vector3d point1, Eigen::Vector3d point2);

double PointDistance(Eigen::Vector3f point1, Eigen::Vector3f point2);

double PointDistance(pcl::PointXYZ point1, pcl::PointXYZ point2);

void BuildProblem(const pcl::PointCloud<pcl::PointXYZ> &k_cloud,
                  const pcl::PointCloud<pcl::PointXYZ> &l_cloud,
                  const vector<int> &nearest_neighbors,
                  const vector<int> &start_points, double l_pose[6],
                  const pcl::PointCloud<pcl::Normal> &normals,
                  const pcl::PointCloud<pcl::Normal> &normals_l,
                  ceres::Problem *problem);

double ResidualDist(const pcl::PointCloud<pcl::PointXYZ> &k_cloud,
                    const pcl::PointCloud<pcl::PointXYZ> &l_cloud,
                    const pcl::PointCloud<pcl::Normal> &k_normal,
                    const pcl::PointCloud<pcl::Normal> &l_normal, double *pose);

Eigen::Matrix<double, 4, 1> TransformDifference(double *transform_base,
                                                double *transform);

double *CombineTransform(double *pose1, double *pose2);

pcl::PointCloud<pcl::PointXYZ> CloudFromObj(string obj);

void VisualizeNN(const pcl::PointCloud<pcl::PointXYZ> &base_cloud,
                 const pcl::PointCloud<pcl::PointXYZ> &k1_cloud,
                 const vector<int> &nearest_neighbors,
                 const vector<int> &start_points,
                 const ros::Publisher &marker_pub);

void VisualizePoses(const vector<double *> &poses,
                    const vector<double *> &poses2, const vector<int> &keys,
                    const ros::Publisher &marker_pub);

void VisualizeReferenceFrame(const double *transform,
                             const ros::Publisher &marker_pub, const int id);

Eigen::MatrixXd CalculateJTJ(const ceres::CRSMatrix &jacobian);

Eigen::MatrixXd CalculateCovariance(const Eigen::MatrixXd &mat);

void VisualizeCovariance(const int num, const string covarianceFolder,
                         const string bagfile,
                         const ceres::CRSMatrix &jacobian);

pcl::PointCloud<pcl::PointXYZ>
VoxelFilter(const pcl::PointCloud<pcl::PointXYZ> &cloud);

pcl::PointCloud<pcl::PointXYZ>
BrassVoxelFilter(const pcl::PointCloud<pcl::PointXYZ> &cloud);

pcl::PointCloud<pcl::Normal>
GetNormals(const pcl::PointCloud<pcl::PointXYZ> &cloud);

void ConstructICP_problem(

    const vector<ros::Publisher> &publishers,
    const pcl::PointCloud<pcl::PointXYZ> &cloud_1,
    const pcl::PointCloud<pcl::PointXYZ> &cloud_2,
    const pcl::PointCloud<pcl::Normal> &normal_1,
    const pcl::PointCloud<pcl::Normal> &normal_2,
    const vector<Eigen::Vector2d> &image_coords_1,
    const vector<Eigen::Vector2d> &image_coords_2, const double nn_dist,
    double *transform, ceres::Problem *problem);

bool GetKnownCor(const pcl::PointCloud<pcl::PointXYZ> &cloud,
                 const pcl::PointCloud<pcl::Normal> &normal_1, const int k2,
                 const vector<int> &index, pcl::Normal *normal_k1,
                 pcl::PointXYZ *base_point, int &num);

void ConstructICPKnown(const vector<ros::Publisher> &publishers,
                       const pcl::PointCloud<pcl::PointXYZ> &cloud_1,
                       const pcl::PointCloud<pcl::PointXYZ> &cloud_2,
                       const pcl::PointCloud<pcl::Normal> &normal_1,
                       const pcl::PointCloud<pcl::Normal> &normal_2,
                       const vector<int> index_k1, const vector<int> index_k2,
                       double *transform, ceres::Problem *problem);

void PlaneCorrections(pcl::PointCloud<pcl::PointXYZ> cloud_1,
                      pcl::PointCloud<pcl::PointXYZ> cloud_2,
                      vector<Eigen::Vector3d> k1_centroids,
                      vector<Eigen::Vector3d> k2_centroids,
                      vector<Eigen::Vector4d> normal_equations,
                      vector<Eigen::Vector4d> k2_normal_equations,
                      vector<ros::Publisher> publishers, double *transform);

Eigen::Matrix3d CalcScatterMatrix(pcl::PointCloud<pcl::Normal> normals);

double CalcConditionNumber(Eigen::Matrix3d mat1);

void ICP(const int k, const double nn_dist,
         const vector<ros::Publisher> &publishers,
         const string &covarianceFolder, const string &bagFile,
         const pcl::PointCloud<pcl::PointXYZ> &cloud_1,
         const pcl::PointCloud<pcl::PointXYZ> &cloud_2,
         const pcl::PointCloud<pcl::Normal> &normal_1,
         const pcl::PointCloud<pcl::Normal> &normal_2,
         const vector<Eigen::Vector2d> &image_coords_1,
         const vector<Eigen::Vector2d> &image_coords_2, double *transform,
         double *final_rmse);

double *ICPKnownC(const int k, const vector<ros::Publisher> &publishers,
                  const string &covarianceFolder, const string &bagFile,
                  const pcl::PointCloud<pcl::PointXYZ> &cloud_1,
                  const pcl::PointCloud<pcl::PointXYZ> &cloud_2,
                  const pcl::PointCloud<pcl::Normal> &normal_1,
                  const pcl::PointCloud<pcl::Normal> &normal_2,
                  const vector<int> index_k1, const vector<int> index_k2,
                  double *transform, double *final_mean);

double *ClosedForm(const pcl::PointCloud<pcl::PointXYZ> &cloud_1,
                   const pcl::PointCloud<pcl::PointXYZ> &cloud_2,
                   double *transform);

pcl::PointCloud<pcl::PointXYZ>
CloudFromVector(const vector<Eigen::Vector3f> &pointCloud,
                const vector<int> &pixelLocs);

// Reads clouds from a given iterator, saves to buffer if they are over
rosbag::View::iterator
GetClouds(rosbag::View::iterator it,
          std::deque<pcl::PointCloud<pcl::PointXYZ>> *buffer1,
          std::deque<pcl::PointCloud<pcl::PointXYZ>> *buffer2,
          std::deque<double> *timestamps_1, std::deque<double> *timestamps_2);

rosbag::View::iterator
GetCloudsBag(rosbag::View::iterator it, rosbag::View::iterator end,
             std::deque<pcl::PointCloud<pcl::PointXYZ>> *buffer1,
             std::deque<pcl::PointCloud<pcl::PointXYZ>> *buffer2,
             std::deque<double> *timestamps_1, std::deque<double> *timestamps_2,
             pcl::PointCloud<pcl::PointXYZ> *cloud1,
             pcl::PointCloud<pcl::PointXYZ> *cloud2, double *time1,
             double *time2);

rosbag::View::iterator TimeAlignedCloudsSlamBag(
    rosbag::View::iterator it, rosbag::View::iterator end,
    rosbag::View::iterator *it2, rosbag::View::iterator end2,
    std::deque<pcl::PointCloud<pcl::PointXYZ>> *buffer1,
    std::deque<pcl::PointCloud<pcl::PointXYZ>> *buffer2,
    std::deque<double> *timestamps_1, std::deque<double> *timestamps_2,
    pcl::PointCloud<pcl::PointXYZ> *cloud1,
    pcl::PointCloud<pcl::PointXYZ> *cloud2, double *time1, double *time2);

rosbag::View::iterator
OneSensorClouds(rosbag::View::iterator it, rosbag::View::iterator end,
                std::deque<pcl::PointCloud<pcl::PointXYZ>> *buffer1,
                std::deque<double> *timestamps_1,
                pcl::PointCloud<pcl::PointXYZ> *cloud1, double *time1);

rosbag::View::iterator
OneSensorCloudsBrass(rosbag::View::iterator it, rosbag::View::iterator end,
                     std::deque<pcl::PointCloud<pcl::PointXYZ>> *buffer1,
                     std::deque<double> *timestamps_1,
                     pcl::PointCloud<pcl::PointXYZ> *cloud1, double *time1);

rosbag::View::iterator TimeAlignedClouds(
    rosbag::View::iterator it, rosbag::View::iterator end,
    std::deque<pcl::PointCloud<pcl::PointXYZ>> *buffer1,
    std::deque<pcl::PointCloud<pcl::PointXYZ>> *buffer2,
    std::deque<double> *timestamps_1, std::deque<double> *timestamps_2,
    pcl::PointCloud<pcl::PointXYZ> *cloud1,
    pcl::PointCloud<pcl::PointXYZ> *cloud2, double *time1, double *time2);

bool CheckChangeVel(double *pose, const int degree,
                    const vector<double> &velocity_list);

bool CheckChangeOdom(double *pose, double *previous_pose,
                     const double &timestamp_1, const double &timestamp_2,
                     const int &degree);

bool CheckChange(double *pose, const int degree);

bool CheckResidualDist(const pcl::PointCloud<pcl::PointXYZ> &k_cloud,
                       const pcl::PointCloud<pcl::PointXYZ> &l_cloud,
                       const pcl::PointCloud<pcl::PointXYZ> &keyframe,
                       const pcl::PointCloud<pcl::Normal> &k_normal,
                       const pcl::PointCloud<pcl::Normal> &l_normal,
                       const pcl::PointCloud<pcl::Normal> &key_normal,
                       double *pose, double &mean);

double ComputeVelocity(const double delta, const double prev_time,
                       const double cur_time);
}

#endif // ICP_H