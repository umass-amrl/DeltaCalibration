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

#include "delta_calibration/icp.h"

using std::size_t;
using std::vector;
using namespace std;
using Eigen::Vector3d;

namespace icp {

bool first_nn = true;
int count = 0;
const float nn_dist = .1;
float neighbor_dist = .1;
vector<int> x_pos;
vector<int> y_pos;

// For checking if the mean has not changed
bool DoubleEquals(double x, double y) { return fabs(x - y) < .000005; }

bool HistogramEquals(double x, double y) { return fabs(x - y) < .0001; }

void PublishCloud(const pcl::PointCloud<pcl::PointXYZ> &cloud,
                  ros::Publisher publisher) {
  sensor_msgs::PointCloud2 temp_cloud;
  pcl::PCLPointCloud2 pcl_cloud;
  pcl::toPCLPointCloud2(cloud, pcl_cloud);
  pcl_conversions::fromPCL(pcl_cloud, temp_cloud);
  temp_cloud.header.frame_id = "point_cloud";
  publisher.publish(temp_cloud);
}

void PublishCloud(const pcl::PointCloud<pcl::PointXYZRGB> &cloud,
                  ros::Publisher publisher) {
  sensor_msgs::PointCloud2 temp_cloud;
  pcl::PCLPointCloud2 pcl_cloud;
  pcl::toPCLPointCloud2(cloud, pcl_cloud);
  pcl_conversions::fromPCL(pcl_cloud, temp_cloud);
  temp_cloud.header.frame_id = "map";

  publisher.publish(temp_cloud);
}

// Transforms a given eigen point by the given transform (array input)
template <class T>
Eigen::Matrix<T, 3, 1> TransformPoint(const Eigen::Matrix<T, 3, 1> &point,
                                      const T *transform) {
  T point_t[] = {T(point.x()), T(point.y()), T(point.z())};
  T transformed_point[3];

  ceres::AngleAxisRotatePoint(transform, point_t, transformed_point);
  for (int i = 0; i < 3; ++i) {
    transformed_point[i] += transform[3 + i];
  }
  return (Eigen::Matrix<T, 3, 1>(transformed_point[0], transformed_point[1],
                                 transformed_point[2]));
}

double *InvertTransform(double *transform) {
  // Create the eigen transform from the camera
  Eigen::Matrix<double, 3, 1> axis(transform[0], transform[1], transform[2]);
  const double angle = axis.norm();

  if (angle > 0) {
    axis = axis / angle;
  }

  const Eigen::Transform<double, 3, Eigen::Affine> rotation =
      Eigen::Transform<double, 3, Eigen::Affine>(
          Eigen::AngleAxis<double>(angle, axis));

  const Eigen::Translation<double, 3> translation =
      Eigen::Translation<double, 3>(transform[3], transform[4], transform[5]);
  // Compute the full transform
  Eigen::Transform<double, 3, Eigen::Affine> affine_transform =
      translation * rotation;
  // invert it
  affine_transform = affine_transform.inverse();
  // Retrieve the rotation
  Eigen::AngleAxis<double> angle_axis(affine_transform.rotation());
  // Get the axis
  Eigen::Matrix<double, 3, 1> normal_axis = angle_axis.axis();
  double combined_angle = (double)angle_axis.angle();
  Eigen::Matrix<double, 3, 1> combined_axis = normal_axis * combined_angle;
  double *posek = new double[6];
  // retrieve the translation
  Eigen::Translation<double, 3> combined_translation(
      affine_transform.translation());
  posek[0] = combined_axis(0);
  posek[1] = combined_axis(1);
  posek[2] = combined_axis(2);
  posek[3] = combined_translation.x();
  posek[4] = combined_translation.y();
  posek[5] = combined_translation.z();
  return posek;
}

// Transforms a given eigen point by the given transform's inverse.
template <class T>
Eigen::Matrix<T, 3, 1> TransformPointInv(const Eigen::Matrix<T, 3, 1> &point,
                                         const T transform[6]) {

  // Create the eigen transform from the input transform
  Eigen::Matrix<T, 3, 1> axis(transform[0], transform[1], transform[2]);
  const T angle = axis.norm();
  if (angle > T(0)) {
    axis = axis / angle;
  }

  const Eigen::Transform<T, 3, Eigen::Affine> rotation =
      Eigen::Transform<T, 3, Eigen::Affine>(Eigen::AngleAxis<T>(angle, axis));

  Eigen::Matrix<T, 3, 1> transformed_point = point;
  Eigen::Transform<T, 3, Eigen::Affine> affine_transform = rotation;

  transformed_point = affine_transform.inverse() * transformed_point;
  for (int i = 0; i < 3; ++i) {
    transformed_point[i] += transform[3 + i];
  }
  return transformed_point;
}

// Transforms a given eigen point by the given quaternion transform.
Eigen::Matrix<double, 3, 1>
TransformPointQuaternion(const Eigen::Matrix<double, 3, 1> &point,
                         double *pose) {
  Eigen::Quaternion<double> quat_transform =
      Eigen::Quaternion<double>(pose[3], pose[0], pose[1], pose[2]);
  quat_transform.normalize();
  Eigen::Matrix<double, 3, 1> transformed_point;
  transformed_point[0] = point[0];
  transformed_point[1] = point[1];
  transformed_point[2] = point[2];
  Eigen::Matrix<double, 3, 1> trans;
  trans[0] = pose[4];
  trans[1] = pose[5];
  trans[2] = pose[6];
  Eigen::Transform<double, 3, Eigen::Affine> transform;
  transform = quat_transform;
  Eigen::Translation<double, 3> translation =
      Eigen::Translation<double, 3>(trans[0], trans[1], trans[2]);
  transform = translation * transform;
  transformed_point = transform * transformed_point;

  return (Eigen::Matrix<double, 3, 1>(
      transformed_point[0], transformed_point[1], transformed_point[2]));
}

// Translates along a given vector by given amount
template <class T>
Eigen::Matrix<T, 3, 1> VectorTranslate(const Eigen::Matrix<T, 3, 1> &point,
                                       const T magnitude,
                                       const Eigen::Matrix<T, 3, 1> &vector) {

  Eigen::Matrix<T, 3, 1> transformed_point;
  transformed_point = point + (magnitude * vector);
  return (Eigen::Matrix<T, 3, 1>(transformed_point[0], transformed_point[1],
                                 transformed_point[2]));
}

Eigen::Matrix<double, 4, 1> TransformDifference(double *transform_base,
                                                double *transform) {

  Eigen::Matrix<double, 4, 1> output;
  // Create the eigen transform from the first pose
  Eigen::Matrix<double, 3, 1> axis(transform[0], transform[1], transform[2]);
  const double angle = axis.norm();
  if (angle != double(0)) {
    axis = axis / angle;
  }
  Eigen::Transform<double, 3, Eigen::Affine> rotation =
      Eigen::Transform<double, 3, Eigen::Affine>(
          Eigen::AngleAxis<double>(angle, axis));

  Eigen::Translation<double, 3> translation =
      Eigen::Translation<double, 3>(transform[3], transform[4], transform[5]);
  Eigen::Transform<double, 3, Eigen::Affine> affine_transform =
      translation * rotation;

  // Create the eigen transform from the second pose
  Eigen::Matrix<double, 3, 1> axis_base(transform_base[0], transform_base[1],
                                        transform_base[2]);
  const double angle_base = axis_base.norm();
  if (angle_base != double(0)) {
    axis_base = axis_base / angle_base;
  }
  Eigen::Transform<double, 3, Eigen::Affine> rotation_base =
      Eigen::Transform<double, 3, Eigen::Affine>(
          Eigen::AngleAxis<double>(angle_base, axis_base));

  Eigen::Translation<double, 3> translation_base =
      Eigen::Translation<double, 3>(transform_base[3], transform_base[4],
                                    transform_base[5]);

  Eigen::Transform<double, 3, Eigen::Affine> affine_transform_base =
      translation_base * rotation_base;

  Eigen::Transform<double, 3, Eigen::Affine> difference =
      affine_transform.inverse() * affine_transform_base;

  // Find the rotation component
  // Find the angle axis format
  Eigen::AngleAxis<double> angle_axis(difference.rotation());
  Eigen::Vector3d trans1 = {transform[3], transform[4], transform[5]};
  Eigen::Vector3d trans2 = {transform_base[3], transform_base[4],
                            transform_base[5]};
  // Get the axis
  //   Eigen::Vector3d normal_axis = angle_axis.axis();

  // Recompute the rotation angle
  double combined_angle = angle_axis.angle();
  // Compute Translation
  //   Eigen::Matrix<double, 3,1> combined_translation(
  //     difference.translation());
  Eigen::Vector3d combined_translation = trans1 - trans2;
  output[0] = combined_angle;
  output[1] = combined_angle / angle;
  output[2] = combined_translation.norm();
  if (!DoubleEquals(trans2.norm(), 0)) {
    output[3] = (double)combined_translation.norm() / trans2.norm();
  } else {
    output[3] = (double)combined_translation.norm();
  }

  return output;
}

// Transforms a given eigen point by the given transform (array input)
template <class T>
Eigen::Matrix<T, 2, 1> TransformDifference(const T transform_base[6],
                                           const T transform[6]) {

  Eigen::Matrix<T, 2, 1> output;
  // Create the eigen transform from the first pose
  Eigen::Matrix<T, 3, 1> axis(transform[0], transform[1], transform[2]);
  const T angle = axis.norm();
  if (angle != T(0)) {
    axis = axis / angle;
  }
  Eigen::Transform<T, 3, Eigen::Affine> rotation =
      Eigen::Transform<T, 3, Eigen::Affine>(Eigen::AngleAxis<T>(angle, axis));

  Eigen::Translation<T, 3> translation =
      Eigen::Translation<T, 3>(transform[3], transform[4], transform[5]);

  Eigen::Transform<T, 3, Eigen::Affine> affine_transform =
      translation * rotation;

  // Create the eigen transform from the second pose
  Eigen::Matrix<T, 3, 1> axis_base(transform_base[0], transform_base[1],
                                   transform_base[2]);
  const T angle_base = axis_base.norm();
  if (angle_base != T(0)) {
    axis_base = axis_base / angle_base;
  }
  Eigen::Transform<T, 3, Eigen::Affine> rotation_base =
      Eigen::Transform<T, 3, Eigen::Affine>(
          Eigen::AngleAxis<T>(angle_base, axis_base));

  Eigen::Translation<T, 3> translation_base = Eigen::Translation<T, 3>(
      transform_base[3], transform_base[4], transform_base[5]);

  Eigen::Transform<T, 3, Eigen::Affine> affine_transform_base =
      translation_base * rotation_base;

  Eigen::Transform<T, 3, Eigen::Affine> difference =
      affine_transform.inverse() * affine_transform_base;

  Eigen::Matrix<T, 3, 3> matrix(difference.rotation());

  // Recompute the rotation angle
  T final_angle =
      acos((matrix(0, 0), +matrix(1, 1) + matrix(2, 2) - (T)1) / (T)2);
  output[0] = final_angle;
  // Compute Translation
  Eigen::Matrix<T, 3, 1> combined_translation(difference.translation());
  output[1] = (T)combined_translation.norm();
  return output;
}

// Transforms a given eigen point by the given transform (array input)
template <class T>
Eigen::Matrix<T, 4, 1> TransformPlane(const Eigen::Matrix<T, 4, 1> &point,
                                      const T transform[6]) {
  T point_t[] = {T(point[0]), T(point[1]), T(point[2])};
  T transformed_point[3];
  ceres::AngleAxisRotatePoint(transform, point_t, transformed_point);
  return (Eigen::Matrix<T, 4, 1>(transformed_point[0], transformed_point[1],
                                 transformed_point[2], T(point[3])));
}

// Transforms a given eigen vector (not point!) by the given transform
// (array input)
template <class T>
Eigen::Matrix<T, 3, 1> TransformVector(const Eigen::Matrix<T, 3, 1> &vector,
                                       const T transform[6]) {
  T vector_t[] = {T(vector.x()), T(vector.y()), T(vector.z())};
  T transformed_vector[3];
  ceres::AngleAxisRotatePoint(transform, vector_t, transformed_vector);
  return (Eigen::Matrix<T, 3, 1>(transformed_vector[0], transformed_vector[1],
                                 transformed_vector[2]));
}

void GeometryTransform(geometry_msgs::Point *point, const double transform[6]) {
  Eigen::Matrix<double, 3, 1> point_eig;
  point_eig[0] = point->x;
  point_eig[1] = point->y;
  point_eig[2] = point->z;
  point_eig = TransformPoint(point_eig, transform);
  point->x = point_eig[0];
  point->y = point_eig[1];
  point->z = point_eig[2];
}

Eigen::Vector3d PointToDepthImage(const pcl::PointXYZ &point) {
  Eigen::Vector3d depth_point;
  double width = 480;
  double height = 640;
  double fovH = RAD(58.5);
  double fovV = RAD(46.6);
  double tanHFovH = tan(fovH * 0.5);
  double tanHFovV = tan(fovV * 0.5);
  double col = point.y / point.x;
  double row = point.z / point.x;
  col = (.5 * (double(width) - 1.0) * (tanHFovH - col)) / tanHFovH;
  row = (.5 * (double(height) - 1.0) * (tanHFovV - row)) / tanHFovV;
  //   double y = -(float(col)/((float(width)-1.0)*0.5)-1.0)*tanHFovH;
  //   double z = -(float(row)/((float(height)-1.0)*0.5)-1.0)*tanHFovV;
  depth_point[0] = col;
  depth_point[1] = row;
  depth_point[2] = point.x;
  return depth_point;
}

// Returns true if a point is not outside a central window
// of the original depth image
bool NotEdge(const pcl::PointXYZ &point) {

  double width = 640;
  double height = 480;
  double fovH = RAD(58.5);
  double fovV = RAD(46.6);
  //   double f = 600.0;
  double tanHFovH = tan(fovH * 0.5);
  double tanHFovV = tan(fovV * 0.5);

  double col = point.y / point.x;
  double row = point.z / point.x;
  col = (.5 * (double(width) - 1.0) * (tanHFovH - col)) / tanHFovH;
  row = (.5 * (double(height) - 1.0) * (tanHFovV - row)) / tanHFovV;
  double y = -(float(col) / ((float(width) - 1.0) * 0.5) - 1.0) * tanHFovH;
  double z = -(float(row) / ((float(height) - 1.0) * 0.5) - 1.0) * tanHFovV;
  y = y * point.x;
  z = z * point.x;
  //   col = col / point.x;
  //   row = row / point.x;
  if (col < 640 && row < 460 && col > 20 && row > 20) {

    return true;
  } else if (col < 641 && row < 481) {

    return false;
  } else if (col > 640 || row > 480) {
    //     fprintf(stdout, "Col: %f \t Row: %f \n", col, row);
    //     fprintf(stdout, "O-Z: %f \t Z: %f \t O-Y: %f \t Y: %f \n", point.z,
    //     z, point.y, y);
  }
  return true;
}

double KdTreeNN(const double nn_dist,
                const pcl::PointCloud<pcl::PointXYZ> &pointcloud,
                const pcl::PointCloud<pcl::PointXYZ> &transformed_cloud,
                const pcl::PointCloud<pcl::Normal> &normal_1,
                const pcl::PointCloud<pcl::Normal> &normal_2,
                const vector<Eigen::Vector2d> &image_coords_1,
                const vector<Eigen::Vector2d> &image_coords_2,
                vector<int> &nearest_neigbors, vector<int> &start_points) {

  vector<double> residual_magnitudes;

  typedef float KDTree_t;
  static const KDTree_t kThreshold = nn_dist;
  vector<KDNodeValue<KDTree_t, 3>> points;
  vector<KDNodeValue<KDTree_t, 3>> transform_points;
  // Initialize the points for the kd_tree (pulled from the pcl clouds)
  for (size_t i = 0; i < pointcloud.size(); i++) {
    KDNodeValue<KDTree_t, 3> temp1;
    if (!isnan(pointcloud.points[i].x) && !isnan(pointcloud.points[i].y) &&
        !isnan(pointcloud.points[i].z)) {
      temp1.point(0) = pointcloud.points[i].x;
      temp1.point(1) = pointcloud.points[i].y;
      temp1.point(2) = pointcloud.points[i].z;
      temp1.index = i;
      if (temp1.point(0) != 0 && temp1.point(1) != 0 && temp1.point(2) != 0) {
        points.push_back(temp1);
      }
    }
  }

  for (size_t i = 0; i < transformed_cloud.size(); i++) {
    KDNodeValue<KDTree_t, 3> temp2;
    if (!isnan(transformed_cloud.points[i].x) &&
        !isnan(transformed_cloud.points[i].y) &&
        !isnan(transformed_cloud.points[i].z)) {
      temp2.point(0) = transformed_cloud.points[i].x;
      temp2.point(1) = transformed_cloud.points[i].y;
      temp2.point(2) = transformed_cloud.points[i].z;

      temp2.index = i;
      if (temp2.point(0) != 0 && temp2.point(1) != 0 && temp2.point(2) != 0) {
        transform_points.push_back(temp2);
      }
    }
  }
  // Run the actual nearest neighbor calculation
  KDTree<KDTree_t, 3> tree(points);

  double mean = 0;
  // double maxError = 0;
  vector<int> nearest_neigbors_temp;
  vector<double> start_points_temp;
  nearest_neigbors_temp.resize(transform_points.size());
  start_points_temp.resize(transform_points.size());
  vector<double> dists;
  dists.resize(transform_points.size());
  residual_magnitudes.resize(transform_points.size());
  OMP_PARALLEL_FOR
  for (unsigned int i = 0; i < transform_points.size(); i++) {
    pcl::PointXYZ neighborPoint;
    pcl::PointXYZ startPoint;
    KDNodeValue<KDTree_t, 3> neighbor;
    tree.FindNearestPoint(transform_points[i].point, kThreshold, &neighbor);

    double dist = (transform_points[i].point - neighbor.point).norm();
    Eigen::Vector3f normal1, normal2;
    normal1[0] = normal_1[neighbor.index].normal_x;
    normal1[1] = normal_1[neighbor.index].normal_y;
    normal1[2] = normal_1[neighbor.index].normal_z;
    normal2[0] = normal_2[transform_points[i].index].normal_x;
    normal2[1] = normal_2[transform_points[i].index].normal_y;
    normal2[2] = normal_2[transform_points[i].index].normal_z;
    Eigen::Vector3f diff1 = (transform_points[i].point - neighbor.point);
    Eigen::Vector3f diff2 = (transform_points[i].point - neighbor.point);
    double res_dist = diff1.dot(normal1);
    double res_dist2 = diff2.dot(normal2);
    //       double res_magnitude = sqrt(pow(res_dist,2) + pow(res_dist2,2));
    residual_magnitudes[i] = dist;
    if (abs(dist) < nn_dist && abs(res_dist) < nn_dist &&
        abs(res_dist2) < nn_dist) {
      nearest_neigbors_temp[i] = neighbor.index;
      start_points_temp[i] = transform_points[i].index;
      dists[i] = dist;
      mean += dist;
    } else {
      dists[i] = 9999;
    }
  }

  for (size_t i = 0; i < transform_points.size(); i++) {
    if (dists[i] < 9999) {
      //       if(image_coords_2[nearest_neigbors[i]][0] < 620
      //         && image_coords_2[nearest_neigbors[i]][1] < 440
      //         && image_coords_2[nearest_neigbors[i]][0] > 20
      //         && image_coords_2[nearest_neigbors[i]][1] > 20) {
      nearest_neigbors.push_back(nearest_neigbors_temp[i]);
      start_points.push_back(start_points_temp[i]);
      //        }
    }
  }
  first_nn = false;
  return mean / nearest_neigbors.size();
}

void WritePose(double *pose, ofstream file) {
  if (file.is_open()) {
    for (int j = 0; j < 6; j++) {
      file << pose[j] << "\t";
    }
    file << "\n";
  }
}

Eigen::Matrix3d CalcScatterMatrix(pcl::PointCloud<pcl::Normal> normals) {
  Eigen::Matrix3d scatter;
  scatter << 0, 0, 0, 0, 0, 0, 0, 0, 0;

  for (size_t i = 0; i < normals.size(); i++) {
    Eigen::Vector3d point;
    point[0] = normals[i].normal_x;
    point[1] = normals[i].normal_y;
    point[2] = normals[i].normal_z;

    Eigen::Matrix3d temp = point * point.transpose();
    if (point.norm() > 0) {
      scatter = scatter + temp;
    }
  }
  return scatter / scatter.norm();
}

double CalcConditionNumber(Eigen::Matrix3d mat1) {
  Eigen::JacobiSVD<Eigen::MatrixXd> svd(mat1);
  double cond = svd.singularValues()(0) /
                svd.singularValues()(svd.singularValues().size() - 1);
  return cond;
}

// Transforms all points in a pcl point cloud (array input)
void TransformPointCloud(pcl::PointCloud<pcl::PointXYZ> *cloud,
                         double transform[6]) {
  OMP_PARALLEL_FOR
  for (size_t i = 0; i < cloud->size(); ++i) {
    pcl::PointXYZ point = (*cloud)[i];

    Eigen::Matrix<double, 3, 1> point_matrix;
    Eigen::Matrix<double, 3, 1> transformed_point_matrix;
    point_matrix << point.x, point.y, point.z;
    transformed_point_matrix = TransformPoint(point_matrix, transform);
    point.x = transformed_point_matrix[0];
    point.y = transformed_point_matrix[1];
    point.z = transformed_point_matrix[2];
    (*cloud)[i] = point;
  }
}

// Transforms all points in a pcl point cloud (array input)
void TransformPointCloudQuat(pcl::PointCloud<pcl::PointXYZ> &cloud,
                             double transform[7]) {
  OMP_PARALLEL_FOR
  for (size_t i = 0; i < cloud.size(); ++i) {
    pcl::PointXYZ point = cloud[i];

    Eigen::Matrix<double, 3, 1> point_matrix;
    Eigen::Matrix<double, 3, 1> transformed_point_matrix;
    point_matrix << point.x, point.y, point.z;
    transformed_point_matrix =
        TransformPointQuaternion(point_matrix, transform);
    point.x = transformed_point_matrix[0];
    point.y = transformed_point_matrix[1];
    point.z = transformed_point_matrix[2];
    cloud[i] = point;
  }
}

// Transforms all points in a pcl point cloud (array input)
void VectorTranslatePointCloud(pcl::PointCloud<pcl::PointXYZ> *cloud,
                               const double &magnitude,
                               const Eigen::Matrix<double, 3, 1> &vector) {
  OMP_PARALLEL_FOR
  for (size_t i = 0; i < cloud->size(); ++i) {
    pcl::PointXYZ point = (*cloud)[i];

    Eigen::Matrix<double, 3, 1> point_matrix;
    Eigen::Matrix<double, 3, 1> transformed_point_matrix;
    point_matrix << point.x, point.y, point.z;
    transformed_point_matrix = VectorTranslate(point_matrix, magnitude, vector);
    point.x = transformed_point_matrix[0];
    point.y = transformed_point_matrix[1];
    point.z = transformed_point_matrix[2];
    (*cloud)[i] = point;
  }
}

// Transforms all points in a pcl point cloud (array input)
void TransformPointCloudInv(pcl::PointCloud<pcl::PointXYZ> *cloud,
                            double transform[6]) {

  OMP_PARALLEL_FOR
  for (size_t i = 0; i < cloud->size(); ++i) {
    pcl::PointXYZ point = (*cloud)[i];

    Eigen::Matrix<double, 3, 1> point_matrix;
    Eigen::Matrix<double, 3, 1> transformed_point_matrix;
    point_matrix << point.x, point.y, point.z;
    transformed_point_matrix = TransformPointInv(point_matrix, transform);
    point.x = transformed_point_matrix[0];
    point.y = transformed_point_matrix[1];
    point.z = transformed_point_matrix[2];
    (*cloud)[i] = point;
  }
}

// Transforms all points in a pcl point cloud (vector input)
void TransformPointCloud(pcl::PointCloud<pcl::PointXYZ> *cloud,
                         const vector<double> &transform) {

  double transform_array[6];
  std::copy(transform.begin(), transform.end(), transform_array);
  TransformPointCloud(cloud, transform_array);
}

struct PointToPointErrorNumeric {
  PointToPointErrorNumeric(const Vector3d &point_0, const Vector3d &point_1,
                           const Vector3d &normal_0, const Vector3d &normal_1)
      : point_0(point_0), point_1(point_1), normal_0(normal_0),
        normal_1(normal_1) {}

  bool operator()(const double *const camera, double *residuals) const {
    // Transform point_1 to the base reference frame by applying the camera tf.
    const Vector3d point_1_transformed = TransformPoint(point_1, camera);
    // Transform normal_1 to the base reference frame by applying the camera tf.
    const Vector3d normal_1_transformed = TransformVector(normal_1, camera);
    residuals[0] = (point_1_transformed - point_0).dot(normal_0);
    residuals[1] = (point_0 - point_1_transformed).dot(normal_1_transformed);
    return true;
  }

  // Factory to hide the construction of the CostFunction object from
  // the client code.
  static ceres::CostFunction *Create(const Vector3d &point_0,
                                     const Vector3d &point_1,
                                     const Vector3d &normal_0,
                                     const Vector3d &normal_1) {
    // 3 is number of residuals, 6 number of parameters (camera transform)
    return (new ceres::NumericDiffCostFunction<PointToPointErrorNumeric,
                                               ceres::CENTRAL, 2, 6>(
        new PointToPointErrorNumeric(point_0, point_1, normal_0, normal_1)));
  }

  // The point in the reference frame of the camera being aligned.
  const Vector3d point_0;
  // The point in the base reference frame, the fixed camera.
  const Vector3d point_1;
  // The normal to the point in the reference frame of the camera being aligned.
  const Vector3d normal_0;
  // The normal to the point in the base reference frame, the fixed camera.
  const Vector3d normal_1;
};

struct PointToPointError {
  PointToPointError(const Vector3d &point_0, const Vector3d &point_1,
                    const Vector3d &normal_0, const Vector3d &normal_1)
      : point_0(point_0), point_1(point_1), normal_0(normal_0),
        normal_1(normal_1) {}

  template <class T>
  bool operator()(const T *const camera, T *residuals) const {
    // Transform point_1 to the base reference frame by applying the camera tf.
    const Eigen::Matrix<T, 3, 1> point_1_transformed =
        TransformPoint<T>(point_1.cast<T>(), camera);
    // Transform normal_1 to the base reference frame by applying the camera tf.
    const Eigen::Matrix<T, 3, 1> normal_1_transformed =
        TransformVector<T>(normal_1.cast<T>(), camera);
    // The error is the difference between the predicted and observed position.
    residuals[0] =
        (point_1_transformed - point_0.cast<T>()).dot(normal_0.cast<T>());
    residuals[1] =
        (point_0.cast<T>() - point_1_transformed).dot(normal_1_transformed);
    return true;
  }

  // Factory to hide the construction of the CostFunction object from
  // the client code.
  static ceres::CostFunction *Create(const Vector3d &point_0,
                                     const Vector3d &point_1,
                                     const Vector3d &normal_0,
                                     const Vector3d &normal_1) {
    return (new ceres::AutoDiffCostFunction<PointToPointError, 2, 6>(
        new PointToPointError(point_0, point_1, normal_0, normal_1)));
  }

  const Vector3d point_0;
  const Vector3d point_1;
  const Vector3d normal_0;
  const Vector3d normal_1;
};

struct PlaneToPlaneError {
  PlaneToPlaneError(const Eigen::Vector4d plane_1,
                    const Eigen::Vector4d plane_2,
                    const Eigen::Vector3d centroid_1,
                    const Eigen::Vector3d centroid_2)
      : plane_1(plane_1), plane_2(plane_2), centroid_1(centroid_1),
        centroid_2(centroid_2) {}

  template <class T>
  bool operator()(const T *const camera, T *residuals) const {
    // Transform point_1 to the base reference frame by applying the camera tf.
    const Eigen::Matrix<T, 4, 1> plane_2_transformed =
        TransformPlane<T>(plane_2.cast<T>(), camera);
    const Eigen::Matrix<T, 3, 1> centroid_2_transformed =
        TransformPoint<T>(centroid_2.cast<T>(), camera);
    Eigen::Matrix<T, 3, 1> point_2_transformed;
    point_2_transformed[0] = plane_2_transformed[0];
    point_2_transformed[1] = plane_2_transformed[1];
    point_2_transformed[2] = plane_2_transformed[2];
    Eigen::Matrix<T, 4, 1> plane_1_t = plane_1.cast<T>();
    Eigen::Matrix<T, 3, 1> point_1;
    point_1[0] = plane_1_t[0];
    point_1[1] = plane_1_t[1];
    point_1[2] = plane_1_t[2];
    // The error is the difference between the predicted and observed position.
    residuals[0] =
        ceres::acos(point_1.dot(point_2_transformed) /
                    ceres::sqrt(point_1.norm() * point_2_transformed.norm()));
    residuals[1] = (centroid_2_transformed - centroid_1.cast<T>()).dot(point_1);
    residuals[2] = (centroid_1.cast<T>() - centroid_2_transformed)
                       .dot(point_2_transformed);
    return true;
  }

  // Factory to hide the construction of the CostFunction object from
  // the client code.
  static ceres::CostFunction *Create(const Eigen::Vector4d plane_1,
                                     const Eigen::Vector4d plane_2,
                                     const Eigen::Vector3d centroid_1,
                                     const Eigen::Vector3d centroid_2) {
    return (new ceres::AutoDiffCostFunction<PlaneToPlaneError, 3, 7>(
        new PlaneToPlaneError(plane_1, plane_2, centroid_1, centroid_2)));
  }

  const Eigen::Vector4d plane_1;
  const Eigen::Vector4d plane_2;
  const Eigen::Vector3d centroid_1;
  const Eigen::Vector3d centroid_2;
};

struct TransformRegularizationError {
  TransformRegularizationError(double *base_transform)
      : base_transform(base_transform) {}

  template <class T>
  bool operator()(const T *const camera, T *residuals) const {
    const Eigen::Matrix<T, 3, 1> rotation =
        Eigen::Matrix<T, 3, 1>(camera[0], camera[1], camera[2]);
    const Eigen::Matrix<T, 3, 1> translation =
        Eigen::Matrix<T, 3, 1>(camera[3], camera[4], camera[5]);
    residuals[0] = rotation.norm() * .1;
    residuals[1] = translation.norm() * .1;
    return true;
  }

  // Factory to hide the construction of the CostFunction object from
  // the client code.
  static ceres::CostFunction *Create(double *base_transform) {
    return (new ceres::AutoDiffCostFunction<TransformRegularizationError, 2, 6>(
        new TransformRegularizationError(base_transform)));
  }

  double *base_transform;
};

void PlaneCorrections(const pcl::PointCloud<pcl::PointXYZ> &cloud_1,
                      const pcl::PointCloud<pcl::PointXYZ> &cloud_2,
                      const vector<Eigen::Vector3d> &k1_centroids,
                      const vector<Eigen::Vector3d> &k2_centroids,
                      const vector<Eigen::Vector4d> &normal_equations,
                      const vector<Eigen::Vector4d> &k2_normal_equations,
                      const vector<ros::Publisher> &publishers,
                      double *transform) {

  // Tolerance for RMSE.
  static const double kToleranceError = 0.00001;
  // The maximum number of overall iterations.
  static const int kMaxIterations = 80;
  // The maximum number of repeat iterations while the RMSE is unchanged.
  static const int kMaxRepeatIterations = 5;
  double rmse = 1000000;
  double last_rmse = 1000010;
  // Optimize until we've reached our tolerance, we've reached our max number
  // of iterations, or values repeating too frequently
  double *base_transform = new double[6];
  memcpy(base_transform, transform, sizeof(double) * 6);
  bool not_first = false;
  for (int iteration = 0, repeat_iteration = 0;
       iteration < kMaxIterations && repeat_iteration < kMaxRepeatIterations &&
       rmse > kToleranceError;
       ++iteration) {
    if (DoubleEquals(rmse, last_rmse)) {
      repeat_iteration++;
    } else {
      repeat_iteration = 0;
    }
    last_rmse = rmse;
    pcl::PointCloud<pcl::PointXYZ> transformed_cloud = cloud_2;

    // Shifts cloud by calculated transform
    TransformPointCloud(&transformed_cloud, transform);
    PublishCloud(cloud_1, publishers[0]);
    PublishCloud(transformed_cloud, publishers[1]);
    sleep(3);
    ceres::Problem problem;
    // Build the problem
    ceres::CostFunction *cost_function =
        PlaneToPlaneError::Create(normal_equations[0], k2_normal_equations[1],
                                  k1_centroids[0], k2_centroids[1]);
    problem.AddResidualBlock(cost_function,
                             new ceres::HuberLoss(0.5), // squared loss
                             transform);
    ceres::CostFunction *cost_function_2 =
        PlaneToPlaneError::Create(normal_equations[1], k2_normal_equations[0],
                                  k1_centroids[1], k2_centroids[0]);
    problem.AddResidualBlock(cost_function_2,
                             new ceres::HuberLoss(0.5), // squared loss
                             transform);
    ceres::CostFunction *cost_function_3 =
        TransformRegularizationError::Create(base_transform);
    if (not_first) {
      problem.AddResidualBlock(cost_function_3,
                               new ceres::HuberLoss(0.5), // squared loss
                               transform);
    }
    not_first = true;
    Eigen::Vector3d transformed_center = k2_centroids[0];
    transformed_center = TransformPoint(transformed_center, transform);
    Eigen::Vector4d transformed_plane = k2_normal_equations[0];
    transformed_plane = TransformPlane(transformed_plane, transform);
    transformed_plane = transformed_plane;
    Eigen::Vector3d transformed_center2 = k2_centroids[1];
    Eigen::Vector4d transformed_plane2 = k2_normal_equations[1];
    transformed_plane2 = TransformPlane(transformed_plane2, transform);
    transformed_plane2 = transformed_plane2;
    transformed_center2 = TransformPoint(transformed_center2, transform);

    // Line Lists Publishing (For visualizing normals)
    visualization_msgs::Marker line_list;
    std_msgs::Header header;
    line_list.action = visualization_msgs::Marker::ADD;
    // line_list.pose.orientation.w = 1.0;
    line_list.header.frame_id = "point_cloud";

    line_list.id = 0;
    line_list.type = visualization_msgs::Marker::LINE_LIST;
    line_list.scale.x = 0.001;

    // Line list is red
    line_list.color.r = 1.0;
    line_list.color.a = 1.0;
    geometry_msgs::Point p, p2;
    p.x = transformed_center[0];
    p.y = transformed_center[1];
    p.z = transformed_center[2];
    p2.x = transformed_center[0] + transformed_plane[0] * 2;
    p2.y = transformed_center[1] + transformed_plane[1] * 2;
    p2.z = transformed_center[2] + transformed_plane[2] * 2;

    line_list.points.push_back(p);
    line_list.points.push_back(p2);
    p.x = transformed_center2[0];
    p.y = transformed_center2[1];
    p.z = transformed_center2[2];
    p2.x = transformed_center2[0] + transformed_plane2[0] * 2;
    p2.y = transformed_center2[1] + transformed_plane2[1] * 2;
    p2.z = transformed_center2[2] + transformed_plane2[2] * 2;
    line_list.points.push_back(p);
    line_list.points.push_back(p2);
    p.x = k1_centroids[0][0];
    p.y = k1_centroids[0][1];
    p.z = k1_centroids[0][2];
    p2.x = k1_centroids[0][0] + normal_equations[0][0] * 2;
    p2.y = k1_centroids[0][1] + normal_equations[0][1] * 2;
    p2.z = k1_centroids[0][2] + normal_equations[0][2] * 2;
    line_list.points.push_back(p);
    line_list.points.push_back(p2);
    p.x = k1_centroids[1][0];
    p.y = k1_centroids[1][1];
    p.z = k1_centroids[1][2];
    p2.x = k1_centroids[1][0] + normal_equations[1][0] * 2;
    p2.y = k1_centroids[1][1] + normal_equations[1][1] * 2;
    p2.z = k1_centroids[1][2] + normal_equations[1][2] * 2;
    line_list.points.push_back(p);
    line_list.points.push_back(p2);
    publishers[4].publish(line_list);

    // Run Ceres problem
    ceres::Solver::Options options;
    options.num_threads = 1;
    options.num_linear_solver_threads = 1;
    options.linear_solver_type = ceres::SPARSE_SCHUR;
    ceres::Solver::Summary summary;
    ceres::Solve(options, &problem, &summary);
    rmse =
        sqrt(summary.final_cost / static_cast<double>(summary.num_residuals));
    ceres::Problem::EvaluateOptions evalOptions =
        ceres::Problem::EvaluateOptions();
  }
}

double PointDistance(const pcl::PointXYZ point1, const pcl::PointXYZ point2) {
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

double PointDistance(const Eigen::Vector3d point1,
                     const Eigen::Vector3d point2) {
  double distance = 0;
  double x1 = point1[0];
  double y1 = point1[1];
  double z1 = point1[2];

  double x2 = point2[0];
  double y2 = point2[1];
  double z2 = point2[2];

  distance = sqrt(pow(x2 - x1, 2) + pow(y2 - y1, 2) + pow(z2 - z1, 2));

  return distance;
}

double PointDistance(const Eigen::Vector3f point1,
                     const Eigen::Vector3f point2) {
  double distance = 0;
  double x1 = point1[0];
  double y1 = point1[1];
  double z1 = point1[2];

  double x2 = point2[0];
  double y2 = point2[1];
  double z2 = point2[2];

  distance = sqrt(pow(x2 - x1, 2) + pow(y2 - y1, 2) + pow(z2 - z1, 2));

  return distance;
}

void BuildProblem(const pcl::PointCloud<pcl::PointXYZ> &k_cloud,
                  const pcl::PointCloud<pcl::PointXYZ> &l_cloud,
                  const vector<int> &nearest_neighbors,
                  const vector<int> &start_points, double l_pose[6],
                  const pcl::PointCloud<pcl::Normal> &normals,
                  const pcl::PointCloud<pcl::Normal> &normals_l,
                  ceres::Problem *problem) {
  // Ceres-Solver Problem setup
  for (size_t i = 0; i < start_points.size(); i++) {
    pcl::PointXYZ l_point, k_point;
    l_point = l_cloud[start_points[i]];
    k_point = k_cloud[nearest_neighbors[i]];
    pcl::Normal normal_k = normals[nearest_neighbors[i]];
    pcl::Normal normal_k1 = normals_l[i];

    // Calculate distance to avoid using points too far apart
    double distance = PointDistance(k_point, l_point);

    // Add non nan values to the problem
    if (distance < nn_dist && distance > 0) {
      const Vector3d point_0(k_point.x, k_point.y, k_point.z);
      const Vector3d point_1(l_point.x, l_point.y, l_point.z);
      const Vector3d normal_0(normal_k.normal_x, normal_k.normal_y,
                              normal_k.normal_z);
      const Vector3d normal_1(normal_k1.normal_x, normal_k1.normal_y,
                              normal_k1.normal_z);
      if (point_0.allFinite() && point_1.allFinite() && normal_0.allFinite() &&
          normal_1.allFinite()) {
        ceres::CostFunction *cost_function = PointToPointErrorNumeric::Create(
            point_0, point_1, normal_0, normal_1);
        problem->AddResidualBlock(cost_function,
                                  new ceres::HuberLoss(0.5), // squared loss
                                  l_pose);
      }
    }
  }
}

double ResidualDist(const pcl::PointCloud<pcl::PointXYZ> &k_cloud,
                    const pcl::PointCloud<pcl::PointXYZ> &l_cloud,
                    const pcl::PointCloud<pcl::Normal> &k_normal,
                    const pcl::PointCloud<pcl::Normal> &l_normal,
                    double *pose) {

  // Create a new problem
  ceres::Problem problem;

  //----------  Find Nearest Neighbors  ----------
  vector<int> nearest_neigbors;
  vector<int> start_points;
  vector<Eigen::Vector2d> image_coords_1;
  vector<Eigen::Vector2d> image_coords_2;
  KdTreeNN(nn_dist, k_cloud, l_cloud, k_normal, l_normal, image_coords_1,
           image_coords_2, nearest_neigbors, start_points);

  //----------  Add to ceres problem  ----------
  // We give this pose k instead of l, and switch the order of the point clouds
  // and normals passed in to make sure that it's a transformation from k to l
  // being calculated
  BuildProblem(k_cloud, l_cloud, nearest_neigbors, start_points, pose, k_normal,
               l_normal, &problem);
  //----------  Ceres Solve  ----------
  ceres::Solver::Options options;
  options.num_threads = 12;
  options.num_linear_solver_threads = 12;
  options.linear_solver_type = ceres::ITERATIVE_SCHUR;
  vector<double> residuals;
  residuals.clear();
  ceres::Problem::EvaluateOptions evalOptions =
      ceres::Problem::EvaluateOptions();
  // std::vector<double*> parameter_blocks;
  // parameter_blocks.push_back(arrays[k+1]);
  // evalOptions.parameter_blocks = parameter_blocks;
  // ceres::CRSMatrix jacobian;
  problem.Evaluate(evalOptions, NULL, &residuals, NULL, NULL);
  double rmse = 0.0;
  for (size_t i = 0; i < residuals.size(); ++i) {
    rmse += sq(residuals[i]);
  }
  rmse = sqrt(rmse / static_cast<double>(residuals.size()));
  return rmse;
}

double *CombineTransform(double *pose1, double *pose2) {

  Eigen::Transform<double, 3, Eigen::Affine> combine_transform;

  // For the two poses create a transform
  double *posek = new double[6];
  double *transform = pose1;

  // Create the eigen transform from the first pose
  Eigen::Matrix<double, 3, 1> axis(transform[0], transform[1], transform[2]);
  const double angle = axis.norm();
  if (angle != 0) {
    axis = axis / angle;
  }
  Eigen::Transform<double, 3, Eigen::Affine> rotation =
      Eigen::Transform<double, 3, Eigen::Affine>(
          Eigen::AngleAxis<double>(angle, axis));

  Eigen::Translation<double, 3> translation =
      Eigen::Translation<double, 3>(transform[3], transform[4], transform[5]);

  Eigen::Transform<double, 3, Eigen::Affine> affine_transform =
      translation * rotation;

  // For the second pose
  // double* posek = new double[6];
  double *transform2 = pose2;

  // Create the eigen transform from the pose
  Eigen::Matrix<double, 3, 1> axis_2(transform2[0], transform2[1],
                                     transform2[2]);
  const double angle_2 = axis.norm();
  if (angle_2 != 0) {
    axis_2 = axis_2 / angle_2;
  }
  Eigen::Transform<double, 3, Eigen::Affine> rotation_2 =
      Eigen::Transform<double, 3, Eigen::Affine>(
          Eigen::AngleAxis<double>(angle_2, axis_2));

  Eigen::Translation<double, 3> translation_2 = Eigen::Translation<double, 3>(
      transform2[3], transform2[4], transform2[5]);

  Eigen::Transform<double, 3, Eigen::Affine> affine_transform_2 =
      translation_2 * rotation_2;

  // Combine the two transforms
  combine_transform = affine_transform_2 * affine_transform;

  // Find the rotation component
  // Find the angle axis format
  Eigen::AngleAxis<double> angle_axis(combine_transform.rotation());

  // Get the axis
  Eigen::Vector3d normal_axis = angle_axis.axis();

  // Recompute the rotation angle
  double combined_angle = angle_axis.angle();
  Eigen::Vector3d combined_axis = normal_axis * combined_angle;

  // Recompute the rotation matrix
  // Eigen::Transform<double, 3, Eigen::Affine> combined_rotation =
  //   Eigen::Transform<double, 3, Eigen::Affine>(
  //     Eigen::AngleAxis<double>(combined_angle, normal_axis));

  // Compute Translation
  Eigen::Translation<double, 3> combined_translation(
      combine_transform.translation());

  // Assign values to pose
  //   posek[3] = (combined_rotation.inverse() *
  //       combined_translation).translation().x();
  //   posek[4] = (combined_rotation.inverse() *
  //       combined_translation).translation().y();
  //   posek[5] = (combined_rotation.inverse() *
  //       combined_translation).translation().z();
  posek[3] = combined_translation.x();
  posek[4] = combined_translation.y();
  posek[5] = combined_translation.z();
  // Recompute the rotation angle
  posek[0] = combined_axis(0);
  posek[1] = combined_axis(1);
  posek[2] = combined_axis(2);
  if (DoubleEquals(posek[0], 0) && DoubleEquals(posek[1], 0) &&
      DoubleEquals(posek[2], 0)) {
    posek[0] = pose2[0];
    posek[1] = pose2[1];
    posek[2] = pose2[2];
  }
  return posek;
}

void VisualizeDepthImage(const pcl::PointCloud<pcl::PointXYZ> &cloud,
                         const ros::Publisher &depth_pub) {
  int width = 640;
  int height = 480;
  //   double a = 3.008;
  //   double b = -.002745;
  uint16_t image[(int)width * (int)height];
  for (size_t i = 0; i < cloud.size(); i++) {

    pcl::PointXYZ point = cloud[i];
    Eigen::Vector3d image_point = PointToDepthImage(point);
    //     int ind = image_point[0]*width + image_point[1];
    int ind2 = width * height - i - 1 + 0 * width * height;
    if (ind2 < 640 * 480) {
      //       if(col  < 0 ) {
      //         image[ind2] = 0;
      //       } else {
      image[ind2] = image_point[2];
    }
    //     }
  }
  sensor_msgs::Image out_image;
  out_image.height = 480;
  out_image.width = 640;
  out_image.encoding = sensor_msgs::image_encodings::MONO16;
  out_image.is_bigendian = 0;
  out_image.step = 640 * 2;
  out_image.data.clear();
  out_image.data.resize(640 * 480 * 2);
  // out_image.header.stamp = frame;
  uint8_t *ptrDest = out_image.data.data();
  memcpy(ptrDest, image, 614400);
  out_image.header.frame_id = "point_clouds";
  depth_pub.publish(out_image);
}

void VisualizeNN(const pcl::PointCloud<pcl::PointXYZ> &base_cloud,
                 const pcl::PointCloud<pcl::PointXYZ> &k1_cloud,
                 const vector<int> &nearest_neighbors,
                 const vector<int> &start_points,
                 const ros::Publisher &marker_pub) {

  visualization_msgs::Marker line_list;
  std_msgs::Header header;
  header.frame_id = "point_clouds";
  line_list.action = visualization_msgs::Marker::ADD;
  // line_list.pose.orientation.w = 1.0;
  line_list.header.frame_id = "point_cloud";

  line_list.id = 0;
  line_list.type = visualization_msgs::Marker::LINE_LIST;
  line_list.scale.x = 0.0001;

  // Line list is red
  line_list.color.r = 1.0;
  line_list.color.a = 1.0;
  geometry_msgs::Point combined_result, zero;
  zero.x = 0;
  zero.y = 0;
  zero.z = 0;
  for (size_t i = 0; i < start_points.size(); ++i) {
    pcl::PointXYZ temp_point = k1_cloud[start_points[i]];
    pcl::PointXYZ temp_point2 = base_cloud[nearest_neighbors[i]];
    if (!isnan(temp_point.x) && !isnan(temp_point.y) && !isnan(temp_point.z) &&
        !isnan(temp_point2.x) && !isnan(temp_point2.y) &&
        !isnan(temp_point2.z)) {
      geometry_msgs::Point p, p2;
      p.x = temp_point.x;
      p.y = temp_point.y;
      p.z = temp_point.z;
      p2.x = temp_point2.x; //((temp_point2.x - p.x)*10) + p.x;
      p2.y = temp_point2.y; //((temp_point2.y - p.y)*10) + p.y;
      p2.z = temp_point2.z; //((temp_point2.z - p.z)*10) + p.z;
      Eigen::Vector3d current_residual;
      combined_result.x += p2.x;
      combined_result.y += p2.y;
      combined_result.z += p2.z;

      line_list.points.push_back(p);
      line_list.points.push_back(p2);
    }
  }
  double norm = sqrt(pow(combined_result.x, 2) + pow(combined_result.y, 2) +
                     pow(combined_result.z, 2));
  combined_result.x = (combined_result.x / norm);
  combined_result.y = (combined_result.y / norm);
  combined_result.z = (combined_result.z / norm);
  //   line_list.points.push_back(zero);
  //   line_list.points.push_back(combined_result);
  marker_pub.publish(line_list);
}

void VisualizeReferenceFrame(const double *transform,
                             const ros::Publisher &marker_pub, const int id) {

  visualization_msgs::MarkerArray marker_array =
      visualization_msgs::MarkerArray();
  visualization_msgs::Marker x_arrow, y_arrow, z_arrow;
  x_arrow.header.frame_id = y_arrow.header.frame_id = z_arrow.header.frame_id =
      "point_cloud";
  x_arrow.action = y_arrow.action = z_arrow.action =
      visualization_msgs::Marker::ADD;
  x_arrow.type = y_arrow.type = z_arrow.type =
      visualization_msgs::Marker::ARROW;
  x_arrow.id = id;
  y_arrow.id = id + 1;
  z_arrow.id = id + 2;

  x_arrow.color.a = y_arrow.color.a = z_arrow.color.a = 1.0;
  x_arrow.color.r = 1.0;
  y_arrow.color.g = 1.0;
  z_arrow.color.b = 1.0;

  x_arrow.scale.x = y_arrow.scale.x = z_arrow.scale.x = .01;
  x_arrow.scale.y = y_arrow.scale.y = z_arrow.scale.y = .015;

  geometry_msgs::Point center, x, y, z;
  center.x = center.y = center.z = x.y = x.z = y.x = y.z = z.x = z.y = 0;
  z.z = .5;
  x.x = .5;
  y.y = .5;
  GeometryTransform(&center, transform);
  GeometryTransform(&x, transform);
  GeometryTransform(&y, transform);
  GeometryTransform(&z, transform);

  x_arrow.points.push_back(center);
  x_arrow.points.push_back(x);
  y_arrow.points.push_back(center);
  y_arrow.points.push_back(y);
  z_arrow.points.push_back(center);
  z_arrow.points.push_back(z);

  marker_array.markers.push_back(x_arrow);
  marker_array.markers.push_back(y_arrow);
  marker_array.markers.push_back(z_arrow);
  marker_pub.publish(marker_array);
}

void VisualizePoses(const vector<double *> &poses,
                    const vector<double *> &poses2, const vector<int> &keys,
                    const ros::Publisher &marker_pub) {

  visualization_msgs::MarkerArray markers;

  double last_x = 0.0;
  double last_y = 0.0;
  double last_z = 0.0;
  double last_x2 = 0.0;
  double last_y2 = 0.0;
  double last_z2 = 0.0;

  visualization_msgs::Marker line_list, line_list2;
  std_msgs::Header header;
  header.frame_id = "point_clouds";
  line_list.action = visualization_msgs::Marker::ADD;
  // line_list.pose.orientation.w = 1.0;
  line_list.header.frame_id = "point_cloud";
  line_list2.action = visualization_msgs::Marker::ADD;
  // line_list.pose.orientation.w = 1.0;
  line_list2.header.frame_id = "point_cloud";

  line_list.id = 0;
  line_list.type = visualization_msgs::Marker::LINE_LIST;
  line_list.scale.x = 0.01;

  line_list2.id = 0;
  line_list2.type = visualization_msgs::Marker::LINE_LIST;
  line_list2.scale.x = 0.01;

  // Line list is red
  line_list.color.r = 1.0;
  line_list.color.a = 1.0;

  // Line list is pur
  line_list2.color.r = 1.0;
  line_list2.color.b = 1.0;
  line_list2.color.a = 1.0;

  Eigen::Matrix<double, 3, 1> point_matrix;

  point_matrix << 0.25, 0.0, 0.0;
  Eigen::Matrix<double, 3, 1> point_matrix2;

  point_matrix2 << 0.25, 0.0, 0.0;

  for (size_t i = 0; i < poses.size(); ++i) {
    visualization_msgs::Marker marker, marker2;
    double *pose = poses[i];
    double *pose2 = poses2[i];
    marker.header.frame_id = "point_cloud";
    marker.id = i;
    marker.header.stamp = ros::Time();
    marker.type = visualization_msgs::Marker::ARROW;
    marker.action = visualization_msgs::Marker::ADD;
    marker2.header.frame_id = "point_cloud";
    marker2.id = (i + poses.size());
    marker2.header.stamp = ros::Time();
    marker2.type = visualization_msgs::Marker::ARROW;
    marker2.action = visualization_msgs::Marker::ADD;

    // marker.scale.z = .01;
    marker.id = i;
    if (std::find(keys.begin(), keys.end(), int(i)) != keys.end()) {
      marker.color.r = 0.0;
      marker.color.g = 1.0;
      marker.color.b = 0.0;
      marker.color.a = 1;
      marker.scale.x = .02;
      marker.scale.y = .02;
      marker2.color.r = 1.0;
      marker2.color.g = 1.0;
      marker2.color.b = 0.0;
      marker2.color.a = 1;
      marker2.scale.x = .02;
      marker2.scale.y = .02;

    } else {
      marker.color.r = 0.0;
      marker.color.g = 0.0;
      marker.color.b = 1.0;
      marker.color.a = 1;
      marker.scale.x = .01;
      marker.scale.y = .01;
      marker2.color.r = 1.0;
      marker2.color.g = 0.0;
      marker2.color.b = 0.0;
      marker2.color.a = 1;
      marker2.scale.x = .01;
      marker2.scale.y = .01;
    }

    point_matrix = TransformPoint(point_matrix, pose);
    point_matrix2 = TransformPoint(point_matrix2, pose2);

    geometry_msgs::Point ap, ap2;

    ap.x = last_x + pose[3];
    ap.y = last_y + pose[4];
    ap.z = last_z + pose[5];
    ap2.x = point_matrix[0];
    ap2.y = point_matrix[1];
    ap2.z = point_matrix[2];
    marker.points.push_back(ap);
    marker.points.push_back(ap2);

    geometry_msgs::Point p, p2;
    p.x = last_x;
    p.y = last_y;
    p.z = last_z;
    p2.x = pose[3];
    p2.y = pose[4];
    p2.z = pose[5];
    line_list.points.push_back(p);
    line_list.points.push_back(p2);

    last_x = pose[3];
    last_y = pose[4];
    last_z = pose[5];

    geometry_msgs::Point ap_2, ap2_2;

    ap_2.x = last_x2 + pose2[3];
    ap_2.y = last_y2 + pose2[4];
    ap_2.z = last_z2 + pose2[5];
    ap2_2.x = point_matrix2[0];
    ap2_2.y = point_matrix2[1];
    ap2_2.z = point_matrix2[2];
    marker2.points.push_back(ap_2);
    marker2.points.push_back(ap2_2);

    geometry_msgs::Point p_2, p2_2;
    p_2.x = last_x2;
    p_2.y = last_y2;
    p_2.z = last_z2;
    p2_2.x = pose[3];
    p2_2.y = pose[4];
    p2_2.z = pose[5];
    line_list2.points.push_back(p_2);
    line_list2.points.push_back(p2_2);

    last_x2 = pose2[3];
    last_y2 = pose2[4];
    last_z2 = pose2[5];

    markers.markers.push_back(marker);
    markers.markers.push_back(marker2);
  }
  marker_pub.publish(line_list);
}

Eigen::MatrixXd CalculateJTJ(const ceres::CRSMatrix &jacobian) {

  int num_rows = jacobian.num_rows;
  // Convert the sparse matrix to a dense matrix

  // Convert the CRS Matrix to an eigen matrix
  Eigen::MatrixXd denseJacobian =
      Eigen::MatrixXd::Zero(num_rows, jacobian.num_cols);
  for (size_t k = 0; k < jacobian.rows.size() - 1; ++k) {
    size_t row = jacobian.rows[k];
    size_t nextRow = jacobian.rows[k + 1];
    for (size_t l = row; l < nextRow; ++l) {
      int column = jacobian.cols[l];
      double value = jacobian.values[l];
      denseJacobian(k, column) = value;
    }
  }

  Eigen::MatrixXd jTj = denseJacobian.transpose() * denseJacobian;

  return jTj;
}

Eigen::MatrixXd CalculateCovariance(const Eigen::MatrixXd &mat) {
  Eigen::MatrixXd centered = mat.rowwise() - mat.colwise().mean();
  Eigen::MatrixXd cov =
      (centered.adjoint() * centered) / double(mat.rows() - 1);
  return cov;
}

void VisualizeCovariance(const int num, const string covarianceFolder,
                         const string bagfile,
                         const ceres::CRSMatrix &jacobian) {
  std::stringstream out;

  mkdir(covarianceFolder.c_str(), 0777);
  out << num;
  string filename =
      covarianceFolder + "/" + bagfile + "_" + out.str() + ".covariance";

  ofstream file(filename.c_str());
  // Calculate the jtj and the covariance matrix
  Eigen::MatrixXd jtj = CalculateJTJ(jacobian);
  // Translation Portion
  Eigen::MatrixXd translation = jtj.inverse().block(3, 3, 3, 3);
  Eigen::MatrixXd covarianceMat = translation;
  file << covarianceMat << endl;
  // Solve to retrieve eigenvectors/values
  Eigen::SelfAdjointEigenSolver<Eigen::MatrixXd> eigenSolver(covarianceMat);
  Eigen::MatrixXd eigenvalues = eigenSolver.eigenvalues();
  Eigen::MatrixXd eigenvectors = eigenSolver.eigenvectors();
  //   //Calculate Rotation Matrix and convert to quaternion
  Eigen::Matrix3d rotationMat = eigenvectors;

  Eigen::Quaterniond quat;
  quat = rotationMat;
  // Eigen::Quaterniond quat = aa;

  visualization_msgs::Marker marker;
  marker.action = visualization_msgs::Marker::ADD;
  marker.header.frame_id = "point_cloud";
  marker.id = 0;
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
  marker.scale.z = eigenvalues(2) * 1000;

  // marker_pub.publish(marker);
  file.close();
}

pcl::PointCloud<pcl::PointXYZ>
VoxelFilter(const pcl::PointCloud<pcl::PointXYZ> &cloud) {
  pcl::PCLPointCloud2::Ptr cloud_filtered(new pcl::PCLPointCloud2());
  pcl::PCLPointCloud2::Ptr ptr_cloud(new pcl::PCLPointCloud2());
  pcl::toPCLPointCloud2(cloud, *ptr_cloud);
  pcl::PointCloud<pcl::PointXYZ> cloud2;
  // Create the filtering object
  pcl::VoxelGrid<pcl::PCLPointCloud2> sor;
  sor.setInputCloud(ptr_cloud);
  sor.setLeafSize(.01f, 0.01f, 0.01f);
  sor.filter(*cloud_filtered);

  pcl::fromPCLPointCloud2(*cloud_filtered, cloud2);
  return cloud2;
}

pcl::PointCloud<pcl::PointXYZ>
BrassVoxelFilter(const pcl::PointCloud<pcl::PointXYZ> &cloud) {
  pcl::PCLPointCloud2::Ptr cloud_filtered(new pcl::PCLPointCloud2());
  pcl::PCLPointCloud2::Ptr ptr_cloud(new pcl::PCLPointCloud2());
  pcl::toPCLPointCloud2(cloud, *ptr_cloud);
  pcl::PointCloud<pcl::PointXYZ> cloud2;
  // Create the filtering object
  pcl::VoxelGrid<pcl::PCLPointCloud2> sor;
  sor.setInputCloud(ptr_cloud);
  sor.setLeafSize(.01f, 0.01f, 0.01f);
  sor.filter(*cloud_filtered);

  pcl::fromPCLPointCloud2(*cloud_filtered, cloud2);
  return cloud2;
}

pcl::PointCloud<pcl::Normal>
GetNormals(const pcl::PointCloud<pcl::PointXYZ> &cloud) {

  // Get the cloud
  pcl::PointCloud<pcl::PointXYZ>::Ptr ptr_cloud(
      new pcl::PointCloud<pcl::PointXYZ>);
  ptr_cloud = cloud.makeShared();
  // Create the normal estimation class, and pass the input dataset to it
  pcl::NormalEstimationOMP<pcl::PointXYZ, pcl::Normal> ne;
  ne.setInputCloud(ptr_cloud);
  ne.setNumberOfThreads(12);
  // Create an empty kdtree representation, and pass it to the normal estimation
  //     object.
  // Its content will be filled inside the object,
  // based on the given input dataset (as no other search surface is given).
  pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(
      new pcl::search::KdTree<pcl::PointXYZ>());
  ne.setSearchMethod(tree);
  // Output datasets
  pcl::PointCloud<pcl::Normal>::Ptr cloud_normals(
      new pcl::PointCloud<pcl::Normal>);
  // Use all neighbors in a sphere of radius
  ne.setRadiusSearch(0.03);

  // Compute the features
  ne.compute(*cloud_normals);

  return *cloud_normals;
}

void ConstructICP_problem(const vector<ros::Publisher> &publishers,
                          const pcl::PointCloud<pcl::PointXYZ> &cloud_1,
                          const pcl::PointCloud<pcl::PointXYZ> &cloud_2,
                          const pcl::PointCloud<pcl::Normal> &normal_1,
                          const pcl::PointCloud<pcl::Normal> &normal_2,
                          const vector<Eigen::Vector2d> &image_coords_1,
                          const vector<Eigen::Vector2d> &image_coords_2,
                          const double nn_dist, double *transform,
                          ceres::Problem *problem) {
  // FunctionTimer timer(__FUNCTION__);
  static const bool kUseNumericOverAutoDiff = false;
  //----------  Transform based on calculated transformation  ----------
  pcl::PointCloud<pcl::PointXYZ> transformed_cloud = cloud_2;

  // Shifts cloud by calculated transform
  TransformPointCloud(&transformed_cloud, transform);

  //----------  Find Nearest Neighbors  ----------
  vector<int> nearest_neigbors;
  vector<int> start_points;
  // Reverse order nearest neighbor to make sure we're transforming in the right
  // direction
  KdTreeNN(nn_dist, cloud_1, transformed_cloud, normal_1, normal_2,
           image_coords_1, image_coords_2, nearest_neigbors, start_points);

  //----------  Visualize NN ----------
//   VisualizeNN(cloud_1,
//                transformed_cloud,
//                nearest_neigbors,
//                start_points);
//   sleep(1);

  //----------  Compute Pose with ceres-solver  ----------
  // Add for each pair of nearest neighbors
  int pair_count = 0;
  for (size_t i = 0; i < start_points.size(); i++) {
    pcl::PointXYZ k1_point, base_point;
    // Use the original cloud, not the transformed cloud
    k1_point = cloud_2[start_points[i]];
    // Base cloud is unchanged
    base_point = cloud_1[nearest_neigbors[i]];
    pcl::Normal normal_k = normal_1[nearest_neigbors[i]];
    pcl::Normal normal_k1 = normal_2[i];
    pcl::PointXYZ transformed_k1 = transformed_cloud[start_points[i]];
    double distance = PointDistance(base_point, transformed_k1);

    if (distance < nn_dist) {
      const Vector3d point_0(base_point.x, base_point.y, base_point.z);
      const Vector3d point_1(k1_point.x, k1_point.y, k1_point.z);
      const Vector3d normal_0(normal_k.normal_x, normal_k.normal_y,
                              normal_k.normal_z);
      const Vector3d normal_1(normal_k1.normal_x, normal_k1.normal_y,
                              normal_k1.normal_z);
      if (point_0.allFinite() && point_1.allFinite() && normal_0.allFinite() &&
          normal_1.allFinite()) {
        ceres::CostFunction *cost_function = NULL;
        if (kUseNumericOverAutoDiff) {
          cost_function = PointToPointErrorNumeric::Create(point_0, point_1,
                                                           normal_0, normal_1);
        } else {
          cost_function =
              PointToPointError::Create(point_0, point_1, normal_0, normal_1);
        }
        problem->AddResidualBlock(cost_function,
                                  NULL, // squared loss
                                  transform);
        pair_count += 1;
      }
    }
  }
}

bool GetKnownCor(const pcl::PointCloud<pcl::PointXYZ> &cloud,
                 const pcl::PointCloud<pcl::Normal> &normal_1, const int k2,
                 const vector<int> &index, pcl::Normal *normal_k1,
                 pcl::PointXYZ *base_point, int &num) {
  bool found = false;
  for (size_t i = 0; i < cloud.size(); i++) {

    if (index[i] == k2) {
      found = true;
      num = i;
      *normal_k1 = normal_1[i];
      *base_point = cloud[i];
      return found;
    }
  }
  return found;
}

void ConstructICPKnown(const vector<ros::Publisher> &publishers,
                       const pcl::PointCloud<pcl::PointXYZ> &cloud_1,
                       const pcl::PointCloud<pcl::PointXYZ> &cloud_2,
                       const pcl::PointCloud<pcl::Normal> &normal_1,
                       const pcl::PointCloud<pcl::Normal> &normal_2,
                       const vector<int> index_k1, const vector<int> index_k2,
                       double *transform, ceres::Problem *problem) {
  static const bool kUseNumericOverAutoDiff = false;
  //----------  Transform based on calculated transformation  ----------
  pcl::PointCloud<pcl::PointXYZ> transformed_cloud = cloud_2;

  // Shifts cloud by calculated transform
  TransformPointCloud(&transformed_cloud, transform);
  PublishCloud(cloud_1, publishers[0]);
  PublishCloud(transformed_cloud, publishers[1]);

  //----------  Compute Pose with ceres-solver  ----------
  // Add for each pair of nearest neighbors
  int pair_count = 0;
  for (size_t i = 0; i < cloud_2.size(); i++) {
    pcl::PointXYZ k1_point, base_point;
    // Use the original cloud, not the transformed cloud
    k1_point = cloud_2[i];
    // Base cloud is unchanged

    pcl::Normal normal_k;
    pcl::Normal normal_k1 = normal_2[i];
    int num;
    bool corresponds = GetKnownCor(cloud_1, normal_1, index_k2[i], index_k1,
                                   &normal_k, &base_point, num);
    // pcl::PointXYZ transformed_k1 = transformed_cloud[i];
    // double distance = PointDistance(base_point, transformed_k1);
    if (corresponds) {
      const Vector3d point_0(base_point.x, base_point.y, base_point.z);
      const Vector3d point_1(k1_point.x, k1_point.y, k1_point.z);
      const Vector3d normal_0(normal_k.normal_x, normal_k.normal_y,
                              normal_k.normal_z);
      const Vector3d normal_1(normal_k1.normal_x, normal_k1.normal_y,
                              normal_k1.normal_z);
      if (point_0.allFinite() && point_1.allFinite() && normal_0.allFinite() &&
          normal_1.allFinite()) {
        ceres::CostFunction *cost_function = NULL;
        if (kUseNumericOverAutoDiff) {
          cost_function = PointToPointErrorNumeric::Create(point_0, point_1,
                                                           normal_0, normal_1);
        } else {
          cost_function =
              PointToPointError::Create(point_0, point_1, normal_0, normal_1);
        }
        problem->AddResidualBlock(cost_function,
                                  new ceres::HuberLoss(0.5), // squared loss
                                  transform);
        pair_count += 1;
      }
    }
  }
}

void ICP(const int k, const double nn_dist,
         const vector<ros::Publisher> &publishers,
         const string &covarianceFolder, const string &bagFile,
         const pcl::PointCloud<pcl::PointXYZ> &cloud_1,
         const pcl::PointCloud<pcl::PointXYZ> &cloud_2,
         const pcl::PointCloud<pcl::Normal> &normal_1,
         const pcl::PointCloud<pcl::Normal> &normal_2,
         const vector<Eigen::Vector2d> &image_coords_1,
         const vector<Eigen::Vector2d> &image_coords_2, double *transform,
         double *final_rmse) {

  FunctionTimer timer(__FUNCTION__);
  first_nn = true;
  CHECK_NOTNULL(transform);
  // Tolerance for RMSE.
  static const double kToleranceError = 0.00001;
  // The maximum number of overall iterations.
  static const int kMaxIterations = 50;
  // The maximum number of repeat iterations while the RMSE is unchanged.
  static const int kMaxRepeatIterations = 5;
  double rmse = 1000000;
  double last_rmse = 1000010;
  vector<double> residuals;
  for (int iteration = 0, repeat_iteration = 0;
       iteration < kMaxIterations && repeat_iteration < kMaxRepeatIterations &&
       rmse > kToleranceError;
       ++iteration) {
    if (DoubleEquals(rmse, last_rmse)) {
      repeat_iteration++;
    } else {
      repeat_iteration = 0;
    }
    last_rmse = rmse;
    // Construct ICP problem
    ceres::Problem problem;
    ConstructICP_problem(publishers, cloud_1, cloud_2, normal_1, normal_2,
                         image_coords_1, image_coords_2, nn_dist, transform,
                         &problem);
    // Run Ceres problem
    ceres::Solver::Options options;
    options.num_threads = 12;
    options.num_linear_solver_threads = 12;
    // options.use_explicit_schur_complement = true;
    // options.linear_solver_type = ceres::ITERATIVE_SCHUR;
    // options.linear_solver_type = ceres::SPARSE_SCHUR;
    options.linear_solver_type = ceres::DENSE_QR;
    // options.minimizer_progress_to_stdout = true;
    // options.function_tolerance = 1e-10;
    ceres::Solver::Summary summary;
    ceres::Solve(options, &problem, &summary);
    rmse =
        sqrt(summary.final_cost / static_cast<double>(summary.num_residuals));
    ceres::Problem::EvaluateOptions evalOptions =
        ceres::Problem::EvaluateOptions();
    residuals.clear();
    problem.Evaluate(evalOptions, NULL, &residuals, NULL, NULL);
    // fprintf(stdout, "Residuals size: %f\n", residuals.size());
    pcl::PointCloud<pcl::PointXYZ> transformed_cloud = cloud_2;

    // Shifts cloud by calculated transform
    TransformPointCloud(&transformed_cloud, transform);
    std::cout << "Publishing" << std::endl;
    PublishCloud(cloud_1, publishers[0]);
    PublishCloud(transformed_cloud, publishers[1]);
    vector<int> nearest_neigbors;
    vector<int> start_points;
    KdTreeNN(nn_dist, cloud_1, transformed_cloud, normal_1, normal_2,
             image_coords_1, image_coords_2, nearest_neigbors, start_points);
    VisualizeNN(cloud_1, transformed_cloud, nearest_neigbors, start_points,
    publishers[4]);
    Sleep(1);
  }
  if (final_rmse)
    *final_rmse = rmse;
}

double *ICPKnownC(const int k, const vector<ros::Publisher> &publishers,
                  const string &covarianceFolder, const string &bagFile,
                  const pcl::PointCloud<pcl::PointXYZ> &cloud_1,
                  const pcl::PointCloud<pcl::PointXYZ> &cloud_2,
                  const pcl::PointCloud<pcl::Normal> &normal_1,
                  const pcl::PointCloud<pcl::Normal> &normal_2,
                  const vector<int> index_k1, const vector<int> index_k2,
                  double *transform, double *final_mean) {
  // Tolerance for RMSE.
  static const double kToleranceError = 0.0001;
  // The maximum number of overall iterations.
  static const int kMaxIterations = 60;
  // The maximum number of repeat iterations while the RMSE is unchanged.
  static const int kMaxRepeatIterations = 5;
  double rmse = 1000000;
  double last_rmse = 1000010;
  for (int iteration = 0, repeat_iteration = 0;
       iteration < kMaxIterations && repeat_iteration < kMaxRepeatIterations &&
       rmse > kToleranceError;
       ++iteration) {
    if (DoubleEquals(rmse, last_rmse)) {
      repeat_iteration++;
    } else {
      repeat_iteration = 0;
    }
    last_rmse = rmse;
    // Construct ICP problem
    ceres::Problem problem;
    ConstructICPKnown(publishers, cloud_1, cloud_2, normal_1, normal_2,
                      index_k1, index_k2, transform, &problem);

    // Run Ceres problem
    ceres::Solver::Options options;
    options.num_threads = 12;
    options.num_linear_solver_threads = 12;
    // options.use_explicit_schur_complement = true;
    // options.linear_solver_type = ceres::ITERATIVE_SCHUR;
    options.function_tolerance = 1e-10;
    options.linear_solver_type = ceres::SPARSE_SCHUR;
    // options.minimizer_progress_to_stdout = true;
    ceres::Solver::Summary summary;
    ceres::Solve(options, &problem, &summary);

    rmse =
        sqrt(summary.final_cost / static_cast<double>(summary.num_residuals));
    printf("MSE:%g\n", rmse);
  }

  return transform;
}

pcl::PointCloud<pcl::PointXYZ>
CloudFromVector(const vector<Eigen::Vector3f> &pointCloud,
                const vector<int> &pixelLocs) {

  pcl::PointCloud<pcl::PointXYZ> cloud;
  cloud.resize(pointCloud.size());

  for (uint i = 0; i < cloud.size(); ++i) {
    pcl::PointXYZ point;
    if (pointCloud[i](0) < 3) {
      point.x = pointCloud[i](1);
      point.y = pointCloud[i](0);
      point.z = pointCloud[i](2);
    } else {
      point.x = 0;
      point.y = 0;
      point.z = 0;
    }
    cloud[i] = point;
  }
  return VoxelFilter(cloud);
  //       return cloud;
}

// Reads clouds from a given iterator, saves to buffer if they are over
rosbag::View::iterator
GetCloudsSlamBag(rosbag::View::iterator it,
                 std::deque<pcl::PointCloud<pcl::PointXYZ>> *buffer1,
                 std::deque<double> *timestamps_1) {

  PlaneFilter filter;
  KinectOpenNIDepthCam camera = KinectOpenNIDepthCam();
  filter.setDepthCamera(&camera);
  const rosbag::MessageInstance &depth_m = *it;
  sensor_msgs::ImagePtr depth_msg = depth_m.instantiate<sensor_msgs::Image>();
  // Going to need to also get the mapping from depth to color and depth to
  // point cloud
  if (depth_msg != NULL) {
    vector<Eigen::Vector3f> pointCloud;
    vector<int> pixelLocs;
    filter.GenerateCompletePointCloud((void *)depth_msg->data.data(),
                                      pointCloud, pixelLocs);
    pcl::PointCloud<pcl::PointXYZ> pcl_cloud =
        icp::CloudFromVector(pointCloud, pixelLocs);
    buffer1->push_back(pcl_cloud);
    timestamps_1->push_back(depth_msg->header.stamp.toSec());
  }
  advance(it, 1);
  return it;
}

// Reads clouds from a given iterator, saves to buffer if they are over
rosbag::View::iterator
GetClouds(rosbag::View::iterator it,
          std::deque<pcl::PointCloud<pcl::PointXYZ>> *buffer1,
          std::deque<pcl::PointCloud<pcl::PointXYZ>> *buffer2,
          std::deque<double> *timestamps_1, std::deque<double> *timestamps_2) {

  PlaneFilter filter;
  KinectRawDepthCam camera = KinectRawDepthCam();
  filter.setDepthCamera(&camera);
  string kinect_0 = "kinect_0";
  string kinect_1 = "kinect_1";
  while ((buffer1->size() == 0 || buffer2->size() == 0)) {
    const rosbag::MessageInstance &m = *it;
    sensor_msgs::ImagePtr imageMsg = m.instantiate<sensor_msgs::Image>();
    if (imageMsg != NULL) {
      vector<Eigen::Vector3f> pointCloud;
      vector<int> pixelLocs;
      filter.GenerateCompletePointCloud((void *)imageMsg->data.data(),
                                        pointCloud, pixelLocs);
      pcl::PointCloud<pcl::PointXYZ> pcl_cloud =
          CloudFromVector(pointCloud, pixelLocs);
      if (imageMsg->header.frame_id == kinect_0) {
        buffer1->push_back(pcl_cloud);
        timestamps_1->push_back(imageMsg->header.stamp.toSec());
      } else if (imageMsg->header.frame_id == kinect_1) {
        buffer2->push_back(pcl_cloud);
        timestamps_2->push_back(imageMsg->header.stamp.toSec());
      }
    }
    advance(it, 1);
  }
  return it;
}

// Reads clouds from a given iterator, saves to buffer if they are over
rosbag::View::iterator
GetCloudsOne(rosbag::View::iterator it,
             std::deque<pcl::PointCloud<pcl::PointXYZ>> *buffer1,
             std::deque<double> *timestamps_1) {

  PlaneFilter filter;
  KinectRawDepthCam camera = KinectRawDepthCam();
  filter.setDepthCamera(&camera);
  string kinect_0 = "kinect_0";
  while ((buffer1->size() == 0)) {
    const rosbag::MessageInstance &m = *it;
    sensor_msgs::ImagePtr imageMsg = m.instantiate<sensor_msgs::Image>();
    if (imageMsg != NULL) {
      vector<Eigen::Vector3f> pointCloud;
      vector<int> pixelLocs;
      filter.GenerateCompletePointCloud((void *)imageMsg->data.data(),
                                        pointCloud, pixelLocs);
      pcl::PointCloud<pcl::PointXYZ> pcl_cloud =
          CloudFromVector(pointCloud, pixelLocs);
      buffer1->push_back(pcl_cloud);
      timestamps_1->push_back(imageMsg->header.stamp.toSec());
    }
    advance(it, 1);
  }
  return it;
}

// Reads clouds from a given iterator, saves to buffer if they are over
rosbag::View::iterator
GetCloudsOneBrass(rosbag::View::iterator it,
                  std::deque<pcl::PointCloud<pcl::PointXYZ>> *buffer1,
                  std::deque<double> *timestamps_1) {

  while ((buffer1->size() == 0)) {
    const rosbag::MessageInstance &m = *it;
    sensor_msgs::PointCloud2Ptr imageMsg =
        m.instantiate<sensor_msgs::PointCloud2>();
    if (imageMsg != NULL) {
      pcl::PointCloud<pcl::PointXYZ> pcl_cloud;
      pcl::PCLPointCloud2 pcl_pc2;
      pcl_conversions::toPCL(*imageMsg, pcl_pc2);
      pcl::fromPCLPointCloud2(pcl_pc2, pcl_cloud);
      buffer1->push_back(pcl_cloud);
      timestamps_1->push_back(imageMsg->header.stamp.toSec());
    }
    advance(it, 1);
  }
  return it;
}

rosbag::View::iterator
GetCloudsBag(rosbag::View::iterator it, rosbag::View::iterator end,
             std::deque<pcl::PointCloud<pcl::PointXYZ>> *buffer1,
             std::deque<pcl::PointCloud<pcl::PointXYZ>> *buffer2,
             std::deque<double> *timestamps_1, std::deque<double> *timestamps_2,
             pcl::PointCloud<pcl::PointXYZ> *cloud1,
             pcl::PointCloud<pcl::PointXYZ> *cloud2, double *time1,
             double *time2) {

  for (uint i = 0; i < 2; i++) {
    const rosbag::MessageInstance &m = *it;

    sensor_msgs::PointCloud2Ptr cloudMsg =
        m.instantiate<sensor_msgs::PointCloud2>();
    pcl::PCLPointCloud2 pcl_pc;
    pcl::PointCloud<pcl::PointXYZ> cloud;
    pcl_conversions::toPCL(*cloudMsg, pcl_pc);
    pcl::fromPCLPointCloud2(pcl_pc, cloud);
    if (i == 0) {
      *cloud1 = cloud;
    } else {
      *cloud2 = cloud;
    }
    advance(it, 1);
  }
  return it;
}

rosbag::View::iterator TimeAlignedClouds(
    rosbag::View::iterator it, rosbag::View::iterator end,
    std::deque<pcl::PointCloud<pcl::PointXYZ>> *buffer1,
    std::deque<pcl::PointCloud<pcl::PointXYZ>> *buffer2,
    std::deque<double> *timestamps_1, std::deque<double> *timestamps_2,
    pcl::PointCloud<pcl::PointXYZ> *cloud1,
    pcl::PointCloud<pcl::PointXYZ> *cloud2, double *time1, double *time2) {

  // Fill the buffers
  it = GetClouds(it, buffer1, buffer2, timestamps_1, timestamps_2);
  // Get the earliest cloud from the first kinect
  pcl::PointCloud<pcl::PointXYZ> cloud_k1 = (*buffer1)[0];
  buffer1->pop_front();
  double k1_time = (*timestamps_1)[0];
  timestamps_1->pop_front();
  // Find the closest cloud in buffer_k2
  bool fin = false;
  pcl::PointCloud<pcl::PointXYZ> cloud_k2;
  double k2_time = 0;
  double best_deltaT = 100000;
  int count = 0;
  while (!fin && it != end) {
    // Fill the buffers
    it = GetClouds(it, buffer1, buffer2, timestamps_1, timestamps_2);
    for (size_t i = 0; i < buffer2->size(); i++) {
      pcl::PointCloud<pcl::PointXYZ> temp_cloud_k2 = (*buffer2)[0];
      double temp_k2_time = (*timestamps_2)[0];
      double deltaT = k1_time - temp_k2_time;
      deltaT = abs(deltaT);
      count++;
      if (deltaT < best_deltaT) {
        best_deltaT = deltaT;
        cloud_k2 = temp_cloud_k2;
        k2_time = temp_k2_time;
        buffer2->pop_front();
        timestamps_2->pop_front();
      } else {
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

rosbag::View::iterator
OneSensorClouds(rosbag::View::iterator it, rosbag::View::iterator end,
                std::deque<pcl::PointCloud<pcl::PointXYZ>> *buffer1,
                std::deque<double> *timestamps_1,
                pcl::PointCloud<pcl::PointXYZ> *cloud1, double *time1) {

  // Fill the buffers
  it = GetCloudsOne(it, buffer1, timestamps_1);
  // Get the earliest cloud from the first kinect
  pcl::PointCloud<pcl::PointXYZ> cloud_k1 = (*buffer1)[0];
  buffer1->pop_front();
  double k1_time = (*timestamps_1)[0];
  timestamps_1->pop_front();
  // return those two as the current clouds to use
  (*cloud1) = VoxelFilter(cloud_k1);
  *time1 = k1_time;
  return it;
}

void OrientCloud(pcl::PointCloud<pcl::PointXYZ> *cloud) {
  pcl::PointCloud<pcl::PointXYZ> cloud_2;
  for (size_t i = 0; i < cloud->size(); ++i) {
    pcl::PointXYZ point = (*cloud)[i];
    pcl::PointXYZ point2;
    point2.x = point.z;
    point2.y = -point.x;
    point2.z = -point.y;
    if (point2.x < 4.9) {
      cloud_2.push_back(point2);
    }
  }
  *cloud = cloud_2;
}

rosbag::View::iterator
OneSensorCloudsBrass(rosbag::View::iterator it, rosbag::View::iterator end,
                     std::deque<pcl::PointCloud<pcl::PointXYZ>> *buffer1,
                     std::deque<double> *timestamps_1,
                     pcl::PointCloud<pcl::PointXYZ> *cloud1, double *time1) {

  // Fill the buffers
  it = GetCloudsOneBrass(it, buffer1, timestamps_1);
  // Get the earliest cloud from the first kinect
  pcl::PointCloud<pcl::PointXYZ> cloud_k1 = (*buffer1)[0];
  buffer1->pop_front();
  double k1_time = (*timestamps_1)[0];
  timestamps_1->pop_front();
  // return those two as the current clouds to use
  (*cloud1) = VoxelFilter(cloud_k1);
  OrientCloud(cloud1);
  *time1 = k1_time;
  return it;
}

rosbag::View::iterator TimeAlignedCloudsSlamBag(
    rosbag::View::iterator it, rosbag::View::iterator end,
    rosbag::View::iterator *it2, rosbag::View::iterator end2,
    std::deque<pcl::PointCloud<pcl::PointXYZ>> *buffer1,
    std::deque<pcl::PointCloud<pcl::PointXYZ>> *buffer2,
    std::deque<double> *timestamps_1, std::deque<double> *timestamps_2,
    pcl::PointCloud<pcl::PointXYZ> *cloud1,
    pcl::PointCloud<pcl::PointXYZ> *cloud2, double *time1, double *time2) {

  // Fill the buffers
  it = GetCloudsSlamBag(it, buffer1, timestamps_1);
  (*it2) = GetCloudsSlamBag(*it2, buffer2, timestamps_2);
  // Get the earliest cloud from the first kinect
  pcl::PointCloud<pcl::PointXYZ> cloud_k1 = (*buffer1)[0];
  buffer1->pop_front();
  double k1_time = (*timestamps_1)[0];
  timestamps_1->pop_front();
  // Find the closest cloud in buffer_k2
  bool fin = false;
  pcl::PointCloud<pcl::PointXYZ> cloud_k2;
  double k2_time = 0;
  double best_deltaT = 100000;
  int count = 0;
  while (!fin && it != end) {
    // Fill the buffers
    it = GetCloudsSlamBag(it, buffer1, timestamps_1);
    (*it2) = GetCloudsSlamBag(*it2, buffer2, timestamps_2);
    for (size_t i = 0; i < buffer2->size(); i++) {
      pcl::PointCloud<pcl::PointXYZ> temp_cloud_k2 = (*buffer2)[0];
      double temp_k2_time = (*timestamps_2)[0];
      double deltaT = k1_time - temp_k2_time;
      deltaT = abs(deltaT);
      count++;
      if (deltaT < best_deltaT) {
        best_deltaT = deltaT;
        cloud_k2 = temp_cloud_k2;
        k2_time = temp_k2_time;
        buffer2->pop_front();
        timestamps_2->pop_front();
      } else {
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

bool CheckChangeVel(double *pose, const int degree,
                    const vector<double> &velocity_list) {
  const double trans_eps = .05;
  Eigen::Matrix<double, 3, 1> axis(pose[0], pose[1], pose[2]);
  Eigen::Matrix<double, 3, 1> trans(pose[3], pose[4], pose[5]);
  const double angle = axis.norm();
  double angle_degree = (180 / 3.14) * angle;
  double norm = trans.norm();
  double velocity =
      std::accumulate(velocity_list.begin(), velocity_list.end(), 0.0);
  velocity = velocity / velocity_list.size();
  if (degree > 0) {
    if (abs(velocity) < 2.5) {
      if ((angle_degree > degree)) {
        return true;
      } else {
        return false;
      }
    }
  } else {
    if ((angle_degree > degree) && norm > trans_eps) {
      return true;
    } else {
      return false;
    }
  }
  return false;
}

// Removes components of a transform which cannot be disambiguated by the
// normals
// in the scene. This could create false transforms given translations caused by
// rotations (potentially).
// To insure this doesn't happen, motion has to be controlled to prevent
// ambigous translation from rotation.
// (Assumes the actual motion can be disambiguated by the given scene
void SanitizeTransform(const pcl::PointCloud<pcl::PointXYZ> &cloud,
                       const pcl::PointCloud<pcl::Normal> &normal,
                       double *pose) {
  Eigen::Matrix<double, 3, 1> axis(pose[0], pose[1], pose[2]);
  const double angle = axis.norm();
  Eigen::Matrix<double, 3, 1> translation(pose[3], pose[4], pose[5]);
  Eigen::AngleAxis<double> aa(angle, axis);
  Eigen::Matrix3d rotation;
  rotation = aa.toRotationMatrix();
  Eigen::Vector3d eulers = rotation.eulerAngles(0, 1, 2);
  Eigen::Matrix<double, 3, 1> x(1, 0, 0);
  Eigen::Matrix<double, 3, 1> y(0, 1, 0);
  Eigen::Matrix<double, 3, 1> z(0, 0, 1);
  int x_count = 0;
  int y_count = 0;
  int z_count = 0;
  int transx_count = 0;
  int transy_count = 0;
  int transz_count = 0;

  for (size_t i = 0; i < normal.size(); i++) {
    pcl::Normal norm = normal[i];
    // If an axis of rotation is parallel to all feature normals, then it is
    // ambiguous
    Eigen::Matrix<double, 3, 1> norm_vector(norm.normal_x, norm.normal_y,
                                            norm.normal_z);
    Eigen::Matrix<double, 3, 1> x_cross = norm_vector.cross(x);
    Eigen::Matrix<double, 3, 1> y_cross = norm_vector.cross(y);
    Eigen::Matrix<double, 3, 1> z_cross = norm_vector.cross(z);
    if (x_cross.norm() > 0.01) {
      x_count++;
    }
    if (y_cross.norm() > 0.01) {
      y_count++;
    }
    if (z_cross.norm() > 0.01) {
      z_count++;
    }
    // If a translation is orthogonal to all normals, it is ambigous
    double transx_dot = norm_vector.dot(x);
    double transy_dot = norm_vector.dot(y);
    double transz_dot = norm_vector.dot(x);
    if (transx_dot > 0.01) {
      transx_count++;
    }
    if (transy_dot > 0.01) {
      transy_count++;
    }
    if (transz_dot > 0.01) {
      transz_count++;
    }
  }

  if (transx_count < 20) {
    translation[0] = 0;
  }
  if (transy_count < 20) {
    translation[1] = 0;
  }
  if (transz_count < 20) {
    translation[2] = 0;
  }
  if (x_count < 20) {
    eulers[0] = 0;
  }
  if (y_count < 20) {
    eulers[1] = 0;
  }
  if (z_count < 20) {
    eulers[2] = 0;
  }
  Eigen::Matrix3d m;
  m = Eigen::AngleAxisd(eulers[0], Eigen::Vector3d::UnitX()) *
      Eigen::AngleAxisd(eulers[1], Eigen::Vector3d::UnitY()) *
      Eigen::AngleAxisd(eulers[2], Eigen::Vector3d::UnitZ());
  Eigen::AngleAxis<double> aaa;
  aaa.fromRotationMatrix(m);
  pose[3] = translation[0];
  pose[4] = translation[1];
  pose[5] = translation[2];

  // Get the axis
  Eigen::Vector3d normal_axis = aaa.axis();

  // Recompute the rotation angle
  double combined_angle = aaa.angle();
  Eigen::Vector3d combined_axis = normal_axis * combined_angle;
  pose[0] = combined_axis[0];
  pose[1] = combined_axis[1];
  pose[2] = combined_axis[2];
}

bool CheckChangeOdom(double *pose, double *previous_pose,
                     const double &timestamp_1, const double &timestamp_2,
                     const int &degree) {
  Eigen::Matrix<double, 3, 1> axis(pose[0], pose[1], pose[2]);
  const Eigen::Vector3d trans = {pose[3], pose[4], pose[5]};
  const double dist = fabs(trans.norm());
  const double angle = axis.norm();
  double angle_degree = (180 / 3.14) * angle;
    if ((angle_degree > degree) || dist > .05) {
      return true;
    } else {
      return false;
    }
    return false;
}

bool CheckChange(double *pose, const int degree) {
  double trans_sum = pow(pose[3], 2) + pow(pose[4], 2) + pow(pose[5], 2);
  double dist = sqrt(trans_sum);
  Eigen::Matrix<double, 3, 1> axis(pose[0], pose[1], pose[2]);
  const double angle = axis.norm();
  double angle_degree = (180 / 3.14) * angle;
  if ((dist > .5) || (angle_degree > degree)) {
    return true;
  } else {
    return false;
  }
}

bool CheckResidualDist(const pcl::PointCloud<pcl::PointXYZ> &k_cloud,
                       const pcl::PointCloud<pcl::PointXYZ> &l_cloud,
                       const pcl::PointCloud<pcl::PointXYZ> &keyframe,
                       const pcl::PointCloud<pcl::Normal> &k_normal,
                       const pcl::PointCloud<pcl::Normal> &l_normal,
                       const pcl::PointCloud<pcl::Normal> &key_normal,
                       double *pose, double &mean) {

  mean = 0;

  double threshold = 0.0;
  mean = ResidualDist(k_cloud, l_cloud, k_normal, l_normal, pose);
  double mean2 =
      abs(ResidualDist(k_cloud, keyframe, k_normal, key_normal, pose));
  if (mean2 >= threshold) {
    return true;
  } else {
    return false;
  }
}

void WriteToObj(const string folder, const string bagfile, const int num,
                const pcl::PointCloud<pcl::PointXYZ> &cloud) {

  std::string frame_string;
  std::stringstream out;
  string temp = bagfile;
  mkdir(folder.c_str(), 0777);
  out << num;
  string filename = folder + "/" + bagfile + "_" + out.str() + ".obj";
  ofstream file(filename.c_str());
  for (size_t i = 0; i < cloud.size(); i++) {
    pcl::PointXYZ point = cloud[i];
    file << "v " << point.x << " " << point.y << " " << point.z << " 0 255 0"
         << endl;
  }
  file.close();
}

pcl::PointCloud<pcl::PointXYZ> CloudFromObj(string obj) {
  std::ifstream infile(obj.c_str());
  char v;
  double x, y, z;

  vector<Eigen::Vector3f> cloud_vector;
  vector<int> pixelLocs;
  while (infile >> v >> x >> y >> z) {
    Eigen::Vector3f point;
    point(0) = x;
    point(1) = y;
    point(2) = z;
    cloud_vector.push_back(point);
  }
  return CloudFromVector(cloud_vector, pixelLocs);
}

void WriteToBag(const string topic, rosbag::Bag *bag,
                const pcl::PointCloud<pcl::PointXYZ> cloud) {
  sensor_msgs::PointCloud2 temp_cloud;
  pcl::PCLPointCloud2 pcl_cloud;
  pcl::toPCLPointCloud2(cloud, pcl_cloud);
  pcl_conversions::fromPCL(pcl_cloud, temp_cloud);
  temp_cloud.header.frame_id = "point_cloud";
  bag->write(topic, ros::Time::now(), temp_cloud);
}

double ComputeVelocity(const double delta, const double prev_time,
                       const double cur_time) {

  double time_diff = cur_time - prev_time;

  return delta / time_diff;
}

void WritePoseFile(double *pose, const double &timestamp, const int &count,
                   ofstream &file) {

  if (file.is_open()) {
    for (int j = 0; j < 6; j++) {

      file << pose[j] << "\t";
    }
  }
  file << std::setprecision(20) << timestamp << "\t" << count << "\t";
  file << std::flush;
}
}
