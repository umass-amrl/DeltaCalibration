#include "delta_calibration/icp.h"
#ifndef DELTA_CALC_H
#define DELTA_CALC_H

namespace delta_calc {

void HandleStop(int i);

void DeltaCalculationBrass(std::string bag_name,
                           std::vector<ros::Publisher> publishers,
                           const int degree, const int kMaxClouds);

void DeltaCalculation(std::string bag_name,
                      std::vector<ros::Publisher> publishers, const int degree,
                      const int kMaxClouds);

void DeltaCalculationSingle(string bag_name, vector<ros::Publisher> publishers,
                            const int degree, const int kMaxClouds);

vector<pcl::PointCloud<pcl::PointXYZ>>
ExtractPlanes(const pcl::PointCloud<pcl::PointXYZ> &cloud,
              std::vector<Eigen::Vector4d> *normal_equations,
              std::vector<Eigen::Vector3d> *centroids);

void ExtractUncertainty(const std::vector<Eigen::Vector4d> &normal_equations,
                        Eigen::Vector3d *uncertainty_T,
                        Eigen::Vector3d *uncertainty_R);

void StripUncertainty(const Vector3d &ut, const Vector3d &ur,
                      double *transform);
}

#endif // DELTA_CALC_H