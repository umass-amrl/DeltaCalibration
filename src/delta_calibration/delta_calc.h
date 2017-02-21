#ifndef DELTA_CALC_H
#define DELTA_CALC_H

namespace delta_calc {
  
  void HandleStop(int i);
  
  void DeltaCalculationBrass( string bag_name,
                              vector<ros::Publisher> publishers,
                              const int degree,
                              const int kMaxClouds);
  
  void DeltaCalculation(string bag_name,
                        vector<ros::Publisher> publishers,
                        const int degree,
                        const int kMaxClouds);
  
  void DeltaCalculationSingle(string bag_name,
                              vector<ros::Publisher> publishers,
                              const int degree,
                              const int kMaxClouds);
  
  vector<pcl::PointCloud<pcl::PointXYZ> > ExtractPlanes(
    const pcl::PointCloud<pcl::PointXYZ>& cloud,
    vector<Eigen::Vector4d>* normal_equations,
    vector<Eigen::Vector3d>* centroids
  );
  
  void ExtractUncertainty(
    const vector<Eigen::Vector4d>& normal_equations,
    Eigen::Vector3d* uncertainty_T,
    Eigen::Vector3d* uncertainty_R);
  
  void StripUncertainty(const Vector3d& ut, const Vector3d& ur, double* transform);
}


#endif //DELTA_CALC_H