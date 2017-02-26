#ifndef PARTIAL_CALIBRATE_H
#define PARTIAL_CALIBRATE_H
//----------- INCLUDES
#include <nav_msgs/Odometry.h>
#include "delta_calibration/icp.h"
using std::size_t;
using std::vector;
using namespace std;
using namespace icp;
using Eigen::Vector3d;

namespace partial_calibrate {
  
  void HandleStop(int i);
  
  void ReadUncertaintiesFromFile(string delta_file,
                                 vector<vector<double> >* uncertainty_1,
                                 vector<vector<double> >* uncertainty_2);
  
  void ReadDeltasFromFile(string delta_file,
                          vector<vector<double> >* deltas_1,
                          vector<vector<double> >* deltas_2,
                          vector<vector<double> >* uncertainty_1,
                          vector<vector<double> >* uncertainty_2
  );
  
  void PartialCalibrateR(
    const vector<vector<double> >& deltas_1,
    const vector<vector<double> >& uncertaintyR_1,
    const vector<vector<double> >& uncertaintyT_1,
    const vector<vector<double> >& deltas_2,
    const vector<vector<double> >& uncertaintyR_2,
    const vector<vector<double> >& uncertaintyT_2,
    double* transform,
    double* final_rmse);
  
  void PartialCalibrateT(
    const vector<vector<double> >& deltas_1,
    const vector<vector<double> >& uncertaintyR_1,
    const vector<vector<double> >& uncertaintyT_1,
    const vector<vector<double> >& deltas_2,
    const vector<vector<double> >& uncertaintyR_2,
    const vector<vector<double> >& uncertaintyT_2,
    double* transform,
    double* final_rmse);
  
  double* turtlebot_calibrate(string filename);

}
#endif //DELTA_CALC_H