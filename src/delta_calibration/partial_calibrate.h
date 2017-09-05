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

#ifndef PARTIAL_CALIBRATE_H
#define PARTIAL_CALIBRATE_H
//----------- INCLUDES
#include "delta_calibration/icp.h"
#include <nav_msgs/Odometry.h>
using std::size_t;
using std::vector;
using namespace std;
using namespace icp;
using Eigen::Vector3d;

namespace partial_calibrate {

void HandleStop(int i);

void PartialCalibrateRwT(const vector<vector<double>> &deltas_1,
                         const vector<vector<double>> &uncertaintyR_1,
                         const vector<vector<double>> &uncertaintyT_1,
                         const vector<vector<double>> &deltas_2,
                         const vector<vector<double>> &uncertaintyR_2,
                         const vector<vector<double>> &uncertaintyT_2,
                         const vector<vector<double>> &deltasT_1,
                         const vector<vector<double>> &TuncertaintyR_1,
                         const vector<vector<double>> &TuncertaintyT_1,
                         const vector<vector<double>> &deltasT_2,
                         const vector<vector<double>> &TuncertaintyR_2,
                         const vector<vector<double>> &TuncertaintyT_2,
                         double *transform, double *final_rmse);

void ReadUncertaintiesFromFile(string delta_file,
                               vector<vector<double>> *uncertainty_1,
                               vector<vector<double>> *uncertainty_2);

void ReadDeltasFromFile(string delta_file, vector<vector<double>> *deltas_1,
                        vector<vector<double>> *deltas_2,
                        vector<vector<double>> *uncertainty_1,
                        vector<vector<double>> *uncertainty_2);

void PartialCalibrateR(const vector<vector<double>> &deltas_1,
                       const vector<vector<double>> &uncertaintyR_1,
                       const vector<vector<double>> &uncertaintyT_1,
                       const vector<vector<double>> &deltas_2,
                       const vector<vector<double>> &uncertaintyR_2,
                       const vector<vector<double>> &uncertaintyT_2,
                       double *transform, double *final_rmse);

void PartialCalibrateT(const vector<vector<double>> &deltas_1,
                       const vector<vector<double>> &uncertaintyR_1,
                       const vector<vector<double>> &uncertaintyT_1,
                       const vector<vector<double>> &deltas_2,
                       const vector<vector<double>> &uncertaintyR_2,
                       const vector<vector<double>> &uncertaintyT_2,
                       double *transform, double *final_rmse);

double *turtlebot_calibrate(string filename);
}
#endif // DELTA_CALC_H