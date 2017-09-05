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

#include "delta_calibration/partial_calibrate.h"

using namespace partial_calibrate;

void WritePose(double *pose, ofstream &file) {
  if (file.is_open()) {
    for (int j = 0; j < 6; j++) {
      file << pose[j] << "\t";
    }
    file << "\n";
  }
}

void PrintPose(double *pose) {
  for (uint i = 0; i < 6; i++) {
    cout << pose[i] << " ";
  }
  cout << endl;
}

int main(int argc, char **argv) {
  signal(SIGINT, HandleStop);
  signal(SIGALRM, HandleStop);

  char *filename = (char *)"pair_upright.bag";
  char *filename2 = (char *)"pair_upright.bag";
  bool turtlebot_mode = false;
  // Parse arguments.
  static struct poptOption options[] = {
      {"file", 'f', POPT_ARG_STRING, &filename, 0,
        "Filename to use for all calibration data.",
       "STR"},
       {"trans_file", 'F', POPT_ARG_STRING, &filename2, 0,
         "Filename to use for pure translation calibration data.",
         "STR"},
      {"turtlebot_mode", 't', POPT_ARG_NONE, &turtlebot_mode, 0,
        "Extract Delta-Transforms in TurtlebotMode", "NONE"},
      POPT_AUTOHELP{NULL, 0, 0, NULL, 0, NULL, NULL}};

  // parse options
  POpt popt(NULL, argc, (const char **)argv, options, 0);
  int c;
  while ((c = popt.getNextOpt()) >= 0) {
  }

  // Initialize Ros
  ros::init(argc, argv, "delta_calibrate",
            ros::init_options::NoSigintHandler);
  ros::NodeHandle n;

  cout << "reading deltas" << endl;
  const string file = *filename + ".pose";
  const string uncertainty_t_string = *filename + "_U_T.txt";
  const string uncertainty_r_string = *filename + "_U_R.txt";
  const string output_str = *filename + ".extrinsics";
  vector<vector<double>> deltas_1;
  vector<vector<double>> deltas_2;
  vector<vector<double>> uncertaintyR_1;
  vector<vector<double>> uncertaintyE_1;
  vector<vector<double>> uncertaintyE_2;
  vector<vector<double>> uncertaintyR_2;
  vector<vector<double>> uncertaintyT_1;
  vector<vector<double>> uncertaintyT_2;
  ReadDeltasFromFile(file, &deltas_1, &deltas_2, &uncertaintyE_1,
                     &uncertaintyE_2);

  ReadUncertaintiesFromFile(uncertainty_t_string, &uncertaintyT_1,
                            &uncertaintyT_2);

  ReadUncertaintiesFromFile(uncertainty_r_string, &uncertaintyR_1,
                            &uncertaintyR_2);

  double *transform = new double[6];
  transform[0] = 0;
  transform[1] = 0;
  transform[2] = 0;
  transform[3] = 0;
  transform[4] = 0;
  transform[5] = 0;

  if (turtlebot_mode) {
    double *RMSE = 0;
    vector<vector<double>> TuncertaintyR_1;
    vector<vector<double>> TuncertaintyR_2;
    vector<vector<double>> TuncertaintyT_1;
    vector<vector<double>> TuncertaintyT_2;
    vector<vector<double>> deltasT_1;
    vector<vector<double>> deltasT_2;
    const string file2 = *filename2 + ".pose";
    const string uncertainty_t_string = *filename2 + "_U_T.txt";
    const string uncertainty_r_string = *filename2 + "_U_R.txt";
    ReadDeltasFromFile(file2, &deltasT_1, &deltasT_2, &uncertaintyE_1,
                       &uncertaintyE_2);
    ReadUncertaintiesFromFile(uncertainty_t_string,
                              &TuncertaintyT_1, &TuncertaintyT_2);

    ReadUncertaintiesFromFile(uncertainty_r_string,
                              &TuncertaintyR_1, &TuncertaintyR_2);
    PartialCalibrateRwT(deltas_1, uncertaintyR_1, uncertaintyT_1, deltas_2,
                      uncertaintyR_2, uncertaintyT_2, deltasT_1,
                      TuncertaintyR_1, TuncertaintyT_1, deltasT_2,
                      TuncertaintyR_2, TuncertaintyT_2, transform, RMSE);
  } else {
    double *RMSE = 0;
    PartialCalibrateR(deltas_1, uncertaintyR_1, uncertaintyT_1, deltas_2,
                      uncertaintyR_2, uncertaintyT_2, transform, RMSE);
    PartialCalibrateT(deltas_1, uncertaintyR_1, uncertaintyT_1, deltas_2,
                      uncertaintyR_2, uncertaintyT_2, transform, RMSE);
  }
  PrintPose(transform);
  ofstream output(output_str.c_str());
  WritePose(transform, output);
  return 0;
}