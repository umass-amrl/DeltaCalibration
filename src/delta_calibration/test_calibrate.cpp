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

  // Initialize Ros
  ros::init(argc, argv, "partial_calibrate",
            ros::init_options::NoSigintHandler);
  ros::NodeHandle n;

  cout << "reading deltas" << endl;
  string file = "generated_deltas.txt";
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

  ReadUncertaintiesFromFile("generated_uncertaintiest.txt", &uncertaintyT_1,
                            &uncertaintyT_2);

  ReadUncertaintiesFromFile("generated_uncertaintiesr.txt", &uncertaintyR_1,
                            &uncertaintyR_2);

  cout << deltas_1.size() << endl;
  cout << deltas_2.size() << endl;
  double *transform = new double[6];
  transform[0] = 0;
  transform[1] = 0;
  transform[2] = 0;
  transform[3] = 0;
  transform[4] = 0;
  transform[5] = 0;

  double *RMSE = 0;
  PartialCalibrateR(deltas_1, uncertaintyR_1, uncertaintyT_1, deltas_2,
                    uncertaintyR_2, uncertaintyT_2, transform, RMSE);
  PartialCalibrateT(deltas_1, uncertaintyR_1, uncertaintyT_1, deltas_2,
                    uncertaintyR_2, uncertaintyT_2, transform, RMSE);
  PrintPose(transform);
  ofstream output("calibration.pose");
  WritePose(transform, output);
  return 0;
}