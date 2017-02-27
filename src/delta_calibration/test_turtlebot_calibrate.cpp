#include "delta_calibration/partial_calibrate.h"

using namespace partial_calibrate;

void WritePose(double* pose, ofstream& file) {
  if (file.is_open()) {
    for(int j = 0; j < 6; j++) {
      file << pose[j] << "\t";
    }
    file << "\n";
  }
}
void PrintPose(double* pose){
  for(uint i = 0; i < 6; i ++) {
    cout << pose[i] << " ";
  }
  cout << endl;
}
int main(int argc, char **argv) {
  signal(SIGINT,HandleStop);
  signal(SIGALRM,HandleStop);
  
  // Initialize Ros
  ros::init(argc, argv, "partial_calibrate",
  ros::init_options::NoSigintHandler);
  ros::NodeHandle n;
  
  cout << "reading deltas" << endl;
  string file1 = "generated_deltas_rot.txt";
  string file2 = "generated_deltas_trans.txt";
  vector<vector<double> > deltas_1;
  vector<vector<double> > deltas_2;
  vector<vector<double> > deltasT_1;
  vector<vector<double> > deltasT_2;
  vector<vector<double> > uncertaintyR_1;
  vector<vector<double> > uncertaintyE_1;
  vector<vector<double> > uncertaintyE_2;
  vector<vector<double> > uncertaintyR_2;
  vector<vector<double> > uncertaintyT_1;
  vector<vector<double> > uncertaintyT_2;
  vector<vector<double> > TuncertaintyR_1;
  vector<vector<double> > TuncertaintyR_2;
  vector<vector<double> > TuncertaintyT_1;
  vector<vector<double> > TuncertaintyT_2;
  ReadDeltasFromFile(file1,
                     &deltas_1,
                     &deltas_2,
                     &uncertaintyE_1,
                     &uncertaintyE_2);
  
  ReadDeltasFromFile(file2,
                     &deltasT_1,
                     &deltasT_2,
                     &uncertaintyE_1,
                     &uncertaintyE_2);
  
  ReadUncertaintiesFromFile("generated_uncertaintiest.txt",
                     &uncertaintyT_1,
                     &uncertaintyT_2);
  
  ReadUncertaintiesFromFile("generated_uncertaintiesr.txt",
                            &uncertaintyR_1,
                            &uncertaintyR_2);
  
  ReadUncertaintiesFromFile("generated_uncertaintiest.txt",
                            &TuncertaintyT_1,
                            &TuncertaintyT_2);
  
  ReadUncertaintiesFromFile("generated_uncertaintiesr.txt",
                            &TuncertaintyR_1,
                            &TuncertaintyR_2);

  cout << deltas_1.size() << endl;
  cout << deltas_2.size() << endl;
  cout << uncertaintyR_1.size() << endl;
  cout << uncertaintyR_2.size() << endl;
  cout << uncertaintyT_1.size() << endl;
  cout << uncertaintyT_2.size() << endl;
  double* transform = new double[6];
  transform[0] = 0;
  transform[1] = 0;
  transform[2] = 0;
  transform[3] = 0;
  transform[4] = 0;
  transform[5] = 0;
  
  double* RMSE = 0;
//   PartialCalibrateR(deltas_1, uncertaintyR_1, uncertaintyT_1, deltas_2, uncertaintyR_2, uncertaintyT_2,transform, RMSE);
//   cout << "calibrated" << endl;
  PartialCalibrateRwT(deltas_1, uncertaintyR_1, uncertaintyT_1, deltas_2, uncertaintyR_2, uncertaintyT_2,deltasT_1, TuncertaintyR_1, TuncertaintyT_1, deltasT_2, TuncertaintyR_2, TuncertaintyT_2,transform, RMSE);
//   PartialCalibrateT(deltas_1, uncertaintyR_1, uncertaintyT_1, deltas_2, uncertaintyR_2, uncertaintyT_2,transform, RMSE);
  cout << "calibrated2" << endl;
  PrintPose(transform);
  ofstream output("calibration.pose");
  WritePose(transform, output);
  return 0;
}