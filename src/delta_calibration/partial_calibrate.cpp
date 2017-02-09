//----------- INCLUDES
#include <nav_msgs/Odometry.h>
#include "delta_calibration/icp.h"

using std::size_t;
using std::vector;
using namespace std;
using namespace icp;
using Eigen::Vector3d;

//Intialize empty publishers
ros::Publisher cloud_pub_1;
ros::Publisher cloud_pub_2;
ros::Publisher cloud_pub_3;
ros::Publisher cloud_pub_4;

ros::Publisher marker_pub;
ros::Publisher markerArray_pub;

// Signal handler for breaks (Ctrl-C)
void HandleStop(int i) {
  printf("\nTerminating.\n");
  exit(0);
}

bool DoubleEquals(double x, double y) {
  return fabs(x-y) < .000005;
}

void PrintPose(double* pose){
  for(uint i = 0; i < 6; i ++) {
    cout << pose[i] << " ";
  }
  cout << endl;
}

void ReadDeltasFromFile(string delta_file,
                        vector<vector<double> >* deltas_1,
                        vector<vector<double> >* deltas_2) { 
  cout << " reading" << endl;
  std::ifstream infile(delta_file.c_str());
  std::string line;
  deltas_1->clear();
  deltas_2->clear();
  while (std::getline(infile, line)) {
    vector<double> delta;
    vector<double> delta2;
    delta.resize(8);
    delta2.resize(8);
    std::istringstream iss(line);
    double x, y, x1, y1;
    if (!(iss >> delta[0] >> delta[1] >> delta[2] >> delta[3] >> delta[4] >> delta[5] >> x >> y
      >> delta2[0] >> delta2[1] >> delta2[2] >> delta2[3] >> delta2[4] >> delta2[5] >> x1 >> y1)) { 
      cout << "problem" << endl;
      break; } // error
    deltas_1->push_back(delta);
    deltas_2->push_back(delta2);
  }
}

template <class T> Eigen::Transform<T, 3, Eigen::Affine> AAToTransform(T x, T y, T z) {
  //Create the eigen transform from the components
    Eigen::Matrix<T,3,1> axis(x, y, z);
    const T angle = axis.norm();
    if (angle > T(0)) {
      axis = axis / angle;
    }
    const Eigen::Transform<T, 3, Eigen::Affine> rotation =
    Eigen::Transform<T, 3, Eigen::Affine>(
    Eigen::AngleAxis<T>(angle, axis));
    return rotation;
}

// struct PartialRotationError {
//   PartialRotationError(const vector<double>& delta_1,
//                        const vector<double>& delta_2) :
//       delta_1(delta_1),
//       delta_2(delta_2) {}
// 
//   template <class T>
//   bool operator()(const T* const camera,
//                   T* residuals) const {
//       
// //     // Transform point_1 to the base reference frame by applying the camera tf.
// //     const Eigen::Matrix<T, 3, 1> point_1_transformed =
// //         TransformPoint<T>(point_1.cast<T>(), camera);
// //     // Transform normal_1 to the base reference frame by applying the camera tf.
// //     const Eigen::Matrix<T, 3, 1> normal_1_transformed =
// //         TransformVector<T>(normal_1.cast<T>(), camera);
// //     // The error is the difference between the predicted and observed position.
//                     
//     Eigen::Transform<T, 3, Eigen::Affine> q = 
//         AAToTransform(camera[0], camera[1], camera[2]);
//     Eigen::Transform<T, 3, Eigen::Affine> q1 = 
//         AAToTransform(T(delta_1[0]), T(delta_1[1]), T(delta_1[2]));
//     Eigen::Transform<T, 3, Eigen::Affine> q2 = 
//         AAToTransform(T(delta_2[0]), T(delta_2[1]), T(delta_2[2]));
//     
//     Eigen::Transform<T, 3, Eigen::Affine> left;
//     left = q1 * q;
//     Eigen::Transform<T, 3, Eigen::Affine> right;
//     right = q * q2;
//     Eigen::Transform<T, 3, Eigen::Affine> error_transform;
//     error_transform = left.inverse() * right;
//     
//     // Find the rotation component
//     // Find the angle axis format
//     Eigen::AngleAxis<T> angle_axis(error_transform.rotation());
//   
//     // Get the axis
//     Eigen::Matrix<T, 3, 1> normal_axis = angle_axis.axis();
//     
//     // Recompute the rotation angle
//     T combined_angle = angle_axis.angle();
//     Eigen::Matrix<T, 3, 1> combined_axis = normal_axis * combined_angle;
//     
//     residuals[0] = combined_axis.norm();
//     return true;
//   }
// 
//   // Factory to hide the construction of the CostFunction object from
//   // the client code.
//   static ceres::CostFunction* Create(const vector<double>& delta_1,
//                                      const vector<double>& delta_2) {
//     return (new ceres::AutoDiffCostFunction<PartialRotationError, 1, 6>(
//             new PartialRotationError(delta_1, delta_2)));
//   }
// 
//  const vector<double> delta_1;
//  const vector<double> delta_2;
// };

struct PartialRotationErrorNumeric {
  PartialRotationErrorNumeric(const vector<double>& delta_1,
                       const vector<double>& delta_2) :
      delta_1(delta_1),
      delta_2(delta_2) {}

  bool operator()(const double* const camera,
                  double* residuals) const {

    Eigen::Transform<double, 3, Eigen::Affine> q = 
        AAToTransform(camera[0], camera[1], camera[2]);
    Eigen::Transform<double, 3, Eigen::Affine> q1 = 
        AAToTransform(double(delta_1[0]), double(delta_1[1]), double(delta_1[2]));
    Eigen::Transform<double, 3, Eigen::Affine> q2 = 
        AAToTransform(double(delta_2[0]), double(delta_2[1]), double(delta_2[2]));
    
    Eigen::Transform<double, 3, Eigen::Affine> left;
    left = q1 * q;
    Eigen::Transform<double, 3, Eigen::Affine> right;
    right = q * q2;
    Eigen::Transform<double, 3, Eigen::Affine> error_transform;
    error_transform = left.inverse() * right;
    
    // Find the rotation component
    // Find the angle axis format
    Eigen::AngleAxis<double> angle_axis(error_transform.rotation());
  
    // Get the axis
    Eigen::Matrix<double, 3, 1> normal_axis = angle_axis.axis();
    
    // Recompute the rotation angle
    double combined_angle = angle_axis.angle();
    Eigen::Matrix<double, 3, 1> combined_axis = normal_axis * combined_angle;
    residuals[0] = combined_axis.norm();
    return true;
  }

  // Factory to hide the construction of the CostFunction object from
  // the client code.
  static ceres::CostFunction* Create(const vector<double>& delta_1,
                                     const vector<double>& delta_2) {
    return (new ceres::NumericDiffCostFunction<PartialRotationErrorNumeric, 
            ceres::CENTRAL, 1, 6>(
            new PartialRotationErrorNumeric(delta_1, delta_2)));
  }

 const vector<double> delta_1;
 const vector<double> delta_2;
};

struct PartialTranslationErrorNumeric {
  PartialTranslationErrorNumeric(const vector<double>& delta_1,
                       const vector<double>& delta_2) :
      delta_1(delta_1),
      delta_2(delta_2) {}

  bool operator()(const double* const camera,
                  double* residuals) const {

    Eigen::Transform<double, 3, Eigen::Affine> q = 
        AAToTransform(camera[0], camera[1], camera[2]);
    Eigen::Transform<double, 3, Eigen::Affine> q1 = 
        AAToTransform(double(delta_1[0]), double(delta_1[1]), double(delta_1[2]));
    Eigen::Transform<double, 3, Eigen::Affine> q2 = 
        AAToTransform(double(delta_2[0]), double(delta_2[1]), double(delta_2[2]));
    
    Eigen::Vector3d t1;
    Eigen::Vector3d t2;
    Eigen::Vector3d T;
    t1 << delta_1[3], delta_1[4], delta_1[5];
    t2 << delta_2[3], delta_2[4], delta_2[5];
    T << camera[3], camera[4], camera[5];
    Eigen::Vector3d left;
    left = q * t2 + T;
    Eigen::Vector3d right;
    right = q1 * T + t1;
    
    Eigen::Vector3d error_vector = left - right;
    
    residuals[0] = error_vector.norm();
    return true;
  }

  // Factory to hide the construction of the CostFunction object from
  // the client code.
  static ceres::CostFunction* Create(const vector<double>& delta_1,
                                     const vector<double>& delta_2) {
    return (new ceres::NumericDiffCostFunction<PartialTranslationErrorNumeric, 
            ceres::CENTRAL, 1, 6>(
            new PartialTranslationErrorNumeric(delta_1, delta_2)));
  }

 const vector<double> delta_1;
 const vector<double> delta_2;
};

void PartialCalibrate(
        const vector<vector < double> >& deltas_1,
        const vector<vector < double> >& deltas_2,
        double* transform,
        double* final_rmse) {
  bool kUseNumericOverAutoDiff = false;
  CHECK_NOTNULL(transform);
  // Tolerance for RMSE.
  static const double kToleranceError = 0.00001;
  // The maximum number of overall iterations.
  static const int kMaxIterations = 80;
  // The maximum number of repeat iterations while the RMSE is unchanged.
  static const int kMaxRepeatIterations = 5;
  double rmse = 1000000;
  double last_rmse = 1000010;
  vector<double> residuals;
  
  for (int iteration = 0, repeat_iteration = 0;
       iteration < kMaxIterations &&
       repeat_iteration < kMaxRepeatIterations &&
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
    for(size_t i = 0; i < deltas_1.size(); i++) {
      ceres::CostFunction* cost_function = NULL;
      
      cost_function = PartialRotationErrorNumeric::Create(
              deltas_1[i], deltas_2[i]);
      
      problem.AddResidualBlock(cost_function,
                                NULL, // squared loss
                                transform);
      
      cost_function = PartialTranslationErrorNumeric::Create(
              deltas_1[i], deltas_2[i]);
      
      problem.AddResidualBlock(cost_function,
                                NULL, // squared loss
                                transform);
    }
    
    // Run Ceres problem
    ceres::Solver::Options options;
    options.linear_solver_type = ceres::DENSE_QR;
    ceres::Solver::Summary summary;
    ceres::Solve(options, &problem, &summary);
    rmse =
        sqrt(summary.final_cost / static_cast<double>(summary.num_residuals));
//     std::cout << summary.FullReport() << "\n";
    ceres::Problem::EvaluateOptions evalOptions = ceres::Problem::EvaluateOptions();
    residuals.clear();
    problem.Evaluate(evalOptions, NULL, &residuals, NULL, NULL);
  }
  PrintPose(transform);
  fprintf(stdout, "RMSE: %f\n", rmse);
}

int main(int argc, char **argv) {
  signal(SIGINT,HandleStop);
  signal(SIGALRM,HandleStop);
  
  // Initialize Ros
  ros::init(argc, argv, "partial_calibrate",
  ros::init_options::NoSigintHandler);
  ros::NodeHandle n;
  
  //----------  Setup Publishers ----------
//   cloud_pub_1 = n.advertise<sensor_msgs::PointCloud2> ("cloud_pub_1", 1);
//   cloud_pub_2 = n.advertise<sensor_msgs::PointCloud2> ("cloud_pub_2", 1);
//   cloud_pub_3 = n.advertise<sensor_msgs::PointCloud2> ("cloud_pub_3", 1);
//   cloud_pub_4 = n.advertise<sensor_msgs::PointCloud2> ("cloud_pub_4", 1);
//   vector<ros::Publisher> publishers;
//   publishers.push_back(cloud_pub_1);
//   publishers.push_back(cloud_pub_2);
//   publishers.push_back(cloud_pub_3);
//   publishers.push_back(cloud_pub_4);
//   marker_pub =
//   n.advertise<visualization_msgs::Marker>("visualization_marker", 10);
//   markerArray_pub =
//   n.advertise<visualization_msgs::MarkerArray>(
//       "visualization_marker_array", 10);
  cout << "reading deltas" << endl;
  string file = "generated_deltas.txt";
  vector<vector<double> > deltas_1;
  vector<vector<double> > deltas_2;
  ReadDeltasFromFile(file,
                     &deltas_1,
                     &deltas_2);

  cout << deltas_1.size() << endl;
  cout << deltas_2.size() << endl;
  double* transform = new double[6];
  transform[0] = 0;
  transform[1] = 0;
  transform[2] = 0;
  transform[3] = 0;
  transform[4] = 0;
  transform[5] = 0;
  
  double* RMSE;
  PartialCalibrate(deltas_1, deltas_2, transform, RMSE);
  return 0;
}