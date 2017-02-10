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

void ReadUncertaintiesFromFile(string delta_file,
                               vector<vector<double> >* uncertainty_1,
                               vector<vector<double> >* uncertainty_2) {
  cout << " reading" << endl;
  std::ifstream infile(delta_file.c_str());
  std::string line;
  uncertainty_1->clear();
  uncertainty_2->clear();
  while (std::getline(infile, line)) {
    vector<double> u1;
    vector<double> u2;
    u1.resize(3);
    u2.resize(3);
    std::istringstream iss(line);
    if (!(iss >> u1[0] >> u1[1] >> u1[2]
      >> u2[0] >> u2[1] >> u2[2] )) { 
      cout << "problem" << endl;
      break; } // error
    uncertainty_1->push_back(u1);
    uncertainty_2->push_back(u2);
  }
}

void ReadDeltasFromFile(string delta_file,
                        vector<vector<double> >* deltas_1,
                        vector<vector<double> >* deltas_2,
                        vector<vector<double> >* uncertainty_1,
                        vector<vector<double> >* uncertainty_2
                       ) { 
  cout << " reading" << endl;
  std::ifstream infile(delta_file.c_str());
  std::string line;
  deltas_1->clear();
  deltas_2->clear();
  while (std::getline(infile, line)) {
    vector<double> delta;
    vector<double> delta2;
    vector<double> u;
    vector<double> u2;
    delta.resize(8);
    delta2.resize(8);
    u.resize(3);
    u2.resize(3);
    u[0] = 0;
    u[1] = 0;
    u[2] = 0;
    u2[0] = 0;
    u2[1] = 0;
    u2[2] = 0;
    std::istringstream iss(line);
    double x, y, x1, y1;
    if (!(iss >> delta[0] >> delta[1] >> delta[2] >> delta[3] >> delta[4] >> delta[5] >> x >> y
      >> delta2[0] >> delta2[1] >> delta2[2] >> delta2[3] >> delta2[4] >> delta2[5] >> x1 >> y1)) { 
      cout << "problem" << endl;
      break; } // error
    deltas_1->push_back(delta);
    deltas_2->push_back(delta2);
    uncertainty_1->push_back(u);
    uncertainty_2->push_back(u2);
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


Eigen::Transform<double, 3, Eigen::Affine> TransformUncertainty(
    Eigen::Transform<double, 3, Eigen::Affine> q,
    Eigen::Transform<double, 3, Eigen::Affine> r,
    const vector<double>& u) {
  
  Eigen::Transform<double, 3, Eigen::Affine> UncertainRotation;
  
  Eigen::Quaternion<double> q_q;
  q_q = q.rotation();
  Eigen::Quaternion<double> r_q;
  r_q = r.rotation();
  Eigen::Vector3d q_q_v;
  q_q_v << q_q.x(), q_q.y(), q_q.z();
  Eigen::Vector3d r_q_v;
  r_q_v << r_q.x(), r_q.y(), r_q.z();
  Eigen::Vector3d u_v;
  u_v << u[0], u[1], u[2];
  if(u_v.norm() != 0) {
    u_v.normalize();
    Eigen::Vector3d temp = q_q * r_q_v;
    double mag = temp.dot(u_v);
    Eigen::Vector3d axis = mag * u_v;
    Eigen::Quaternion<double> uncertain_q(axis[0], axis[1],axis[2],r_q.w());
    UncertainRotation = uncertain_q;
  } else{
    Eigen::Quaternion<double> uncertain_q(0,0,0,0);
    UncertainRotation = uncertain_q;
  }
  
  return UncertainRotation;
}

struct PartialRotationErrorNumeric {
  PartialRotationErrorNumeric(const vector<double>& delta_1,
                       const vector<double>& delta_2,
                       const vector<double>& u1,
                       const vector<double>& u2
                             ) :
      delta_1(delta_1),
      delta_2(delta_2),
      u1(u1),
      u2(u2){}

  bool operator()(const double* const camera,
                  double* residuals) const {

    Eigen::Transform<double, 3, Eigen::Affine> q = 
        AAToTransform(camera[0], camera[1], camera[2]);
    Eigen::Transform<double, 3, Eigen::Affine> q1 = 
        AAToTransform(double(delta_1[0]), double(delta_1[1]), double(delta_1[2]));
    Eigen::Transform<double, 3, Eigen::Affine> q2 = 
        AAToTransform(double(delta_2[0]), double(delta_2[1]), double(delta_2[2]));
    
    Eigen::Transform<double, 3, Eigen::Affine> v1 = TransformUncertainty(q.inverse(), q1, u2);    
    Eigen::Transform<double, 3, Eigen::Affine> v2 = TransformUncertainty(q, q2, u1); 
    
    Eigen::Transform<double, 3, Eigen::Affine> left;
    left = q1 * q * v1.inverse();
    Eigen::Transform<double, 3, Eigen::Affine> right;
    right = v2.inverse() * q * q2;
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
                                     const vector<double>& delta_2,
                                     const vector<double>& u1,
                                     const vector<double>& u2
                                    ) {
    return (new ceres::NumericDiffCostFunction<PartialRotationErrorNumeric, 
            ceres::CENTRAL, 1, 6>(
            new PartialRotationErrorNumeric(delta_1, delta_2, u1, u2)));
  }

 const vector<double> delta_1;
 const vector<double> delta_2;
 const vector<double> u1;
 const vector<double> u2;
};

struct PartialTranslationErrorNumeric {
  PartialTranslationErrorNumeric(const vector<double>& delta_1,
                       const vector<double>& delta_2,
                       const vector<double>& u1,
                       const vector<double>& u2) :
      delta_1(delta_1),
      delta_2(delta_2),
      u1(u1),
      u2(u2){}

  bool operator()(const double* const camera,
                  double* residuals) const {

    Eigen::Transform<double, 3, Eigen::Affine> q = 
        AAToTransform(camera[0], camera[1], camera[2]);
    Eigen::Transform<double, 3, Eigen::Affine> q1 = 
        AAToTransform(double(delta_1[0]), double(delta_1[1]), double(delta_1[2]));
    
    Eigen::Vector3d t1;
    Eigen::Vector3d t2;
    Eigen::Vector3d T;
    t1 << delta_1[3], delta_1[4], delta_1[5];
    t2 << delta_2[3], delta_2[4], delta_2[5];
    T << camera[3], camera[4], camera[5];
    
    Eigen::Vector3d u1_v;
    u1_v << u1[0], u1[1], u1[2];
    Eigen::Vector3d u2_v;
    u2_v << u2[0], u2[1], u2[2];
    
    // Vector projections for partials based on uncertainty
    
    Eigen::Vector3d l_u1 = (q1 * T).dot(u1_v) * u1_v;
    Eigen::Vector3d l_u2 = (q1 * T).dot(q * u2_v) * u2_v;
    Eigen::Vector3d r_u1 = (q * t2).dot(u1_v) * u1_v;
    Eigen::Vector3d r_u2 = t1.dot(q * u2_v) * u2_v;
    
    
    Eigen::Vector3d left;
    left = (T - (q1 * T)) - l_u1 - l_u2;
    Eigen::Vector3d right;
    right = (t1 - (q * t2)) + r_u1 - r_u2;
    
    Eigen::Vector3d error_vector = left - right;
    
    residuals[0] = error_vector.norm();
    return true;
  }

  // Factory to hide the construction of the CostFunction object from
  // the client code.
  static ceres::CostFunction* Create(const vector<double>& delta_1,
                                     const vector<double>& delta_2,
                                     const vector<double>& u1,
                                     const vector<double>& u2) {
    return (new ceres::NumericDiffCostFunction<PartialTranslationErrorNumeric, 
            ceres::CENTRAL, 1, 6>(
            new PartialTranslationErrorNumeric(delta_1, delta_2, u1, u2)));
  }

 const vector<double> delta_1;
 const vector<double> delta_2;
 const vector<double> u1;
 const vector<double> u2;
};

void PartialCalibrateR(
        const vector<vector<double> >& deltas_1,
        const vector<vector<double> >& uncertaintyR_1,
        const vector<vector<double> >& uncertaintyT_1,
        const vector<vector<double> >& deltas_2,
        const vector<vector<double> >& uncertaintyR_2,
        const vector<vector<double> >& uncertaintyT_2,
        double* transform,
        double* final_rmse) {
  
//   bool kUseNumericOverAutoDiff = false;
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
              deltas_1[i], deltas_2[i], uncertaintyR_1[i], uncertaintyR_2[i]);
      
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
    std::cout << summary.FullReport() << "\n";
    ceres::Problem::EvaluateOptions evalOptions = ceres::Problem::EvaluateOptions();
    residuals.clear();
    problem.Evaluate(evalOptions, NULL, &residuals, NULL, NULL);
  }
  PrintPose(transform);
  fprintf(stdout, "RMSE: %f\n", rmse);
}

void PartialCalibratet(
        const vector<vector<double> >& deltas_1,
        const vector<vector<double> >& uncertaintyR_1,
        const vector<vector<double> >& uncertaintyT_1,
        const vector<vector<double> >& deltas_2,
        const vector<vector<double> >& uncertaintyR_2,
        const vector<vector<double> >& uncertaintyT_2,
        double* transform,
        double* final_rmse) {
  
//   bool kUseNumericOverAutoDiff = false;
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
      
      cost_function = PartialTranslationErrorNumeric::Create(
              deltas_1[i], deltas_2[i], uncertaintyT_1[i], uncertaintyT_2[i]);
      
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
    std::cout << summary.FullReport() << "\n";
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
  vector<vector<double> > uncertaintyR_1;
  vector<vector<double> > uncertaintyR_2;
  vector<vector<double> > uncertaintyT_1;
  vector<vector<double> > uncertaintyT_2;
  ReadDeltasFromFile(file,
                     &deltas_1,
                     &deltas_2,
                     &uncertaintyR_1,
                     &uncertaintyR_2);
  
  ReadUncertaintiesFromFile("generated_uncertaintiest.txt",
                     &uncertaintyT_1,
                     &uncertaintyT_2);

  cout << deltas_1.size() << endl;
  cout << deltas_2.size() << endl;
  double* transform = new double[6];
  transform[0] = 0;
  transform[1] = 0;
  transform[2] = 0;
  transform[3] = 0;
  transform[4] = 0;
  transform[5] = 0;
  
  double* RMSE = 0;
  PartialCalibrateR(deltas_1, uncertaintyT_1, uncertaintyT_1, deltas_2, uncertaintyT_2, uncertaintyT_2,transform, RMSE);
//   PartialCalibratet(deltas_1, uncertaintyR_1, uncertaintyT_1, deltas_2, uncertaintyR_2, uncertaintyT_2,transform, RMSE);
  return 0;
}