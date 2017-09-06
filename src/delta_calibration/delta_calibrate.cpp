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
#include <pcl/common/common.h>

using namespace partial_calibrate;

void WritePose(double *pose, ofstream &file) {
  if (file.is_open()) {
    for (int j = 0; j < 5; j++) {
      file << pose[j] << "\t";
    }
    file << pose[5];
  }
}

void PrintPose(double *pose) {
  for (uint i = 0; i < 6; i++) {
    cout << pose[i] << " ";
  }
  cout << endl;
}

float FindZ(string bag_file, double* transform) {
  const string bag_name = bag_file + ".bag";
  cout << " Finished reading transform " << endl;
  double *transform_temp = new double[6];
  transform_temp[0] = transform[0];
  transform_temp[1] = transform[1];
  transform_temp[2] = transform[2];
  transform_temp[3] = 0;
  transform_temp[4] = 0;
  transform_temp[5] = 0;
  // Get the first pose from the bag file
  rosbag::Bag bag;
  bag.open(bag_name, rosbag::bagmode::Read);
  std::vector<std::string> topics;
  std::vector<std::string> odom_topics;
  odom_topics.push_back(std::string("/camera/depth/points"));
  rosbag::View odom_view(bag, rosbag::TopicQuery(odom_topics));
  rosbag::View::iterator bag_it = odom_view.begin();
  rosbag::View::iterator end = odom_view.end();
  std::deque<pcl::PointCloud<pcl::PointXYZ>> buffer;
  std::deque<double> timestamps;
  pcl::PointCloud<pcl::PointXYZ> cloud, cloud2;
  double time;
  // Get Cloud
  bag_it = OneSensorCloudsBrass(bag_it, end, &buffer, &timestamps, &cloud, &time);
  cloud2 = cloud;
  TransformPointCloudInv(&cloud2, transform_temp);

  // Naive Solution find the minimum z value as the height
  pcl::PointXYZ min, max;
  pcl::getMinMax3D(cloud2, min, max);
  cout << "Greatest Z: " << max << endl;
  cout << "Min Z: " << min << endl;
  return min.z;
}

template <class T>
Eigen::Transform<T, 3, Eigen::Affine> AAToTransform(T x, T y, T z) {
  // Create the eigen transform from the components
  Eigen::Matrix<T, 3, 1> axis(x, y, z);
  const T angle = axis.norm();
  if (angle > T(0)) {
    axis = axis / angle;
  }
  const Eigen::Transform<T, 3, Eigen::Affine> rotation =
  Eigen::Transform<T, 3, Eigen::Affine>(Eigen::AngleAxis<T>(angle, axis));
  return rotation;
}

Eigen::Transform<double, 3, Eigen::Affine>
TransformUncertainty(Eigen::Transform<double, 3, Eigen::Affine> q,
                     Eigen::Transform<double, 3, Eigen::Affine> r,
                     const vector<double> &u) {

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
  if (u_v.norm() != 0) {
    u_v.normalize();
    Eigen::Vector3d temp = q_q * r_q_v;
    double mag = temp.dot(u_v);
    Eigen::Vector3d axis = mag * u_v;
    Eigen::Quaternion<double> uncertain_q(r_q.w(), axis[0], axis[1], axis[2]);
    UncertainRotation = uncertain_q;
  } else {
    Eigen::Quaternion<double> uncertain_q(0, 0, 0, 0);
    UncertainRotation = uncertain_q;
  }

  return UncertainRotation;
  }

void StripUncertainty(const vector<double> &ut_vector,
                      const vector<double> &ur_vector,
                      vector<double>* transform) {
  Eigen::Vector3d ut = {ut_vector[0], ut_vector[1], ut_vector[2]};
  Eigen::Vector3d ur = {ur_vector[0], ur_vector[1], ur_vector[2]};
  Eigen::Transform<double, 3, Eigen::Affine> R =
      AAToTransform((*transform)[0], (*transform)[1], (*transform)[2]);
  Eigen::Transform<double, 3, Eigen::Affine> I = AAToTransform(0.0, 0.0, 0.0);
  vector<double> ur_v = {ur[0], ur[1], ur[2]};
  Eigen::Vector3d trans = {(*transform)[3], (*transform)[4], (*transform)[5]};

  if (ur.norm() != 0) {
    Eigen::Transform<double, 3, Eigen::Affine> U =
    TransformUncertainty(I, R, ur_v);
    // Remove the rotational uncertainty from R.
    R = U.inverse() * R;
    // If there is only one axis of translational certainty
    if (ut.norm() >= 100) {
      // only the portion of translation along the axis of rotational
      // uncertainty should be kept.
      trans = trans.dot(ur)*ur;
    }
  } else {
    if (ut.norm() != 0) {
      // Remove translation in direction of uncertainty
      trans = trans - trans.dot(ut)*ut;
    }
  }

  // Recombine rotation and translation into a single transform.

  Eigen::AngleAxis<double> angle_axis(R.rotation());
  // Get the axis
  Eigen::Matrix<double, 3, 1> normal_axis = angle_axis.axis();
  // Recompute the rotation angle
  double combined_angle = angle_axis.angle();
  Eigen::Matrix<double, 3, 1> combined_axis = normal_axis * combined_angle;
  (*transform)[0] = combined_axis[0];
  (*transform)[1] = combined_axis[1];
  (*transform)[2] = combined_axis[2];
  (*transform)[3] = trans[0];
  (*transform)[4] = trans[1];
  (*transform)[5] = trans[2];
                      }

int main(int argc, char **argv) {
  signal(SIGINT, HandleStop);
  signal(SIGALRM, HandleStop);

  char *filename = (char *)"test.bag";
  char *filename2 = (char *)"pair_upright.bag";
  bool turtlebot_mode = false;
  // Parse arguments.
  static struct poptOption options[] = {
      {"file", 'f', POPT_ARG_STRING, &filename, 0,
        "Filename to use for all or rotation calibration data.",
       "STR"},
       {"trans_file", 'g', POPT_ARG_STRING, &filename2, 0,
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
  const string filename_str(filename);
  const string filename_str2(filename2);
  cout << filename << std::endl;
  cout << filename_str << std::endl;
  cout << filename_str2 << std::endl;
  const string file = filename_str + ".pose";
  const string uncertainty_t_string = filename_str + "_U_T.txt";
  const string uncertainty_r_string = filename_str + "_U_R.txt";
  const string output_str = filename_str + ".extrinsics";
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
    const string file2 = filename_str2 + ".pose";
    const string uncertainty_t_string = filename_str2 + "_U_T.txt";
    const string uncertainty_r_string = filename_str2 + "_U_R.txt";
    ReadDeltasFromFile(file2, &deltasT_1, &deltasT_2, &uncertaintyE_1,
                       &uncertaintyE_2);
    std::cout << "Deltas 1 size: " << deltas_1.size() << std::endl;
    std::cout << "Deltas 2 size: " << deltas_1.size() << std::endl;
    std::cout << "DeltasT 1 size: " << deltasT_1.size() << std::endl;
    std::cout << "DeltasT 2 size: " << deltasT_2.size() << std::endl;

    ReadUncertaintiesFromFile(uncertainty_t_string,
                              &TuncertaintyT_1, &TuncertaintyT_2);

    ReadUncertaintiesFromFile(uncertainty_r_string,
                              &TuncertaintyR_1, &TuncertaintyR_2);
    std::cout << "uncertainties read" << std::endl;
    for (size_t i = 0; i < deltas_1.size(); ++i) {
      StripUncertainty(uncertaintyT_1[i], uncertaintyR_1[i],
                       &deltas_1[i]);
    }
    for (size_t i = 0; i < deltas_2.size(); ++i) {
      StripUncertainty(uncertaintyT_2[i], uncertaintyR_2[i],
                       &deltas_2[i]);
    }
    for (size_t i = 0; i < deltasT_1.size(); ++i) {
      StripUncertainty(TuncertaintyT_1[i], TuncertaintyR_1[i],
                       &deltasT_1[i]);
    }
    for (size_t i = 0; i < deltasT_2.size(); ++i) {
      StripUncertainty(TuncertaintyT_2[i], TuncertaintyR_2[i],
                       &deltasT_2[i]);
    }
    PartialCalibrateRwT(deltas_1, uncertaintyR_1, uncertaintyT_1, deltas_2,
                      uncertaintyR_2, uncertaintyT_2, deltasT_1,
                      TuncertaintyR_1, TuncertaintyT_1, deltasT_2,
                      TuncertaintyR_2, TuncertaintyT_2, transform, RMSE);
    const float z = -FindZ(filename_str, transform);
    transform[5] = z;
  } else {
    double *RMSE = 0;
    for (size_t i = 0; i < deltas_1.size(); ++i) {
      StripUncertainty(uncertaintyT_1[i], uncertaintyR_1[i],
                       &deltas_1[i]);
    }
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