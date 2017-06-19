//----------- INCLUDES
#include <nav_msgs/Odometry.h>
#include "delta_calibration/icp.h"
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/extract_indices.h>

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
// Controls usage of normal files
bool normal_file = false;

namespace delta_calc {

// Signal handler for breaks (Ctrl-C)
void HandleStop(int i) {
  printf("\nTerminating.\n");
  exit(0);
}

void PrintPose(double* pose){
  for(uint i = 0; i < 6; i ++) {
    cout << pose[i] << " ";
  }
  cout << endl;
}

template<typename T>
void PopFront(std::vector<T>* vec)
{
  assert(!vec.empty());
  vec.erase(vec.begin());
}

// For checking if the mean has not changed
bool DoubleEquals(double x, double y) {
  return fabs(x-y) < .1;
}

void SavePoses(const vector<double*>& poses, string& filename) {
  filename += ".poses";
  ofstream myfile (filename.c_str());
  if (myfile.is_open()) {
    for(size_t i = 0; i < poses.size(); i++) {
        double* pose = poses[i];
         for(int j = 0; j < 6; j++) {
          myfile << pose[j] << "\t";
    }
    myfile << "\n";
  }
    myfile.close();
  }
  else {
    cout << "Unable to open file";
  }
}

void WritePose(double* pose, ofstream file) {
  if (file.is_open()) {
    for(int j = 0; j < 6; j++) {
      file << pose[j] << "\t";
    }
  file << "\n";
  }
}

void WritePoseFile(double* pose,
                   const double& timestamp,
                   const int& count,
                   ofstream& file) {
  if (file.is_open()) {
    //cout << "Writing to bag_name" << endl;
    for(int j = 0; j < 6; j++) {

      file << pose[j] << "\t";
    }
  }
  file << std::setprecision(20) << timestamp << "\t" << count << "\t";
  file << std::flush;
}

void WriteUncertaintyFile(Eigen::Vector3d u, ofstream& file) {
  if (file.is_open()) {
    file << u[0] << "\t" << u[1] << "\t" << u[2] << endl;
    file << std::flush;
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
    Eigen::Quaternion<double> uncertain_q(r_q.w(), axis[0], axis[1],axis[2]);
    UncertainRotation = uncertain_q;
  } else{
    Eigen::Quaternion<double> uncertain_q(0,0,0,0);
    UncertainRotation = uncertain_q;
  }

  return UncertainRotation;
  }

  void StripUncertainty(const Vector3d& ut, const Vector3d& ur, double* transform) {
    Eigen::Transform<double, 3, Eigen::Affine> R = AAToTransform(transform[0], transform[1], transform[2]);
    Eigen::Transform<double, 3, Eigen::Affine> I = AAToTransform(0.0, 0.0, 0.0);
    vector<double> ur_v = {ur[0], ur[1], ur[2]};
    Eigen::Vector3d trans = {transform[3], transform[4], transform[5]};


    if(ur.norm() != 0) {
      Eigen::Transform<double, 3, Eigen::Affine> U = TransformUncertainty(I, R, ur_v);
      R = U.inverse() * R;
      // Find the rotation component
      // Find the angle axis format

//       trans = trans.dot(ur)*ur;
    }
    if(trans.norm() != 0) {
      if(ut[0] > .9) {
	Eigen::Vector3d x = {1,0,0};
	trans = trans - (trans.dot(x)*x);
      } if(ut[1] > .9) {
	Eigen::Vector3d y = {0,1,0};
	trans = trans - (trans.dot(y)*y);
      } if(ut[2] > .9) {
	Eigen::Vector3d z = {0,0,1};
	trans = trans - (trans.dot(z)*z);
      }

    }
    Eigen::AngleAxis<double> angle_axis(R.rotation());

    // Get the axis
    Eigen::Matrix<double, 3, 1> normal_axis = angle_axis.axis();

    // Recompute the rotation angle
    double combined_angle = angle_axis.angle();
    Eigen::Matrix<double, 3, 1> combined_axis = normal_axis * combined_angle;

    transform[0] = combined_axis[0];
    transform[1] = combined_axis[1];
    transform[2] = combined_axis[2];
    transform[3] = trans[0];
    transform[4] = trans[1];
    transform[5] = trans[2];
  }

void CalculateDelta(
    const int k,
    const vector<ros::Publisher>& publishers,
    const string& covarianceFolder,
    const string& bagFile,
    const pcl::PointCloud<pcl::PointXYZ>& cloud_1,
    const pcl::PointCloud<pcl::PointXYZ>& cloud_2,
    const pcl::PointCloud<pcl::PointXYZ>& cloud_3,
    const pcl::PointCloud<pcl::Normal>& normal_1,
    const pcl::PointCloud<pcl::Normal>& normal_2,
    const pcl::PointCloud<pcl::Normal>& normal_3,
    double* transform,
    double* final_rmse) {

  // From this point on, calculating the icp between clouds
  double* calculated_delta = new double[6];
  // Initialize transform ARRAYS
  vector<double> pose0(6, 0.0);
  std::copy(pose0.begin(), pose0.end(), calculated_delta);
  vector<Eigen::Vector2d> empty_coords;
  // Run ICP between clouds k and
  // clouds k-1 (either the keyframe or the last clouds)
  fprintf(stdout, "ICP 1 \n");
  ICP (k,
       .05,
       publishers,
       covarianceFolder,
       bagFile,
       cloud_1,
       cloud_2,
       normal_1,
       normal_2,
       empty_coords,
       empty_coords,
       calculated_delta,
       NULL);
  Eigen::Matrix<double,3,1> axis(calculated_delta[0], calculated_delta[1], calculated_delta[2]);
  Eigen::Matrix<double,3,1> trans(calculated_delta[3], calculated_delta[4], calculated_delta[5]);
  const double angle = axis.norm();
  double angle_degree = (180/3.14) * angle;
  cout << "Instant angular rotation: " << angle_degree << endl;
  *final_rmse = angle_degree;

//    combine the transform returned by ICP with the combined transform,
//    and run ICP between clouds k and the last keyframe
      CombineTransform(transform, calculated_delta);
      fprintf(stdout, "ICP 2 \n");
      ICP (k,
       .05,
       publishers,
       covarianceFolder,
       bagFile,
       cloud_3,
       cloud_2,
       normal_3,
       normal_2,
       empty_coords,
       empty_coords,
       transform,
       NULL);

      double zero_pose[6];
      std::copy(pose0.begin(), pose0.end(), zero_pose);

//       cout << transform[0] << endl;
//       VisualizeReferenceFrame(transform,
//                               markerArray_pub,
//                               0
//       );
//       VisualizeReferenceFrame(zero_pose,
//                               markerArray_pub,
//                               3
//       );

}

void ExtractUncertainty(
    const vector<Eigen::Vector4d>& normal_equations,
    Eigen::Vector3d* uncertainty_T,
    Eigen::Vector3d* uncertainty_R) {

  Eigen::Vector3d rot_u;
  Eigen::Vector3d trans_u = {100, 100, 100};
  Eigen::Vector3d partial_u = {100, 0, 0};
  bool first_normal = true;
  int normal_count = 0;
  for(size_t i = 0; i < normal_equations.size(); i++) {
    Eigen::Vector4d normal_equation = normal_equations[i];
    Eigen::Vector3d normal =
        {normal_equation[0],normal_equation[1], normal_equation[2]};
    if(!first_normal) {
      // If new normal (not sure about second case of if statement).
      if(!DoubleEquals(abs(rot_u.dot(normal)), 1) ||
          DoubleEquals(rot_u.norm(), 0)) {
        // If a partial has not been calculated
        if(DoubleEquals(partial_u.norm(), 100)) {
          // Partial uncertainty is cross product of the two normals received.
          partial_u = rot_u.cross(normal);
          trans_u = partial_u.normalized();
          // Following converts sufficient uncertainty to values of 1.
          //if(abs(partial_u[0]) > .7) {
            //trans_u[0] = 1;
          //} else {
            //trans_u[0] = 0;
          //}
          //if(abs(partial_u[1]) > .7) {
            //trans_u[1] = 1;
          //}   else {
            //trans_u[1] = 0;
          //}
          //if(abs(partial_u[2]) > .7) {
            //trans_u[2] = 1;
          //} else {
            //trans_u[2] = 0;
          }
        }
        // If the two normals are not parallel, no rotation uncertainty.
        if(!DoubleEquals(abs(rot_u.dot(normal)), 1)) {
          rot_u[0] = 0;
          rot_u[1] = 0;
          rot_u[2] = 0;
        }
        // If we have a third normal, and it is not perpendicular to the current
        // translational uncertainty, no translational uncertainty.
        if(normal_count > 2 &&
          !DoubleEquals(partial_u.norm(), 100) &&
          !DoubleEquals(abs(partial_u.dot(normal)), 0)) {
          trans_u = {0,0,0};
        }
    // For the first normal
    } else {
      first_normal = false;
      // Rotational axis of uncertainty is the normal
      rot_u = normal;
    }
  }
  *uncertainty_T = trans_u;
  *uncertainty_R = rot_u;
}

vector<pcl::PointCloud<pcl::PointXYZ> > ExtractPlanes(
  const pcl::PointCloud<pcl::PointXYZ>& cloud,
  vector<Eigen::Vector4d>* normal_equations,
  vector<Eigen::Vector3d>* centroids) {

  vector<pcl::PointCloud<pcl::PointXYZ> > output;
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered (new pcl::PointCloud<pcl::PointXYZ>),
  cloud_p (new pcl::PointCloud<pcl::PointXYZ>),
  cloud_f (new pcl::PointCloud<pcl::PointXYZ>);

  // Convert to the templated PointCloud
  cloud_filtered = cloud.makeShared();
  vector<Eigen::Vector4d> equations;
  pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients ());
  pcl::PointIndices::Ptr inliers (new pcl::PointIndices ());
  // Create the segmentation object
  pcl::SACSegmentation<pcl::PointXYZ> seg;
  // Optional
  seg.setOptimizeCoefficients (true);
  // Mandatory
  seg.setModelType (pcl::SACMODEL_PLANE);
  seg.setMethodType (pcl::SAC_RANSAC);
  seg.setMaxIterations (1000);
  seg.setDistanceThreshold (0.01);

  // Create the filtering object
  pcl::ExtractIndices<pcl::PointXYZ> extract;

  int i = 0;
  double nr_points = (int) cloud_filtered->points.size ();
  // While 10% of the original cloud is still there
  int num_planes = 0;
  while (nr_points > (.1 * cloud.size())) {

    Eigen::Vector4d equation;
    num_planes +=1;
    // Segment the largest planar component from the remaining cloud
    seg.setInputCloud (cloud_filtered);
    seg.segment (*inliers, *coefficients);
    if (inliers->indices.size () == 0)
    {
      break;
    }

    // Extract the inliers
    extract.setInputCloud (cloud_filtered);
    extract.setIndices (inliers);
    extract.setNegative (false);
    extract.filter (*cloud_p);
    // Create the filtering object
    extract.setNegative (true);
    extract.filter (*cloud_f);
    cloud_filtered.swap (cloud_f);
    i++;
    nr_points = (int) cloud_filtered->points.size ();
    equation[0] = coefficients->values[0];
    equation[1] = coefficients->values[1];
    equation[2] = coefficients->values[2];
    equation[3] = coefficients->values[3];
    equations.push_back(equation);
    output.push_back(*cloud_p);
  }
  for(size_t i = 0; i < output.size(); i++) {
    Eigen::Vector3d centroid;
    pcl::PointCloud<pcl::PointXYZ> current_cloud = output[i];
    for(size_t j = 0; j < output[i].size(); j++) {
      pcl::PointXYZ point = current_cloud[j];
      centroid[0] += point.x;
      centroid[1] += point.y;
      centroid[2] += point.z;
    }
    centroid = centroid / current_cloud.size();
    centroids->push_back(centroid);
  }
  *normal_equations = equations;
  return output;
}

struct DeltaCalVariables {
  string bag_name;
  rosbag::Bag bag;
  string k1_output_name;
  string k2_output_name;
  string object_file;
  string covariance_file;
  string k1_covariance;
  string k2_covariance;
  string k1_base_name;
  string k2_base_name;
  rosbag::View::iterator bag_it;
  rosbag::View::iterator end;
  string pose_name;
  ofstream* pose_file;
  string velocity_name;
  ofstream* velocity_file;
  string trajectory_name;
  ofstream* traj_file;
  std::deque<pcl::PointCloud<pcl::PointXYZ> > k1_buffer;
  std::deque<pcl::PointCloud<pcl::PointXYZ> > k2_buffer;
  std::deque<double> k1_timestamps;
  std::deque<double> k2_timestamps;
  pcl::PointCloud<pcl::PointXYZ> k1_keyframe;
  pcl::PointCloud<pcl::PointXYZ> k2_keyframe;
  double k1_timestamp;
  double k2_timestamp;
  pcl::PointCloud<pcl::Normal> k1_key_normal;
  pcl::PointCloud<pcl::Normal> k2_key_normal;
  // Previous clouds
  pcl::PointCloud<pcl::PointXYZ> k1_prev;
  pcl::PointCloud<pcl::PointXYZ> k2_prev;
  // Previous normals
  pcl::PointCloud<pcl::Normal> k1_prev_normal;
  pcl::PointCloud<pcl::Normal> k2_prev_normal;
  double k1_prev_timestamp;
  double k2_prev_timestamp;
  double k1_combined_transform[6];
  double k2_combined_transform[6];
  vector<double> k1_velocity_list;
  vector<double> k2_velocity_list;
  vector<int> keys;
  vector<double*> k1_trajectory;
  vector<double*> k2_trajectory;
};

rosbag::View::iterator InitializeVariables(
    string bag_name,
    int degree,
    rosbag::View::iterator bag_it,
    rosbag::View::iterator end,
    DeltaCalVariables* input) {

  // --- INITIALIZATION ---
  // Degree to string
  std::stringstream out;
  out << degree;

  // Initializing output file names

  input->bag_name = bag_name +
  "_" + out.str();
  input->k1_output_name = "cloud_" + out.str() + "_1";
  input->k2_output_name = "cloud_" + out.str() + "_2";

  input->object_file = bag_name + "_objects";
  input->covariance_file = bag_name + "_covariance";
  input->k1_covariance = "covariance_" + out.str() + "_1";
  input->k2_covariance = "covariance_" + out.str() + "_2";

  input->k1_base_name =  input->k1_output_name + ".base";
  input->k2_base_name = input->k2_output_name + ".base";

  // Opening pose and trajectory files
  input->pose_name = bag_name + ".pose";
  input->velocity_name = bag_name + ".velocity";
  input->trajectory_name = bag_name + ".traj";
  ofstream traj_file (input->trajectory_name.c_str());
  ofstream pose_file (input->pose_name.c_str());
  ofstream velocity_file (input->velocity_name.c_str());
  input->pose_file = &pose_file;
  input->traj_file = &traj_file;
  input->velocity_file = &velocity_file;
  vector<double> k1_velocity_list(10);
  vector<double> k2_velocity_list(10);
  input->k1_velocity_list = k1_velocity_list;
  input->k2_velocity_list = k2_velocity_list;
  // Read in the first cloud from each kinect, set it as the keyframe
  // and cloud k - 1, set the combined transform to the zero transform
  bag_it = TimeAlignedClouds(bag_it, end,
      &input->k1_buffer, &input->k2_buffer, &(input->k1_timestamps), &input->k2_timestamps,
     &input->k1_keyframe, &input->k2_keyframe, &input->k1_timestamp, &input->k2_timestamp);
  // Key normals
  input->k1_key_normal = GetNormals(input->k1_keyframe);
  input->k2_key_normal = GetNormals(input->k2_keyframe);
  // Previous clouds
  input->k1_prev = input->k1_keyframe;
  input->k2_prev = input->k2_keyframe;
  input->k1_prev_timestamp = input->k1_timestamp;
  // Previous normals
  input->k1_prev_normal = input->k1_key_normal;
  input->k2_prev_normal = input->k2_key_normal;
  vector<double> pose0(6, 0.0);
  std::copy(pose0.begin(), pose0.end(), input->k1_combined_transform);
  std::copy(pose0.begin(), pose0.end(), input->k2_combined_transform);
  // Lists used to threshold over velocity
  return bag_it;
  //--- End Initialization ---
}

rosbag::View::iterator InitializeVariablesSingle(
    string bag_name,
    int degree,
    rosbag::View::iterator bag_it,
    rosbag::View::iterator end,
    DeltaCalVariables* input) {

  // --- INITIALIZATION ---
  // Degree to string
  std::stringstream out;
  out << degree;

  // Initializing output file names

  input->bag_name = bag_name +
  "_" + out.str();
  input->k1_output_name = "cloud_" + out.str() + "_1";

  input->object_file = bag_name + "_objects";
  input->covariance_file = bag_name + "_covariance";
  input->k1_covariance = "covariance_" + out.str() + "_1";

  input->k1_base_name =  input->k1_output_name + ".base";

  // Opening pose and trajectory files
  input->pose_name = bag_name + ".pose";
  input->velocity_name = bag_name + ".velocity";
  input->trajectory_name = bag_name + ".traj";
  ofstream traj_file (input->trajectory_name.c_str());
  ofstream pose_file (input->pose_name.c_str());
  ofstream velocity_file (input->velocity_name.c_str());
  input->pose_file = &pose_file;
  input->traj_file = &traj_file;
  input->velocity_file = &velocity_file;
  vector<double> k1_velocity_list(10);
  input->k1_velocity_list = k1_velocity_list;
  // Read in the first cloud from each kinect, set it as the keyframe
  // and cloud k - 1, set the combined transform to the zero transform
  bag_it = OneSensorClouds(bag_it, end, &input->k1_buffer,
                           &(input->k1_timestamps),
                           &input->k1_keyframe, &input->k1_timestamp);
  // Key normals
  //   cout << "Calculating Normals" << endl;
  input->k1_key_normal = GetNormals(input->k1_keyframe);
  // Previous clouds
  input->k1_prev = input->k1_keyframe;
  // Previous normals
  input->k1_prev_normal = input->k1_key_normal;
  vector<double> pose0(6, 0.0);
  std::copy(pose0.begin(), pose0.end(), input->k1_combined_transform);
  // Lists used to threshold over velocity
  return bag_it;
  //--- End Initialization ---
  }

rosbag::View::iterator InitializeVariablesBrass(
    string bag_name,
    int degree,
    rosbag::View::iterator bag_it,
    rosbag::View::iterator end,
    DeltaCalVariables* input) {

  // --- INITIALIZATION ---
  // Degree to string
  std::stringstream out;
  out << degree;

  // Initializing output file names

  input->bag_name = bag_name +
  "_" + out.str();
  input->k1_output_name = "cloud_" + out.str() + "_1";

  input->object_file = bag_name + "_objects";
  input->covariance_file = bag_name + "_covariance";
  input->k1_covariance = "covariance_" + out.str() + "_1";

  input->k1_base_name =  input->k1_output_name + ".base";

  // Opening pose and trajectory files
  input->pose_name = bag_name + "_brass.pose";
  input->velocity_name = bag_name + ".velocity";
  input->trajectory_name = bag_name + ".traj";
  ofstream traj_file (input->trajectory_name.c_str());
  ofstream pose_file (input->pose_name.c_str());
  ofstream velocity_file (input->velocity_name.c_str());
  input->pose_file = &pose_file;
  input->traj_file = &traj_file;
  input->velocity_file = &velocity_file;
  vector<double> k1_velocity_list(10);
  input->k1_velocity_list = k1_velocity_list;
  // Read in the first cloud from each kinect, set it as the keyframe
  // and cloud k - 1, set the combined transform to the zero transform
  bag_it = OneSensorCloudsBrass(bag_it, end, &input->k1_buffer,
                           &(input->k1_timestamps),
                           &input->k1_keyframe, &input->k1_timestamp);
  // Key normals
  //   cout << "Calculating Normals" << endl;
  input->k1_key_normal = GetNormals(input->k1_keyframe);
  // Previous clouds
  input->k1_prev = input->k1_keyframe;
  // Previous normals
  input->k1_prev_normal = input->k1_key_normal;
  vector<double> pose0(6, 0.0);
  std::copy(pose0.begin(), pose0.end(), input->k1_combined_transform);
  // Lists used to threshold over velocity
  return bag_it;
  //--- End Initialization ---
  }

rosbag::View::iterator InitializeVariablesSlam(
    string bag_name,
    int degree,
    rosbag::View::iterator bag_it,
    rosbag::View::iterator end,
    rosbag::View::iterator& bag_it2,
    rosbag::View::iterator end2,
    DeltaCalVariables* input) {

  // --- INITIALIZATION ---
  // Degree to string
  std::stringstream out;
  out << degree;

  // Initializing output file names

  input->bag_name = bag_name +
  "_" + out.str();
  input->k1_output_name = "cloud_" + out.str() + "_1";
  input->k2_output_name = "cloud_" + out.str() + "_2";

  input->object_file = bag_name + "_objects";
  input->covariance_file = bag_name + "_covariance";
  input->k1_covariance = "covariance_" + out.str() + "_1";
  input->k2_covariance = "covariance_" + out.str() + "_2";

  input->k1_base_name =  input->k1_output_name + ".base";
  input->k2_base_name = input->k2_output_name + ".base";

  // Opening pose and trajectory files
  input->pose_name = bag_name + ".pose";
  input->velocity_name = bag_name + ".velocity";
  input->trajectory_name = bag_name + ".traj";
  ofstream traj_file (input->trajectory_name.c_str());
  ofstream pose_file (input->pose_name.c_str());
  ofstream velocity_file (input->velocity_name.c_str());
  input->pose_file = &pose_file;
  input->traj_file = &traj_file;
  input->velocity_file = &velocity_file;
  vector<double> k1_velocity_list(10);
  vector<double> k2_velocity_list(10);
  input->k1_velocity_list = k1_velocity_list;
  input->k2_velocity_list = k2_velocity_list;
  // Read in the first cloud from each kinect, set it as the keyframe
  // and cloud k - 1, set the combined transform to the zero transform
  bag_it = TimeAlignedCloudsSlamBag(bag_it, end, &bag_it2, end2,
                                    &input->k1_buffer, &input->k2_buffer, &(input->k1_timestamps), &input->k2_timestamps,
                                    &input->k1_keyframe, &input->k2_keyframe, &input->k1_timestamp, &input->k2_timestamp);
  // Key normals
  //   cout << "Calculating Normals" << endl;
  input->k1_key_normal = GetNormals(input->k1_keyframe);
  input->k2_key_normal = GetNormals(input->k2_keyframe);
  // Previous clouds
  input->k1_prev = input->k1_keyframe;
  input->k2_prev = input->k2_keyframe;
  // Previous normals
  input->k1_prev_normal = input->k1_key_normal;
  input->k2_prev_normal = input->k2_key_normal;
  vector<double> pose0(6, 0.0);
  std::copy(pose0.begin(), pose0.end(), input->k1_combined_transform);
  std::copy(pose0.begin(), pose0.end(), input->k2_combined_transform);
  // Lists used to threshold over velocity
  return bag_it;
  //--- End Initialization ---
  }

  // Read Normals from a pcl normal file
std::vector<pcl::PointCloud<pcl::Normal> > ReadNormals(std::string filename) {
  ifstream fin(filename.c_str(), ios::in | ios::binary);
  vector<pcl::PointCloud<pcl::Normal> > normals;

  int normals_size;
  // Calculate size and save to variables
  fin.seekg(0, ifstream::end);
  int size = fin.tellg();
  fin.seekg(0, ifstream::beg);
  fin.read((char*)&normals_size, sizeof(int));
  size = (size - sizeof(int)) / normals_size;

  cout << normals_size << endl;
  cout << size << endl;

  pcl::PointCloud<pcl::Normal> temp[size];
  fin.read((char*)&temp, 1 * normals_size);
  for(int i = 0; i < size; i++) {
    normals.push_back(temp[i]);
  }
  fin.close();
  fprintf(stdout, "Num Normals: %d", (int)normals.size());
  return normals;
}

// Performs DeltaCalculation given a bagfile containing data recorded from
// openni
void DeltaCalculationSlam(string bag_name,
                          vector<ros::Publisher> publishers,
                          const int degree,
                          const int kMaxClouds) {

  // --- INITIALIZATION ---
  DeltaCalVariables* variables = new DeltaCalVariables();

  // Opening bagfile
  string bag_file = bag_name + ".bag";
  string pcd_folder = bag_name + "pcd_file/";
  mkdir(pcd_folder.c_str(), 0777);
  string keyframe_bag_name = bag_name + "_keyframes.bag";
  string normal_name = bag_name + "_normals.dat";
  vector<pcl::PointCloud<pcl::Normal> > all_normals;
  rosbag::Bag bag, keyframe_bag;
  bag.open(bag_file, rosbag::bagmode::Read);
  keyframe_bag.open(keyframe_bag_name, rosbag::bagmode::Write);
  cout << "Bag open" << endl;
  std::vector<std::string> topics;

  // Topics to read data from
  topics.push_back(std::string("/kinect_1/depth_registered/image_raw"));
  std::vector<std::string> topics2;
  topics2.push_back(std::string("/kinect_2/depth_registered/image_raw"));
  rosbag::View view(bag, rosbag::TopicQuery(topics));
  rosbag::View view2(bag, rosbag::TopicQuery(topics2));

  // Values used for reading/writing normals to a file
  string str_key_normal_k1 = pcd_folder + "key_normal_k1";
  string str_key_normal_k2 = pcd_folder + "key_normal_k2";
  string str_prev_normal_k1 = pcd_folder + "prev_normal_k1";
  string str_prev_normal_k2 =pcd_folder + "prev_normal_k2";
  string str_normal_k1 = pcd_folder + "normal_k1";
  string str_normal_k2 =pcd_folder + "normal_k2";

  // Iterator over bag bag_name
  rosbag::View::iterator bag_it = view.begin();
  rosbag::View::iterator end = view.end();
  rosbag::View::iterator bag_it2 = view2.begin();
  rosbag::View::iterator end2 = view2.end();
  bag_it = InitializeVariablesSlam(bag_name, degree, bag_it, end, bag_it2, end2, variables);
  cout << "Variables initialized" << endl;
  ofstream pose_file (variables->pose_name.c_str());
  int avg_len = 5;
  bool dist_okay = false;
  int count = 0;

  // Write keyframes to object files
  WriteToObj(variables->object_file, variables->k1_output_name, count, variables->k1_keyframe);
  WriteToObj(variables->object_file, variables->k2_output_name, count, variables->k2_keyframe);
  WriteToBag("kinect_1", &keyframe_bag, variables->k1_keyframe);
  WriteToBag("kinect_2", &keyframe_bag, variables->k2_keyframe);

  // Save normals and get size of normals
  all_normals.push_back(variables->k1_key_normal);
  all_normals.push_back(variables->k2_key_normal);
  // size_t size = all_normals[0].size();
  int normal_size;

  vector<pcl::PointCloud<pcl::Normal> > saved_normals;
  if(normal_file) {
    // No behavior for using normals
  }
  ofstream fout(normal_name.c_str(), ios::out | ios::binary);
  normal_size = sizeof(all_normals[0]);
  cout << "Normal size: " << normal_size << endl;
  fout.write((char*)&normal_size, sizeof(normal_size));

  // While there are still clouds in both datasets
  while(((variables->k1_buffer.size() != 0 && variables->k2_buffer.size() != 0 )
    || (variables->bag_it != variables->end))) {
    count += 1;
  cout << "Frame: " << count << endl;
  // Read in a new cloud from each dataset
  pcl::PointCloud<pcl::PointXYZ> k1_cloud;
  pcl::PointCloud<pcl::PointXYZ> k2_cloud;
  bag_it = TimeAlignedCloudsSlamBag(bag_it, end, &bag_it2, end2, &variables->k1_buffer,
                             &variables->k2_buffer, &variables->k1_timestamps, &variables->k2_timestamps,
                             &k1_cloud, &k2_cloud, &variables->k1_timestamp, &variables->k2_timestamp);

  pcl::PointCloud<pcl::Normal> k1_normal;
  pcl::PointCloud<pcl::Normal> k2_normal;

  // Get normals for the two clouds
  if(normal_file) {
    std::stringstream out;
    out << count;
    pcl::PCLPointCloud2 cloud_blob;
    pcl::io::loadPCDFile (str_normal_k1 + "_" + out.str() + ".pcd", cloud_blob);
    pcl::fromPCLPointCloud2 (cloud_blob, k1_normal);
    pcl::io::loadPCDFile (str_normal_k2 + "_" + out.str() + ".pcd", cloud_blob);
    pcl::fromPCLPointCloud2 (cloud_blob, k2_normal);
  } else {
    k1_normal = GetNormals(k1_cloud);
    k2_normal = GetNormals(k2_cloud);
    // Save normals to binary file
    all_normals.push_back(k1_normal);
    all_normals.push_back(k2_normal);
    fout.write((char*)&all_normals[0], all_normals.size() * sizeof(all_normals[0]));
    fout.flush();
    all_normals.clear();
    std::stringstream out;
    out << count;
    pcl::io::savePCDFileASCII (str_key_normal_k1 + "_" + out.str() + ".pcd", variables->k1_key_normal);
    pcl::io::savePCDFileASCII (str_key_normal_k2 + "_" +out.str() + ".pcd", variables->k2_key_normal);
    pcl::io::savePCDFileASCII (str_prev_normal_k1 + "_" +out.str() + ".pcd", variables->k1_prev_normal);
    pcl::io::savePCDFileASCII (str_prev_normal_k2 + "_" +out.str() + ".pcd", variables->k2_prev_normal);
    pcl::io::savePCDFileASCII (str_normal_k1 + "_" +out.str() + ".pcd", k1_normal);
    pcl::io::savePCDFileASCII (str_normal_k2 + "_" +out.str() + ".pcd", k2_normal);

  }
  // If the residual distance between either of these clouds (unmodified) and
  // the clouds k - 1 is large enough continue (otherwise read in new clouds)
//   double k1_calculated_delta[6];
  double k2_calculated_delta[6];

  // Check the residual distance against threshold
  // Check the residual distance against threshold
  //     const double k1_residual = ResidualDist(variables->k1_prev,
  //                                           k1_cloud,
  //                                                variables->k1_prev_normal,
  //                                           k1_normal,
  //                                           k1_calculated_delta);
  const double k1_residual = 0;
  const double k2_residual = ResidualDist(variables->k2_prev,
                                          k2_cloud,
                                          variables->k2_prev_normal,
                                          k2_normal,
                                          k2_calculated_delta);

  // Accumulate and write velocities
  const double k1_velocity = k1_residual / (variables->k1_timestamp - variables->k1_prev_timestamp);
  const double k2_velocity = k2_residual / (variables->k2_timestamp - variables->k2_prev_timestamp);
  cout << k1_residual << endl;
  cout << k2_residual << endl;
  cout << variables->k1_timestamp << endl;
  cout << variables->k2_timestamp << endl;
  cout << "Velocity List " << endl;
  cout << k1_velocity << endl;
  cout << k2_velocity << endl << endl;
  variables->k1_velocity_list[count % avg_len] = k1_velocity;
  variables->k2_velocity_list[count % avg_len] = k2_velocity;
  double k1_acc_velocity =
  std::accumulate(variables->k1_velocity_list.begin(), variables->k1_velocity_list.end(), 0.0);
  k1_acc_velocity = k1_acc_velocity / avg_len;
  double k2_acc_velocity =
  std::accumulate(variables->k2_velocity_list.begin(), variables->k2_velocity_list.end(), 0.0);
  k2_acc_velocity = k2_acc_velocity / avg_len;

  // If our residual is large enough, or we are far enough from keyframe
  if ((k1_residual > 0.003 && k2_residual > 0.003) || dist_okay) {
    // Run ICP
    fprintf(stdout, "Kinect 1\n");
    CalculateDelta(count,
                   publishers,
                   variables->covariance_file,
                   variables->k1_covariance,
                   variables->k1_prev,
                   k1_cloud,
                   variables->k1_keyframe,
                   variables->k1_prev_normal,
                   k1_normal,
                   variables->k1_key_normal,
                   variables->k1_combined_transform,
                   NULL);

    fprintf(stdout, "Kinect 2\n");
    CalculateDelta(count,
                   publishers,
                   variables->covariance_file,
                   variables->k2_covariance,
                   variables->k2_prev,
                   k2_cloud,
                   variables->k2_keyframe,
                   variables->k2_prev_normal,
                   k2_normal,
                   variables->k2_key_normal,
                   variables->k2_combined_transform,
                   NULL);
  }
  // Check the magnitude of translation and angle of rotation, if larger
  // than some threshold, this is our next keyframe
  // If there has been sufficient change update keyframe and save deltas
  bool k1_change = CheckChangeVel(variables->k1_combined_transform, degree, variables->k1_velocity_list);
  bool k2_change = CheckChangeVel(variables->k2_combined_transform, degree, variables->k2_velocity_list);
  if(k1_change && k2_change) {
    dist_okay = false;
    vector<double> pose0(6, 0.0);
    variables->k1_keyframe = k1_cloud;
    variables->k2_keyframe = k2_cloud;
    variables->k2_key_normal = k2_normal;
    variables->k1_key_normal = k1_normal;
    variables->keys.push_back(count);

    WritePoseFile(variables->k1_combined_transform, variables->k1_timestamp, count, pose_file);
    WritePoseFile(variables->k2_combined_transform, variables->k2_timestamp, count, pose_file);
    pose_file << endl;

    //Transform the clouds and then write them to object files
    pcl::PointCloud<pcl::PointXYZ> temp_cloud1 = k1_cloud;
    pcl::PointCloud<pcl::PointXYZ> temp_cloud2 = k2_cloud;
    WriteToBag("kinect_1", &keyframe_bag, k1_cloud);
    WriteToBag("kinect_2", &keyframe_bag, k2_cloud);
    TransformPointCloud(&temp_cloud1, variables->k1_combined_transform);
    TransformPointCloud(&temp_cloud2, variables->k2_combined_transform);
    // Zero Combined transforms
    std::copy(pose0.begin(), pose0.end(), variables->k1_combined_transform);
    std::copy(pose0.begin(), pose0.end()  , variables->k2_combined_transform);
    WriteToObj(variables->object_file, variables->k1_output_name, count, temp_cloud1);
    WriteToObj(variables->object_file, variables->k2_output_name, count, temp_cloud2);
    WriteToObj(variables->object_file, variables->k1_base_name, count, k1_cloud);
    WriteToObj(variables->object_file, variables->k2_base_name, count, k2_cloud);

  }
  // Checking to see if we have a significant change from the predecessor
  else{
    if(CheckChange(variables->k1_combined_transform, degree)
      && CheckChange(variables->k2_combined_transform, degree)) {
      dist_okay = true;
      }
      else{
        dist_okay = false;
      }
  }
  // Update predecessor clouds
  variables->k1_prev = k1_cloud;
  variables->k2_prev = k2_cloud;
  variables->k1_prev_normal = k1_normal;
  variables->k2_prev_normal = k2_normal;
  variables->k1_prev_timestamp = variables->k1_timestamp;
  variables->k2_prev_timestamp = variables->k2_timestamp;
  }
  keyframe_bag.close();
}

void DeltaCalculation(string bag_name,
                      vector<ros::Publisher> publishers,
                      const int degree,
                      const int kMaxClouds) {

  // --- INITIALIZATION ---
  DeltaCalVariables* variables = new DeltaCalVariables();
  //Opening bagfile
  string bag_file = bag_name + ".bag";
  string pcd_folder = bag_name + "pcd_file/";
  mkdir(pcd_folder.c_str(), 0777);
  string keyframe_bag_name = bag_name + "_keyframes.bag";
  string normal_name = bag_name + "_normals.dat";
  vector<pcl::PointCloud<pcl::Normal> > all_normals;
  rosbag::Bag bag, keyframe_bag;
  bag.open(bag_file, rosbag::bagmode::Read);
  keyframe_bag.open(keyframe_bag_name, rosbag::bagmode::Write);
  std::vector<std::string> topics;
  topics.push_back(std::string("/Cobot/Kinect/Depth"));
  rosbag::View view(bag, rosbag::TopicQuery(topics));

  string str_key_normal_k1 = pcd_folder + "key_normal_k1";
  string str_key_normal_k2 = pcd_folder + "key_normal_k2";
  string str_prev_normal_k1 = pcd_folder + "prev_normal_k1";
  string str_prev_normal_k2 =pcd_folder + "prev_normal_k2";
  string str_normal_k1 = pcd_folder + "normal_k1";
  string str_normal_k2 =pcd_folder + "normal_k2";
  // Iterator over bag bag_name
  rosbag::View::iterator bag_it = view.begin();
  rosbag::View::iterator end = view.end();
  bag_it = InitializeVariables(bag_name, degree, bag_it, end, variables);
  variables->k2_prev_timestamp = variables->k2_timestamp;
  ofstream pose_file (variables->pose_name.c_str());
  int avg_len = 5;
  bool dist_okay = false;
  int count = 0;

  // Write keyframes to object files
  WriteToObj(variables->object_file, variables->k1_output_name, count, variables->k1_keyframe);
  WriteToObj(variables->object_file, variables->k2_output_name, count, variables->k2_keyframe);
  WriteToBag("kinect_1", &keyframe_bag, variables->k1_keyframe);
  WriteToBag("kinect_2", &keyframe_bag, variables->k2_keyframe);

  // Save normals and get size of normals
  all_normals.push_back(variables->k1_key_normal);
  all_normals.push_back(variables->k2_key_normal);
  int normal_size;

  vector<pcl::PointCloud<pcl::Normal> > saved_normals;
  if(normal_file) {

  }
    ofstream fout(normal_name.c_str(), ios::out | ios::binary);
    normal_size = sizeof(all_normals[0]);
    cout << "Normal size: " << normal_size << endl;
    fout.write((char*)&normal_size, sizeof(normal_size));

  // While there are still clouds in both datasets
  while(((variables->k1_buffer.size() != 0 && variables->k2_buffer.size() != 0 )
        || (variables->bag_it != variables->end))) {
    count += 1;
    cout << "Frame: " << count << endl;
    // Read in a new cloud from each dataset
    pcl::PointCloud<pcl::PointXYZ> k1_cloud;
    pcl::PointCloud<pcl::PointXYZ> k2_cloud;
    bag_it = TimeAlignedClouds(bag_it, end, &variables->k1_buffer,
        &variables->k2_buffer, &variables->k1_timestamps, &variables->k2_timestamps,
        &k1_cloud, &k2_cloud, &variables->k1_timestamp, &variables->k2_timestamp);

    pcl::PointCloud<pcl::Normal> k1_normal;
    pcl::PointCloud<pcl::Normal> k2_normal;
    // Get normals for the two clouds
    if(normal_file) {
      std::stringstream out;
      out << count;
      pcl::PCLPointCloud2 cloud_blob;
      pcl::io::loadPCDFile (str_normal_k1 + "_" + out.str() + ".pcd", cloud_blob);
      pcl::fromPCLPointCloud2 (cloud_blob, k1_normal);
      pcl::io::loadPCDFile (str_normal_k2 + "_" + out.str() + ".pcd", cloud_blob);
      pcl::fromPCLPointCloud2 (cloud_blob, k2_normal);
    } else {
      k1_normal = GetNormals(k1_cloud);
      k2_normal = GetNormals(k2_cloud);
      // Save normals to binary file
      all_normals.push_back(k1_normal);
      all_normals.push_back(k2_normal);
      fout.write((char*)&all_normals[0], all_normals.size() * sizeof(all_normals[0]));
      fout.flush();
      all_normals.clear();
      std::stringstream out;
      out << count;
      pcl::io::savePCDFileASCII (str_key_normal_k1 + "_" + out.str() + ".pcd", variables->k1_key_normal);
      pcl::io::savePCDFileASCII (str_key_normal_k2 + "_" +out.str() + ".pcd", variables->k2_key_normal);
      pcl::io::savePCDFileASCII (str_prev_normal_k1 + "_" +out.str() + ".pcd", variables->k1_prev_normal);
      pcl::io::savePCDFileASCII (str_prev_normal_k2 + "_" +out.str() + ".pcd", variables->k2_prev_normal);
      pcl::io::savePCDFileASCII (str_normal_k1 + "_" +out.str() + ".pcd", k1_normal);
      pcl::io::savePCDFileASCII (str_normal_k2 + "_" +out.str() + ".pcd", k2_normal);

    }
    // If the residual distance between either of these clouds (unmodified) and
    // the clouds k - 1 is large enough continue (otherwise read in new clouds)
    double k1_calculated_delta[6];
    double k2_calculated_delta[6];

    // Check the residual distance against threshold
    const double k1_residual = ResidualDist(variables->k1_prev,
                                            k1_cloud,
                                            variables->k1_prev_normal,
                                            k1_normal,
                                            k1_calculated_delta);
    const double k2_residual = ResidualDist(variables->k2_prev,
                                            k2_cloud,
                                            variables->k2_prev_normal,
                                            k2_normal,
                                            k2_calculated_delta);

    // Accumulate and write velocities

//     cout << "k1_residual: " << k1_residual << " Time 1: " << variables->k1_timestamp << "Time 2: " << variables->k1_prev_timestamp << endl;
//     cout << "k2_residual: " << k2_residual << " Time 1: " << variables->k2_timestamp << "Time 2: " << variables->k2_prev_timestamp << endl;


    // If our residual is large enough, or we are far enough from keyframe
    if ((k1_residual > 0.003 || k2_residual > 0.003) || dist_okay) {
      // Run ICP
      double rot1, rot2; // Amount of instananeous rotation
      fprintf(stdout, "Kinect 1\n");
      CalculateDelta(count,
                     publishers,
                     variables->covariance_file,
                     variables->k1_covariance,
                     variables->k1_prev,
                     k1_cloud,
	             variables->k1_keyframe,
                     variables->k1_prev_normal,
                     k1_normal,
                     variables->k1_key_normal,
                     variables->k1_combined_transform,
                     &rot1);

      fprintf(stdout, "Kinect 2\n");
      CalculateDelta(count,
                     publishers,
                     variables->covariance_file,
                     variables->k2_covariance,
                     variables->k2_prev,
                     k2_cloud,
	             variables->k2_keyframe,
                     variables->k2_prev_normal,
                     k2_normal,
	             variables->k2_key_normal,
                     variables->k2_combined_transform,
                     &rot2);

      const double k1_velocity = rot1 / (variables->k1_timestamp - variables->k1_prev_timestamp);
      const double k2_velocity = rot2 / (variables->k2_timestamp - variables->k2_prev_timestamp);
      variables->k1_velocity_list[count % avg_len] = k1_velocity;
      variables->k2_velocity_list[count % avg_len] = k2_velocity;
      double k1_acc_velocity =
      std::accumulate(variables->k1_velocity_list.begin(), variables->k1_velocity_list.end(), 0.0);
      k1_acc_velocity = k1_acc_velocity / avg_len;
      double k2_acc_velocity =
      std::accumulate(variables->k2_velocity_list.begin(), variables->k2_velocity_list.end(), 0.0);
      k2_acc_velocity = k2_acc_velocity / avg_len;
    }
    // Check the magnitude of translation and angle of rotation, if larger
    // than some threshold, this is our next keyframe
    // If there has been sufficient change update keyframe and save deltas
    bool k1_change = CheckChangeVel(variables->k1_combined_transform, degree, variables->k1_velocity_list);
    bool k2_change = CheckChangeVel(variables->k2_combined_transform, degree, variables->k2_velocity_list);
    if(k1_change && k2_change) {
      dist_okay = false;
      vector<double> pose0(6, 0.0);
      variables->k1_keyframe = k1_cloud;
      variables->k2_keyframe = k2_cloud;
      variables->k2_key_normal = k2_normal;
      variables->k1_key_normal = k1_normal;
      variables->keys.push_back(count);
      SanitizeTransform(k1_cloud, k1_normal, variables->k1_combined_transform);
      SanitizeTransform(k2_cloud, k2_normal, variables->k2_combined_transform);
      WritePoseFile(variables->k1_combined_transform, variables->k1_timestamp, count, pose_file);
      WritePoseFile(variables->k2_combined_transform, variables->k2_timestamp, count, pose_file);
      pose_file << endl;

      //Transform the clouds and then write them to object files
      pcl::PointCloud<pcl::PointXYZ> temp_cloud1 = k1_cloud;
      pcl::PointCloud<pcl::PointXYZ> temp_cloud2 = k2_cloud;
      WriteToBag("kinect_1", &keyframe_bag, k1_cloud);
      WriteToBag("kinect_2", &keyframe_bag, k2_cloud);
      TransformPointCloud(&temp_cloud1, variables->k1_combined_transform);
      TransformPointCloud(&temp_cloud2, variables->k2_combined_transform);
      // Zero Combined transforms
      std::copy(pose0.begin(), pose0.end(), variables->k1_combined_transform);
      std::copy(pose0.begin(), pose0.end()  , variables->k2_combined_transform);
      WriteToObj(variables->object_file, variables->k1_output_name, count, temp_cloud1);
      WriteToObj(variables->object_file, variables->k2_output_name, count, temp_cloud2);
      WriteToObj(variables->object_file, variables->k1_base_name, count, k1_cloud);
      WriteToObj(variables->object_file, variables->k2_base_name, count, k2_cloud);

    }
    // Checking to see if we have a significant change from the predecessor
    else{
      if(CheckChange(variables->k1_combined_transform, degree)
          && CheckChange(variables->k2_combined_transform, degree)) {
        dist_okay = true;
      }
      else{
        dist_okay = false;
      }
    }
    // Update predecessor clouds
      variables->k1_prev = k1_cloud;
      variables->k2_prev = k2_cloud;
      variables->k1_prev_normal = k1_normal;
      variables->k2_prev_normal = k2_normal;
      variables->k1_prev_timestamp = variables->k1_timestamp;
      variables->k2_prev_timestamp = variables->k2_timestamp;
    //VisualizePoses(trajectory1, trajectory2, keys);
  }
  fout.close();
  keyframe_bag.close();
}

void DeltaCalculationSingle(string bag_name,
                            vector<ros::Publisher> publishers,
                            const int degree,
                            const int kMaxClouds) {
  // --- INITIALIZATION ---
  DeltaCalVariables* variables = new DeltaCalVariables();
  //Opening bagfile
  string bag_file = bag_name + ".bag";
  string pcd_folder = bag_name + "pcd_file/";
  mkdir(pcd_folder.c_str(), 0777);
  string keyframe_bag_name = bag_name + "_keyframes.bag";
  string normal_name = bag_name + "_normals.dat";
  vector<pcl::PointCloud<pcl::Normal> > all_normals;
  rosbag::Bag bag, keyframe_bag;
  bag.open(bag_file, rosbag::bagmode::Read);
  keyframe_bag.open(keyframe_bag_name, rosbag::bagmode::Write);
  std::vector<std::string> topics;
  topics.push_back(std::string("/Cobot/Kinect/Depth"));
  rosbag::View view(bag, rosbag::TopicQuery(topics));

  string str_key_normal_k1 = pcd_folder + "key_normal_k1";
  string str_key_normal_k2 = pcd_folder + "key_normal_k2";
  string str_prev_normal_k1 = pcd_folder + "prev_normal_k1";
  string str_prev_normal_k2 =pcd_folder + "prev_normal_k2";
  string str_normal_k1 = pcd_folder + "normal_k1";
  string str_normal_k2 =pcd_folder + "normal_k2";
  // Iterator over bag bag_name
  rosbag::View::iterator bag_it = view.begin();
  rosbag::View::iterator end = view.end();
  bag_it = InitializeVariablesSingle(bag_name, degree, bag_it, end, variables);
  ofstream pose_file (variables->pose_name.c_str());
  int avg_len = 5;
  bool dist_okay = false;
  int count = 0;
  // Save normals and get size of normals
  all_normals.push_back(variables->k1_key_normal);
  // size_t size = all_normals[0].size();

  if((variables->k1_buffer.size() == 0)){
    cout << "first " << endl;
  }
  if(bag_it == variables->end){
    cout << "second " << endl;
    cout << variables->k1_keyframe.size() << endl;
  }
  // While there are still clouds in both datasets
  while(((variables->k1_buffer.size() != 0)
    || (bag_it != variables->end))) {
    count += 1;
  cout << "Frame: " << count << endl;
  // Read in a new cloud from each dataset
  pcl::PointCloud<pcl::PointXYZ> k1_cloud;
  bag_it = OneSensorClouds(bag_it, end, &variables->k1_buffer,
                              &variables->k1_timestamps,
                             &k1_cloud, &variables->k1_timestamp);

  pcl::PointCloud<pcl::Normal> k1_normal;
  pcl::PointCloud<pcl::Normal> k2_normal;
  // Get normals for the two clouds
    k1_normal = GetNormals(k1_cloud);

  // If the residual distance between either of these clouds (unmodified) and
  // the clouds k - 1 is large enough continue (otherwise read in new clouds)
  double k1_calculated_delta[6];

  // Check the residual distance against threshold
  const double k1_residual = ResidualDist(variables->k1_prev,
                                          k1_cloud,
                                          variables->k1_prev_normal,
                                          k1_normal,
                                          k1_calculated_delta);
  // Accumulate and write velocities
  const double k1_velocity = k1_residual / (variables->k1_timestamp - variables->k1_prev_timestamp);
  variables->k1_velocity_list[count % avg_len] = k1_velocity;
  double k1_acc_velocity =
  std::accumulate(variables->k1_velocity_list.begin(), variables->k1_velocity_list.end(), 0.0);
  k1_acc_velocity = k1_acc_velocity / avg_len;
  // If our residual is large enough, or we are far enough from keyframe

  if ((k1_residual > 0.003 ) || dist_okay) {
    // Run ICP
    fprintf(stdout, "Kinect 1\n");
    CalculateDelta(count,
                   publishers,
                   variables->covariance_file,
                   variables->k1_covariance,
                   variables->k1_prev,
                   k1_cloud,
                   variables->k1_keyframe,
                   variables->k1_prev_normal,
                   k1_normal,
                   variables->k1_key_normal,
                   variables->k1_combined_transform,
                   NULL);
  }
  // Check the magnitude of translation and angle of rotation, if larger
  // than some threshold, this is our next keyframe
  // If there has been sufficient change update keyframe and save deltas
  bool k1_change = CheckChangeVel(variables->k1_combined_transform, degree, variables->k1_velocity_list);
  if(k1_change) {
    dist_okay = false;
    vector<double> pose0(6, 0.0);
    variables->k1_keyframe = k1_cloud;
    variables->k1_key_normal = k1_normal;
    variables->keys.push_back(count);

    WritePoseFile(variables->k1_combined_transform, variables->k1_timestamp, count, pose_file);
    pose_file << endl;

    //Transform the clouds and then write them to object files
    pcl::PointCloud<pcl::PointXYZ> temp_cloud1 = k1_cloud;
    WriteToBag("kinect_1", &keyframe_bag, k1_cloud);
    TransformPointCloud(&temp_cloud1, variables->k1_combined_transform);
    // Zero Combined transforms
    std::copy(pose0.begin(), pose0.end(), variables->k1_combined_transform);
    WriteToObj(variables->object_file, variables->k1_output_name, count, temp_cloud1);
    WriteToObj(variables->object_file, variables->k1_base_name, count, k1_cloud);

  }
  // Checking to see if we have a significant change from the predecessor
  else{
    if(CheckChange(variables->k1_combined_transform, degree)
      && CheckChange(variables->k2_combined_transform, degree)) {
      dist_okay = true;
      }
      else{
        dist_okay = false;
      }
  }
  // Update predecessor clouds
  variables->k1_prev = k1_cloud;
  variables->k1_prev_normal = k1_normal;
  variables->k1_prev_timestamp = variables->k1_timestamp;
  }
    keyframe_bag.close();
}

rosbag::View::iterator ClosestOdom(rosbag::View::iterator it,
                                   rosbag::View::iterator end,
                                   const double& timestamp,
                                   vector<double>* keyframe_odom) {


    double best_deltaT = 1000;
    nav_msgs::OdometryPtr best_odom;
    bool done = false;
    double time;
    double best_time;
    while(!done && it != end) {
      const rosbag::MessageInstance &m = *it;

      nav_msgs::OdometryPtr odomMsg = m.instantiate<nav_msgs::Odometry>();
      time = odomMsg->header.stamp.toSec();
      double deltaT = time - timestamp;
      deltaT = abs(deltaT);
      if(deltaT < best_deltaT) {
        best_time = time;
        best_deltaT = deltaT;
        best_odom = odomMsg;
        advance(it, 1);
      } else {
        done = true;
      }
    }
    keyframe_odom->clear();
    keyframe_odom->push_back(best_time);
    keyframe_odom->push_back(best_odom->pose.pose.orientation.x);
    keyframe_odom->push_back(best_odom->pose.pose.orientation.y);
    keyframe_odom->push_back(best_odom->pose.pose.orientation.z);
    keyframe_odom->push_back(best_odom->pose.pose.orientation.w);
    keyframe_odom->push_back(best_odom->pose.pose.position.x);
    keyframe_odom->push_back(best_odom->pose.pose.position.y);
    keyframe_odom->push_back(best_odom->pose.pose.position.z);

    return it;
}


double* DeltaFromOdom(const vector<double>& current,
                      const vector<double>& previous) {

  Eigen::Transform<double, 3, Eigen::Affine> current_transform;
  Eigen::Transform<double, 3, Eigen::Affine> previous_transform;
  // For the two poses create a transform
  double* posek = new double[6];

  Eigen::Vector3d cur_translation;
  cur_translation[0] = current[5];
  cur_translation[1] = current[6];
  cur_translation[2] = current[7];

  // Build the affine transforms for the previous pose
  Eigen::Quaternion<double> previous_quat(previous[4], previous[1], previous[2], previous[3]);
  Eigen::Translation<double, 3> previous_translation =
  Eigen::Translation<double, 3>(previous[5], previous[6], previous[7]);

  // Build the affine transforms based on the current pose
  Eigen::Quaternion<double> current_quat(current[4], current[1], current[2], current[3]);

  // Calculate delta rotation
  Eigen::Quaternion<double> rotation = previous_quat.inverse() * current_quat;

  Eigen::Translation<double, 3> translation =
  Eigen::Translation<double, 3>(-cur_translation[0] + previous_translation.x(), -cur_translation[1] + previous_translation.y(), -cur_translation[2] + previous_translation.z());
  Eigen::Transform<double, 3, Eigen::Affine> transform =
  translation * rotation;

  transform = transform.inverse();
  Eigen::AngleAxis<double> test(transform.rotation());
  // Get the axis
  Eigen::Vector3d test_axis = test.axis();
  // Recompute the rotation angle
  double test_angle = test.angle();
  Eigen::Vector3d test_trans = test_axis * test_angle;
  transform = transform;

  // Find the rotation component
  // Find the angle axis format
  Eigen::AngleAxis<double> angle_axis(transform.rotation());
  Eigen::Quaternion<double> final_quat(transform.rotation());
  // Get the axis
  Eigen::Vector3d normal_axis = angle_axis.axis();

  // Recompute the rotation angle
  double combined_angle = angle_axis.angle();
  Eigen::Vector3d combined_axis = normal_axis * combined_angle;

  // Compute Translation
  Eigen::Translation<double, 3> combined_translation(
    transform.translation());
  Eigen::Vector3d combined_trans = {combined_translation.x(), combined_translation.y(), combined_translation.z()};
  combined_trans = current_quat.inverse() * combined_trans;
  posek[3] = -combined_trans[0];
  posek[4] = -combined_trans[1];
  posek[5] = -combined_trans[2];

  // Recompute the rotation angle
  posek[0] = combined_axis[0];
  posek[1] = combined_axis[1];
  posek[2] = combined_axis[2];
  return posek;
}


void DeltaCalculationOdometry(string bag_name,
                              vector<ros::Publisher> publishers,
                              const int degree,
                              const int kMaxClouds) {

  // --- INITIALIZATION ---
  DeltaCalVariables* variables = new DeltaCalVariables();
  //Opening bagfile
  string bag_file = bag_name + ".bag";
  string pcd_folder = bag_name + "pcd_file/";
  mkdir(pcd_folder.c_str(), 0777);
  string keyframe_bag_name = bag_name + "_keyframes.bag";
  string normal_name = bag_name + "_normals.dat";
  vector<pcl::PointCloud<pcl::Normal> > all_normals;
  rosbag::Bag bag, keyframe_bag;
  bag.open(bag_file, rosbag::bagmode::Read);
  keyframe_bag.open(keyframe_bag_name, rosbag::bagmode::Write);
  std::vector<std::string> topics;
  topics.push_back(std::string("/Cobot/Kinect/Depth"));
  rosbag::View view(bag, rosbag::TopicQuery(topics));
  std::vector<std::string> odom_topics;
  odom_topics.push_back(std::string("/odom"));

  rosbag::View odom_view(bag, rosbag::TopicQuery(odom_topics));
  string str_key_normal_k1 = pcd_folder + "key_normal_k1";
  string str_key_normal_k2 = pcd_folder + "key_normal_k2";
  string str_prev_normal_k1 = pcd_folder + "prev_normal_k1";
  string str_prev_normal_k2 =pcd_folder + "prev_normal_k2";
  string str_normal_k1 = pcd_folder + "normal_k1";
  string str_normal_k2 =pcd_folder + "normal_k2";
  // Iterator over bag bag_name
  rosbag::View::iterator bag_it = view.begin();
  rosbag::View::iterator end = view.end();
  rosbag::View::iterator odom_it = odom_view.begin();
  rosbag::View::iterator odom_end = odom_view.end();
  bag_it = InitializeVariablesSingle(bag_name, degree, bag_it, end, variables);
  vector<double> keyframe_odom, previous_odom;

  odom_it = ClosestOdom(odom_it, odom_end, variables->k1_timestamp, &keyframe_odom);
  previous_odom = keyframe_odom;
  ofstream pose_file (variables->pose_name.c_str());
  ofstream odom_file ("selected_odom.txt");
  int avg_len = 5;
  bool dist_okay = false;
  int count = 0;
  // Save normals and get size of normals
  all_normals.push_back(variables->k1_key_normal);
  // size_t size = all_normals[0].size();

  if((variables->k1_buffer.size() == 0)){
  }
  if(bag_it == variables->end){
    cout << variables->k1_keyframe.size() << endl;
  }
  // While there are still clouds in both datasets
  while(((variables->k1_buffer.size() != 0)
    || (bag_it != variables->end))) {
    count += 1;
  cout << "Frame: " << count << endl;
  // Read in a new cloud from each dataset
  pcl::PointCloud<pcl::PointXYZ> k1_cloud;
  bag_it = OneSensorClouds(bag_it, end, &variables->k1_buffer,
                           &variables->k1_timestamps,
                           &k1_cloud, &variables->k1_timestamp);
  vector<double> current_odom;

  odom_it = ClosestOdom(odom_it, odom_end, variables->k1_timestamp, &current_odom);
  pcl::PointCloud<pcl::Normal> k1_normal;
  pcl::PointCloud<pcl::Normal> k2_normal;
  // Get normals for the two clouds
  k1_normal = GetNormals(k1_cloud);
  PublishCloud(k1_cloud,cloud_pub_3);
  // If the residual distance between either of these clouds (unmodified) and
  // the clouds k - 1 is large enough continue (otherwise read in new clouds)
  double k1_calculated_delta[6];

  // Check the residual distance against threshold
  const double k1_residual = ResidualDist(variables->k1_prev,
                                          k1_cloud,
                                          variables->k1_prev_normal,
                                          k1_normal,
                                          k1_calculated_delta);
  // Accumulate and write velocities
  const double k1_velocity = k1_residual / (variables->k1_timestamp -
    variables->k1_prev_timestamp);
  variables->k1_velocity_list[count % avg_len] = k1_velocity;
  double k1_acc_velocity =
  std::accumulate(variables->k1_velocity_list.begin(), variables->k1_velocity_list.end(), 0.0);
  k1_acc_velocity = k1_acc_velocity / avg_len;

  if ((k1_residual > 0.003 ) || dist_okay) {
    // Run ICP
    fprintf(stdout, "Kinect 1\n");
    CalculateDelta(count,
                   publishers,
                   variables->covariance_file,
                   variables->k1_covariance,
                   variables->k1_prev,
                   k1_cloud,
                   variables->k1_keyframe,
                   variables->k1_prev_normal,
                   k1_normal,
                   variables->k1_key_normal,
                   variables->k1_combined_transform,
                   NULL);
  }
  // Check the magnitude of translation and angle of rotation, if larger
  // than some threshold, this is our next keyframe
  // If there has been sufficient change update keyframe and save deltas
  double* previous_odom_delta = DeltaFromOdom(current_odom, previous_odom);
  double* odom_delta = DeltaFromOdom(current_odom, keyframe_odom);
//   PrintPose(current_odom);
//   PrintPose(previous_odom);
  PrintPose(previous_odom_delta);
  PrintPose(odom_delta);
  cout << endl;
  cout << "check delta odom" << endl;
  bool k1_change = CheckChangeVel(variables->k1_combined_transform, degree, variables->k1_velocity_list);
  cout << "check change odom" << endl;
  bool k2_change = CheckChangeOdom(odom_delta, previous_odom_delta, current_odom[0], previous_odom[0],  degree);
  cout << endl;
  if(k1_change && k2_change) {
    dist_okay = false;
    double* odom_delta = DeltaFromOdom(current_odom, keyframe_odom);
    cout << endl;
    keyframe_odom = current_odom;
    vector<double> pose0(6, 0.0);
    variables->k1_keyframe = k1_cloud;
    variables->k1_key_normal = k1_normal;
    variables->keys.push_back(count);

    WritePoseFile(variables->k1_combined_transform, variables->k1_timestamp,
        count, pose_file);
    WritePoseFile(odom_delta, variables->k1_timestamp, count, pose_file);
    pose_file << endl;

    //Transform the clouds and then write them to object files
    pcl::PointCloud<pcl::PointXYZ> temp_cloud1 = k1_cloud;
    WriteToBag("kinect_1", &keyframe_bag, k1_cloud);
    TransformPointCloud(&temp_cloud1, variables->k1_combined_transform);
    // Zero Combined transforms
    std::copy(pose0.begin(), pose0.end(), variables->k1_combined_transform);
    WriteToObj(variables->object_file, variables->k1_output_name, count, temp_cloud1);
    WriteToObj(variables->object_file, variables->k1_base_name, count, k1_cloud);

  }
  // Checking to see if we have a significant change from the predecessor
  else{
    if(CheckChange(variables->k1_combined_transform, degree)) {
      dist_okay = true;
      }
      else{
        dist_okay = false;
      }
  }
  // Update predecessor clouds
  variables->k1_prev = k1_cloud;
  variables->k1_prev_normal = k1_normal;
  variables->k1_prev_timestamp = variables->k1_timestamp;
  previous_odom = current_odom;
    }
    keyframe_bag.close();
}

void DeltaCalculationBrass(string bag_name,
                              vector<ros::Publisher> publishers,
                              const int degree,
                              const int kMaxClouds) {

  // --- INITIALIZATION ---
  bool log = true;
  DeltaCalVariables* variables = new DeltaCalVariables();
  //Opening bagfile
  string bag_file = bag_name + ".bag";
  vector<pcl::PointCloud<pcl::Normal> > all_normals;
  rosbag::Bag bag, keyframe_bag;
  bag.open(bag_file, rosbag::bagmode::Read);

  std::vector<std::string> topics;
  topics.push_back(std::string("/camera/depth/points"));
  rosbag::View view(bag, rosbag::TopicQuery(topics));
  std::vector<std::string> odom_topics;
  odom_topics.push_back(std::string("/odom"));

  rosbag::View odom_view(bag, rosbag::TopicQuery(odom_topics));

  // Iterator over bag bag_name
  rosbag::View::iterator bag_it = view.begin();
  rosbag::View::iterator end = view.end();
  rosbag::View::iterator odom_it = odom_view.begin();
  rosbag::View::iterator odom_end = odom_view.end();
  bag_it = InitializeVariablesBrass(bag_name, degree, bag_it, end, variables);

  vector<double> keyframe_odom, previous_odom;
  odom_it =
      ClosestOdom(odom_it, odom_end, variables->k1_timestamp, &keyframe_odom);
  previous_odom = keyframe_odom;
  ofstream pose_file (variables->pose_name.c_str());
  string uncertainty_t_string = bag_name + "_uncertainty_t.txt";
  string uncertainty_r_string = bag_name + "_uncertainty_r.txt";
  ofstream uncertaintyT_file (uncertainty_t_string.c_str());
  ofstream uncertaintyR_file (uncertainty_r_string.c_str());

  int avg_len = 1;
  bool dist_okay = false;
  int count = 0;
  // Save normals and get size of normals

  // While there are still clouds in both datasets
  while(((variables->k1_buffer.size() != 0)
    || (bag_it != variables->end)) && (odom_it != odom_end) ) {
    count += 1;

  cout << "Frame: " << count << endl;

  // Read in a new cloud from each dataset
  pcl::PointCloud<pcl::PointXYZ> k1_cloud;
  bag_it = OneSensorCloudsBrass(bag_it, end, &variables->k1_buffer,
                           &variables->k1_timestamps,
                           &k1_cloud, &variables->k1_timestamp);

  vector<double> current_odom;

  odom_it = ClosestOdom(odom_it, odom_end, variables->k1_timestamp, &current_odom);
  pcl::PointCloud<pcl::Normal> k1_normal;
  pcl::PointCloud<pcl::Normal> k2_normal;

  // Get normals for the two clouds
  k1_normal = GetNormals(k1_cloud);
  PublishCloud(k1_cloud,cloud_pub_3);
  // If the residual distance between either of these clouds (unmodified) and
  // the clouds k - 1 is large enough continue (otherwise read in new clouds)
  double k1_calculated_delta[6];

  // Check the residual distance against threshold
  const double k1_residual = ResidualDist(variables->k1_prev,
                                          k1_cloud,
                                          variables->k1_prev_normal,
                                          k1_normal,
                                          k1_calculated_delta);
  // Accumulate and write velocities
  const double k1_velocity = k1_residual / (variables->k1_timestamp -
    variables->k1_prev_timestamp);
  variables->k1_velocity_list[count % avg_len] = k1_velocity;
  double k1_acc_velocity =
  std::accumulate(variables->k1_velocity_list.begin(), variables->k1_velocity_list.end(), 0.0);
  k1_acc_velocity = k1_acc_velocity / avg_len;

  // I don't know if these residual values are reasonable. Although we
  // have the advantage of only needing one residual (will check against lab computers and other methods)
  if ((k1_residual > 0.003 ) || dist_okay) {
    // Run ICP
    double rot1;
    fprintf(stdout, "Kinect 1\n");
    CalculateDelta(count,
                   publishers,
                   variables->covariance_file,
                   variables->k1_covariance,
                   variables->k1_prev,
                   k1_cloud,
                   variables->k1_keyframe,
                   variables->k1_prev_normal,
                   k1_normal,
                   variables->k1_key_normal,
                   variables->k1_combined_transform,
                   &rot1);
    const double k1_velocity = rot1 / (variables->k1_timestamp - variables->k1_prev_timestamp);
    variables->k1_velocity_list[count % avg_len] = k1_velocity;
    double k1_acc_velocity =
    std::accumulate(variables->k1_velocity_list.begin(), variables->k1_velocity_list.end(), 0.0);
    k1_acc_velocity = k1_acc_velocity / avg_len;
  }
  // Check the magnitude of translation and angle of rotation, if larger
  // than some threshold, this is our next keyframe
  // If there has been sufficient change update keyframe and save deltas

  // Calculate Delta From Odometry between this and the previous odometry frame
  double* previous_odom_delta = DeltaFromOdom(current_odom, previous_odom);
  // Calculate the keyframe odometry delta
  double* odom_delta = DeltaFromOdom(current_odom, keyframe_odom);
  PrintPose(previous_odom_delta);
  PrintPose(odom_delta);
  // Have to watch the check changes when I test this
  cout << endl;
  cout << "check delta odom" << endl;
  bool k1_change = CheckChangeVel(variables->k1_combined_transform, degree, variables->k1_velocity_list);
  cout << "check change odom" << endl;
  bool k2_change = CheckChangeOdom(odom_delta, previous_odom_delta, current_odom[0], previous_odom[0],  degree);
  cout << endl;
  // If there is enough change
  if(k1_change && k2_change) {
    dist_okay = false;
    // This is getting calculated twice but I don't know why.
    double* odom_delta = DeltaFromOdom(keyframe_odom, current_odom);
    cout << endl;
    keyframe_odom = current_odom;
    vector<double> pose0(6, 0.0);
    variables->k1_keyframe = k1_cloud;
    variables->k1_key_normal = k1_normal;
    variables->keys.push_back(count);
    vector<Eigen::Vector4d> plane_normals;
    vector<Eigen::Vector3d> plane_centroids;
    cout << "Extracting Planes" << endl;
    ExtractPlanes(k1_cloud, &plane_normals, &plane_centroids);

    Eigen::Vector3d uncertainty_t;
    Eigen::Vector3d uncertainty_r;
    cout << "exctracting Uncertainty" << endl;
    ExtractUncertainty(plane_normals, &uncertainty_t, &uncertainty_r);

    cout << "Writing Uncertainty " << endl;
    // Writes both deltas to the same file
    WriteUncertaintyFile(uncertainty_t, uncertaintyT_file);
    WriteUncertaintyFile(uncertainty_r, uncertaintyR_file);

//     StripUncertainty(uncertainty_t, uncertainty_r, variables->k1_combined_transform);
    // Writes both deltas to the same file
    WritePoseFile(variables->k1_combined_transform, variables->k1_timestamp,
        count, pose_file);
    WritePoseFile(odom_delta, variables->k1_timestamp, count, pose_file);
    pose_file << endl;
    // Zero Combined transforms
    std::copy(pose0.begin(), pose0.end(), variables->k1_combined_transform);

    // Transform the clouds and then write them to object files
    if(log) {
    pcl::PointCloud<pcl::PointXYZ> temp_cloud1 = k1_cloud;
// //     WriteToBag("kinect_1", &keyframe_bag, k1_cloud);
    TransformPointCloud(&temp_cloud1, variables->k1_combined_transform);

    WriteToObj(variables->object_file, variables->k1_output_name, count, temp_cloud1);
    WriteToObj(variables->object_file, variables->k1_base_name, count, k1_cloud);
    }

  }
  // Checking to see if we have a significant change from the predecessor
  else{
    if(CheckChange(variables->k1_combined_transform, degree)) {
      dist_okay = true;
      }
      else{
        dist_okay = false;
      }
  }
  // Update predecessor clouds
  variables->k1_prev = k1_cloud;
  variables->k1_prev_normal = k1_normal;
  variables->k1_prev_timestamp = variables->k1_timestamp;
  previous_odom = current_odom;
    }
    keyframe_bag.close();
}
}
