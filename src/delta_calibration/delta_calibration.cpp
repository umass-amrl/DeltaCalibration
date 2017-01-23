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
// Controls usage of normal files
bool normal_file = false;

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
//       fprintf(stdout, "ICP 1 \n");
//    combine the transform returned by ICP with the combined transform,
//    and run ICP between clouds k and the last keyframe
      CombineTransform(transform, calculated_delta);

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
//       fprintf(stdout, "ICP 2 \n");
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
  input->pose_name = "brass.pose";
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
  double k1_calculated_delta[6];
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
    
    cout << "k1_residual: " << k1_residual << " Time 1: " << variables->k1_timestamp << "Time 2: " << variables->k1_prev_timestamp << endl;
    cout << "k2_residual: " << k2_residual << " Time 1: " << variables->k2_timestamp << "Time 2: " << variables->k2_prev_timestamp << endl;
    
    
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
    keyframe_odom->push_back(best_odom->pose.pose.position.x);
    keyframe_odom->push_back(best_odom->pose.pose.position.y);
    keyframe_odom->push_back(best_odom->pose.pose.position.z);
    keyframe_odom->push_back(best_odom->pose.pose.orientation.x);
    keyframe_odom->push_back(best_odom->pose.pose.orientation.y);
    keyframe_odom->push_back(best_odom->pose.pose.orientation.z);
    keyframe_odom->push_back(best_odom->pose.pose.orientation.w);
    return it;
}

double* DeltaFromOdom(const vector<double>& current, 
                      const vector<double>& previous) {
  Eigen::Transform<double, 3, Eigen::Affine> current_transform;
  Eigen::Transform<double, 3, Eigen::Affine> previous_transform;
  // For the two poses create a transform
  double* posek = new double[6];
  
  Eigen::Vector3d cur_translation;
  cur_translation[0] = current[1];
  cur_translation[1] = current[2];
  cur_translation[2] = current[3];
  
    // Build the affine transforms for the previous pose
  Eigen::Quaternion<double> previous_quat(previous[7], previous[4], previous[5], previous[6]);
  Eigen::Translation<double, 3> previous_translation =
      Eigen::Translation<double, 3>(previous[1], previous[2], previous[3]);
  
  // Build the affine transforms based on the current pose
  Eigen::Quaternion<double> current_quat(current[7], current[4], current[5], current[6]);
  // Calculate delta rotation
  Eigen::Quaternion<double> rotation = previous_quat.inverse() * current_quat;
  // Calculate Delta Translation
  cur_translation = (previous_quat * current_quat.inverse() * cur_translation);
  Eigen::Translation<double, 3> translation =
  Eigen::Translation<double, 3>(-cur_translation[0] + previous_translation.x(), -cur_translation[1] + previous_translation.y(), -cur_translation[2] + previous_translation.z());
  Eigen::Transform<double, 3, Eigen::Affine> transform =
      translation * rotation;
      
  transform = transform.inverse();
 
      
  // Find the rotation component
  // Find the angle axis format
      Eigen::AngleAxis<double> angle_axis(transform.rotation());
  
  // Get the axis
  Eigen::Vector3d normal_axis = angle_axis.axis();
  
  // Recompute the rotation angle
  double combined_angle = angle_axis.angle();
  Eigen::Vector3d combined_axis = normal_axis * combined_angle;
  
  // Compute Translation
  Eigen::Translation<double, 3> combined_translation(
    transform.translation());
  
  cout << "Current translation" << endl;
  cout << current[1] << " " << current[2] << " " << current[3] << endl;
  cout << "combined Translation" << endl;
  cout << combined_translation.x() << " " << combined_translation.y() << " " << combined_translation.z() <<  endl;
  cout << "Previous translation" << endl;
  cout << previous_translation.x() << " " << previous_translation.y() << " " << previous_translation.z() <<  endl;
  posek[3] = combined_translation.x();
  posek[4] = combined_translation.y();
  posek[5] = combined_translation.z();
  cout << "Delta translation" << endl;
  cout << posek[3] << " " << posek[4] << " " << posek[5] << " " << endl;
  // Recompute the rotation angle
  posek[0] = combined_axis(0);
  posek[1] = combined_axis(1);
  posek[2] = combined_axis(2);
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
  bool log = false;
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
  odom_it = ClosestOdom(odom_it, odom_end, variables->k1_timestamp, &keyframe_odom);
  previous_odom = keyframe_odom;
  ofstream pose_file (variables->pose_name.c_str());
  ofstream odom_file ("brass_odom.txt"); // Figure out odometry file?
  
  int avg_len = 5;
  bool dist_okay = false;
  int count = 0;
  // Save normals and get size of normals
  
  // While there are still clouds in both datasets
  while(((variables->k1_buffer.size() != 0)
    || (bag_it != variables->end))) {
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
  
  // Calculate Delta From Odometry between this and the previous odometry frame
  double* previous_odom_delta = DeltaFromOdom(current_odom, previous_odom);
  // Calculate the keyframe odometry delta
  double* odom_delta = DeltaFromOdom(current_odom, keyframe_odom);
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
    double* odom_delta = DeltaFromOdom(current_odom, keyframe_odom);
    cout << endl;
    keyframe_odom = current_odom;
    vector<double> pose0(6, 0.0);
    variables->k1_keyframe = k1_cloud;
    variables->k1_key_normal = k1_normal;
    variables->keys.push_back(count);
    
    // Writes both deltas to the same file
    WritePoseFile(variables->k1_combined_transform, variables->k1_timestamp, 
        count, pose_file);
    WritePoseFile(odom_delta, variables->k1_timestamp, count, pose_file);
    pose_file << endl;
    
    // Transform the clouds and then write them to object files
    if(log) {
    pcl::PointCloud<pcl::PointXYZ> temp_cloud1 = k1_cloud;
    WriteToBag("kinect_1", &keyframe_bag, k1_cloud);
    TransformPointCloud(&temp_cloud1, variables->k1_combined_transform);
    // Zero Combined transforms
    std::copy(pose0.begin(), pose0.end(), variables->k1_combined_transform);
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

int main(int argc, char **argv) {
  signal(SIGINT,HandleStop);
  signal(SIGALRM,HandleStop);

  int max_clouds = INT_MAX;
  int max_delta_degrees = 0;
  char* bag_file = (char*)"pair_upright.bag";
  int mode = 0;
  bool normal_mode = false;
  // Parse arguments.
  static struct poptOption options[] = {
    { "max-clouds" , 'k', POPT_ARG_INT , &max_clouds ,0, "Max Clouds" , "NUM" },
    { "delta" , 'd', POPT_ARG_INT , &max_delta_degrees ,0, "Angular Change" ,
        "NUM" },
    { "bag-file" , 'B', POPT_ARG_STRING, &bag_file ,0, "Process bag file" ,
        "STR" },
    { "mode-mode", 'M', POPT_ARG_NONE, &mode, 0, "Selec input mode",
        "NONE" },
    POPT_AUTOHELP
    { NULL, 0, 0, NULL, 0, NULL, NULL }
  };

  // parse options
  POpt popt(NULL,argc,(const char**)argv,options,0);
  int c;
  while((c = popt.getNextOpt()) >= 0) {
  }
  // Print option values
  printf("Max Frames: %d\nBagfile: %s\nDelta Size: %d deg\n",
         max_clouds,
         bag_file,
         max_delta_degrees);

  // Initialize Ros
  ros::init(argc, argv, "delta_calibration",
  ros::init_options::NoSigintHandler);
  ros::NodeHandle n;
  //----------  Setup Publishers ----------
  cloud_pub_1 = n.advertise<sensor_msgs::PointCloud2> ("cloud_pub_1", 1);
  cloud_pub_2 = n.advertise<sensor_msgs::PointCloud2> ("cloud_pub_2", 1);
  cloud_pub_3 = n.advertise<sensor_msgs::PointCloud2> ("cloud_pub_3", 1);
  cloud_pub_4 = n.advertise<sensor_msgs::PointCloud2> ("cloud_pub_4", 1);
  vector<ros::Publisher> publishers;
  publishers.push_back(cloud_pub_1);
  publishers.push_back(cloud_pub_2);
  publishers.push_back(cloud_pub_3);
  publishers.push_back(cloud_pub_4);
  marker_pub =
  n.advertise<visualization_msgs::Marker>("visualization_marker", 10);
  markerArray_pub =
  n.advertise<visualization_msgs::MarkerArray>(
      "visualization_marker_array", 10);

  if (mode == 0) {
    DeltaCalculation(bag_file, publishers, max_delta_degrees, 
                     max_clouds);
  } else if(mode == 1) {
    DeltaCalculationOdometry(bag_file, publishers, max_delta_degrees, 
                             max_clouds);
  } else {
    DeltaCalculationSingle(bag_file, publishers, max_delta_degrees, 
                           max_clouds);
  }
  return 0;
}
