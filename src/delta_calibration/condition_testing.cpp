#include "delta_calibration/icp.h"
#include "delta_calibration/delta_calc.h"
#include "delta_calibration/partial_calibrate.h"
#include <stdio.h>      /* printf, NULL */
#include <stdlib.h>     /* srand, rand */
#include <time.h>  
//Intialize empty publishers
ros::Publisher cloudx_pub_1;
ros::Publisher cloudx_pub_2;
ros::Publisher cloudx_pub_3;
ros::Publisher cloudx_pub_4;

ros::Publisher xmarker_pub;
ros::Publisher xmarkerArray_pub;

using namespace delta_calc;
using namespace icp;

vector<double> conditions = {0, 10, 50, 100, 200, 300, 400, 500, 600, 700, 800, 900, 1000};
double extrinsics[6] = { 0.0915,    0.3334,    0.2165,    0.0527,    0.0454,    0.0017};
double fRand(double fMin, double fMax)
{
  double f = (double)rand() / RAND_MAX;
  return fMin + f * (fMax - fMin);
}

// For checking if the mean has not changed
bool DoubleEquals(double x, double y) {
  return fabs(x-y) < .001;
}


pcl::PointCloud<pcl::PointXYZ> GenerateCorner() {
  pcl::PointCloud<pcl::PointXYZ> cloud;
  
  for(size_t i = 0; i < 5000; ++i) {
    pcl::PointXYZ point;
    point.z = 0;
    point.x = fRand(0.0, 2.0);
    point.y = fRand(0.0,2.0);
    cloud.points.push_back(point);
  }
  for(size_t i = 0; i < 5000; ++i) {
    pcl::PointXYZ point;
    point.z = fRand(0.0, 2.0);
    point.x = 2.0;
    point.y = fRand(0.0,2.0);
    cloud.points.push_back(point);
  }
  for(size_t i = 0; i < 5000; ++i) {
    pcl::PointXYZ point;
    point.z = fRand(0.0, 2.0);
    point.x = fRand(0.0, 2.0);
    point.y = 2.0;
    cloud.points.push_back(point);
  }
  return cloud;
}

vector<pcl::PointCloud<pcl::PointXYZ> > CreateSequence(const pcl::PointCloud<pcl::PointXYZ>& cloud, vector<vector<double> > deltas) {
  vector<pcl::PointCloud<pcl::PointXYZ> > clouds;
  
  for(size_t i = 0; i < deltas.size(); i++) {
    vector<double> delta = deltas[i];
    double transform_array[6];
    std::copy(delta.begin(), delta.end(), transform_array);
    pcl::PointCloud<pcl::PointXYZ> new_cloud;
    copyPointCloud(cloud, new_cloud);
    TransformPointCloud(&new_cloud, transform_array);
    clouds.push_back(cloud);
    clouds.push_back(new_cloud);
  }
  
  return clouds;
}


pcl::PointCloud<pcl::PointXYZ> SubsamplePlane(pcl::PointCloud<pcl::PointXYZ> cloud, int axis, int count) {
  pcl::PointCloud<pcl::PointXYZ> sampled;
  int counter = count;
  for(size_t i = 0; i < cloud.size(); ++i) {
    pcl::PointXYZ point = cloud[i];
    if(counter == 0) {
      sampled.push_back(point);
    } else {
      if(axis == 0) {
        if(DoubleEquals(point.x, 2.0)) {
          counter--;
        } else{
          sampled.push_back(point);
        }
      } else if (axis == 1) {
        if(DoubleEquals(point.y, 2.0)) {
          counter--;
        } else{
          sampled.push_back(point);
        }
      } else if (axis == 2) {
        if(DoubleEquals(point.z, 0)) {
          counter--;
        } else{
          sampled.push_back(point);
        }
      }
    }
  }
  return sampled;
}

double* ReadTransform(string file) {
  
  std::ifstream infile(file.c_str());
  std::string line;
  std::getline(infile, line);
  vector<double> delta(6, 0.0);
  std::istringstream iss(line);
  if (!(iss >> delta[0] >> delta[1] >> delta[2] >> delta[3] >> delta[4] >> delta[5])) { 
    cout << "problem" << endl;
  } 
  double* transform = new double[6];
  std::copy(delta.begin(), delta.end(), transform);
  return transform;
}

void RunConditionExperiment(string file, pcl::PointCloud<pcl::PointXYZ> cloud, vector<vector<double> > deltas_1, vector<vector<double> > deltas_2) {
  cout << "creating sequence" << endl;
  vector<pcl::PointCloud<pcl::PointXYZ> > clouds = CreateSequence(cloud, deltas_1);
  double time = 0;
  ofstream error_file (file.c_str());
  for(size_t i = 0; i < 1; ++i) {
    string variance_string = "generated_variance.txt";
    ofstream variance_file (variance_string.c_str());
    
    string pose_string = "generated_deltas.txt";
    ofstream pose_file (pose_string.c_str());
    Eigen::Vector3d zero_un = {0, 0, 0};
    string uncertainty_t_string = "generated_uncertaintiest.txt";
    string uncertainty_r_string = "generated_uncertaintiesr.txt";
    
    ofstream uncertaintyT_file (uncertainty_t_string.c_str());
    ofstream uncertaintyR_file (uncertainty_r_string.c_str());
    int count = 0;
    for(size_t j = 0; j < clouds.size() / 2; j=j+2) {
      cout << "internal setup" << endl;
      pcl::PointCloud<pcl::PointXYZ> cloud1 = clouds[j];
      pcl::PointCloud<pcl::PointXYZ> cloud2 = clouds[j+1];
      double* transform = new double[6];
      double* variance = new double[6];
      double* zero_unc = new double[6];
      vector<double> delta(6, 0.0);
      pcl::PointCloud<pcl::Normal> normals_1 = GetNormals(cloud1);
      pcl::PointCloud<pcl::Normal> normals_2 = GetNormals(cloud2);
      vector<double> pose0(6, 0.0);
      std::copy(pose0.begin(), pose0.end(), transform);
      std::copy(pose0.begin(), pose0.end(), zero_unc);
      vector<Eigen::Vector2d> empty_coords;
      vector<ros::Publisher> publishers;
      publishers.push_back(cloudx_pub_1);
      publishers.push_back(cloudx_pub_2);
      publishers.push_back(cloudx_pub_3);
      cout << "run icp" << endl;
      ICP (10,
           .5,
           publishers,
           "",
           "",
           cloud2,
           cloud1,
           normals_2,
           normals_1,
           empty_coords,
           empty_coords,
           transform,
           variance,
           NULL);
      
      WriteVarianceFile(zero_unc, uncertaintyT_file);
      WriteVarianceFile(zero_unc, uncertaintyR_file);
      
      cout << "Writing Variance" << endl;
      // Writes both deltas to the same file
      WriteVarianceFile(variance, variance_file);
      // Writes both deltas to the same file
      WritePoseFile(transform, j, 
                    j, pose_file);
      double transform_array[6];
      std::copy(deltas_2[count].begin(), deltas_2[count].end(), transform_array);
      WritePoseFile(transform_array, j, j, pose_file);
      count++;
      pose_file << endl;
    }
    cout << "calibrate" << endl;
    
    cout << "read in and calc error" << endl;
    system("./bin/partial_calibrate");
    double* pose = ReadTransform("calibration.pose");
    Eigen::Matrix<double,4,1> error = TransformDifference(pose, extrinsics);
    error_file << error[0] << "\t" << error[2] << endl;
  }
}

int main(int argc, char **argv) {
  signal(SIGINT,HandleStop);
  signal(SIGALRM,HandleStop);
  
  srand (time(NULL));
  // Initialize Ros
  ros::init(argc, argv, "condition_testing",
            ros::init_options::NoSigintHandler);
  ros::NodeHandle n;
  //----------  Setup Publishers ----------
  cloudx_pub_1 = n.advertise<sensor_msgs::PointCloud2> ("cloud_pub_1", 1);
  cloudx_pub_2 = n.advertise<sensor_msgs::PointCloud2> ("cloud_pub_2", 1);
  cloudx_pub_3 = n.advertise<sensor_msgs::PointCloud2> ("cloud_pub_3", 1);
  cloudx_pub_4 = n.advertise<sensor_msgs::PointCloud2> ("cloud_pub_4", 1);
  vector<ros::Publisher> publishers;
  publishers.push_back(cloudx_pub_1);
  publishers.push_back(cloudx_pub_2);
  publishers.push_back(cloudx_pub_3);
  publishers.push_back(cloudx_pub_4);
  xmarker_pub =
  n.advertise<visualization_msgs::Marker>("visualization_marker", 10);
  xmarkerArray_pub =
  n.advertise<visualization_msgs::MarkerArray>(
    "visualization_marker_array", 10);
  
  string file = "condition_test_deltas.txt";
  vector<vector<double> > deltas_1;
  vector<vector<double> > deltas_2;
  vector<vector<double> > uncertaintyE_1;
  vector<vector<double> > uncertaintyE_2;
  
  partial_calibrate::ReadDeltasFromFile(file,
                     &deltas_1,
                     &deltas_2,
                     &uncertaintyE_1,
                     &uncertaintyE_2); 
  
  cout << deltas_1.size() << endl;
  cout << deltas_2.size() << endl;
  sleep(5);
  
  visualization_msgs::Marker line_list;
  std_msgs::Header header;
  header.frame_id = "point_clouds";
  line_list.action = visualization_msgs::Marker::ADD;
  //line_list.pose.orientation.w = 1.0;
  line_list.header.frame_id = "point_cloud";
  
  line_list.id = 0;
  line_list.type = visualization_msgs::Marker::LINE_LIST;
  line_list.scale.x = 0.01;
  
  // Line list is red
  line_list.color.r = 1.0;
  line_list.color.a = 1.0;
  geometry_msgs::Point center, x, y, z;
  center.x = center.y = center.z = x.y = x.z = y.x = y.z = z.x = z.y = 0;
  z.z = .5;
  x.x = .5;
  y.y = .5;
  
//   ReadDeltasFromFile(&deltas_1, &deltas_2) {
//   }
  pcl::PointCloud<pcl::PointXYZ> corner = GenerateCorner();
  pcl::PointCloud<pcl::Normal> normals = icp::GetNormals(corner);
  Eigen::Matrix3d scatter = icp::CalcScatterMatrix(normals);
  double condition =  icp::CalcConditionNumber(scatter);
  cout << "Condition: " <<condition << endl;
  int axis = 0;
  
  for(size_t i = 0; i < conditions.size(); ++i) {
    cout << "checkpoint" << endl;
    double checkpoint = conditions[i];
    if(checkpoint > 500) {
      axis = 1;
    }
    while(condition < checkpoint) {
      cout << "subsample" << endl;
      corner = SubsamplePlane(corner, axis, 100);
      normals = icp::GetNormals(corner);
      scatter = icp::CalcScatterMatrix(normals);
      condition =  icp::CalcConditionNumber(scatter);
      cout << condition << endl;
    }
    cout << "setting experiment" << endl;
    std::ostringstream strs;
    strs << checkpoint;
    std::string str = strs.str();
    string output = str + "condition_error.txt";
    cout << "running experiment" << endl;
    RunConditionExperiment(output, corner, deltas_1, deltas_2);
  }
  
  return 0;
}