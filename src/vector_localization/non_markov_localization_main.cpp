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
// Copyright 2012 joydeepb@ri.cmu.edu
// Robotics Institute, Carnegie Mellon University
//
// Main entry point for non-Markov localization.

#include <stdint.h>
#include <stdio.h>

#include <algorithm>
#include <cmath>
#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/Eigenvalues>
#include <gflags/gflags.h>
#include <glog/logging.h>
#include <pthread.h>
#include <queue>
#include <string>
#include <termios.h>
#include <utility>
#include <vector>

#include "eigen_helper.h"
#include "helpers.h"
#include "non_markov_localization.h"
#include "CImg/CImg.h"
#include "cobot_msgs/CobotEventsMsg.h"
#include "cobot_msgs/CobotLocalizationMsg.h"
#include "cobot_msgs/CobotOdometryMsg.h"
#include "cobot_msgs/CobotRemoteInterfaceSrv.h"
#include "cobot_msgs/GuiKeyboardEvent.h"
#include "cobot_msgs/GuiMouseClickEvent.h"
#include "cobot_msgs/LidarDisplayMsg.h"
#include "cobot_msgs/LocalizationGuiCaptureSrv.h"
#include "cobot_msgs/LocalizationMsg.h"
#include "cobot_msgs/CobotLocalizationSrv.h"
#include "cross_entropy_minimizer/cross_entropy_minimizer.h"
#include "gui_publisher_helper.h"
#include "nav_msgs/Odometry.h"
#include "object_map.h"
#include "perception_tools/perception_2d.h"
#include "popt_pp.h"
#include "proghelp.h"
#include "pthread_utils.h"
#include "ros/ros.h"
#include "ros/package.h"
#include "rosbag/bag.h"
#include "rosbag/view.h"
#include "shared/math/geometry.h"
#include "shared/util/configreader.h"
#include "sensor_msgs/LaserScan.h"
#include "timer.h"
#include "util.h"
#include "vector_map.h"
#include "vector_localization/relocalization.h"
#include "vector_localization/residual_functors.h"

using cobot_gui::DrawLine;
using cobot_gui::DrawPoint;
using cobot_gui::ClearDrawingMessage;
using cobot_msgs::CobotEventsMsg;
using cobot_msgs::CobotLocalizationSrv;
using cobot_msgs::CobotRemoteInterfaceSrv;
using Eigen::Affine2d;
using Eigen::Affine2f;
using Eigen::Matrix2d;
using Eigen::Matrix2f;
using Eigen::Matrix3d;
using Eigen::Matrix3f;
using Eigen::Perp2;
using Eigen::Rotation2Dd;
using Eigen::Rotation2Df;
using Eigen::ScalarCross;
using Eigen::Translation2d;
using Eigen::Translation2f;
using Eigen::Vector2d;
using Eigen::Vector2f;
using Eigen::Vector3d;
using EnmlMaps::PersistentObject;
using perception_2d::GenerateNormals;
using perception_2d::NormalCloudf;
using perception_2d::PointCloudf;
using perception_2d::Pose2Df;
using ros::ServiceServer;
using ros::Subscriber;
using std::pair;
using std::queue;
using std::size_t;
using std::sort;
using std::string;
using std::vector;
using vector_localization::NonMarkovLocalization;

typedef KDNodeValue<float, 2> KDNodeValue2f;

namespace {

class DestructorTrace {
 public:
  DestructorTrace(const std::string& file_name, const int line_number) :
      kFileName(file_name), kLineNumber(line_number) {}
  ~DestructorTrace() {
    printf("Global Destructors called @ %s:%d\n",
           kFileName.c_str(), kLineNumber);
  }
 private:
  const std::string kFileName;
  const int kLineNumber;
};

}  // namespace

// Name of the topic on Cobot's software stack that laser data is published on.
static const string kCobotLaserTopic("/Cobot/Laser");
// Name of the topic on Cobot's software stack that Kinect scan data is
// published on.
static const string kKinectScanTopic("/Cobot/Kinect/Scan");
// Name of the topic in a standardized data bag files that Kinect scan data is
// published on.
static const string kStandardKinectScanTopic("kinect_scan");
// Name of the topic on Cobot's software stack that odometry is published on.
static const string kCobotOdometryTopic("/Cobot/Odometry");
// Name of the topic in a standardized data bag files that laser data is
// published on.
static const string kStandardLaserTopic("laser");
// Name of the topic in a standardized data bag files that odometry is
// published on.
static const string kStandardOdometryTopic("odom");
// Name of the topic in a standardized data bag files that location reset
// commands are published on.
static const string kStandardSetLocationTopic("set_location");
// Name of the map to localize the robot on.
string kMapName;
// Robot's starting location.
Vector2f kStartingLocation = Vector2f(-9.0, 42.837);
// Robot's starting angle.
float kStartingAngle = RAD(-165.0);
// Uncertainty of translation in the direction of travel.
float kRadialTranslationUncertainty = 0.05;
// Uncertainty of translation perpendicular to the direction of travel.
float kTangentialTranslationUncertainty = 0.05;
// Uncertainty of rotation in radians after moving 1 radian.
float kAngleUncertainty = 0.05;
// Scaling constant to correct for error in angular odometry.
float kOdometryRotationScale = 1.0;
// Scaling constant to correct for error in translation odometry.
float kOdometryTranslationScale = 1.0;
// Minimum distance of observed points from the robot.
float kMinPointCloudRange = 0.2;
// Maximum distance of observed points from the robot.
float kMaxPointCloudRange = 6.0;
// Maximum distance between adjacent points to use for computation of normals.
float kMaxNormalPointDistance = 0.03;
// Indicates whether the bag file being read is a standardized data bag file.
bool kStandardizedData = false;
// The maximum expected odometry-reported translation difference per timestep.
float kSqMaxOdometryDeltaLoc = sq(0.2);
// The maximum expected odometry-reported rotation difference per timestep.
float kMaxOdometryDeltaAngle = RAD(15.0);
// Mutex to ensure only a single relocalization call is made at a time.
pthread_mutex_t relocalization_mutex_ = PTHREAD_MUTEX_INITIALIZER;

// The path of the cobot_linux ROS stack.
static const string kCobotStackPath(ros::package::getPath("cobot_linux"));

// The directory where all the maps are stored.
static const string kMapsDirectory(kCobotStackPath + "/../maps");

// Index of test set. This will determine which file the results of the test are
// saved to.
int test_set_index_ = -1;

// Indicates that a statistical test is being run, and to save the localization
// results to the file with the specified index.
int statistical_test_index_ = -1;

// The fraction of additive odometry noise for statistical tests.
double odometry_additive_noise_ = 0.05;

//static const uint32_t kTrajectoryColor = 0x6FFF0000;
static const uint32_t kTrajectoryColor = 0x7F000000;
static const uint32_t kPoseCovarianceColor = 0xFF808080;
static const uint32_t kOdometryColor = 0x70FF0000;
// static const uint32_t kTrajectoryColor = 0xFFC0C0C0;
static const uint32_t kLtfCorrespondenceColor = 0x7FFF7700;
static const uint32_t kLtfPointColor = 0xFFFF7700;
static const uint32_t kStfPointColor = 0xFF994CD9;
static const uint32_t kStfCorrespondenceColor = 0x7F994CD9;
static const uint32_t kDfPointColor  = 0x7F37B30C;
static const uint32_t kObjectColor = 0xFF56C4C3;

bool run_ = true;
int debug_level_ = -1;

// Indicates that /Cobot/Kinect/Scan should be used instead of
// /Cobot/Laser, to localize using the Kinect sensor instead of the laser
// rangefinder.
bool use_kinect_ = false;

// Indicates that the user will provide an estimate of an object instance via
// the localization_gui.
bool get_object_instance_ = false;

// Display message for drawing debug vizualizations on the localization_gui.
cobot_msgs::LidarDisplayMsg display_message_;

// Service client to ask localization_gui to save images.
ros::ServiceClient gui_capture_client;

// ROS publisher to publish display_message_ to the ROS topic
// Cobot/VectorLocalization/Gui
ros::Publisher display_publisher_;

// ROS publisher to the CoBot events topic.
ros::Publisher events_publisher_;

// ROS publisher to publish the latest robot localization estimate to
// Cobot/Localization
ros::Publisher localization_publisher_;

// Parameters and settings for Non-Markov Localization.
NonMarkovLocalization::LocalizationOptions localization_options_;

// Main class instance for Non-Markov Localization.
NonMarkovLocalization localization_(kMapsDirectory);

// Relocalization interface.
vector_localization::Relocalization relocalization_(kMapsDirectory);

// The last observed laser scan, used for auto localization.
sensor_msgs::LaserScan last_laser_scan_;

// Parameters used for relocalization.
VectorLocalization2D::LidarParams relocalization_lidar_params_;

// File name of the final image of all observations from all poses with their
// classifications.
char* save_image_file = NULL;

// Directory where images of every episode at every timestep will be saved.
char* episode_images_path = NULL;

// Determines whether STFS will be saved for later object mapping or not.
bool save_stfs_ = false;

// WatchFiles to track changes to config files.
WatchFiles watch_files_;

// Config reader for localization options.
ConfigReader config_((kCobotStackPath + "/").c_str());

// Corrections to angle-dependent errors of the laser scanner.
vector<float> laser_corrections_;

// Resolution of the laser corrections lookup table.
float laser_corrections_resolution_ = 0.0;

// Boolean flag to indicate use of laser scanner corrections.
bool use_laser_corrections_ = false;

// Flag to write LTF observations to disk for error correction computation.
bool save_ltfs_ = false;

// Flag to save episode statistics.
bool save_episode_stats_ = false;

// Suppress stdout.
bool quiet_ = false;

// Accept noise-free odometry, and apply noise by mimicing encoders of a
// four=-wheel omnidirectional robot.
void ApplyNoiseModel(
    const float dx,
    const float dy,
    const float da,
    const float e,
    float* dx_n,
    float* dy_n,
    float* da_n) {
  // Wheel radius.
  const float R = 0.1;
  Eigen::Matrix<float, 4, 3> M_vel_to_enc;
  Eigen::Matrix<float, 3, 4> M_enc_to_vel;
  const float C = cos(RAD(45.0));
  M_vel_to_enc <<
      C, C, R,
      -C, C, R,
      -C, -C, R,
      C, -C, R;
  const float kSqRt2 = sqrt(2.0);
  M_enc_to_vel <<
      kSqRt2, -kSqRt2, -kSqRt2, kSqRt2,
      kSqRt2, kSqRt2, -kSqRt2, -kSqRt2,
      1.0 / R, 1.0 / R, 1.0 / R, 1.0 / R;
  M_enc_to_vel = M_enc_to_vel / 4.0;
  const Eigen::Vector3f delta(dx, dy, da);
  // Compute noise-free encoder reading.
  const Eigen::Vector4f enc = M_vel_to_enc * delta;
  Eigen::Vector4f enc_noisy = Eigen::Vector4f::Zero();
  // Add noise.
  for (int i = 0; i < 4; ++i) {
    enc_noisy(i) = enc(i) + randn(e * enc(i));
  }
  const Eigen::Vector3f delta_noisy = M_enc_to_vel * enc_noisy;
  *dx_n = delta_noisy(0);
  *dy_n = delta_noisy(1);
  *da_n = delta_noisy(2);
}

void PublishLocation(
    const string& map_name, const float x, const float y, const float angle) {
  cobot_msgs::CobotLocalizationMsg localization_msg_;
  localization_msg_.timeStamp = GetTimeSec();
  localization_msg_.map = map_name;
  localization_msg_.x = x;
  localization_msg_.y = y;
  localization_msg_.angle = angle;
  localization_publisher_.publish(localization_msg_);
}

void PublishLocation() {
  Pose2Df latest_pose = localization_.GetLatestPose();
  cobot_msgs::CobotLocalizationMsg localization_msg_;
  localization_msg_.timeStamp = GetTimeSec();
  localization_msg_.map = localization_.GetCurrentMapName();
  localization_msg_.x = latest_pose.translation.x();
  localization_msg_.y = latest_pose.translation.y();
  localization_msg_.angle = latest_pose.angle;
  localization_publisher_.publish(localization_msg_);
}

void PublishTrace()
{
  Pose2Df latest_pose = localization_.GetLatestPose();
  static vector<vector2f> trace;
  static vector2f lastLoc;
  static bool initialized = false;
  const vector2f curLoc(
        latest_pose.translation.x(), latest_pose.translation.y());
  if(!initialized){
    trace.clear();
    trace.push_back(curLoc);
    lastLoc = curLoc;
    if(lastLoc.sqlength()>sq(5.0))
      initialized = true;
    return;
  }
  if((curLoc-lastLoc).sqlength()>sq(0.05)){
    trace.push_back(curLoc);
    lastLoc = curLoc;
  }

  for(unsigned int i=0; i<trace.size()-1;i++){
    if((trace[i]-trace[i+1]).sqlength()>1.0)
      continue;
    DrawLine(trace[i], trace[i + 1], 0xFFc0c0c0, &display_message_);
  }
  // printf("trace: %d locations\n",int(trace.size()));
}

void ClearDisplay() {
  ClearDrawingMessage(&display_message_);
  display_publisher_.publish(display_message_);
}

void PublishDisplay() {
  display_publisher_.publish(display_message_);
  ClearDrawingMessage(&display_message_);
}

int kbhit() {
  if(!run_) return 0;
  struct timeval tv;
  fd_set fds;
  tv.tv_sec = 0;
  tv.tv_usec = 0;
  FD_ZERO(&fds);
  FD_SET(STDIN_FILENO, &fds); //STDIN_FILENO is 0
  select(STDIN_FILENO+1, &fds, NULL, NULL, &tv);
  return FD_ISSET(STDIN_FILENO, &fds);
}

void nonblock(const bool blocking) {
  struct termios ttystate;

  //get the terminal state
  tcgetattr(STDIN_FILENO, &ttystate);

  if (blocking) {
    //turn off canonical mode
    ttystate.c_lflag &= ~ICANON;
    //minimum of number input read.
    ttystate.c_cc[VMIN] = 1;
  } else {
    //turn on canonical mode
    ttystate.c_lflag |= ICANON;
  }
  //set the terminal attributes.
  tcsetattr(STDIN_FILENO, TCSANOW, &ttystate);
}

bool LoadConfiguration(NonMarkovLocalization::LocalizationOptions* options) {
  if (!config_.readFiles()) return false;

  ConfigReader::SubTree c(config_,"NonMarkovLocalization");
  bool error = false;
  error = error || !c.getReal("starting_location.x", kStartingLocation.x());
  error = error || !c.getReal("starting_location.y", kStartingLocation.y());
  error = error || !c.getReal("starting_angle", kStartingAngle);
  error = error || !c.getReal("radial_translation_uncertainty",
                              kRadialTranslationUncertainty);
  error = error || !c.getReal("tangential_translation_uncertainty",
                              kTangentialTranslationUncertainty);
  error = error || !c.getReal("angle_uncertainty", kAngleUncertainty);
  error = error || !c.getReal("odometry_translation_scale",
                              kOdometryTranslationScale);
  error = error || !c.getReal("odometry_rotation_scale",
                              kOdometryRotationScale);
  error = error || !c.getReal("max_odometry_delta_loc",
                              kSqMaxOdometryDeltaLoc);
  kSqMaxOdometryDeltaLoc = sq(kSqMaxOdometryDeltaLoc);
  error = error || !c.getReal("max_odometry_delta_angle",
                              kMaxOdometryDeltaAngle);
  error = error || !c.getReal("min_point_cloud_range", kMinPointCloudRange);
  error = error || !c.getReal("max_point_cloud_range", kMaxPointCloudRange);
  error = error || !c.getReal("max_normal_point_distance",
                              kMaxNormalPointDistance);
#ifdef NDEBUG
  error = error || !c.getInt("num_threads", options->kNumThreads);
#else
  options->kNumThreads = 1;
#endif

  options->kMinRange = kMinPointCloudRange;
  options->kMaxRange = kMaxPointCloudRange;

  error = error || !c.getReal("robot_laser_offset.x",
                              options->sensor_offset.x());
  error = error || !c.getReal("robot_laser_offset.y",
                              options->sensor_offset.y());
  error = error || !c.getReal("min_rotation", options->minimum_node_rotation);
  error = error || !c.getReal("min_translation",
                              options->minimum_node_translation);
  error = error || !c.getInt("max_correspondences_per_point",
                             options->kMaxCorrespondencesPerPoint);
  error = error || !c.getReal("max_point_to_line_distance",
                              options->kMaxPointToLineDistance);
  error = error || !c.getReal("laser_std_dev",
                              options->kLaserStdDev);
  error = error || !c.getReal("map_correlation_factor",
                              options->kPointMapCorrelationFactor);
  error = error || !c.getReal("point_correlation_factor",
                              options->kPointPointCorrelationFactor);
  error = error || !c.getReal("object_correlation_factor",
                              options->kPointObjectCorrelationFactor);
  error = error || !c.getReal("odometry_radial_stddev_rate",
                              options->kOdometryRadialStdDevRate);
  error = error || !c.getReal("odometry_tangential_stddev_rate",
                              options->kOdometryTangentialStdDevRate);
  error = error || !c.getReal("odometry_angular_stddev_rate",
                              options->kOdometryAngularStdDevRate);
  error = error || !c.getReal("odometry_translation_min_stddev",
                              options->kOdometryTranslationMinStdDev);
  error = error || !c.getReal("odometry_translation_max_stddev",
                              options->kOdometryTranslationMaxStdDev);
  error = error || !c.getReal("odometry_rotation_min_stddev",
                              options->kOdometryAngularMinStdDev);
  error = error || !c.getReal("odometry_rotation_max_stddev",
                              options->kOdometryAngularMaxStdDev);
  error = error || !c.getReal("point_match_threshold",
                              options->kPointMatchThreshold);
  error = error || !c.getReal("max_stf_angle_error",
                              options->kMaxStfAngleError);
  error = error || !c.getReal("max_angle_error",
                              options->kMaxAngleError);
  error = error || !c.getInt("pose_increment",
                              options->kPoseIncrement);
  error = error || !c.getInt("max_history",
                             options->kMaxHistory);
  error = error || !c.getReal("map_huber_loss",
                              options->kMapHuberLossThreshold);
  error = error || !c.getInt("max_solver_iterations",
                             options->max_solver_iterations);
  error = error || !c.getInt("max_repeat_iterations",
                             options->kMaxRepeatIterations);
  error = error || !c.getInt("num_repeat_iterations",
                             options->kNumRepeatIterations);
  error = error || !c.getReal("min_ltf_ratio",
                              options->kMinLtfRatio);
  error = error || !c.getUInt("min_episode_length",
                              options->kMinEpisodeLength);
  error = error || !c.getBool("use_object_constraints",
                              options->use_object_constraints);
  error = error || !c.getUInt("min_object_overlap",
                              options->min_object_overlap);
  error = error || !c.getReal("object_match_threshold",
                              options->kObjectMatchThreshold);
  error = error || !c.getBool("use_visibility_constraints",
                              options->use_visibility_constraints);
  error = error || !c.getReal("visibility_correlation_factor",
                              options->kVisibilityCorrelationFactor);
  error = error || !c.getUInt("num_skip_readings",
                             options->num_skip_readings);
  error = error || !c.getReal("max_update_period",
                              options->max_update_period);
  error = error || !relocalization_lidar_params_.LoadFromConfig(&config_);
  const char* map_name_lua = c.getStr("map_name");
  if (map_name_lua != NULL) {
    kMapName = string(map_name_lua);
  } else {
    error = true;
  }
  return !error;
}

void DrawImageLine(const vector2f& p0, const vector2f& p1,
              const vector2f& origin, const float& scale,
              const uint8_t* color,
              cimg_library::CImg<uint8_t>* image) {
  static const int kThickness = 2;
  vector2f p0_transformed = (p0 - origin) / scale;
  vector2f p1_transformed = (p1 - origin) / scale;
  const int height = image->height();
  for (int dx = -kThickness; dx < kThickness; ++dx) {
    for (int dy = -kThickness; dy < kThickness; ++dy) {
      image->draw_line(p0_transformed.x + dx,
                       height - 1 - p0_transformed.y + dy,
                       p1_transformed.x + dx,
                       height - 1 - p1_transformed.y + dy,
                       color);
    }
  }
}

void SaveMap(const char* bag_name,
             const VectorMap& map) {
  static const float kImageResolution = 0.02;
  static const uint8_t kMapColor[] = { 255 };

  const int width = (map.maxX - map.minX) / kImageResolution;
  const int height = (map.maxY - map.minY) / kImageResolution;
  const vector2f g_origin(map.minX, map.minY);
  const Vector2f origin(map.minX, map.minY);

  // Save Observations
  cimg_library::CImg<uint8_t> image(width, height, 1, 1, 0);
  for (size_t i = 0; i < map.Lines().size(); ++i) {
    const line2f& line = map.Line(i);
    DrawImageLine(line.P0(), line.P1(), g_origin,
              kImageResolution, kMapColor, &image);
  }
  image.save("ghc7.png");
}

void SaveResults(const string& bag_name,
                 const vector<Pose2Df>& poses,
                 const vector<PointCloudf>& point_clouds,
                 const vector<NormalCloudf>& normal_clouds,
                 const string& map_name) {
  // Scale of the image in meters per pixel.
  static const float kImageResolution = 0.02;
  static const uint8_t kMapColor[] = { 255 };
  static const uint8_t kPointColor[] = { 255 };

  const VectorMap map(kMapName.c_str(), kMapsDirectory.c_str(), true);
  const int width = (map.maxX - map.minX) / kImageResolution;
  const int height = (map.maxY - map.minY) / kImageResolution;
  const vector2f g_origin(map.minX, map.minY);
  const Vector2f origin(map.minX, map.minY);

  // Save test set reults.
  if (test_set_index_ >=0) {
    const string file_name =
        StringPrintf("non_markov_test_%d.txt", test_set_index_);
    ScopedFile fid(file_name, "a");
    CHECK_NOTNULL(fid());
    for (unsigned int i = 0; i < poses.size(); ++i) {
      fprintf(fid, "%f,%f,%f, ",
              poses[i].translation.x(),
              poses[i].translation.y(),
              poses[i].angle);
    }
    fprintf(fid, "\n");
  }

  // Save Observations
  {
    cimg_library::CImg<uint8_t> image(width, height, 1, 1, 0);
    if (false) {
      for (size_t i = 0; i < map.Lines().size(); ++i) {
        const line2f& line = map.Line(i);
        DrawImageLine(line.P0(), line.P1(), g_origin,
                      kImageResolution, kMapColor, &image);
      }
    }
    for (unsigned int i = 0; i < point_clouds.size() - 1; ++i) {
      const PointCloudf& point_cloud = point_clouds[i];
      const Rotation2Df rotation(poses[i].angle);
      for (unsigned int j = 0; j < point_cloud.size(); ++j) {
        const Vector2f p = (Vector2f(rotation * point_cloud[j] +
            poses[i].translation) - origin) / kImageResolution;
        image.draw_point(p.x(), height - 1 - p.y(), kPointColor);
      }
    }
    image.save((string(bag_name) + string(".")
        + map.mapName + string(".points.png")).c_str());
  }

  // Save Pose locations
  {
    cimg_library::CImg<uint8_t> image(width, height, 1, 1, 0);
    if (false) {
      for (size_t i = 0; i < map.Lines().size(); ++i) {
        const line2f& line = map.Line(i);
        DrawImageLine(line.P0(), line.P1(), g_origin,
                      kImageResolution, kMapColor, &image);
      }
    }
    for (unsigned int i = 0; i < poses.size(); ++i) {
      const Vector2f p = (poses[i].translation - origin) / kImageResolution;
      image.draw_point(p.x(), height - 1 - p.y(), kPointColor);
    }
    image.save((string(bag_name) + string(".") +
        map.mapName + string(".poses.png")).c_str());
  }

  // Save raw data.
  {
    const string data_file = string(bag_name) + string(".") +
        map.mapName + string(".data");
    FILE* fid = fopen(data_file.c_str(), "wb");
    const int num_poses = poses.size();
    // Write all the pose locations.
    fwrite(&num_poses, sizeof(num_poses), 1, fid);
    for (int i = 0; i < num_poses; ++i) {
      fwrite(&(poses[i].translation.x()), sizeof(poses[i].translation.x()),
             1, fid);
      fwrite(&(poses[i].translation.y()), sizeof(poses[i].translation.y()),
             1, fid);
      fwrite(&(poses[i].angle), sizeof(poses[i].angle), 1, fid);
    }
    // Write the observation points.
    fwrite(&num_poses, sizeof(num_poses), 1, fid);
    for (int i = 0; i < num_poses; ++i) {
      const int num_points = point_clouds[i].size();
      fwrite(&num_points, sizeof(num_points), 1, fid);
      const PointCloudf& point_cloud = point_clouds[i];
      for (int j = 0; j < num_points; ++j) {
        fwrite(&(point_cloud[j].x()), sizeof(point_cloud[j].x()), 1, fid);
        fwrite(&(point_cloud[j].y()), sizeof(point_cloud[j].y()), 1, fid);
      }
    }
    fclose(fid);
  }
}

void AlignObjects(vector<vector<KDNodeValue2f> >* clusters_ptr,
                  vector<PointCloudf>* objects_ptr,
                  vector<Vector2f>* object_locs_ptr) {
  static const bool kSubSample = false;
  static const float kMinPointSeparation = 0.1;
  vector<vector<KDNodeValue2f> >& clusters = *clusters_ptr;
  Eigen::SelfAdjointEigenSolver<Matrix2d> solver;
  vector<PointCloudf>& objects = *objects_ptr;
  vector<Vector2f>& object_locs = *object_locs_ptr;
  objects.clear();
  for (size_t i = 0; i < clusters.size(); ++i) {
    Vector2d centroid(0.0, 0.0);
    if (kSubSample) {
      for (size_t j = 0; j < clusters[i].size(); ++j) {
        const Vector2f& p1 = clusters[i][j].point;
        for (size_t k = j + 1; k < clusters[i].size(); ++k) {
          if ((clusters[i][k].point - p1).norm() < kMinPointSeparation) {
            clusters[i].erase(clusters[i].begin() + k);
            --k;
          }
        }
      }
    }
    const size_t num_points = clusters[i].size();
    for (size_t j = 0; j < num_points; ++j) {
      centroid += clusters[i][j].point.cast<double>();
    }
    centroid = centroid / static_cast<double>(num_points);
    PointCloudf object(num_points);
    Matrix2d scatter_matrix = Matrix2d::Zero();
    for (size_t j = 0; j < num_points; ++j) {
      const Vector2d p = clusters[i][j].point.cast<double>() - centroid;
      object[j] = p.cast<float>();
      scatter_matrix += (p * (p.transpose()));
    }
    solver.compute(scatter_matrix);
    Matrix2d eigen_vectors = solver.eigenvectors();
    Vector2d eigen_values = solver.eigenvalues();
    const int major_index = (eigen_values(0) >= eigen_values(1)) ? 0 : 1;
    const int minor_index = 1 - major_index;
    const Vector2d b0 =
        Vector2d(eigen_vectors(0, major_index),
                 eigen_vectors(1, major_index)).normalized();
    const Vector2d b1 =
        Vector2d(eigen_vectors(0, minor_index),
                 eigen_vectors(1, minor_index)).normalized();
    Matrix2f rotation_matrix;
    rotation_matrix << b0.x(), b0.y(), b1.x(), b1.y();
    for (size_t j = 0; j < num_points; ++j) {
      object[j] = rotation_matrix * object[j];
    }
    objects.push_back(object);
    object_locs.push_back(centroid.cast<float>());
  }
}

void SaveStfs(
    const string& map_name,
    const vector<Pose2Df>& poses,
    const vector<PointCloudf>& point_clouds,
    const vector<NormalCloudf>& normal_clouds,
    const vector<vector<NonMarkovLocalization::ObservationType> >&
        classifications,
    const string& bag_file,
    double timestamp) {
  static const bool kSaveAllObservations = true;
  static const bool kDisplaySteps = false;
  static const float kMaxMapLineOffset = 0.2;
  const string stfs_file = bag_file + ".stfs";
  ScopedFile fid(stfs_file, "w");
  fprintf(fid(), "%s\n", map_name.c_str());
  fprintf(fid(), "%lf\n", timestamp);
  VectorMap map(map_name, kMapsDirectory, true);
  if (kDisplaySteps) {
    ClearDrawingMessage(&display_message_);
    nonblock(true);
  }
  for (size_t i = 0; i < point_clouds.size(); ++i) {
    const Rotation2Df pose_rotation(poses[i].angle);
    const Affine2f pose_transform =
        Translation2f(poses[i].translation) * pose_rotation;
    const vector<int>& visibility_list = *map.getVisibilityList(
        poses[i].translation.x(), poses[i].translation.y());
    for (unsigned int j = 0; j < point_clouds[i].size(); ++j) {
      const Vector2f p = pose_transform * point_clouds[i][j];
      if (kDisplaySteps) {
        DrawLine(p, poses[i].translation, 0x1FC0C0C0, &display_message_);
      }
      if (!kSaveAllObservations &&
          classifications[i][j] != NonMarkovLocalization::kStfObservation) {
        if (kDisplaySteps) {
          DrawPoint(p, kLtfPointColor, &display_message_);
        }
        continue;
      }
      if (!kSaveAllObservations) {
        const vector2f p_g(p.x(), p.y());
        bool map_match = false;
        for (size_t l = 0; !map_match && l < visibility_list.size(); ++l) {
          const float line_offset =
              map.Line(visibility_list[l]).closestDistFromLine(p_g, false);
          const bool along_line =
              (p_g - map.Line(visibility_list[l]).P0()).dot(
              p_g - map.Line(visibility_list[l]).P1()) < 0.0;
          map_match = along_line && line_offset < kMaxMapLineOffset;
        }
        if (map_match) {
          if (kDisplaySteps) {
            DrawPoint(p, kLtfPointColor, &display_message_);
          }
          continue;
        }
      }
      const Vector2f n = pose_rotation * normal_clouds[i][j];
      fprintf(fid(), "%.4f,%.4f,%.4f, %.4f,%.4f, %.4f,%.4f\n",
          poses[i].translation.x(), poses[i].translation.y(),
          poses[i].angle, p.x(), p.y(), n.x(), n.y());
      if (kDisplaySteps) {
        DrawPoint(p, kStfPointColor, &display_message_);
      }
    }
    if (kDisplaySteps) {
      PublishLocation(kMapName, poses[i].translation.x(),
                      poses[i].translation.y(), poses[i].angle);
      PublishDisplay();
      while (kbhit() == 0) {
        Sleep(0.02);
      }
      fgetc(stdin);
      printf("\r");
      fflush(stdout);
      const string file_name = StringPrintf(
          "poses/%04d.png", static_cast<int>(i));
      cobot_msgs::LocalizationGuiCaptureSrv::Request req;
      cobot_msgs::LocalizationGuiCaptureSrv::Response res;
      req.filename = string(file_name);
      printf("Saving image to %s\n", req.filename.c_str());
      gui_capture_client.call(req, res);
    }
  }
  if (kDisplaySteps) {
    PublishDisplay();
  }
}

void DisplayPoses(const vector<Pose2Df>& poses,
                  const vector<PointCloudf>& point_clouds,
                  const vector<NormalCloudf>& normal_clouds,
                  const vector<vector<NonMarkovLocalization::ObservationType> >&
                      classifications,
                  const char* save_image_file) {
  if (debug_level_ < 0) return;
  static const int kSkipLaserScans = 1;
  CHECK_EQ(poses.size(), point_clouds.size());
  CHECK_GT(poses.size(), 0);

  // Check if the observation classifications are valid.
  bool classifications_valid = (classifications.size() == point_clouds.size());
  for (size_t i = 0; i + 1 < poses.size(); ++i) {
    classifications_valid = classifications_valid &&
        (classifications[i].size() == point_clouds[i].size());
  }

  ClearDrawingMessage(&display_message_);
  // Add robot localization trajectory.
  for (size_t i = 0; i + 1 < poses.size(); ++i) {
    DrawLine(poses[i].translation, poses[i + 1].translation, kTrajectoryColor,
             &display_message_);
  }

  // Add laser scans.
  for (size_t i = 0; i < point_clouds.size(); i += kSkipLaserScans) {
    const PointCloudf& point_cloud = point_clouds[i];
    const Rotation2Df rotation(poses[i].angle);
    // const Vector2f& pose_location = poses[i].translation;
    const Vector2f pose_location = poses[i].translation;
    for (unsigned int j = 0; j < point_cloud.size(); ++j) {
      const Vector2f p(rotation * point_cloud[j] + pose_location);
      uint32_t color = kLtfPointColor;
      if (classifications_valid) {
        switch (classifications[i][j]) {
          case NonMarkovLocalization::kLtfObservation : {
            color = kLtfPointColor;
          } break;

          case NonMarkovLocalization::kStfObservation : {
            color = kStfPointColor;
          } break;

          case NonMarkovLocalization::kDfObservation : {
            color = kDfPointColor;
          } break;

          case NonMarkovLocalization::kObjectObservation : {
            color = kObjectColor;
          } break;
        }
      }
      DrawPoint(p, color, &display_message_);
      // DrawLine(p, pose_location, 0x3Fc0c0c0, &display_message_);
    }
  }
  if (save_image_file != NULL) {
    const string filename = string(save_image_file) + ".bag";
    rosbag::Bag bag(filename, rosbag::bagmode::Write);
    bag.write("/Cobot/VectorLocalization/Gui",
              ros::Time::now(),
              display_message_);
    bag.close();
  }
  PublishDisplay();
}

void SaveEpisodeStats(FILE* fid) {
  vector<Pose2Df> poses;
  vector<PointCloudf> point_clouds;
  vector<Pose2Df> pending_poses;
  vector<uint64_t> pose_ids;
  vector<vector<NonMarkovLocalization::ObservationType> > observation_classes;
  Pose2Df latest_pose;
  if (!localization_.GetNodeData(
          &poses,
          &pose_ids,
          &point_clouds,
          &pending_poses,
          &observation_classes,
          &latest_pose) ||
      poses.size() < 1 ||
      poses.size() != observation_classes.size()
     ) {
    return;
  }
  const string map_name = localization_.GetCurrentMapName();
  int num_ltfs = 0;
  int num_stfs = 0;
  int num_dfs = 0;
  for (size_t i = 0; i < point_clouds.size(); ++i) {
    const PointCloudf& point_cloud = point_clouds[i];
    for (unsigned int j = 0; j < point_cloud.size(); ++j) {
      switch (observation_classes[i][j]) {
        case NonMarkovLocalization::kLtfObservation : {
          ++num_ltfs;
        } break;
        case NonMarkovLocalization::kStfObservation : {
          ++num_stfs;
        } break;
        case NonMarkovLocalization::kDfObservation : {
          ++num_dfs;
        } break;
        default : {
        } break;
      }
    }
  }
  const int episode_length = static_cast<int>(point_clouds.size());
  fprintf(fid,
          "%s, %f, %f, %f, %f, %f, %f, %d, %d, %d, %d\n",
          map_name.c_str(),
          poses.back().translation.x(),
          poses.back().translation.y(),
          poses.back().angle,
          poses.front().translation.x(),
          poses.front().translation.y(),
          poses.front().angle,
          episode_length,
          num_ltfs,
          num_stfs,
          num_dfs);
}

void DisplayDebug() {
  static const bool debug = false;
  static Pose2Df last_pose_(0, 0, 0);
  static double t_last = GetTimeSec();
  vector<Pose2Df> poses;
  vector<uint64_t> pose_ids;
  vector<PointCloudf> point_clouds;
  vector<Pose2Df> pending_poses;
  vector<vector<NonMarkovLocalization::ObservationType> > observation_classes;

  Pose2Df latest_pose;
  if (!localization_.GetNodeData(
      &poses,
      &pose_ids,
      &point_clouds,
      &pending_poses,
      &observation_classes,
      &latest_pose)) {
    // Unable to get node data at this time: that's okay, do nothing.
    if (debug || debug_level_ > 1) {
      printf("---- nodes, --- pending nodes\n");
    }
    return;
  }
  // There's probably nothing new to display if the latest pose MLE is unchaged.
  if (latest_pose.translation == last_pose_.translation) {
    return;
  }
  last_pose_ = latest_pose;

  if (point_clouds.size() != observation_classes.size()) {
    printf("EnML Error: point_clouds.size() != observation_classes.size(), "
           "%d vs. %d\n",
           static_cast<int>(point_clouds.size()),
           static_cast<int>(observation_classes.size()));
    return;
  }
  if (debug_level_ < 1 && GetTimeSec() < t_last + 0.5) {
    return;
  }
  t_last = GetTimeSec();
  const int skip_scans = (debug_level_ < 1) ? 8 : 2;
  ClearDrawingMessage(&display_message_);
  if (debug || debug_level_ > 1) {
    printf("%4lu nodes, %3lu pending nodes\n",
           poses.size(), pending_poses.size());
  }
  // Add robot localization trajectory.
  for (size_t i = 0; i + 1 < poses.size(); ++i) {
    DrawLine(poses[i].translation, poses[i + 1].translation, kTrajectoryColor,
             &display_message_);
  }

  // Add laser scans.
  for (size_t i = 0; i + 1 < point_clouds.size(); i += skip_scans) {
    const PointCloudf& point_cloud = point_clouds[i];
    const Rotation2Df rotation(poses[i].angle);
    const Vector2f& translation = poses[i].translation;
    for (unsigned int j = 0; j < point_cloud.size(); ++j) {
      const Vector2f p(rotation * point_cloud[j] + translation);
      uint32_t color = kTrajectoryColor;
      switch (observation_classes[i][j]) {
        case NonMarkovLocalization::kLtfObservation : {
          color = kLtfPointColor;
        } break;
        case NonMarkovLocalization::kStfObservation : {
          color = kStfPointColor;
        } break;
        case NonMarkovLocalization::kDfObservation : {
          color = kDfPointColor;
        } break;
        default : {
        } break;
      }
      DrawPoint(p, color, &display_message_);
    }
  }

  // Add pending poses.
  Pose2Df pose = localization_.GetLastMLEPose();
  for (size_t i = 0; i < pending_poses.size(); ++i) {
    pose.ApplyPose(pending_poses[i]);
    const Vector2f p0 = pose.translation;
    const Vector2f p1 = pose.translation +
        Rotation2Df(pose.angle) * Vector2f(0.3, 0.0);
    //DrawLine(p0, p1, 0x7FC0C0C0, &display_message_);
    DrawLine(p0, p1, 0x7F0000FF, &display_message_);
    DrawPoint(p0, 0xFFFF0000, &display_message_);
  }
  display_publisher_.publish(display_message_);
}

void GetCovarianceFromRelativePose(const Vector2f& relative_location,
                                   const float& relative_angle,
                                   Matrix3f* covariance) {
  covariance->setZero();
  if (relative_location.norm() > FLT_MIN) {
    const Vector2f radial_direction(relative_location.normalized());
    const Vector2f tangential_direction(Rotation2Df(M_PI_2) * radial_direction);
    Matrix2f eigenvectors;
    eigenvectors.leftCols<1>() = radial_direction;
    eigenvectors.rightCols<1>() = tangential_direction;
    Matrix2f eigenvalues;
    eigenvalues.setZero();
    eigenvalues(0, 0) = kRadialTranslationUncertainty;
    eigenvalues(1, 1) = kTangentialTranslationUncertainty;
    covariance->block<2, 2>(0, 0) =
        eigenvectors * eigenvalues * (eigenvectors.transpose());
  }
  (*covariance)(2, 2) = kAngleUncertainty * fabs(relative_angle);
}

bool AddPose(const sensor_msgs::LaserScanPtr& laser_message,
             const vector<double>& keyframes,
             Vector2f* relative_location_ptr,
             float* relative_angle_ptr,
             Vector2f* global_location_ptr,
             float* global_angle_ptr,
             vector<PointCloudf>* point_clouds,
             vector<NormalCloudf>* normal_clouds,
             vector<Pose2Df>* poses,
              vector<double>* timestamps,
             Vector2f* odometry_location,
             float* odometry_angle) {
  static const bool debug = false;
  // Aliasing dereferenced pointers for readability.
  float& relative_angle = *relative_angle_ptr;
  Vector2f& relative_location = *relative_location_ptr;
  float& global_angle = *global_angle_ptr;
  Vector2f& global_location = *global_location_ptr;
  const bool keyframe = binary_search(keyframes.begin(), keyframes.end(),
                                      laser_message->header.stamp.toSec());
  if (!keyframe && relative_location.norm() <
      localization_options_.minimum_node_translation &&
      fabs(relative_angle) < localization_options_.minimum_node_rotation) {
    // Ignore this pose since the robot has not moved since the last update.
    return false;
  }
  if (debug) {
    printf("Relative pose: %6.3f, %6.3f %4.1f\u00b0\n", relative_location.x(),
        relative_location.y(), DEG(relative_angle));
  }
  // Update global pose
  global_location = Rotation2Df(global_angle) * relative_location +
      global_location;
  global_angle = relative_angle + global_angle;

  // Update odometry pose
  *odometry_location = Rotation2Df(*odometry_angle) * relative_location +
      *odometry_location;
  *odometry_angle = relative_angle + *odometry_angle;

  // Compute global pose and covariance.
  Pose2Df pose;
  pose.translation = global_location;
  pose.angle = global_angle;

  // Add laser scan to list of entries.
  PointCloudf point_cloud;
  for (unsigned int i = 0; i < laser_message->ranges.size(); ++i) {
    if (laser_message->ranges[i] < kMinPointCloudRange ||
        laser_message->ranges[i] > kMaxPointCloudRange ||
        !std::isfinite(laser_message->ranges[i])) continue;
    const float angle = laser_message->angle_min +
        laser_message->angle_increment * static_cast<float>(i);
    if (angle < laser_message->angle_min + RAD(15.0) ||
        angle > laser_message->angle_max - RAD(15.0)) continue;
    float range = laser_message->ranges[i];
    if (use_laser_corrections_) {
      const int angle_index =
          floor((angle + M_PI) / laser_corrections_resolution_);
      CHECK_GE(angle_index, 0);
      CHECK_LT(angle_index, static_cast<int>(laser_corrections_.size()));
      range = range * laser_corrections_[angle_index];
    }
    point_cloud.push_back(localization_options_.sensor_offset +
        Rotation2Df(angle) * Vector2f(range, 0.0));
  }

  NormalCloudf normal_cloud;
  GenerateNormals(kMaxNormalPointDistance, &point_cloud, &normal_cloud);
  if (point_cloud.size() > 1 || 1) {
    point_clouds->push_back(point_cloud);
    normal_clouds->push_back(normal_cloud);
    poses->push_back(pose);
    timestamps->push_back(laser_message->header.stamp.toSec());
    // Reset odometry accumulation.
    relative_angle = 0.0;
    relative_location = Vector2f(0.0, 0.0);
  }
  return true;
}

bool LoadLaserMessage(const rosbag::MessageInstance& message,
                      const vector<double>& keyframes,
                      Vector2f* relative_location,
                      float* relative_angle,
                      Vector2f* global_location,
                      float* global_angle,
                      vector<PointCloudf>* point_clouds,
                      vector<NormalCloudf>* normal_clouds,
                      vector<Pose2Df>* poses,
                      vector<double>* timestamps,
                      Vector2f* odometry_location,
                      float* odometry_angle) {
  sensor_msgs::LaserScanPtr laser_message =
      message.instantiate<sensor_msgs::LaserScan>();
  const bool is_cobot_laser = message.getTopic() == kCobotLaserTopic;
  const bool is_cobot_kinect = message.getTopic() == kKinectScanTopic;
  const bool is_standard_laser =
      kStandardizedData && message.getTopic() == kStandardLaserTopic;
  if (laser_message != NULL && (((!use_kinect_ && is_cobot_laser) ||
      (use_kinect_ && is_cobot_kinect)) || is_standard_laser)) {
    if (false && debug_level_ > 1) {
      printf("Laser Msg,    t:%.2f\n", message.getTime().toSec());
      fflush(stdout);
    }
    AddPose(laser_message, keyframes, relative_location, relative_angle,
            global_location, global_angle, point_clouds, normal_clouds,
            poses, timestamps, odometry_location, odometry_angle);
    return true;
  }
  return false;
}

bool LoadOdometryMessage(const rosbag::MessageInstance& message,
                         const Vector2f& odometry_location,
                         const float& odometry_angle,
                         Vector2f* relative_location,
                         float* relative_angle) {
  if (kStandardizedData) {
    nav_msgs::OdometryPtr odometry_message =
        message.instantiate<nav_msgs::Odometry>();
    const string topic_name = message.getTopic();
    if (odometry_message != NULL &&
        message.getTopic() == kStandardOdometryTopic) {
      if (false && debug_level_ > 1) {
        printf("Odometry Msg, t:%.2f\n", message.getTime().toSec());
        fflush(stdout);
      }
      const Vector2f odometry_message_location(
          odometry_message->pose.pose.position.x,
          odometry_message->pose.pose.position.y);
      *relative_location =  kOdometryTranslationScale * (
          Rotation2Df(-odometry_angle) *
          (odometry_message_location - odometry_location));
      const float odometry_message_angle =
          2.0 * atan2(odometry_message->pose.pose.orientation.z,
                      odometry_message->pose.pose.orientation.w);
      *relative_angle = kOdometryRotationScale *
          angle_diff(odometry_message_angle , odometry_angle);
      if (test_set_index_ >= 0 || statistical_test_index_ >= 0) {
        relative_location->x() +=
            randn(odometry_additive_noise_ * relative_location->x());
        relative_location->y() +=
            randn(odometry_additive_noise_ * relative_location->y());
        (*relative_angle) +=
            randn(odometry_additive_noise_ * (*relative_angle));
      }
      return true;
    }
  } else {
    cobot_msgs::CobotOdometryMsgPtr odometry_message =
        message.instantiate<cobot_msgs::CobotOdometryMsg>();
    const string topic_name = message.getTopic();
    if (odometry_message != NULL && message.getTopic() == kCobotOdometryTopic) {
      if (false && debug_level_ > 1) {
        printf("Odometry Msg, t:%.2f\n", message.getTime().toSec());
        fflush(stdout);
      }
      if (test_set_index_ >= 0 || statistical_test_index_ >= 0) {
        odometry_message->dx +=
            randn(odometry_additive_noise_ * odometry_message->dx);
        odometry_message->dy +=
            randn(odometry_additive_noise_ * odometry_message->dy);
        odometry_message->dr +=
            randn(odometry_additive_noise_ * odometry_message->dr);
      }
      // Accumulate odometry to update robot pose estimate.
      const Vector2f delta(odometry_message->dx, odometry_message->dy);
      // printf("Delta: %f, %f\n", odometry_message->dx, odometry_message->dy);
      *relative_location = *relative_location +
          kOdometryTranslationScale * (Rotation2Df(*relative_angle) *
          delta);
      *relative_angle = *relative_angle +
          kOdometryRotationScale *  odometry_message->dr;
          //- angle_mod(fabs(odometry_message->dr)) / RAD(5.0) * RAD(1.0);
      return true;
    }
  }
  return false;
}

bool LoadSetLocationMessage(const rosbag::MessageInstance& message,
                            Vector2f* global_location,
                            float* global_angle,
                            string* map_name) {
  cobot_msgs::LocalizationMsgPtr set_location_message =
      message.instantiate<cobot_msgs::LocalizationMsg>();
  const string topic_name = message.getTopic();
  if (set_location_message != NULL &&
      message.getTopic() == kStandardSetLocationTopic) {
    if (debug_level_ > 1) {
      printf("Set Location, t:%.2f\n", message.getTime().toSec());
      fflush(stdout);
    }
    *global_angle = set_location_message->angle;
    global_location->x() = set_location_message->location.x;
    global_location->y() = set_location_message->location.y;
    *map_name = set_location_message->map_name;
    return true;
  }
  return false;
}

void LoadLaserCorrections() {
  static const char kCorrectionsFile[] =
      "results/sensor_corrections.txt";
  if (!FileExists(kCorrectionsFile)) return;
  ScopedFile fid(kCorrectionsFile, "r");
  if (fid() == NULL) {
    printf("No laser corrections found.\n");
    return;
  }
  int num_bins = 0;
  if (fscanf(fid(), "%d,0\n", &num_bins) != 1) {
    printf("Error parsing laser corrections file!\n");
    return;
  }
  laser_corrections_.resize(num_bins, 1.0);
  laser_corrections_resolution_ = 2.0 * M_PI / static_cast<float>(num_bins);
  bool error = false;
  for (int i = 0; !error && i < num_bins; ++i) {
    float angle = 0.0;
    error = (fscanf(fid(), "%f,%f\n", &angle, &(laser_corrections_[i])) != 2);
    laser_corrections_[i] += 1.0;
  }
  if (error) {
    printf("Error parsing laser corrections file!\n");
    return;
  }
  printf("Loaded %d laser corrections\n", num_bins);
  use_laser_corrections_ = true;
}

void LoadRosBag(const string& bagName, int max_laser_poses, double time_skip,
                const vector<double>& keyframes,
                vector<PointCloudf>* point_clouds,
                vector<NormalCloudf>* normal_clouds,
                vector<Pose2Df>* poses,
                vector<double>* timestamps,
                double* t_duration,
                string* map_name) {
  CHECK_NOTNULL(point_clouds);
  CHECK_NOTNULL(normal_clouds);
  CHECK_NOTNULL(poses);
  // Last angle as determined by integration of robot odometry from an arbitrary
  // starting location.
  float odometry_angle(0.0);
  // Last location as determined by integration of robot odometry from an
  // arbitrary starting location.
  Vector2f odometry_location(0.0, 0.0);
  // Last angle as determined by integration of robot odometry since location of
  // last pose update.
  float relative_angle(0.0);
  // Last location as determined by integration of robot odometry since location
  // of last pose update.
  Vector2f relative_location(0.0, 0.0);
  // Estimated robot global angle as determined by integration of robot odometry
  // from set starting location.
  float global_angle(kStartingAngle);
  // Estimated robot global location as determined by integration of robot
  // odometry from set starting location.
  Vector2f global_location(kStartingLocation);
  rosbag::Bag bag;
  printf("Opening bag file %s...", bagName.c_str()); fflush(stdout);
  bag.open(bagName,rosbag::bagmode::Read);
  printf(" Done.\n"); fflush(stdout);

  std::vector<std::string> topics;
  if (kStandardizedData) {
    printf("Warning: Interpreting bag file as standardized data.\n");
    topics.push_back(kStandardLaserTopic);
    topics.push_back(kStandardOdometryTopic);
    topics.push_back(kStandardSetLocationTopic);
  } else {
    topics.push_back(kCobotOdometryTopic);
    if (use_kinect_) {
      topics.push_back(kKinectScanTopic);
    } else {
      topics.push_back(kCobotLaserTopic);
    }
  }

  bool localization_initialized = false;
  printf("Reading bag file..."); fflush(stdout);
  if (false && debug_level_ > 1) printf("\n");
  rosbag::View view(bag, rosbag::TopicQuery(topics));
  double bag_time_start = -1.0;
  double bag_time = 0.0;
  for (rosbag::View::iterator it = view.begin();
       run_ && it != view.end() &&
       (max_laser_poses < 0 ||
       static_cast<int>(poses->size()) < max_laser_poses); ++it) {
    const rosbag::MessageInstance &message = *it;
    bag_time = message.getTime().toSec();
    if (bag_time_start < 0.0) {
      // Initialize bag starting time.
      bag_time_start = bag_time;
    }

    // Ignore messages before elapsed time_skip.
    if (!kStandardizedData && bag_time < bag_time_start + time_skip) continue;

    if (kStandardizedData && !localization_initialized) {
      if (LoadSetLocationMessage(message, &global_location,
                                 &global_angle, map_name)) {
        localization_initialized = true;
      }
      // If this message was a set_location message, we don't need to process
      // anything else. If it wasn't, that means localization is still not
      // initialized; so no point processing this message any further.
      continue;
    }

    // Check to see if this is a laser scan message.
    if (LoadLaserMessage(message, keyframes, &relative_location,
        &relative_angle, &global_location, &global_angle, point_clouds,
        normal_clouds, poses, timestamps, &odometry_location,
        &odometry_angle)) {
      continue;
    }

    // Check to see if this is an odometry message.
    if (LoadOdometryMessage(message, odometry_location, odometry_angle,
      &relative_location, &relative_angle)) {
      continue;
    }
  }
  *t_duration = bag_time - bag_time_start;
  if (false && debug_level_ > 1) printf("\n");
  printf(" Done.\n"); fflush(stdout);
  printf("%d poses loaded.\n", static_cast<int>(point_clouds->size()));
}

void DrawVisibilityConstraints(
    const vector<vector_localization::VisibilityGlobConstraint*>&
        visibility_constraints,
    const vector<double>& poses) {
  for (size_t i = 0; i < visibility_constraints.size(); ++i) {
    const vector_localization::VisibilityGlobConstraint& constraint =
        *visibility_constraints[i];
    const size_t& pose_index = constraint.pose_index;
    const Vector2f pose_translation(
        poses[3 * pose_index], poses[3 * pose_index + 1]);
    const Affine2f pose_transform =
        Translation2f(poses[3 * pose_index], poses[3 * pose_index + 1]) *
        Rotation2Df(poses[3 * pose_index + 2]);
    for (size_t j = 0; j < constraint.points.size(); ++j) {
      const Vector2f point_global = pose_transform * constraint.points[j];
      // Compute the line offset error from the constraint.
      const float point_offset =
          constraint.line_normals[j].dot(point_global - constraint.line_p1s[j]);
      const Vector2f point_anchor =
          point_global - constraint.line_normals[j] * point_offset;
      DrawLine(point_global, point_anchor, 0xFFFF0000, &display_message_);
      if (true) {
        const float alpha = bound(fabs(point_offset) / 5.0, 0.0, 0.2);
        const uint32_t colour =
            (static_cast<uint32_t>(255.0 * alpha) << 24) | 0xFF0000;
        DrawLine(point_global, pose_translation, colour, &display_message_);
      }
    }
  }
}

void DrawStfs(
    const vector<NonMarkovLocalization::PointToPointGlobCorrespondence>&
    point_point_correspondences, const vector<double>& poses,
    const vector<vector< vector2f> >& point_clouds,
    const std::vector< NormalCloudf >& normal_clouds) {
  for (size_t i = 0; i < point_point_correspondences.size(); ++i) {
    const int pose_index0 = point_point_correspondences[i].pose_index0;
    const int pose_index1 = point_point_correspondences[i].pose_index1;
    CHECK_EQ(point_point_correspondences[i].points0.size(),
             point_point_correspondences[i].points1.size());
    const Affine2f pose_tf0 =
        Translation2f(poses[3 * pose_index0 + 0], poses[3 * pose_index0 + 1]) *
        Rotation2Df(poses[3 * pose_index0 + 2]);
    const Affine2f pose_tf1 =
        Translation2f(poses[3 * pose_index1 + 0], poses[3 * pose_index1 + 1]) *
        Rotation2Df(poses[3 * pose_index1 + 2]);
    for (size_t j = 0; j < point_point_correspondences[i].points0.size(); ++j) {
      const Vector2f p0 = pose_tf0 * point_point_correspondences[i].points0[j];
      const Vector2f p1 = pose_tf1 * point_point_correspondences[i].points1[j];
      DrawLine(p0, p1, kStfCorrespondenceColor, &display_message_);
    }
  }
}


void DrawLtfs(
    const size_t start_pose, const size_t end_pose,
    const vector<double>& poses, const vector<vector< vector2f> >& point_clouds,
    const vector<vector<NonMarkovLocalization::ObservationType> >&
        classifications,
    const vector<vector<line2f> >& ray_cast_lines,
    const vector<vector<int> >& point_line_correspondences) {
  for (size_t i = start_pose; i <= end_pose; ++i) {
    const vector2f pose_location(poses[3 * i + 0], poses[3 * i + 1]);
    const float pose_angle = poses[3 * i + 2];
    for (size_t j = 0; j < point_clouds[i].size(); ++j) {
      if (classifications[i][j] != NonMarkovLocalization::kLtfObservation) {
        continue;
      }
      const vector2f& point =
          point_clouds[i][j].rotate(pose_angle) + pose_location;
      const int correspondence = point_line_correspondences[i][j];
      const line2f& line = ray_cast_lines[i][correspondence];
      const vector2f point_projected = line.point_projection(point);
      DrawLine(point, point_projected, kLtfCorrespondenceColor,
              &display_message_);
    }
  }
}

void DrawObservations(
    const size_t start_pose, const size_t end_pose,
    const vector<double>& poses,
    const vector<vector< vector2f> >& point_clouds,
    const std::vector< NormalCloudf >& normal_clouds,
    const vector<vector<NonMarkovLocalization::ObservationType> >&
        classifications) {
  static const bool kDisplayTangents = false;
  for (size_t i = start_pose; i <= end_pose; ++i) {
    const vector<vector2f> &point_cloud = point_clouds[i];
    const vector<Vector2f> &normal_cloud = normal_clouds[i];
    const vector2f pose_location(poses[3 * i + 0], poses[3 * i + 1]);
    const float pose_angle = poses[3 * i + 2];
    const Rotation2Df pose_rotation(pose_angle);
    for (size_t j = 0; j < point_cloud.size(); ++j) {
      const vector2f& point = point_cloud[j].rotate(pose_angle) + pose_location;
      uint32_t point_color = 0xF0C0C0C0;
      switch (classifications[i][j]) {
        case NonMarkovLocalization::kLtfObservation : {
          point_color = kLtfPointColor;
        } break;
        case NonMarkovLocalization::kStfObservation : {
          // DrawLine(point, pose_location, 0x1F994CD9, &display_message_);
          point_color = kStfPointColor;
        } break;
        case NonMarkovLocalization::kObjectObservation : {
          point_color = kObjectColor;
        } break;
        case NonMarkovLocalization::kDfObservation : {
          point_color = kDfPointColor;
          if (i == end_pose) continue;
        } break;
      }
      if (kDisplayTangents) {
        const Vector2f normal_e = pose_rotation * normal_cloud[j];
        const vector2f normal(normal_e.x(), normal_e.y());
        const vector2f tangent = 0.05 * normal.perp();
        DrawLine(
            point + tangent, point - tangent, point_color, &display_message_);
      }
      DrawPoint(point, point_color, &display_message_);
    }
  }
}

void DrawObjects(
    const vector<PersistentObject>& objects,
    const vector<NonMarkovLocalization::ObjectCorrespondence>&
        object_correspondences,
    const vector<double>& object_poses) {
  const uint32_t kColour = 0xFFFF0000;
  for (size_t i = 0; i < object_correspondences.size(); ++i) {
    const size_t& model_id = object_correspondences[i].object_model;
    const PersistentObject& model = objects[model_id];
    const Affine2f object_transform =
        Translation2f(object_poses[3 * model_id + 0],
                      object_poses[3 * model_id + 1]) *
        Rotation2Df(object_poses[3 * model_id + 2]);
    // Build a visualization of the model
    for (unsigned int x = 0; x < model.image_width; ++x) {
      for (unsigned int y = 0; y < model.image_height; ++y) {
        if (model.occupancy_image(x, y) > 0.4) {
          const Vector2f p =
              object_transform * (model.image_resolution * Vector2f(x, y));
          DrawPoint(p, kColour, &display_message_);
        }
      }
    }
  }
}

void DrawGradients(
    const size_t start_pose, const size_t end_pose,
    const vector<double>& gradients,
    const vector<double>& poses) {
  if (gradients.size() != poses.size()) return;
  for (size_t i = start_pose, j = 3 * start_pose;
       i <= end_pose; ++i, j += 3) {
    const Vector2f location_gradient(gradients[j], gradients[j + 1]);
    const Vector2f pose_location(poses[j], poses[j + 1]);
    const Rotation2Df pose_rotation(poses[j + 2]);
    const Vector2f p2 = pose_location - location_gradient;
    DrawLine(pose_location, p2, 0xFF0000FF, &display_message_);
  }
}

void DrawPoseCovariance(const Vector2f& pose, const Matrix2f& covariance) {
  static const float kDTheta = RAD(15.0);
  Eigen::SelfAdjointEigenSolver<Matrix2f> solver;
  solver.compute(covariance);
  const Matrix2f eigenvectors = solver.eigenvectors();
  const Vector2f eigenvalues = solver.eigenvalues();
  for (float a = 0; a < 2.0 * M_PI; a += kDTheta) {
    const Vector2f v1(cos(a) * sqrt(eigenvalues(0)),
                      sin(a) * sqrt(eigenvalues(1)));
    const Vector2f v2(cos(a + kDTheta) * sqrt(eigenvalues(0)),
                      sin(a + kDTheta) * sqrt(eigenvalues(1)));
    const Vector2f v1_global = eigenvectors.transpose() * v1 + pose;
    const Vector2f v2_global = eigenvectors.transpose() * v2 + pose;
    DrawLine(v1_global, v2_global, kPoseCovarianceColor, &display_message_);
  }
}

void DrawPoses(const size_t start_pose, const size_t end_pose,
    const vector<Pose2Df>& odometry_poses, const vector<double>& poses,
    const vector<Matrix2f>& covariances) {
  static const bool kDrawCovariances = false;
  Vector2f pose_location_last(0.0, 0.0);
  float pose_angle_last = 0.0;
  const bool valid_covariances = (covariances.size() > end_pose);
  for (size_t i = start_pose; i <= end_pose; ++i) {
    const Vector2f pose_location(poses[3 * i + 0], poses[3 * i + 1]);
    if (i > start_pose) {
      DrawLine(pose_location, pose_location_last, kTrajectoryColor,
               &display_message_);
      const Vector2f odometry =
          Rotation2Df(pose_angle_last) *
          Rotation2Df(-odometry_poses[i - 1].angle) *
          (odometry_poses[i].translation - odometry_poses[i - 1].translation);
      DrawLine(pose_location, Vector2f(pose_location_last + odometry),
               kOdometryColor, &display_message_);
      if (kDrawCovariances && valid_covariances) {
        DrawPoseCovariance(pose_location, covariances[i]);
      }
    }
    pose_location_last = pose_location;
    pose_angle_last = poses[3 * i + 2];
  }
}

void DrawRayCasts(
    const size_t start_pose, const size_t end_pose,
    const vector<vector<line2f> >& ray_cast_lines,
    const vector<double>& poses) {
  const uint32_t colour = 0xFFCF8D00;
  for (size_t i = start_pose; i <= end_pose; ++i) {
    const vector2f pose_location(poses[3 * i + 0], poses[3 * i + 1]);
    for (size_t j = 0; j < ray_cast_lines[i].size(); ++j) {
      DrawLine(ray_cast_lines[i][j].P0(), ray_cast_lines[i][j].P1(),
               colour, &display_message_);
      /*
      DrawLine(pose_location, ray_cast_lines[i][j].P0(),
               colour, &display_message_);
      DrawLine(pose_location, ray_cast_lines[i][j].P1(),
               colour, &display_message_);
      */
    }
  }
}

void DrawFactor(const Vector2f& p0,
                const Vector2f& p1,
                const uint32_t edge_color,
                const uint32_t factor_node_color,
                const uint32_t variable_node_color) {
  DrawPoint(p0, variable_node_color, &display_message_);
  DrawPoint(p1, variable_node_color, &display_message_);
  DrawPoint(Vector2f(0.5 * (p0 + p1)), factor_node_color, &display_message_);
  DrawLine(p0, p1, edge_color, &display_message_);
}

void DrawFactorGraph(
    const vector<double>& poses,
    const vector<vector<int> >& point_line_correspondences,
    const vector<NonMarkovLocalization::PointToPointGlobCorrespondence>&
        point_point_correspondences,
    const size_t start_pose, const size_t end_pose) {
  static const uint32_t kFactorEdgeColor = 0xC0000000;
  static const uint32_t kFactorFactorNodeColor = 0xAF00FF00;
  static const uint32_t kFactorVariableNodeColor = 0x6F0000FF;
  static const bool kDrawFactorGraph = true;

  if (!kDrawFactorGraph) return;

  // Draw factor for odometry.
  for (size_t i = start_pose; i < end_pose; ++i) {
    const Vector2f p0(poses[3 * i], poses[3 * i + 1]);
    const Vector2f p1(poses[3 * (i + 1)], poses[3 * (i + 1) + 1]);
    DrawFactor(p0, 
               p1,
               kFactorEdgeColor, 
               kFactorFactorNodeColor,
               kFactorVariableNodeColor);
  }

  // Draw factors for STFs
  for (size_t i = 0; i < point_point_correspondences.size(); ++i) {
    if (point_point_correspondences[i].points0.size() < 1) continue;
    const int pose_index0 = point_point_correspondences[i].pose_index0;
    const int pose_index1 = point_point_correspondences[i].pose_index1;
    const Vector2f p0(poses[3 * pose_index0 + 0], poses[3 * pose_index0 + 1]);
    const Vector2f p1(poses[3 * pose_index1 + 0], poses[3 * pose_index1 + 1]);
    DrawFactor(p0, 
               p1,
               kFactorEdgeColor, 
               kFactorFactorNodeColor,
               kFactorVariableNodeColor);
  }
}

void CorrespondenceCallback(
    const vector<double>& poses,
    const vector<vector< vector2f> >& point_clouds,
    const vector< NormalCloudf >& normal_clouds,
    const vector<vector<line2f> >& ray_cast_lines,
    const vector<vector<int> >& point_line_correspondences,
    const vector<NonMarkovLocalization::PointToPointGlobCorrespondence>&
        point_point_correspondences,
    const vector<NonMarkovLocalization::ObjectCorrespondence>&
        object_correspondences,
    const vector<PersistentObject>& objects,
    const vector<double>& object_poses,
    const vector<vector<NonMarkovLocalization::ObservationType> >&
        classifications,
    const vector<vector_localization::VisibilityGlobConstraint*>&
        visibility_constraints,
    const vector<double>& gradients,
    const vector<Matrix2f>& covariances,
    const vector<Pose2Df>& odometry_poses,
    const size_t start_pose, const size_t end_pose) {
  static const bool kDisplayLtfCorrespondences = true;
  static const bool kDisplayStfCorrespondences = true;
  static const bool kDisplayRayCasts = false;
  static const bool kDisplayFactorGraph = false;
  CHECK_EQ(poses.size(), point_clouds.size() * 3);
  CHECK_EQ(point_clouds.size(), ray_cast_lines.size());
  CHECK_EQ(point_clouds.size(), point_line_correspondences.size());
  ClearDrawingMessage(&display_message_);
  if (!run_) {
    localization_.Terminate();
  }
  DrawPoses(start_pose, end_pose, odometry_poses, poses, covariances);
  DrawGradients(start_pose, end_pose, gradients, poses);
  DrawObservations(start_pose, end_pose, poses, point_clouds, normal_clouds,
                   classifications);
  DrawObjects(objects, object_correspondences, object_poses);
  DrawVisibilityConstraints(visibility_constraints, poses);
  if (kDisplayRayCasts) {
    DrawRayCasts(start_pose, end_pose, ray_cast_lines, poses);
  }
  if (kDisplayFactorGraph) {
    DrawFactorGraph(poses, 
                    point_line_correspondences, 
                    point_point_correspondences,
                    start_pose,
                    end_pose);
  }
  if (kDisplayLtfCorrespondences) {
    DrawLtfs(start_pose, end_pose, poses, point_clouds, classifications,
             ray_cast_lines, point_line_correspondences);
  }
  if (kDisplayStfCorrespondences) {
    DrawStfs(point_point_correspondences, poses, point_clouds, normal_clouds);
  }
  if (debug_level_ >= 1) {
    PublishTrace();
  }
  display_publisher_.publish(display_message_);
}

void SaveLoggedPoses(const string& filename,
                     const vector<Pose2Df>& logged_poses,
                     const vector<double>& timestamps) {
  ScopedFile fid(filename, "w");
  for (size_t i = 0; i < logged_poses.size(); ++i) {
    fprintf(fid(), "%f %f %f %f\n",
            timestamps[i],
            logged_poses[i].translation.x(),
            logged_poses[i].translation.y(),
            logged_poses[i].angle);
  }
}

void LoadKeyframes(const string& file, vector<double>* keyframes) {
  ScopedFile fid(file, "r");
  if (fid() == NULL) return;
  double t = 0.0;
  keyframes->clear();
  while (fscanf(fid(), "%lf\n", &t) == 1) {
    keyframes->push_back(t);
  }
  sort(keyframes->begin(), keyframes->end());
  printf("Loaded %lu keyframes\n", keyframes->size());
}

void SaveSensorErrors(
    const vector<Pose2Df>& poses,
    const vector<PointCloudf>& point_clouds,
    const vector<vector<NonMarkovLocalization::ObservationType> >&
        classifications) {
  VectorMap map(kMapName, kMapsDirectory, true);
  ScopedFile fid("results/sensor_errors.txt", "w");
  printf("Saving sensor errors... ");
  fflush(stdout);
  ClearDrawingMessage(&display_message_);
  const vector2f laser_loc(
      localization_options_.sensor_offset.x(),
      localization_options_.sensor_offset.y());
  for (size_t i = 0; i < poses.size(); ++i) {
    const PointCloudf& point_cloud = point_clouds[i];
    const vector2f pose_loc_g =
        vector2f(poses[i].translation.x(), poses[i].translation.y()) +
        laser_loc.rotate(poses[i].angle);
    vector<vector2f> point_cloud_g(point_cloud.size());
    for (size_t j = 0; j < point_cloud.size(); ++j) {
      point_cloud_g[j] = vector2f(point_cloud[j].x(), point_cloud[j].y());
    }
    vector<line2f> ray_cast;
    const vector<int> line_correspondences = map.getRayToLineCorrespondences(
        pose_loc_g, poses[i].angle,
        -M_PI + FLT_MIN, M_PI - FLT_MIN, point_cloud_g,
        0.0, localization_options_.kMaxRange,
        true, &ray_cast);
    for (size_t j = 0; j < point_cloud_g.size(); ++j) {
      const vector2f observed_point =
          point_cloud_g[j].rotate(poses[i].angle) + pose_loc_g;
      if (classifications[i][j] != NonMarkovLocalization::kLtfObservation ||
          line_correspondences[j] < 0) {
        continue;
      }
      const line2f& line = ray_cast[line_correspondences[j]];
      const vector2f expected_point = line.intersection(
          pose_loc_g, observed_point, true, true);
      DrawPoint(expected_point, 0xFFFF0000, &display_message_);
      DrawPoint(observed_point, 0x4FFF7700, &display_message_);
      DrawLine(expected_point, pose_loc_g, 0x4FC0C0C0, &display_message_);
      const float angle = point_cloud_g[j].angle();
      const float observed_range = (point_cloud_g[j] - laser_loc).length();
      const float expected_range = (expected_point - pose_loc_g).length();
      fprintf(fid(), "%.4f %.4f %.4f\n", angle, expected_range, observed_range);
    }
  }
  PublishDisplay();
  printf("Done.\n");
}

void BatchLocalize(const string& bag_file, const string& keyframes_file,
                   int max_laser_poses, bool use_point_constraints,
                   double time_skip, bool return_initial_poses) {
  vector<PointCloudf> point_clouds;
  vector<NormalCloudf> normal_clouds;
  vector<Pose2Df> poses;
  vector<double> timestamps;
  vector<double> keyframes;
  vector<vector<NonMarkovLocalization::ObservationType> > classifications;

  localization_options_.use_STF_constraints = use_point_constraints;
  double bag_duration;
  printf("Processing %s\n", bag_file.c_str());

  LoadKeyframes(keyframes_file, &keyframes);
  LoadRosBag(bag_file, max_laser_poses, time_skip, keyframes, &point_clouds,
             &normal_clouds, &poses, &timestamps, &bag_duration, &kMapName);

  if (debug_level_ > 0) {
    localization_options_.CorrespondenceCallback = CorrespondenceCallback;
    // Reset localization_gui to correct map.
    cobot_msgs::CobotLocalizationMsg localization_msg_;
    localization_msg_.timeStamp = GetTimeSec();
    localization_msg_.map = kMapName;
    localization_msg_.x = 0;
    localization_msg_.y = 0;
    localization_msg_.angle = 0;
    localization_publisher_.publish(localization_msg_);
  }
  ClearDrawingMessage(&display_message_);
  DisplayPoses(poses, point_clouds, normal_clouds, classifications, NULL);
  if (debug_level_ > 2) {
    Sleep(1.0);
    return;
  }

  const double t_start = GetTimeSec();
  localization_.BatchLocalize(localization_options_, kMapName, point_clouds,
                              normal_clouds, (debug_level_ > 1),
                              return_initial_poses, &poses, &classifications);

  const double process_time = GetTimeSec() - t_start;
  printf("Done in %.3fs, bag time %.3fs (%.3fx).\n",
         process_time, bag_duration, bag_duration / process_time);
  ClearDrawingMessage(&display_message_);
  DisplayPoses(poses, point_clouds, normal_clouds, classifications,
               save_image_file);
  if (save_ltfs_) {
    SaveSensorErrors(poses, point_clouds, classifications);
  }
  SaveLoggedPoses(string(bag_file) + ".poses", poses, timestamps);
  if (kStandardizedData ||
      test_set_index_ >= 0 ||
      statistical_test_index_ >= 0) {
    SaveResults(bag_file, poses, point_clouds, normal_clouds, kMapName);
  }
  if (save_image_file != NULL) {
    cobot_msgs::LocalizationGuiCaptureSrv::Request req;
    cobot_msgs::LocalizationGuiCaptureSrv::Response res;
    req.filename = string(save_image_file);
    printf("Saving image to %s\n", req.filename.c_str());
    gui_capture_client.call(req, res);
  }
  if (save_stfs_) {
    SaveStfs(kMapName, poses, point_clouds, normal_clouds, classifications,
             bag_file, timestamps[0]);
  }
}

void StandardOdometryCallback(const nav_msgs::Odometry& last_odometry_msg,
                              const nav_msgs::Odometry& odometry_msg) {
  const float old_theta = 2.0 * atan2(
      last_odometry_msg.pose.pose.orientation.z,
      last_odometry_msg.pose.pose.orientation.w);
  const float new_theta = 2.0 * atan2(
      odometry_msg.pose.pose.orientation.z,
      odometry_msg.pose.pose.orientation.w);
  const Vector2f p_old(last_odometry_msg.pose.pose.position.x,
                       last_odometry_msg.pose.pose.position.y);
  const Vector2f p_new(odometry_msg.pose.pose.position.x,
                       odometry_msg.pose.pose.position.y);
  Vector2f p_delta =
      Rotation2Df(-old_theta) * (p_new - p_old);
  float d_theta = angle_diff<float>(new_theta, old_theta);
  if (debug_level_ > 1) {
    printf("Standard Odometry %f %f %f, t=%f\n",
           p_delta.x(), p_delta.y(), DEG(d_theta),
           odometry_msg.header.stamp.toSec());
  }
  if (test_set_index_ >= 0 || statistical_test_index_ >= 0) {
    ApplyNoiseModel(p_delta.x(),
                    p_delta.y(),
                    d_theta,
                    odometry_additive_noise_,
                    &(p_delta.x()),
                    &(p_delta.y()),
                    &(d_theta));
  }
  localization_.OdometryUpdate(
      kOdometryTranslationScale * p_delta.x(),
      kOdometryTranslationScale * p_delta.y(), d_theta);
  PublishLocation();
}

void OdometryCallback(const cobot_msgs::CobotOdometryMsg& odometry_msg) {
  if((sq(odometry_msg.dx) + sq(odometry_msg.dy)) > kSqMaxOdometryDeltaLoc ||
      fabs(odometry_msg.dr) > kMaxOdometryDeltaAngle){
      printf("EnML Odometry out of bounds: x:%7.3f y:%7.3f a:%7.3f\u00b0\n",
             odometry_msg.dx, odometry_msg.dy, DEG(odometry_msg.dr));
    return;
  }
  if (debug_level_ > 1) {
    printf("Odometry, t=%f\n", odometry_msg.header.stamp.toSec());
  }
  localization_.OdometryUpdate(
      kOdometryTranslationScale * odometry_msg.dx,
      kOdometryTranslationScale * odometry_msg.dy,
      kOdometryRotationScale * odometry_msg.dr);
  PublishLocation();
}

void LaserCallback(const sensor_msgs::LaserScan& laser_message) {
  if (debug_level_ > 1) {
    printf("LaserScan, t=%f\n", laser_message.header.stamp.toSec());
  }
  PointCloudf point_cloud;
  for (size_t i = 0; i < laser_message.ranges.size(); ++i) {
    if (laser_message.ranges[i] < kMinPointCloudRange ||
        laser_message.ranges[i] > kMaxPointCloudRange ||
        !std::isfinite(laser_message.ranges[i])) continue;
    const float angle = laser_message.angle_min +
        laser_message.angle_increment * static_cast<float>(i);
    if (angle < laser_message.angle_min + RAD(15.0) ||
        angle > laser_message.angle_max - RAD(15.0)) continue;
    float range = laser_message.ranges[i];
    point_cloud.push_back(localization_options_.sensor_offset +
        Rotation2Df(angle) * Vector2f(range, 0.0));
  }
  NormalCloudf normal_cloud;
  GenerateNormals(kMaxNormalPointDistance, &point_cloud, &normal_cloud);
  if (point_cloud.size() > 1) {
    localization_.SensorUpdate(point_cloud, normal_cloud);
  }
  last_laser_scan_ = laser_message;
}

void SaveOrebroLoggedPoses(const vector<Pose2Df>& logged_poses,
                     const vector<int>& logged_episode_lengths,
                     const vector<vector<float> >& laser_scans,
                     const string& bag_file) {
  static const uint32_t kLoggedPosesColor = 0xFF00A80E;

  // Display a trace of the logged poses.
  ClearDrawingMessage(&display_message_);
  for (size_t i = 1; i < logged_poses.size(); ++i) {
    DrawLine(logged_poses[i - 1].translation, logged_poses[i].translation,
             kLoggedPosesColor, &display_message_);
  }
  PublishDisplay();

  // Save bmap (Orebro format) localization log.
  if (false) {
    CHECK_EQ(laser_scans.size(), logged_poses.size());
    ScopedFile fid(bag_file + ".bmap", "w");
    fprintf(fid(), "%% Non-Markov Localization results\n");
    fprintf(fid(), "%% Format: TimeMs x y phi laserScanReadings\n");
    fprintf(fid(), "%% 181 # nr of samples per laser scan\n");
    fprintf(fid(), "V2\n");
    for (size_t i = 0; i < logged_poses.size(); ++i) {
      const Pose2Df& pose = logged_poses[i];
      fprintf(fid(), "0 %f %f %f",
              pose.translation.x() * 100.0, pose.translation.y() * 100.0,
              pose.angle);
      const vector<float>& scan = laser_scans[i];
      for (size_t j = 0; j < scan.size(); ++j) {
        fprintf(fid(), " %f", scan[j] * 100.0);
      }
      fprintf(fid(), "\n");
    }
  }

  // Save the logged poses.
  {
    ScopedFile fid(bag_file + ".poses", "w");
    for (size_t i = 0; i < logged_poses.size(); ++i) {
      const Pose2Df& pose = logged_poses[i];
      fprintf(fid(), "%f, %f, %f\n",
              pose.translation.x(), pose.translation.y(),
              pose.angle);
    }
  }

  // Save the episode lengths.
  {
    ScopedFile fid(bag_file + ".episodes", "w");
    for (size_t i = 0; i < logged_episode_lengths.size(); ++i) {
      fprintf(fid(), "%d\n", logged_episode_lengths[i]);
    }
  }
  Sleep(1.0);
}

// GUI interaction state enumeration.
enum GuiInteractionState {
  kGuiStateNone = 0,
  kGuiStateDraw = 1,
};

// Current GUI interaction state.
GuiInteractionState interaction_state_ = kGuiStateNone;

// The type of object pose being set.
uint32_t object_type_ = 0;

void KeyboardCallback(const cobot_msgs::GuiKeyboardEventConstPtr& msg) {
  if (msg->modifiers != 0) return;

  if (msg->keycode >= '0' && msg->keycode <= '9') {
    object_type_ = msg->keycode - '0';
    printf("Set object type %u pose estimate.\n", object_type_);
    interaction_state_ = kGuiStateDraw;
  }
}

void MouseClickCallback(const cobot_msgs::GuiMouseClickEventConstPtr& msg) {
  if (interaction_state_ != kGuiStateDraw) return;
  const Vector2f mouse_down(V2COMP(msg->mouse_down));
  const Vector2f mouse_up(V2COMP(msg->mouse_up));
  const Vector2f delta = mouse_up - mouse_down;
  const float angle = atan2(delta.y(), delta.x());
  interaction_state_ = kGuiStateNone;
  printf("Added object instance of type %u at %f,%f, %f\n",
         object_type_, mouse_down.x(), mouse_down.y(), DEG(angle));
}

bool LaserPointComparator(const pair<float, Vector2f>& p1,
                          const pair<float, Vector2f>& p2) {
  return (p1.first < p2.first);
}

float CheckLaserOverlap(const sensor_msgs::LaserScan& scan1,
                       const Pose2Df& pose1,
                       const sensor_msgs::LaserScan& scan2,
                       const Pose2Df& pose2) {
  static ScopedFile fid("overlap.txt", "w");
  const vector<float>& ranges1 = scan1.ranges;
  const vector<float>& ranges2 = scan2.ranges;
  if (ranges1.size() != ranges2.size()) return 0.0;

  // Compute the relative delta transform.
  const Vector2f delta_loc =
      Rotation2Df(-pose1.angle) * (pose2.translation - pose1.translation);
  const float delta_angle = angle_diff(pose2.angle, pose1.angle);
  const Affine2f delta_transform =
      Translation2f(delta_loc) * Rotation2Df(delta_angle);
  vector<pair<float, Vector2f> > new_point_cloud(ranges2.size());
  for (size_t i = 0; i < ranges2.size(); ++i) {
    const float scan_angle =
        scan2.angle_min + scan2.angle_increment * static_cast<float>(i);
    const Vector2f scan_point = delta_transform * Rotation2Df(scan_angle) *
        Vector2f(ranges2[i], 0.0);
    const float point_angle = atan2(scan_point.y(), scan_point.x());
    new_point_cloud[i] = std::make_pair(point_angle, scan_point);
  }
  std::sort(new_point_cloud.begin(), new_point_cloud.end(),
      LaserPointComparator);
  float diff = 0.0;
  size_t j = 0;
  for (size_t i = 0; i < ranges1.size(); ++i) {
    if (ranges1[i] < scan1.range_min || ranges1[i] > scan1.range_max ||
        ranges2[i] < scan2.range_min || ranges2[i] > scan2.range_max) {
      continue;
    }
    const float scan_angle =
        scan1.angle_min + scan1.angle_increment * static_cast<float>(i);
    for(; j < new_point_cloud.size() &&
        new_point_cloud[j].first < scan_angle; ++j) {
    }
    diff += fabs(ranges1[i] - new_point_cloud[j].second.norm());
  }
  fprintf(fid(), "%f\n", diff);
  // printf("Laser diff: %6.2f mean: %6.4f\n",
  //        diff, diff / static_cast<float>(ranges1.size()));
  return diff;
}

void SRLVisualize(
    const vector<double>& poses,
    const vector<vector2f>& point_cloud_g,
    const PointCloudf& point_cloud_e,
    const NormalCloudf& normal_cloud,
    const vector<vector_localization::LTSConstraint*>& constraints) {
  const size_t num_samples = poses.size() / 3;
  ClearDrawingMessage(&display_message_);
  double max_weight = 0.0;
  size_t best_pose = 0;
  vector<double> pose_weights(num_samples);
  for (size_t i = 0; i < num_samples; ++i) {
    const Pose2Df pose(poses[3 * i + 2], poses[3 * i + 0], poses[3 * i + 1]);
    pose_weights[i] = localization_.ObservationLikelihood(
        pose, point_cloud_e, point_cloud_g, normal_cloud);
    if (pose_weights[i] > max_weight) {
      max_weight = pose_weights[i];
      best_pose = i;
    }
  }
  printf("Best SRL weight: %23.20f\n", max_weight);
  for (size_t i = 0; i < num_samples; ++i) {
    const Affine2f pose_transform =
        Translation2f(poses[3 * i + 0], poses[3 * i + 1]) *
        Rotation2Df(poses[3 * i + 2]);
    const Vector2f line_p0(poses[3 * i + 0], poses[3 * i + 1]);
    const Vector2f line_p1 =
        line_p0 + Rotation2Df(poses[3 * i + 2]) * Vector2f(0.3, 0.0);
    const double alpha = max(0.05, pose_weights[i] / max_weight);
    const uint32_t alpha_channel =
        (static_cast<uint32_t>(255.0 * alpha) << 24) & 0xFF000000;
    DrawPoint(line_p0, alpha_channel | 0xFF0000, &display_message_);
    DrawLine(line_p0, line_p1, alpha_channel | 0x3F3F3F, &display_message_);
    if (pose_weights[i] == max_weight) {
      for (size_t j = 0; j < point_cloud_e.size(); ++j) {
        const Vector2f point = pose_transform * point_cloud_e[j];
        DrawPoint(point, 0xFFFF7700, &display_message_);
        DrawLine(point, line_p0, 0x1FFF7700, &display_message_);
      }
    }
  }
  const Affine2f best_pose_transform =
      Translation2f(poses[3 * best_pose + 0], poses[3 * best_pose + 1]) *
      Rotation2Df(poses[3 * best_pose + 2]);
  for (size_t i = 0; i < constraints.size(); ++i) {
    if (constraints[i]->pose_index != best_pose) continue;
    const vector_localization::LTSConstraint& constraint = *constraints[i];
    const Vector2f p = best_pose_transform * constraint.point;
    const float offset = p.dot(constraint.line_normal) + constraint.line_offset;
    const Vector2f p_line = p - constraint.line_normal * offset;
    DrawLine(p, p_line, 0xFF00FF00, &display_message_);
  }
  PublishDisplay();
}

void SensorResettingResample(const sensor_msgs::LaserScan& laser_message) {
  printf("SRL with laser t=%f\n", laser_message.header.stamp.toSec());
  PointCloudf point_cloud_e;
  vector<vector2f> point_cloud_g;
  for (size_t i = 0; i < laser_message.ranges.size(); ++i) {
    if (laser_message.ranges[i] < kMinPointCloudRange ||
        laser_message.ranges[i] > kMaxPointCloudRange ||
        !std::isfinite(laser_message.ranges[i])) continue;
    const float angle = laser_message.angle_min +
        laser_message.angle_increment * static_cast<float>(i);
    point_cloud_e.push_back(localization_options_.sensor_offset +
        Rotation2Df(angle) * Vector2f(laser_message.ranges[i], 0.0));
    point_cloud_g.push_back(
        vector2f(point_cloud_e.back().x(), point_cloud_e.back().y()));
  }
  NormalCloudf normal_cloud;
  GenerateNormals(kMaxNormalPointDistance, &point_cloud_e, &normal_cloud);

  vector<Pose2Df> poses;
  vector<double> pose_weights;
  vector<vector_localization::LTSConstraint*> constraints;
  static const size_t kNumSamples = 500;
  static const float kTangentialStdDev = 0.3;
  static const float kRadialStdDev = 0.3;
  static const float kAngularStdDev = RAD(20.0);
  printf("Running SRL...");
  fflush(stdout);
  localization_.SensorResettingResample(
      point_cloud_g, normal_cloud, kNumSamples, kRadialStdDev,
      kTangentialStdDev, kAngularStdDev, &SRLVisualize, &poses, &pose_weights,
      &constraints);
  printf(" Done.\n");
  fflush(stdout);

  double max_weight = 0.0;
  for (size_t i = 0; i < pose_weights.size(); ++i) {
    max_weight = max(max_weight, pose_weights[i]);
  }
  printf("Best weight: %f\n", max_weight);

  vector<double> pose_array(poses.size() * 3);
  for (size_t i = 0; i < poses.size(); ++i) {
    pose_array[3 * i + 0] = poses[i].translation.x();
    pose_array[3 * i + 1] = poses[i].translation.y();
    pose_array[3 * i + 2] = poses[i].angle;
  }
  SRLVisualize(pose_array, point_cloud_g, point_cloud_e, normal_cloud,
               constraints);
}



bool RemoteInterfaceCallback(CobotRemoteInterfaceSrv::Request& req,
                             CobotRemoteInterfaceSrv::Response& res) {
  static const bool kUseLocationSearch = true;
  static const unsigned int CmdSetLocation = 0x0002;
  if(req.command_type & CmdSetLocation){
    const Vector2f loc(req.loc_x, req.loc_y);
    const string event =
        StringPrintf("Setting location: %f %f %f %s\n",
                     loc.x(), loc.y(), DEG(req.orientation), req.map.c_str());
    if (debug_level_ > 0) printf("%s", event.c_str());
    CobotEventsMsg msg;
    msg.module = "Episodic non-Markov Localization";
    msg.timestamp = GetTimeSec();
    msg.events.push_back(event);
    events_publisher_.publish(msg);
    if (kUseLocationSearch) {
      static const float kRelocalizationResolution = 0.2;
      static const float kRelocalizationAngResolution = RAD(5.0);
      static const float kRelocalizationWindow = 0.4;
      static const float kRelocalizationAngWindow = RAD(10.0);
      // Mutex used to prevent multiple simultaneous calls to this service.
      ScopedLock lock(relocalization_mutex_);

      VectorLocalization2D::LidarScan lidar_scan;
      lidar_scan.angleResolution = last_laser_scan_.angle_increment;
      lidar_scan.minAngle = last_laser_scan_.angle_min;
      lidar_scan.maxAngle = last_laser_scan_.angle_max;
      lidar_scan.minRange = last_laser_scan_.range_min;
      lidar_scan.maxRange = last_laser_scan_.range_max;
      lidar_scan.numRays = last_laser_scan_.ranges.size();
      lidar_scan.ranges = last_laser_scan_.ranges.data();
      lidar_scan.laserToBaseRot = Matrix2f::Identity();
      lidar_scan.laserToBaseTrans = localization_options_.sensor_offset;
      lidar_scan.initialize();

      vector2f bestLoc(0.0, 0.0);
      float bestAngle(0.0);
      float bestWeight(0.0);
      relocalization_.Relocalize(req.map,
                                vector2f(req.loc_x, req.loc_y),
                                req.orientation,
                                lidar_scan,
                                relocalization_lidar_params_,
                                kRelocalizationResolution,
                                kRelocalizationAngResolution,
                                kRelocalizationWindow,
                                kRelocalizationAngWindow,
                                &bestLoc,
                                &bestAngle,
                                &bestWeight);
      ClearDrawingMessage(&display_message_);
      relocalization_.VisualizeSamples(&display_message_);
      display_publisher_.publish(display_message_);
      ClearDrawingMessage(&display_message_);
      localization_.Initialize(Pose2Df(bestAngle, bestLoc), req.map);
    } else {
      localization_.Initialize(Pose2Df(req.orientation, loc), req.map);
      // Publish this updated location.
      cobot_msgs::CobotLocalizationMsg localization_msg_;
      localization_msg_.timeStamp = GetTimeSec();
      localization_msg_.map = req.map;
      localization_msg_.x = loc.x();
      localization_msg_.y = loc.y();
      localization_msg_.angle = req.orientation;
      localization_publisher_.publish(localization_msg_);
    }
  }
  return true;
}

void InteractivePause() {
  bool ready = false;
  while(run_ && !ready) {
    if (kbhit() != 0) {
      switch(fgetc(stdin)) {
        case ' ': {
          ready = true;
        } break;
        case 'q': {
          run_ = false;
        } break;
      }
      printf("\n"); fflush(stdout);
    }
    Sleep(0.05);
  }
}

void CheckIfLost() {
  static double t_last = 0.0;
  if (GetTimeSec() - t_last < 1.0) return;
  // Experimental setup to see if we can detect when EnML is lost.
  const double lost_metric = localization_.GetLostMetric();
  if (lost_metric > 0.2) {
    printf("\nPossibly lost: %.3f\n", lost_metric);
    InteractivePause();
    t_last = GetTimeSec();
  }
}


void ComputeLidarError(
    const sensor_msgs::LaserScan& msg,
    double* lidar_error_ptr,
    double* num_lidar_points_ptr) {
  /*
  static const float kMaxError = 1.0;
  double& lidar_error = *lidar_error_ptr;
  double& num_lidar_points = *num_lidar_points_ptr;
  localization_.GetLastMLEPose();
  const VectorMap& map = localization->getCurrentMap();
  vector<vector2f> laser_points;
  vector<line2f> lines;
  vector<int> lineCorrespondences;
  // Transform laserpoints to robot frame
  vector<Vector2f> laserPoints(lidarScan.numRays);
  for(unsigned int i=0; i<lidarScan.numRays; i++){
    laserPoints[i] = lidarScan.laserToBaseTrans +
        lidarScan.laserToBaseRot * lidarScan.scanHeadings[i] * lidarScan.ranges[i];
  }

  Matrix2f robotAngle;
  robotAngle = Rotation2Df(curAngle);
  const Vector2f locE(V2COMP(curLoc));
  const vector2f laserLoc = curLoc + vector2f(
      lidarScan.laserToBaseTrans.x(),
      lidarScan.laserToBaseTrans.y()).rotate(curAngle);
  lineCorrespondences = map.getRayToLineCorrespondences(
      laserLoc, curAngle, lidarScan.angleResolution, lidarScan.numRays,
      lidarScan.minRange, lidarScan.maxRange, true, &lines);
  for (unsigned int i = 0; i < lidarScan.numRays; ++i) {
    if (lidarScan.ranges[i] > lidarScan.maxRange ||
        lidarScan.ranges[i] < lidarScan.minRange) {
      continue;
    }
    num_lidar_points += 1.0;
    if(lineCorrespondences[i]>=0){
      const Vector2f scanPoint = locE + robotAngle*laserPoints[i];
      const vector2f scanPoint_g = vector2f(scanPoint.x(), scanPoint.y());
      const vector2f perp_from_line =
          lines[lineCorrespondences[i]].perpFromLine(scanPoint_g, true);
      const float error = perp_from_line.length();
      lidar_error += min(kMaxError, error);
    } else {
      lidar_error += kMaxError;
    }
  }
  */
}

void PlayBagFile(const string& bag_file,
                 int max_poses,
                 bool use_point_constraints,
                 double time_skip,
                 ros::NodeHandle* node_handle,
                 double* observation_error_ptr) {
  const bool compute_error = (observation_error_ptr != NULL);
  double& observation_error = *observation_error_ptr;
  double num_observed_points = 0.0;
  FILE* episode_stats_file = NULL;
  if (save_episode_stats_) {
    const string stats_file = bag_file + ".stats";
    episode_stats_file = fopen(stats_file.c_str(), "w");
    // CHECK_NOTNULL(episode_stats_file);
    if (episode_stats_file == NULL) {
      fprintf(stderr, "Unable to open file %s", stats_file.c_str());
      perror("");
      exit(1);
    }
  }

  if (compute_error) {
    observation_error = 0.0;
  }
  localization_options_.use_STF_constraints = use_point_constraints;
  localization_options_.log_poses = true;
  localization_options_.CorrespondenceCallback =
      ((debug_level_ > 0) ? CorrespondenceCallback : NULL);
  localization_.SetOptions(localization_options_);
  localization_.Initialize(Pose2Df(kStartingAngle, kStartingLocation),
                           kMapName);

  rosbag::Bag bag;
  if (!quiet_) printf("Processing bag file %s\n", bag_file.c_str());
  bag.open(bag_file.c_str(), rosbag::bagmode::Read);
  const double t_start = GetTimeSec();

  std::vector<std::string> topics;
  topics.push_back(kCobotOdometryTopic);
  if (use_kinect_) {
    topics.push_back(kKinectScanTopic);
    topics.push_back(kStandardKinectScanTopic);
  } else {
    topics.push_back(kCobotLaserTopic);
    topics.push_back(kStandardLaserTopic);
  }
  topics.push_back(kStandardOdometryTopic);
  topics.push_back(kStandardSetLocationTopic);

  rosbag::View view(bag, rosbag::TopicQuery(topics));
  double bag_time_start = -1.0;
  double bag_time = 0.0;
  int num_laser_scans = 0;

  Subscriber mouse_click_subscriber = node_handle->subscribe(
      "Cobot/VectorLocalization/GuiMouseClickEvents", 1, MouseClickCallback);
  Subscriber keyboard_subscriber = node_handle->subscribe(
      "Cobot/VectorLocalization/GuiKeyboardEvents", 1, KeyboardCallback);
  events_publisher_ = node_handle->advertise<CobotEventsMsg>(
      "Cobot/Events",1,true);

  // RemoteInterface server.
  ServiceServer remote_interface_server = node_handle->advertiseService(
      "Cobot/VectorLocalization/RemoteInterface", &RemoteInterfaceCallback);
  int episode_image_number = 0;
  nav_msgs::Odometry last_standard_odometry;
  sensor_msgs::LaserScan last_laser_scan;
  Pose2Df last_laser_pose;
  bool standard_odometry_initialized = false;
  bool standard_localization_initialized = false;
  vector<Pose2Df> pose_trajectory;
  nonblock(true);
  for (rosbag::View::iterator it = view.begin();
       run_ && it != view.end(); ++it) {
    const rosbag::MessageInstance &message = *it;
    bag_time = message.getTime().toSec();
    if (bag_time_start < 0.0) {
      // Initialize bag starting time.
      bag_time_start = bag_time;
    }
    if (!quiet_ && debug_level_ > 1) {
      printf("message from %s\n", message.getTopic().c_str());
    }
    // Ignore messages before elapsed time_skip.
    if (bag_time < bag_time_start + time_skip) continue;
    if (!quiet_) {
      const double elapsed_time = bag_time - bag_time_start;
      const int hh = rint(elapsed_time / 3600.0);
      const int mm = rint(fmod(elapsed_time, 3600.0) / 60.0);
      const float ss = fmod(elapsed_time, 60.0);
      printf("\r%02d:%02d:%04.1f (%.1f) Lost:%.3f",
            hh, mm, ss, elapsed_time, localization_.GetLostMetric());
    }
    fflush(stdout);
    int keycode = -1;
    if (run_ && kbhit() != 0) {
      keycode = fgetc(stdin);
      printf("\n"); fflush(stdout);
      switch(keycode){
        case ' ': {
          printf("Paused\n");
          keycode = 0;
          while (keycode !=' ' && run_) {
            Sleep(0.01);
            ros::spinOnce();
            if (kbhit()!=0) {
              keycode = fgetc(stdin);
            }
            if (keycode == 'r') {
              SensorResettingResample(last_laser_scan);
              keycode = 0;
            } else if (keycode == 'q') {
              printf("\nExiting.\n");
              exit(0);
            }
          }
        } break;
        case 'q': {
          printf("\nExiting.\n");
          exit(0);
        } break;
      }
    }

    // Check to see if this is a laser scan message.
    {
      sensor_msgs::LaserScanPtr laser_message =
          message.instantiate<sensor_msgs::LaserScan>();
      if (laser_message != NULL &&
          (message.getTopic() == kCobotLaserTopic ||
           message.getTopic() == kKinectScanTopic ||
          (standard_localization_initialized &&
              (message.getTopic() == kStandardLaserTopic ||
               message.getTopic() == kStandardKinectScanTopic)))) {
        ++num_laser_scans;
        LaserCallback(*laser_message);
        const bool ran_solver = localization_.RunningSolver();
        while(localization_.RunningSolver()) {
          Sleep(0.01);
        }
        last_laser_scan = *laser_message;
        last_laser_pose = localization_.GetLatestPose();
        double lidar_error = 0.0;
        if (compute_error && ran_solver) {
          ComputeLidarError(last_laser_scan,
                            &lidar_error,
                            &num_observed_points);
        }
        if (ran_solver && save_episode_stats_) {
          SaveEpisodeStats(episode_stats_file);
        }
        if (debug_level_ > 1) CheckIfLost();
        if (episode_images_path != NULL && ran_solver) {
          ++episode_image_number;
          cobot_msgs::LocalizationGuiCaptureSrv::Request req;
          cobot_msgs::LocalizationGuiCaptureSrv::Response res;
          req.filename = StringPrintf(
              "%s/%06d.png", episode_images_path, episode_image_number);
          printf("Saving image to %s\n", req.filename.c_str());
          Sleep(0.1);
          gui_capture_client.call(req, res);
        }
        pose_trajectory.push_back(localization_.GetLatestPose());
        continue;
      }
    }

    // Check to see if this is an odometry message.
    {
      cobot_msgs::CobotOdometryMsgPtr odometry_message =
          message.instantiate<cobot_msgs::CobotOdometryMsg>();
      if (odometry_message != NULL &&
          message.getTopic() == kCobotOdometryTopic) {
        OdometryCallback(*odometry_message);
      }
    }

    // Check to see if this is a standardized odometry message.
    {
      nav_msgs::OdometryPtr odometry_message =
          message.instantiate<nav_msgs::Odometry>();
      if (odometry_message != NULL &&
          message.getTopic() == kStandardOdometryTopic) {
        if (standard_odometry_initialized) {
          if (standard_localization_initialized) {
            StandardOdometryCallback(last_standard_odometry, *odometry_message);
          }
        } else {
          standard_odometry_initialized = true;
        }
        last_standard_odometry = *odometry_message;
      }
    }

    {
      Vector2f init_location;
      float init_angle;
      string init_map;
      if (LoadSetLocationMessage(message, &init_location, &init_angle,
          &init_map)) {
        if (!quiet_ && debug_level_ > 0) {
          printf("Initializing location to %s: %f,%f, %f\u00b0\n",
                 init_map.c_str(), init_location.x(), init_location.y(),
                 DEG(init_angle));
        }
        const Pose2Df init_pose(init_angle, init_location.x(),
                                init_location.y());
        localization_.Initialize(init_pose, init_map);
        standard_localization_initialized = true;
      }
    }
  }
  localization_.Finalize();
  const double process_time = GetTimeSec() - t_start;
  const double bag_duration = bag_time - bag_time_start;
  const vector<Pose2Df> logged_poses = localization_.GetLoggedPoses();
  const vector<int> episode_lengths = localization_.GetLoggedEpisodeLengths();
  if (!quiet_) {
    printf("Done in %.3fs, bag time %.3fs (%.3fx).\n",
          process_time, bag_duration, bag_duration / process_time);
    printf("%d laser scans, %lu logged poses\n",
          num_laser_scans, logged_poses.size());
  }

  if (statistical_test_index_ >= 0) {
    const string file_name = StringPrintf(
        "results/enml/non_markov_test_%d.txt", statistical_test_index_);
    ScopedFile fid(file_name, "w");
    CHECK_NOTNULL(fid());
    for (unsigned int i = 0; i < pose_trajectory.size(); ++i) {
      fprintf(fid, "%f,%f,%f\n",
              pose_trajectory[i].translation.x(),
              pose_trajectory[i].translation.y(),
              pose_trajectory[i].angle);
    }
  }
  if (save_image_file != NULL && logged_poses.size() > 2) {
    cobot_msgs::LocalizationGuiCaptureSrv::Request req;
    cobot_msgs::LocalizationGuiCaptureSrv::Response res;
    req.filename = string(save_image_file);
    printf("Saving image to %s\n", req.filename.c_str());
    gui_capture_client.call(req, res);
  }
  if (compute_error) {
    observation_error = observation_error / num_observed_points;
  }
  if (save_episode_stats_) {
    fclose(episode_stats_file);
  }
}

bool AutoLocalizeCallBack(
    CobotLocalizationSrv::Request& req, CobotLocalizationSrv::Response& res) {
  static const float kRelocalizationResolution = 0.2;
  static const float kRelocalizationAngResolution = RAD(20.0);
  static const float kRelocalizationWindow = 4.0;
  // Mutex used to prevent multiple simultaneous calls to this service.
  ScopedTryLock lock(relocalization_mutex_);
  if (!lock.Locked()) {
    // There is a concurrent call in progress.
    return false;
  }

  VectorLocalization2D::LidarScan lidar_scan;
  lidar_scan.angleResolution = last_laser_scan_.angle_increment;
  lidar_scan.minAngle = last_laser_scan_.angle_min;
  lidar_scan.maxAngle = last_laser_scan_.angle_max;
  lidar_scan.minRange = last_laser_scan_.range_min;
  lidar_scan.maxRange = last_laser_scan_.range_max;
  lidar_scan.numRays = last_laser_scan_.ranges.size();
  lidar_scan.ranges = last_laser_scan_.ranges.data();
  lidar_scan.laserToBaseRot = Matrix2f::Identity();
  lidar_scan.laserToBaseTrans = localization_options_.sensor_offset;
  lidar_scan.initialize();

  vector2f bestLoc(0.0, 0.0);
  float bestAngle(0.0);
  float bestWeight(0.0);
  relocalization_.Relocalize(localization_.GetCurrentMapName(),
                             vector2f(req.x, req.y),
                             req.angle,
                             lidar_scan,
                             relocalization_lidar_params_,
                             kRelocalizationResolution,
                             kRelocalizationAngResolution,
                             kRelocalizationWindow,
                             M_PI,
                             &bestLoc,
                             &bestAngle,
                             &bestWeight);
  ClearDrawingMessage(&display_message_);
  relocalization_.VisualizeSamples(&display_message_);
  display_publisher_.publish(display_message_);
  ClearDrawingMessage(&display_message_);
  res.x = bestLoc.x;
  res.y = bestLoc.y;
  res.angle = bestAngle;
  res.map = localization_.GetCurrentMapName();
  localization_.Initialize(Pose2Df(res.angle, res), res.map);
  return true;
}

void OnlineLocalize(bool use_point_constraints, ros::NodeHandle* node) {
  localization_options_.use_STF_constraints = use_point_constraints;
  localization_.SetOptions(localization_options_);
  localization_.Initialize(Pose2Df(kStartingAngle, kStartingLocation),
                           kMapName);

  // Subscribe to odometry.
  Subscriber odometry_subscriber = node->subscribe(
    "Cobot/Odometry", 40, OdometryCallback);

  // Subscribe to laser scanner.
  Subscriber laser_subscriber = use_kinect_ ?
      (node->subscribe("Cobot/Kinect/Scan", 1, LaserCallback)) :
      (node->subscribe("Cobot/Laser", 2, LaserCallback));

  // RemoteInterface server.
  ServiceServer remote_interface_server = node->advertiseService(
      "Cobot/VectorLocalization/RemoteInterface", &RemoteInterfaceCallback);

  // AutoLocalize service request server.
  ServiceServer auto_localize_server = node->advertiseService(
      "Cobot/VectorLocalization/AutoLocalize",
      &AutoLocalizeCallBack);
  events_publisher_ = node->advertise<CobotEventsMsg>("Cobot/Events",1,true);

  ClearDrawingMessage(&display_message_);
  display_publisher_.publish(display_message_);
  {
    CobotEventsMsg msg;
    msg.module = "Episodic non-Markov Localization";
    msg.timestamp = GetTimeSec();
    msg.events.push_back("Episodic non-Markov Localization Started Up");
    events_publisher_.publish(msg);
  }
  while (run_ && ros::ok()) {
    Sleep(0.02);
    if (debug_level_ > -1) {
      ClearDrawingMessage(&display_message_);
      DisplayDebug();
    }
    if (watch_files_.getEvents() != 0) {
      printf("Reloading parameters.\n");
      if (config_.isFileModified())
        printf("config modified\n");
      CHECK(LoadConfiguration(&localization_options_));
      localization_.SetOptions(localization_options_);
      watch_files_.clearEvents();
    }
    ros::spinOnce();
  }
}

void SegfaultHandler(int t) {
  printf("Segmentation Fault\n");
  PrintBackTrace();
  exit(-1);
}

// Signal handler for breaks (Ctrl-C)
void HandleStop(int i) {
  printf("\nTerminating.\n");
  exit(0);
}

int main(int argc, char** argv) {
  //InitHandleStop(&run_, 0);
  signal(SIGINT,HandleStop);
  signal(SIGALRM,HandleStop);
  google::InitGoogleLogging(argv[0]);
  google::LogToStderr();
  // CHECK(signal(SIGSEGV, &SegfaultHandler) != SIG_ERR);

  char* bag_file = NULL;
  char* keyframes_file = NULL;
  int max_laser_poses = -1;
  bool disable_stfs = false;
  double time_skip = 0;
  bool batch_mode = false;
  bool unique_node_name = false;
  bool return_initial_poses = false;

  static struct poptOption options[] = {
    { "debug" , 'd', POPT_ARG_INT, &debug_level_, 1, "Debug level", "NUM" },
    { "bag-file", 'b', POPT_ARG_STRING, &bag_file, 1, "ROS bagfile to use",
        "STRING"},
    { "batch-mode", 'B', POPT_ARG_NONE, &batch_mode, 0, "Run in batch mode",
        "NONE"},
    { "max-poses" , 'n', POPT_ARG_INT, &max_laser_poses, 1,
        "Maximum number of laser poses to optimize", "NUM" },
    { "time-skip", 's', POPT_ARG_DOUBLE, &time_skip, 1,
      "Time to skip from the bag file", "NUM" },
    { "standardized", 'S', POPT_ARG_NONE, &kStandardizedData, 0,
        "Load standardized data bag file.", "NONE" },
    { "test-set", 't', POPT_ARG_INT, &test_set_index_, 1,
        "Test set index", "NUM" },
    { "statistical-test", 'T', POPT_ARG_INT, &statistical_test_index_, 1,
        "Statistical test index", "NUM" },
    { "noise", 'N', POPT_ARG_DOUBLE, &odometry_additive_noise_, 1,
        "Statistical test additive random noise", "NUM" },
    { "keyframes", 'k', POPT_ARG_STRING, &keyframes_file, 1,
        "Keyframes file", "STRING" },
    { "image", 'I', POPT_ARG_STRING, &save_image_file, 1,
        "Image file", "STRING" },
    { "episode-images", 'e', POPT_ARG_STRING, &episode_images_path, 1,
        "Epidode images path", "STRING" },
    { "unique_node_name", 'u', POPT_ARG_NONE, &unique_node_name, 0,
        "Use unique ROS node name", "NONE" },
    { "initial_poses", 'i', POPT_ARG_NONE, &return_initial_poses, 0,
        "Return intial, instead of final, pose estimates", "NONE" },
    { "object_clustering", 'o', POPT_ARG_NONE, &save_stfs_, 0,
        "Do object clustering", "NONE"},
    { "disable-stfs", 'p', POPT_ARG_NONE, &disable_stfs, 0,
        "Disable STFs", "NONE"},
    { "save-ltfs", 'l', POPT_ARG_NONE, &save_ltfs_, 0,
        "Save LTFs", "NONE"},
    { "use-kinect", 'K', POPT_ARG_NONE, &use_kinect_, 0,
        "Use Kinect", "NONE"},
    { "save-episode-stats", 'v', POPT_ARG_NONE, &save_episode_stats_, 0,
        "Save episode stats", "NONE"},
    { "quiet", 'q', POPT_ARG_NONE, &quiet_, 0,
        "Quiet", "NONE"},
    POPT_AUTOHELP
    { NULL, 0, 0, NULL, 0, NULL, NULL }
  };
  POpt popt(NULL,argc,(const char**)argv,options,0);
  int c;
  while((c = popt.getNextOpt()) >= 0){
  }
  config_.init(watch_files_);
  config_.addFile("../robot.cfg");
  config_.addFile("config/non_markov_localization.cfg");
  config_.addFile("config/localization_parameters.cfg");
  CHECK(LoadConfiguration(&localization_options_));

  // if (bag_file == NULL) unique_node_name = true;
  const bool running_tests =
      (test_set_index_ >= 0 || statistical_test_index_ >= 0);
  if (running_tests) {
    const unsigned int seed = time(NULL);
    srand(seed);
    printf("Seeding with %u test=%.5f\n", seed, randn(1.0f));
  }
  const string node_name =
      (running_tests || unique_node_name) ?
      StringPrintf("NonMarkovLocalization_%lu",
                   static_cast<uint64_t>(GetTimeSec() * 1000000.0)) :
      string("NonMarkovLocalization");
  ros::init(argc, argv, node_name, ros::init_options::NoSigintHandler);
  ros::NodeHandle ros_node;
  display_publisher_ =
      ros_node.advertise<cobot_msgs::LidarDisplayMsg>(
      "Cobot/VectorLocalization/Gui",1,true);
  localization_publisher_ =
      ros_node.advertise<cobot_msgs::CobotLocalizationMsg>(
      "Cobot/Localization", 1, true);
  gui_capture_client =
      ros_node.serviceClient<cobot_msgs::LocalizationGuiCaptureSrv>(
      "VectorLocalization/Capture");

  LoadLaserCorrections();
  if (bag_file != NULL) {
    if (batch_mode) {
      const string keyframes = (keyframes_file == NULL) ? "" : keyframes_file;
      BatchLocalize(bag_file, keyframes, max_laser_poses,
                    !disable_stfs, time_skip, return_initial_poses);
    } else {
      PlayBagFile(bag_file, max_laser_poses, !disable_stfs,
                  time_skip, &ros_node, NULL);
    }
  } else {
    OnlineLocalize(!disable_stfs, &ros_node);
  }
  return 0;
}
