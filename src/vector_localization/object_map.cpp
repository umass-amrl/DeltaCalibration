#include "object_map.h"

#include <algorithm>
#include <dirent.h>
#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/Geometry>
#include <glog/logging.h>
#include <list>
#include <map>
#include <signal.h>
#include <stdio.h>
#include <string>
#include <utility>
#include <vector>

#include "libraries/CImg/CImg.h"
#include "libraries/connected_components/connected_components.h"
#include "perception_tools/perception_2d.h"
#include "shared/math/eigen_helper.h"
#include "shared/util/helpers.h"
#include "shared/util/timer.h"
#include "vector_localization/non_markov_localization.h"

#ifdef NDEBUG
  #define OMP_PARALLEL_FOR _Pragma("omp parallel for")
#else
  #define OMP_PARALLEL_FOR {}
#endif

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
using std::fwrite;
using std::fprintf;
using std::list;
using std::make_pair;
using std::map;
using std::pair;
using std::size_t;
using std::sort;
using std::string;
using std::vector;
using perception_2d::Pose2Df;

namespace EnmlMaps {
GridObject::GridObject(const PersistentObject& obj) {
  location = Vector2f(0.0, 0.0);
  angle = 0.0;
  pose_transform = Affine2f::Identity();
  image_resolution = obj.image_resolution;
  width = obj.width;
  height = obj.height;
  image_width = obj.image_width;
  image_height = obj.image_height;
  occupancy_image = obj.occupancy_image;
  occupancy_gradients = obj.occupancy_gradients;
}

bool GridObject::Save(const string& filename) const {
  // Save object model.
  // Object Model parameters:
  // * Location
  // * Angle
  // * Location covariance
  // * Angular variance
  // * Grid resolution
  // * Size
  // * Occupancy model
  // * Gradients of Occupancy model

  ScopedFile fid(filename, "w");
  if (fid() == NULL) return false;
  fwrite(&(location.x()), sizeof(location.x()), 1,
          fid());
  fwrite(&(location.y()), sizeof(location.y()), 1,
          fid());
  fwrite(&(angle), sizeof(angle), 1, fid());
  const Matrix2f location_covariance = Matrix2f::Zero();
  fwrite(location_covariance.data(), sizeof(float), 4, fid());
  const float angular_variance = 0.0;
  fwrite(&angular_variance, sizeof(angular_variance), 1, fid());
  fwrite(&image_resolution, sizeof(image_resolution), 1, fid());
  fwrite(&image_width, sizeof(image_width), 1, fid());
  fwrite(&image_height, sizeof(image_height), 1, fid());
  const unsigned int occupancy_image_size = image_width * image_height;
  const unsigned int occupancy_gradients_size =
      image_width * image_height * 2;
  // Sanity checks:
  if (occupancy_image_size != occupancy_image.size() ||
      occupancy_gradients_size != occupancy_gradients.size()) {
    fprintf(stderr, "ERROR: Occupancy Image size mismatch\n");
    return false;
  }
  bool error = (fwrite(
      occupancy_image.data(), sizeof(occupancy_image(0)),
      occupancy_image.size(), fid()) != occupancy_image_size);
  error = error || (fwrite(
      occupancy_gradients.data(), sizeof(occupancy_gradients(0)),
      occupancy_gradients.size(), fid()) != occupancy_gradients_size);
  if (error) {
    fprintf(stderr, "ERROR: Unable to save occupancy images\n");
    return false;
  }
  return true;
}

bool PersistentObject::Save(const string& filename) const {
  CHECK_EQ(instance_poses.size(), instance_timestamps.size());
  ScopedFile fid(filename, "w");
  if (fid() == NULL) return false;
  // TODO: Error handling.
  const size_t num_poses = instance_poses.size();
  fwrite(&num_poses, sizeof(num_poses),1, fid());
  for (size_t i = 0; i < num_poses; ++i) {
    fwrite(&(instance_poses[i].translation.x()),
        sizeof(instance_poses[i].translation.x()),1, fid());
    fwrite(&(instance_poses[i].translation.y()),
        sizeof(instance_poses[i].translation.y()),1, fid());
    fwrite(&(instance_poses[i].angle),
        sizeof(instance_poses[i].angle), 1, fid());
    fwrite(&(instance_timestamps[i]),
        sizeof(instance_timestamps[i]), 1, fid());
  }
  fwrite(&(image_resolution), sizeof(image_resolution), 1, fid());
  fwrite(&(image_width), sizeof(image_width), 1, fid());
  fwrite(&(image_height), sizeof(image_height), 1, fid());
  fwrite(occupancy_image.data(), sizeof(float),
         occupancy_image.size(), fid());
  fwrite(occupancy_gradients.data(), sizeof(float),
         occupancy_gradients.size(), fid());
  cimg_library::CImg<float> vis_image = 255.0 * occupancy_image;
  const string image_file = (filename.substr(0, filename.length() - 3) + "png");
  vis_image.save_png(image_file.c_str());
  return true;
}

PersistentObject::PersistentObject(const string& filename) {
  static const bool debug = false;
  ScopedFile fid(filename, "r");
  CHECK_NOTNULL(fid());
  // TODO: Error handling.
  bool error = false;
  size_t num_poses = 0;
  error = error || (fread(&num_poses, sizeof(num_poses),1, fid()) != 1);
  CHECK(!error);
  instance_poses.resize(num_poses);
  instance_timestamps.resize(num_poses);
  if (debug) {
    printf("Persistent object '%s', %d poses\n",
          filename.c_str(), static_cast<int>(num_poses));
  }
  for (size_t i = 0; !error && i < num_poses; ++i) {
    error = error || (fread(&(instance_poses[i].translation.x()),
        sizeof(instance_poses[i].translation.x()),1, fid()) != 1);
    error = error || (fread(&(instance_poses[i].translation.y()),
        sizeof(instance_poses[i].translation.y()),1, fid()) != 1);
    error = error || (fread(&(instance_poses[i].angle),
        sizeof(instance_poses[i].angle), 1, fid()) != 1);
    error = error || (fread(&(instance_timestamps[i]),
        sizeof(instance_timestamps[i]), 1, fid()) != 1);
  }
  error = error || (fread(&(image_resolution), sizeof(image_resolution),
      1, fid()) != 1);
  error = error || (fread(&(image_width), sizeof(image_width),
      1, fid()) != 1);
  error = error || (fread(&(image_height), sizeof(image_height),
      1, fid()) != 1);
  width = image_width * image_resolution;
  height = image_height * image_resolution;
  const unsigned int occupancy_image_size = image_width * image_height;
  occupancy_image = cimg_library::CImg<float>(image_width, image_height);
  error = error || (fread(occupancy_image.data(), sizeof(float),
      occupancy_image_size, fid()) != occupancy_image_size);
  const unsigned int occupancy_gradients_size =
      image_width * image_height * 2;
  occupancy_gradients = cimg_library::CImg<float>(
      image_width, image_height, 1, 2);
  error = error || (fread(occupancy_gradients.data(), sizeof(float),
      occupancy_gradients_size, fid()) != occupancy_gradients_size);
  if (error) {
    fprintf(stderr, "Error loading %s\n", filename.c_str());
  }
}

PersistentObject::PersistentObject(
    int object_id, const vector<size_t>& cluster,
    const vector<GridObject>& objects,
    const vector<vector<double> >& error_matrix,
    const vector<vector<Affine2f> >& transforms, const double log_timestamp) {
  float best_instance_score = 0.0;
  CHECK_GT(cluster.size(), 0);
  size_t best_instance = cluster[0];
  for (size_t j = 0; j < cluster.size(); ++j) {
    const size_t instance_j = cluster[j];
    float instance_score = 0.0;
    for (size_t k = 0; k < cluster.size(); ++k) {
      if (j == k) continue;
      const size_t instance_k = cluster[k];
      instance_score += error_matrix[instance_j][instance_k];
      instance_score += error_matrix[instance_k][instance_j];
    }
    if (instance_score > best_instance_score) {
      best_instance_score = instance_score;
      best_instance = cluster[j];
    }
  }
  // Find all the poses of the instances relative to the best instance and add
  // them.
  instance_poses.push_back(Pose2Df(0.0, objects[best_instance].location));
  for (size_t j = 0; j < cluster.size(); ++j) {
    if (cluster[j] == best_instance) continue;
    const Affine2f& transform = transforms[cluster[j]][best_instance];
    Rotation2Df rotation_matrix(0.0);
    rotation_matrix.fromRotationMatrix(transform.rotation());
    const float rotation = -rotation_matrix.angle();
    const Vector2f translation =
        transform.inverse(Eigen::Affine) * Vector2f(0.0, 0.0);
    instance_poses.push_back(Pose2Df(rotation, translation));
  }
  const GridObject& best_model = objects[best_instance];
  this->height = best_model.height;
  this->width = best_model.width;
  this->image_height = best_model.image_height;
  this->image_width = best_model.image_width;
  this->image_resolution = best_model.image_resolution;
  this->occupancy_gradients = best_model.occupancy_gradients;
  this->occupancy_image = best_model.occupancy_image;
  this->points = best_model.points;
  instance_timestamps = vector<double>(instance_poses.size(), log_timestamp);
  // const float num_comparisons =
  //     2.0 * static_cast<float>(cluster.size() - 1);
  // printf("Object %d has best cluster %d, score %.2f\n",
  //         object_id, static_cast<int>(best_instance),
  //         best_instance_score / num_comparisons);
}

void PersistentObject::Merge(
    const PersistentObject& other, const Affine2f& tf) {
  for (size_t j = 0; j < other.instance_poses.size(); ++j) {
    const Affine2f old_pose =
        Translation2f(other.instance_poses[j].translation) *
        Rotation2Df(other.instance_poses[j].angle);
    const Affine2f new_pose =
        old_pose * (tf.inverse(Eigen::Affine));
    Rotation2Df rotation_matrix(0.0);
    rotation_matrix.fromRotationMatrix(new_pose.rotation());
    const float rotation = rotation_matrix.angle();
    const Vector2f translation = new_pose.translation();
    instance_poses.push_back(Pose2Df(rotation, translation));
  }
  instance_timestamps.insert(
      instance_timestamps.end(), other.instance_timestamps.begin(),
      other.instance_timestamps.end());
}

bool LoadPersistentObjects(
    const string& map_directory, vector<PersistentObject>* objects_ptr) {
  CHECK_NOTNULL(objects_ptr);
  vector<PersistentObject>& objects = *objects_ptr;
  DIR *dp = NULL;
  struct dirent *dirp;
  dp = opendir(map_directory.c_str());
  if (dp == NULL) {
    fprintf(stderr,
            "ERROR: Unable to get directory listing for object map %s\n",
            map_directory.c_str());
    return false;
  }
  // Read filenames from directory
  while ((dirp = readdir(dp)) != NULL) {
    const string filename = string(dirp->d_name);
    // Read only .dat files
    if (filename.find(".dat") == string::npos) continue;
    objects.push_back(PersistentObject(map_directory + "/" + filename));
  }
  printf("Loaded %d persistent objects.\n", static_cast<int>(objects.size()));
  return true;
}


bool SavePersistentObjects(
    const string& map_directory, const vector<PersistentObject>& objects) {
  bool error = false;
  for (size_t i = 0; !error && i < objects.size(); ++i) {
    const string file_name = map_directory +
        StringPrintf("%05d.dat", static_cast<int>(i));
    error = !(objects[i].Save(file_name));
  }
  return (!error);
}

}  // namespace EnmlMaps
