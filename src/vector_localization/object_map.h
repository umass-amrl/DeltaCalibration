#ifndef MODEL_INSTANCE_MAP_H_
#define MODEL_INSTANCE_MAP_H_

#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/Geometry>
#include <string>
#include <vector>

#include "libraries/CImg/CImg.h"
#include "perception_tools/perception_2d.h"
#include "vector_localization/non_markov_localization.h"

namespace EnmlMaps {

struct PersistentObject;

struct ObjectMapOptions {
  // The maximum separation for connected points in the same object.
  float object_distance_threshold;

  // The minimum size of an object.
  unsigned int min_object_points;

  float laser_angular_resolution;
  float image_resolution;
  float image_border;
  float min_sdf_value;
  float max_sdf_value;
  float min_sdf_weight;
  float max_sdf_weight;
  float min_sdf_inference_weight;
  bool generate_sdf;
  float sdf_mistrust_weight;
  float matching_angular_resolution;
  float matching_delta_loc;
  float matching_loc_resolution;
  float max_clique_kld_value;
  float min_clique_overlap_value;
  float laser_std_dev;
  float epsilon_occupancy;
  float occupancy_threshold;
  float good_match_threshold;
};


// Struct that represents an instance of an object as an occupancy grid.
struct GridObject {
  Eigen::Vector2f location;
  float angle;
  Eigen::Affine2f pose_transform;
  float image_resolution;
  float width;
  float height;
  unsigned int image_width;
  unsigned int image_height;
  cimg_library::CImg<float> occupancy_image;
  cimg_library::CImg<float> occupancy_gradients;
  std::vector<Eigen::Vector2f> points;

  // Saves the GridObject to the specified filename, returning true on
  // success and false on error.
  bool Save(const std::string& filename) const;
  explicit GridObject(const PersistentObject& obj);
  GridObject() {}
};

// Struct that represents an instance of an object as an occupancy grid.
struct PersistentObject {
  std::vector<perception_2d::Pose2Df> instance_poses;
  std::vector<double> instance_timestamps;
  float image_resolution;
  float width;
  float height;
  unsigned int image_width;
  unsigned int image_height;
  cimg_library::CImg<float> occupancy_image;
  cimg_library::CImg<float> occupancy_gradients;
  std::vector<Eigen::Vector2f> points;

  // Saves the PersistentObject to the specified filename, returning true on
  // success and false on error.
  bool Save(const std::string& filename) const;
  explicit PersistentObject(const std::string& filename);
  PersistentObject(
      int object_id, const std::vector<std::size_t>& cluster,
      const std::vector<GridObject>& objects,
      const std::vector<std::vector<double> >& error_matrix,
      const std::vector<std::vector<Eigen::Affine2f> >& transforms,
      const double log_timestamp);
  void Merge(const PersistentObject& other, const Eigen::Affine2f& tf);
};

// Load all persistent objects found in the specified map directory into the
// provided vector, and return true on success, false on failure.
bool LoadPersistentObjects(
    const std::string& map_directory,
    std::vector<PersistentObject>* objects_ptr);

// Save all persistent objects to the specified map directory from the
// provided vector, and return true on success, false on failure.
bool SavePersistentObjects(
    const std::string& map_directory,
    const std::vector<PersistentObject>& objects);

/*
// Merge an old set of objects with a new set of objects, maintaining and
// merging the models as necessary based on the strongly connected components
// of the similarity graph.
void MergePersistentObjects(
    const ObjectMapOptions& map_options,
    std::vector<PersistentObject>* new_objects_ptr,
    std::vector<PersistentObject>* old_objects_ptr);
*/

}  // namespace EnmlMaps

#endif
