//----------- INCLUDES
#include "delta_calibration/icp.h"
#include <string>
#include <sstream>
#include <iostream>
#include <fstream>
#include <cmath>
#include <math.h>
// ROS INCLUDES
#include <ros/ros.h>
#include <rosbag/bag.h>
#include <rosbag/view.h>
#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/PointCloud2.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
// PCL specific includes
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/common/transforms.h>
#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
#include <pcl/io/obj_io.h>
#include <pcl/pcl_exports.h>
#include <pcl/console/parse.h>
#include <pcl/features/normal_3d.h>
#include <pcl/features/normal_3d_omp.h>
#include <pcl/io/pcd_io.h>
#include "pcl/point_types.h"
#include "pcl/impl/instantiate.hpp"
#include <pcl/filters/voxel_grid.h>
// Opencv includes
#include "opencv2/opencv_modules.hpp"
#include <stdio.h>
#include <opencv2/opencv.hpp>
# include "opencv2/core/core.hpp"
# include "opencv2/features2d/features2d.hpp"
# include "opencv2/highgui/highgui.hpp"
#include <cv_bridge/cv_bridge.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseArray.h>
// # include "opencv2/nonfree/features2d.hpp"
# include "opencv2/nonfree/features2d.hpp"
#include "delta_calibration/icp.h"

// #include <libfreenect2/src/opencl_depth_packet_processor.cl>

using namespace cv;
using namespace icp;

//Intialize empty publishers
ros::Publisher cloud_pub_1;
ros::Publisher cloud_pub_2;
ros::Publisher cloud_pub_3;
ros::Publisher cloud_pub_4;

ros::Publisher marker_pub;
ros::Publisher markerArray_pub;
ros::Publisher poses;

// Signal handler for breaks (Ctrl-C)
void HandleStop(int i) {
  printf("\nTerminating.\n");
  exit(0);
}


// For checking if the mean has not changed
bool DoubleEquals(double x, double y) {
  return fabs(x-y) < .00005;
}

bool TimeEquals(double time1, double time2) {
  return abs(time1 - time2) < .00001;

}

bool TimeEquals_Depth(double time1, double time2) {
  return abs(time1 - time2) < .015;

}

string type2str(int type) {
  string r;

  uchar depth = type & CV_MAT_DEPTH_MASK;
  uchar chans = 1 + (type >> CV_CN_SHIFT);

  switch ( depth ) {
    case CV_8U:  r = "8U"; break;
    case CV_8S:  r = "8S"; break;
    case CV_16U: r = "16U"; break;
    case CV_16S: r = "16S"; break;
    case CV_32S: r = "32S"; break;
    case CV_32F: r = "32F"; break;
    case CV_64F: r = "64F"; break;
    default:     r = "User"; break;
  }

  r += "C";
  r += (chans+'0');

  return r;
}

// Transforms a given eigen point by the given transform (array input)
template <class T> Eigen::Matrix<T,3,1> TransformPointQuat(
  const Eigen::Matrix<T,3,1>& point,
  const T* transform) {
  
  T point_t[] = {T(point.x()), T(point.y()), T(point.z())};
  T transformed_point[3];
  //   const T* transform_rotated[4];
  //   transform_rotated[0] = transform[3];
  //   transform_rotated[1] = transform[0];
  //   transform_rotated[2] = transform[1];
  //   transform_rotated[3] = transform[2];
  ceres::QuaternionRotatePoint(transform, point_t, transformed_point);
  for (int i = 0; i < 3; ++i) {
    transformed_point[i] += transform[4 + i];
  }
  return (Eigen::Matrix<T, 3, 1>(
    transformed_point[0],
    transformed_point[1],
    transformed_point[2]));
  }
  
  // Transforms all points in a pcl point cloud (array input)
  void TransformPointCloudQuaternion(
    pcl::PointCloud<pcl::PointXYZ>& cloud,
    double transform[7]) {
    OMP_PARALLEL_FOR
    for(size_t i = 0; i < cloud.size(); ++i) {
      pcl::PointXYZ point = cloud[i];
      
      Eigen::Matrix<double,3,1> point_matrix;
      Eigen::Matrix<double,3,1> transformed_point_matrix;
      point_matrix << point.x, point.y, point.z;
      transformed_point_matrix = TransformPointQuat(point_matrix, transform);
      point.x = transformed_point_matrix[0];
      point.y = transformed_point_matrix[1];
      point.z = transformed_point_matrix[2];
      cloud[i] = point;
    }
    }

void KeyframesFromBag(rosbag::View::iterator it,
                      rosbag::View::iterator end,
                      rosbag::View::iterator it_rgb,
                      rosbag::View::iterator end_rgb,
                      const vector<double*>& trajectory,
                      const vector<double>& times,
                      vector<Mat>* images,
                      vector<sensor_msgs::Image>* depth_images,
                      vector<vector<int> >* pixel_mappings,
                      vector<std::map<int,int> >* depths_to_clouds,
                      vector<pcl::PointCloud<pcl::PointXYZ> >* clouds) {

  PlaneFilter filter;
  KinectOpenNIDepthCam camera = KinectOpenNIDepthCam();
  filter.setDepthCamera(&camera);
  for(size_t i = 0; i < times.size(); i++) {
    double time = times[i];
    bool found = false;
    while(!found && it != end && it_rgb != end_rgb) {
      const rosbag::MessageInstance &depth_m = *it;
      sensor_msgs::ImagePtr depth_msg = depth_m.instantiate<sensor_msgs::Image>();
      const rosbag::MessageInstance &rgb_m = *it_rgb;
      sensor_msgs::ImagePtr rgb_msg = rgb_m.instantiate<sensor_msgs::Image>();
      // Going to need to also get the mapping from depth to color and depth to point cloud
      if (depth_msg != NULL) {
        vector<Eigen::Vector3f> pointCloud;
        vector<int> pixelLocs;
        fprintf(stdout, "Current Key Time: %f \t Msg Time: %f  Depth Time: %f  \n", time, rgb_msg->header.stamp.toSec(), depth_msg->header.stamp.toSec());
        if(TimeEquals(rgb_msg->header.stamp.toSec(), time)) {
          found = true;

         // Get RGB Image
         cv_bridge::CvImagePtr cv_ptr;
         try {
           cv_ptr = cv_bridge::toCvCopy(rgb_msg, sensor_msgs::image_encodings::BGR8);
         }
         catch (cv_bridge::Exception& e) {
           ROS_ERROR("cv_bridge exception: %s", e.what());
           return;
         }
         Mat rgb_mat = cv_ptr->image;
         int num_pixels = 640*480;
         Mat depth_colorized(480,640, CV_8UC3, Scalar(0,0,0));
         string ty =  type2str( depth_colorized.type() );
//          printf("Matrix: %s %dx%d \n", ty.c_str(), depth_colorized.cols, depth_colorized.rows );
         for (int j = 0; j < num_pixels; ++j) {
           if (camera.isValidDepth(j, (void*)depth_msg->data.data())) {
             void* depthimage = (void*)depth_msg->data.data();
             uint16_t* _depth = (uint16_t*)depthimage;
             uint16_t depthVal = _depth[j];
             if(depthVal != 0) {
             int x = j % 640;
             int y = j / 640;
//              cout << x << endl;
//              cout << y << endl;
             depth_colorized.at<Vec3b>(y,x) = rgb_mat.at<Vec3b>(y,x);
             }
           }
         }
         cv::Point point;
         point.x = 639;
         point.y =479;
//          circle(depth_colorized, point, 10, 1);
//          circle(rgb_mat, point, 10, 1);
//          imshow( "Good Matches", depth_colorized );
//          waitKey(0);
//
//          imshow("color", rgb_mat);
//          waitKey(0);
         images->push_back(depth_colorized);
         // Get Cloud
//          cout << "Depth Retrieved" << endl;
         filter.GenerateCompletePointCloud((void*)depth_msg->data.data(),
                                           pointCloud, pixelLocs);
         pcl::PointCloud<pcl::PointXYZ> pcl_cloud =
         icp::CloudFromVector(pointCloud, pixelLocs);
         clouds->push_back(pcl_cloud);
         pixel_mappings->push_back(pixelLocs); // Index in cloud gives index in image
         std::map<int,int> depth_to_cloud;
         depth_images->push_back(*depth_msg);
         for(size_t j = 0; j < pixelLocs.size(); j++) {
           depth_to_cloud[pixelLocs[j]] = j;
         }
         depths_to_clouds->push_back(depth_to_cloud); // index in image gives index in cloud
        }
        if(TimeEquals_Depth(depth_msg->header.stamp.toSec(), time)) {

        }
      }
      //cout << "advance" << endl;
      advance(it, 1);
      advance(it_rgb, 1);
    }
  }
}

void FeatureIdentification(const vector<Mat>& images, vector<Mat>* descriptors, vector<vector<KeyPoint> >* keypoints) {
  // Default parameters of ORB
//   int nfeatures=500;
//   float scaleFactor=1.2f;
//   int nlevels=8;
//   int edgeThreshold=15; // Changed default (31);
//   int firstLevel=0;
//   int WTA_K=2;
//   int scoreType=ORB::HARRIS_SCORE;
//   int patchSize=31;
//   int fastThreshold=20;

//   OrbFeatureDetector detector(1000, 1.2f, 8, 15, 0, 4);
//   OrbDescriptorExtractor extractor;
  //-- Step 1: Detect the keypoints using SURF Detector
  int minHessian = 400;

  SurfFeatureDetector detector( minHessian );
  SurfDescriptorExtractor extractor;
  for(size_t i = 0; i < images.size(); i++) {
    vector<KeyPoint> key_points;
    Mat grayscale;
    cvtColor(images[i], grayscale, CV_RGB2GRAY);
    detector.detect(images[i], key_points);
//     std::/*cout*/ << "Found " << key_points.size() << " Keypoints " << std::endl;
    Mat out;
    drawKeypoints(images[i], key_points, out, Scalar::all(255));

// //     imshow("Kpts", out);
// //
// //     waitKey(0);
    Mat descriptors_1;

    extractor.compute( images[i], key_points, descriptors_1 );
    keypoints->push_back(key_points);
    descriptors->push_back(descriptors_1);

  }
}

void FeatureMatching(vector<pcl::PointCloud<pcl::PointXYZ> > clouds_1,
                     vector<pcl::PointCloud<pcl::PointXYZ> > clouds_2,
                     vector<Mat> descriptors_1,
                     vector<Mat> descriptors_2,
                     vector<vector<DMatch> >* all_matches,
                      vector<vector<DMatch> >* all_good_matches) {
  BFMatcher matcher(NORM_HAMMING);
  
  for(size_t i = 0; i < descriptors_1.size(); i++) {
    cout << "Descriptors 1: " << i << endl;
    Mat descriptor1 = descriptors_1[i];
    for(size_t j = 0; j < descriptors_2.size(); j++) {
      cout << "Descriptors 2: " << j << endl;
      std::vector< DMatch > matches;
      Mat descriptor2 = descriptors_2[j];
      if ( descriptor1.empty() )
        cvError(0,"MatchFinder","1st descriptor empty",__FILE__,__LINE__);
      if ( descriptor2.empty() )
        cvError(0,"MatchFinder","2nd descriptor empty",__FILE__,__LINE__);
      matcher.match( descriptor1, descriptor2, matches );
      cout << "matched" << endl;
      std::vector<cv::Point2f> fp[2];
      std::vector<cv::DMatch>::iterator iter;
      fp[0].clear();
      fp[1].clear();
      cout << matches.size() << endl;
//       for (iter = matches.begin(); iter != matches.end(); ++iter)
//       {
//         //if (iter->distance > 1000)
//         //  continue;
//        fp[0].push_back(keypoints_1[i].at(iter->queryIdx).pt);
//        fp[1].push_back(keypoints_2[j].at(iter->trainIdx).pt);
//       }
//       // remove outliers
//       std::vector<uchar> mask;
// 
//       findFundamentalMat(fp[0], fp[1], cv::FM_RANSAC, 1, 1, mask);
// 
//       std::vector<cv::Point2f> fp_refined[2];
//       for (size_t k = 0; k < mask.size(); ++k)
//       {
//         if (mask[k] != 0)
//         {
//           fp_refined[0].push_back(fp[0][k]);
//           fp_refined[1].push_back(fp[1][k]);
//         }
//       }
// 
//       std::swap(fp_refined[0], fp[0]);
//       std::swap(fp_refined[1], fp[1]);
//       cout << "ransaced" << endl;
      all_matches->push_back(matches);

      double max_dist = 0; double min_dist = 100;
      // Naive way of finding quality matches
      //-- Quick calculation of max and min distances between keypoints
      for( int k = 0; k < descriptor1.rows; k++ )
      { double dist = matches[k].distance;
        if( dist < min_dist ) min_dist = dist;
        if( dist > max_dist ) max_dist = dist;
      }
      cout << "dist calculated" << endl;

      //-- only "good" matches (i.e. whose distance is less than 2*min_dist,
      //-- or a small arbitary value ( 0.02 ) in the event that min_dist is very
      //-- small)
//       std::vector< DMatch > good_matches;
      std::vector< DMatch > best_matches;
//       for( int k = 0; k < descriptor1.rows; k++ )
//       { if( mask[k] != 0 )
//         { good_matches.push_back( matches[k]); }
//       }
      best_matches.clear();
      for( size_t k = 0; k < matches.size(); k++ ) { 
        cout <<  min_dist << endl;
        if( matches[k].distance <= 20) { 
          best_matches.push_back( matches[k]); 
        }
      } 
      cout << "best matches selected " << endl;
      all_good_matches->push_back(best_matches);

      visualization_msgs::Marker line_list;
      std_msgs::Header header;
      header.frame_id = "point_clouds";
      line_list.action = visualization_msgs::Marker::ADD;
      //line_list.pose.orientation.w = 1.0;
      line_list.header.frame_id = "point_cloud";

      line_list.id = 0;
      line_list.type = visualization_msgs::Marker::LINE_LIST;
      line_list.scale.x = 0.001;

      // Line list is red
      line_list.color.r = 1.0;
      line_list.color.a = 1.0;
      pcl::PointCloud<pcl::PointXYZ> cloud_1 = clouds_1[i];
      pcl::PointCloud<pcl::PointXYZ> cloud_2 = clouds_2[j];
      cout << "vis setup " << endl;
      for( int k = 0; k < (int)best_matches.size(); k++ ){ 
//         KeyPoint key_1 = keypoints_1[i][best_matches[k].queryIdx];
//         KeyPoint key_2 = keypoints_2[j][best_matches[k].trainIdx];
//         Point point, point2;
//         point.x = key_1.pt.x;
//         point.y = key_1.pt.y;
//         int NUM_COLS = 640;
//         int index_1, index_2;
//         index_1 = (640*(int)key_1.pt.y) + (int)key_1.pt.x;
//         point2.x = key_2.pt.x + 640;
//         point2.y = key_2.pt.y;
        geometry_msgs::Point combined_result, zero;
        zero.x = 0;
        zero.y = 0;
        zero.z = 0;
        pcl::PointXYZ l_point, k_point;
//         vector<Eigen::Vector3f> allPoints_1, allPoints_2;
//         vector<int> pixelLocs;
        
//         index_2 = NUM_COLS*(int)key_2.pt.y + (int)key_2.pt.x
        k_point = cloud_2[best_matches[k].trainIdx];
        l_point = cloud_1[best_matches[k].queryIdx];
        geometry_msgs::Point p, p2;
        p.x = k_point.x;
        p.y = k_point.y;
        p.z = k_point.z;
        p2.x = l_point.x;
        p2.y = l_point.y;
        p2.z = l_point.z;
        line_list.points.push_back(p);
        line_list.points.push_back(p2);
      }
      PublishCloud(cloud_1, cloud_pub_1);
      PublishCloud(cloud_2, cloud_pub_2);
      marker_pub.publish(line_list);
      }
      cout << "published" << endl;
    }
  }
  
  void VisualizeMatches(vector<pcl::PointCloud<pcl::PointXYZ> > clouds_1,
                       vector<pcl::PointCloud<pcl::PointXYZ> > clouds_2,
                       vector<double*> trajectory_1,
                       vector<double*> trajectory_2,
                       vector<Mat> descriptors_1,
                       vector<Mat> descriptors_2,
                       vector<vector<DMatch> > good_matches) {
    
    visualization_msgs::Marker line_list;
    std_msgs::Header header;
    header.frame_id = "point_cloud";
    line_list.action = visualization_msgs::Marker::ADD;
    //line_list.pose.orientation.w = 1.0;
    line_list.header.frame_id = "point_cloud";
    
    line_list.id = 50;
    line_list.type = visualization_msgs::Marker::LINE_LIST;
    line_list.scale.x = .0001;
    // Line list is red
    line_list.color.r = 1.0;
    line_list.color.a = 1.0;
    pcl::PointCloud<pcl::PointXYZ> map_1, map_2;
    for(size_t i = 0; i < clouds_1.size(); i++) {
      pcl::PointCloud<pcl::PointXYZ> cloud_1 = clouds_1[i];
      double* transform_1 = trajectory_1[i];
      TransformPointCloudQuaternion(cloud_1, transform_1);
      map_1 += cloud_1;
      for(size_t j = 0; j < clouds_2.size(); j++) {
        pcl::PointCloud<pcl::PointXYZ> cloud_2 = clouds_2[j];
        double* transform_2 = trajectory_2[j];
        
        TransformPointCloudQuaternion(cloud_2, transform_2);
        if(i == 0) {
          map_2 += cloud_2;
        }
        vector<DMatch> matches = good_matches[i * (int)clouds_2.size() + j];
        for( size_t k = 0; k < matches.size(); k++ ) { 
          geometry_msgs::Point combined_result, zero;
          zero.x = 0;
          zero.y = 0;
          zero.z = 0;
          pcl::PointXYZ l_point, k_point;
          k_point = cloud_2[matches[k].trainIdx];
          l_point = cloud_1[matches[k].queryIdx];
          geometry_msgs::Point p, p2;
          p.x = k_point.x;
          p.y = k_point.y;
          p.z = k_point.z;
          p2.x = l_point.x;
          p2.y = l_point.y;
          p2.z = l_point.z;
          line_list.points.push_back(p);
          line_list.points.push_back(p2);
        }
        
      }
    }
    while(true){
    PublishCloud(map_1, cloud_pub_1);
    PublishCloud(map_2, cloud_pub_2);
    cout << line_list.points.size() << endl;
    marker_pub.publish(line_list);
    sleep(3);
    }
}


//Individual camera trajectory transforms only
struct FeatureError {
  FeatureError(const Vector3d& point_0,
               const Vector3d& point_1,
               double* transform
              ) :
      point_0(point_0),
      point_1(point_1),
      transform(transform){}

  template <class T>
  bool operator()(const T* const camera,
                  T* residuals) const {
    // Transform point_1 to the base reference frame by applying the camera tf.
    Eigen::Matrix<T, 3, 1> point_start;
    point_start[0] = (T)point_1[0];
    point_start[1] = (T)point_1[1];
    point_start[2] = (T)point_1[2];
    const Eigen::Matrix<T, 3, 1> point_1_transformed =
    TransformPointQuat<T>(point_1.cast<T>(), camera);
    T* Ttransform = new T[7];
    Ttransform[0] = (T)transform[0];
    Ttransform[1] = (T)transform[1];
    Ttransform[2] = (T)transform[2];
    Ttransform[3] = (T)transform[3];
    Ttransform[4] = (T)transform[4];
    Ttransform[5] = (T)transform[5];
    Ttransform[6] = (T)transform[6];
    const Eigen::Matrix<T, 3, 1> point_0_transformed =
    TransformPointQuat<T>(point_0.cast<T>(), Ttransform);
    // The error is the difference between the predicted and observed position.
    residuals[0] =
    point_0_transformed[0] - point_1_transformed[0];
    residuals[1] =
    point_0_transformed[1] - point_1_transformed[1];
    residuals[2] =
    point_0_transformed[2] - point_1_transformed[2];
    return true;
  }

  // Factory to hide the construction of the CostFunction object from
  // the client code.
  static ceres::CostFunction* Create(const Vector3d& point_0,
                                     const Vector3d& point_1,
                                     double* transform
                                    ) {
    return (new ceres::AutoDiffCostFunction<FeatureError, 3, 7>(
            new FeatureError(point_0, point_1, transform)));
  }

 const Vector3d point_0;
 const Vector3d point_1;
 double* transform;
};

// Feature error with extrinsics and individual camera trajectory transforms
struct FeatureErrorCrossMap {
  FeatureErrorCrossMap(const Vector3d& point_0,
               const Vector3d& point_1,
               double* pose_1,
               double* pose_2
  ) :
  point_0(point_0),
  point_1(point_1),
  pose_1(pose_1),
  pose_2(pose_2){}

  template <class T>
  bool operator()(const T* const extrinsic,
                  T* residuals) const {
// //       Transform point_1 to the base reference frame by applying the camera tf.
      T* Ttransform = new T[7];
      Ttransform[0] = (T)pose_1[0];
      Ttransform[1] = (T)pose_1[1];
      Ttransform[2] = (T)pose_1[2];
      Ttransform[3] = (T)pose_1[3];
      Ttransform[4] = (T)pose_1[4];
      Ttransform[5] = (T)pose_1[5];
      Ttransform[6] = (T)pose_1[6];           
      T* Ttransform2 = new T[7];
      Ttransform2[0] = (T)pose_2[0];
      Ttransform2[1] = (T)pose_2[1];
      Ttransform2[2] = (T)pose_2[2];
      Ttransform2[3] = (T)pose_2[3];
      Ttransform2[4] = (T)pose_2[4];
      Ttransform2[5] = (T)pose_2[5];
      Ttransform2[6] = (T)pose_2[6];      
//       T point_1_transformed[3];
//       T point_2_transformed[3];
//       point_1_transformed[0] = (T)point_0[0];
//       point_1_transformed[1] = (T)point_0[1];
//       point_1_transformed[2] = (T)point_0[2];
//       point_2_transformed[0] = (T)point_1[0];
//       point_2_transformed[1] = (T)point_1[1];
//       point_2_transformed[2] = (T)point_1[2];
//       ceres::QuaternionRotatePoint(Ttransform2, point_2_transformed, point_2_transformed);
//       point_2_transformed[0] = point_2_transformed[0] + pose_2[4];
//       point_2_transformed[1] = point_2_transformed[1] + pose_2[5];
//       point_2_transformed[2] = point_2_transformed[2] + pose_2[6];
//       ceres::QuaternionRotatePoint(extrinsic, point_2_transformed, point_2_transformed);
//       point_2_transformed[0] = point_2_transformed[0] + extrinsic[4];
//       point_2_transformed[1] = point_2_transformed[1] + extrinsic[5];
//       point_2_transformed[2] = point_2_transformed[2] + extrinsic[6];
//       ceres::QuaternionRotatePoint(Ttransform, point_1_transformed, point_1_transformed);
//       point_1_transformed[0] = point_1_transformed[0] + pose_1[4];
//       point_1_transformed[1] = point_1_transformed[1] + pose_1[5];
//       point_1_transformed[2] = point_1_transformed[2] + pose_1[6];
      // The error is the difference between the predicted and observed position.
//       TransformPointQuat<T>(point_1_transformed, Ttransform);
//       TransformPointQuat<T>(point_2_transformed, Ttransform2);
//       TransformPointQuat<T>(point_2_transformed, extrinsic);
      const Eigen::Matrix<T, 3, 1> point_0_transformed =
          TransformPointQuat<T>(point_1.cast<T>(), Ttransform2);
      const Eigen::Matrix<T, 3, 1> point_2_transformed =
          TransformPointQuat<T>(point_0_transformed, extrinsic);
      const Eigen::Matrix<T, 3, 1> point_1_transformed =
      TransformPointQuat<T>(point_0.cast<T>(), Ttransform);
      residuals[0] =
      point_2_transformed[0] - point_1_transformed[0];
      residuals[1] =
      point_2_transformed[1] - point_1_transformed[1];
      residuals[2] =
      point_2_transformed[2] - point_1_transformed[2];
      return true;
    }

    // Factory to hide the construction of the CostFunction object from
    // the client code.
    static ceres::CostFunction* Create(const Vector3d& point_0,
                                       const Vector3d& point_1,
                                       double* pose_1,
                                       double* pose_2
    ) {
      return (new ceres::AutoDiffCostFunction<FeatureErrorCrossMap, 3, 7>(
        new FeatureErrorCrossMap(point_0, point_1, pose_1, pose_2)));
    }

const Vector3d point_0;
const Vector3d point_1;
double* pose_1;
double* pose_2;
};

// Feature error with extrinsics and individual camera trajectory transforms
struct FeatureErrorCrossMapFullRefine {
  FeatureErrorCrossMapFullRefine(const Vector3d& point_0,
               const Vector3d& point_1
  ) :
  point_0(point_0),
  point_1(point_1){}

  template <class T>
  bool operator()(const T* const extrinsic, const T* const pose_1, const T* const pose_2,
                  T* residuals) const {
// //       Transform point_1 to the base reference frame by applying the camera tf.
//       T point_1_transformed[3];
//       T point_2_transformed[3];
//       point_1_transformed[0] = (T)point_0[0];
//       point_1_transformed[1] = (T)point_0[1];
//       point_1_transformed[2] = (T)point_0[2];
//       point_2_transformed[0] = (T)point_1[0];
//       point_2_transformed[1] = (T)point_1[1];
//       point_2_transformed[2] = (T)point_1[2];
//       ceres::QuaternionRotatePoint(Ttransform2, point_2_transformed, point_2_transformed);
//       point_2_transformed[0] = point_2_transformed[0] + pose_2[4];
//       point_2_transformed[1] = point_2_transformed[1] + pose_2[5];
//       point_2_transformed[2] = point_2_transformed[2] + pose_2[6];
//       ceres::QuaternionRotatePoint(extrinsic, point_2_transformed, point_2_transformed);
//       point_2_transformed[0] = point_2_transformed[0] + extrinsic[4];
//       point_2_transformed[1] = point_2_transformed[1] + extrinsic[5];
//       point_2_transformed[2] = point_2_transformed[2] + extrinsic[6];
//       ceres::QuaternionRotatePoint(Ttransform, point_1_transformed, point_1_transformed);
//       point_1_transformed[0] = point_1_transformed[0] + pose_1[4];
//       point_1_transformed[1] = point_1_transformed[1] + pose_1[5];
//       point_1_transformed[2] = point_1_transformed[2] + pose_1[6];
      // The error is the difference between the predicted and observed position.
//       TransformPointQuat<T>(point_1_transformed, Ttransform);
//       TransformPointQuat<T>(point_2_transformed, Ttransform2);
//       TransformPointQuat<T>(point_2_transformed, extrinsic);
      const Eigen::Matrix<T, 3, 1> point_0_transformed =
      TransformPointQuat<T>(point_1.cast<T>(), pose_1);
      const Eigen::Matrix<T, 3, 1> point_2_transformed =
          TransformPointQuat<T>(point_0_transformed, extrinsic);
      const Eigen::Matrix<T, 3, 1> point_1_transformed =
      TransformPointQuat<T>(point_0.cast<T>(), pose_2);
      residuals[0] =
      point_2_transformed[0] - point_1_transformed[0];
      residuals[1] =
      point_2_transformed[1] - point_1_transformed[1];
      residuals[2] =
      point_2_transformed[2] - point_1_transformed[2];
      return true;
    }

    // Factory to hide the construction of the CostFunction object from
    // the client code.
    static ceres::CostFunction* Create(const Vector3d& point_0,
                                       const Vector3d& point_1
    ) {
      return (new ceres::AutoDiffCostFunction<FeatureErrorCrossMapFullRefine, 3, 7, 7 , 7>(
        new FeatureErrorCrossMapFullRefine(point_0, point_1)));
    }

const Vector3d point_0;
const Vector3d point_1;
};

// Optimizes a single map based on feature correspondences
void IndividualMapOptimize(vector<Mat> images,
                      vector<pcl::PointCloud<pcl::PointXYZ> > clouds,
                      vector<Mat> descriptors,
                      const vector<sensor_msgs::Image>& depths_1,
                      vector<vector<DMatch> > matches,
                      vector<vector<KeyPoint> > key_points,
                      vector<vector<int> >  cloud_to_depth,
                      vector<map<int, int> > depths_to_clouds,
                      vector<double*> trajectory) {

  PlaneFilter filter;
  KinectOpenNIDepthCam camera = KinectOpenNIDepthCam();
  filter.setDepthCamera(&camera);


    for(size_t i = 1; i < images.size(); i++) {
//       int j = i -1;
      // Tolerance for RMSE.
      static const double kToleranceError = 0.00001;
      // The maximum number of overall iterations.
      static const int kMaxIterations = 40;
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
        ceres::Problem problem;
        if (DoubleEquals(rmse, last_rmse)) {
          repeat_iteration++;
        } else {
          repeat_iteration = 0;
        }
        last_rmse = rmse;
      sensor_msgs::Image depth_1 = depths_1[i];
      pcl::PointCloud<pcl::PointXYZ> cloud_1 = clouds[i];
      Mat descriptor_1 = descriptors[i];
      vector<KeyPoint> keypoint_1 = key_points[i];
      double* pose = trajectory[i];
      map<int,int> depth_to_cloud_1 = depths_to_clouds[i];


//       for(int j = (int)i - 1; j >= 0; j--) {
        int j = i - 1;
        double* pose_2 = trajectory[j];
        sensor_msgs::Image depth_2 = depths_1[j];
        cout << "J: " << j << endl;

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
        geometry_msgs::Point combined_result, zero;
        zero.x = 0;
        zero.y = 0;
        zero.z = 0;


        vector<KeyPoint> keypoint_2 = key_points[j];
        pcl::PointCloud<pcl::PointXYZ> cloud_2 = clouds[j];
        TransformPointCloud(cloud_2, pose_2);
        Mat descriptor_2 = descriptors[j];
        map<int,int> depth_to_cloud_2 = depths_to_clouds[j];
        vector<DMatch> current_matches  = matches[(i * clouds.size()) + j];

          // Construct the ceres Problem

          for(size_t k = 0; k < current_matches.size(); k++) {
            pcl::PointXYZ l_point, k_point;
            KeyPoint k_key = keypoint_2[current_matches[k].trainIdx];
            KeyPoint l_key = keypoint_1[current_matches[k].queryIdx];
            int NUM_COLS = 640;
            int k_index, l_index;
            k_index = NUM_COLS*(int)k_key.pt.y + (int)k_key.pt.x;
            l_index = NUM_COLS*(int)l_key.pt.y + (int)l_key.pt.x;
            k_point = cloud_2[depth_to_cloud_2[k_index]];
            l_point = cloud_1[depth_to_cloud_1[l_index]];
//             k_index = NUM_COLS*320 + 240;
//             l_index = NUM_COLS*320 + 240;
//             k_point = cloud_2[depth_to_cloud_2[k_index]];
//             l_point = cloud_1[depth_to_cloud_1[l_index]];

            Eigen::Vector3f cloudpt_1, cloudpt_2;
            Eigen::Matrix<double, 3, 1> pt_1, pt_2;
            cloudpt_1 = camera.depthPixelTo3D(l_index, (void*)depth_1.data.data());
            cloudpt_2 = camera.depthPixelTo3D(k_index, (void*)depth_2.data.data());
            pt_1[0] = cloudpt_1[0];
            pt_1[1] = cloudpt_1[1];
            pt_1[2] = cloudpt_1[2];
            pt_2[0] = cloudpt_2[0];
            pt_2[1] = cloudpt_2[1];
            pt_2[2] = cloudpt_2[2];
            pt_2 = TransformPoint(pt_2, pose_2);
            const Vector3d point_0(cloudpt_1[0], cloudpt_1[1], cloudpt_1[2]);
            const Vector3d point_1(cloudpt_2[0], cloudpt_2[1], cloudpt_2[2]);
            pt_1 = TransformPoint(pt_1, pose);
//             double dist = PointDistance(cloudpt_1, cloudpt_2);
            if(camera.isValidDepth(l_index, depth_1.data.data()) && camera.isValidDepth(k_index, depth_2.data.data())) {
              geometry_msgs::Point p, p2;
              p.x = pt_1[0];
              p.y = pt_1[1];
              p.z = pt_1[2];
              p2.x = pt_2[0]; //((temp_point2.x - p.x)*10) + p.x;
              p2.y = pt_2[1]; //((temp_point2.y - p.y)*10) + p.y;
              p2.z = pt_2[2]; //((temp_point2.z - p.z)*10) + p.z;
              line_list.points.push_back(p);
              line_list.points.push_back(p2);
              ceres::CostFunction* cost_function =
              FeatureError::Create(point_1,
                                    point_0,
                                   pose_2
                                    );
                problem.AddResidualBlock(cost_function,
                                         new ceres::HuberLoss(0.5),
                                          pose);
            }

          }


//       }
      // Run Ceres problem
      ceres::Solver::Options options;
      options.num_threads = 12;
      options.num_linear_solver_threads = 12;
      // options.use_explicit_schur_complement = true;
      // options.linear_solver_type = ceres::ITERATIVE_SCHUR;
      //           options.linear_solver_type = ceres::SPARSE_SCHUR;
      options.linear_solver_type = ceres::DENSE_QR;
      // options.minimizer_progress_to_stdout = true;
      // options.function_tolerance = 1e-10;
      ceres::Solver::Summary summary;
      ceres::Solve(options, &problem, &summary);
      rmse =
      sqrt(summary.final_cost / static_cast<double>(summary.num_residuals));
      cout << setprecision(20) << "RMSE: " << rmse << endl;
      //     (*trajectory)[i] = pose;
      pcl::PointCloud<pcl::PointXYZ> transformed_cloud = cloud_1;
      // Shifts cloud by calculated transform
      TransformPointCloud(transformed_cloud, pose);
      //                           TransformPointCloud(cloud_2, pose_2);
      PublishCloud(cloud_2, cloud_pub_1);
      PublishCloud(transformed_cloud, cloud_pub_2);
      marker_pub.publish(line_list);
//       sleep(3);

    }
  }
}

void DrawPoints(const vector<vector<Eigen::Vector3d> >& points) {
  visualization_msgs::Marker line_list;
  std_msgs::Header header;
  header.frame_id = "point_clouds";
  line_list.action = visualization_msgs::Marker::ADD;
  //line_list.pose.orientation.w = 1.0;
  line_list.header.frame_id = "point_cloud";
  
  line_list.id = 0;
  line_list.type = visualization_msgs::Marker::LINE_LIST;
  line_list.scale.x = 0.001;
  
  // Line list is red
  line_list.color.r = 1.0;
  line_list.color.a = 1.0;
  for(size_t i = 0; i < points[0].size(); i++) {
    geometry_msgs::Point p, p2;
    p.x = points[0][i][0];
    p.y = points[0][i][1];
    p.z = points[0][i][2];
    p2.x = points[1][i][0];
    p2.y = points[1][i][1];
    p2.z = points[1][i][2];
    line_list.points.push_back(p);
    line_list.points.push_back(p2);
  }
  marker_pub.publish(line_list);
  //   sleep(1);
}

// Optimizes a single map with icp
void ICPOptimizeMap(vector<pcl::PointCloud<pcl::PointXYZ> > clouds,
                    vector<double*>* trajectory) {

  for(size_t i = 1; i < clouds.size(); i++) {
    cout << "I: " << i << endl;
    pcl::PointCloud<pcl::PointXYZ> cloud_1 = clouds[i];
    double* pose_1 = (*trajectory)[i];
    int j = i -1;
    pcl::PointCloud<pcl::PointXYZ> cloud_2 = clouds[j];
    double* pose_2 = (*trajectory)[j];
    TransformPointCloud(cloud_2, pose_2);
    pcl::PointCloud<pcl::Normal> normal_1 = GetNormals(cloud_1);
    pcl::PointCloud<pcl::Normal> normal_2 = GetNormals(cloud_2);
    cout << "normals_computed" << endl;
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
      cout << iteration << endl;
      double nn_dist = .08;
      const std::vector<ros::Publisher> publishers;
      const std::vector<Eigen::Matrix<double, 2, 1> > empty;
      vector<int > nearest_neigbors;
      vector<int > start_points;
      ConstructICP_problem(publishers,
                           cloud_2,
                           cloud_1,
                           normal_2,
                           normal_1,
                           empty,
                           empty,
                           nn_dist,
                           pose_1,
                           &problem);
      // Run Ceres problem
      ceres::Solver::Options options;
      options.num_threads = 12;
      options.num_linear_solver_threads = 12;
      // options.use_explicit_schur_complement = true;
      // options.linear_solver_type = ceres::ITERATIVE_SCHUR;
      options.linear_solver_type = ceres::DENSE_QR;
      // options.minimizer_progress_to_stdout = true;
      // options.function_tolerance = 1e-10;
      ceres::Solver::Summary summary;
      ceres::Solve(options, &problem, &summary);
      rmse =
      sqrt(summary.final_cost / static_cast<double>(summary.num_residuals));
      cout << "RMSE: " << rmse << endl;
      //std::cout << summary.FullReport() << "\n";
      residuals.clear();
      //fprintf(stdout, "Residuals size: %f\n", residuals.size());
      pcl::PointCloud<pcl::PointXYZ> transformed_cloud = cloud_1;

      // Shifts cloud by calculated transform
      cout << "kdtree publish" << endl;
      TransformPointCloud(transformed_cloud, pose_1);
      PublishCloud(cloud_2, cloud_pub_1);
      PublishCloud(transformed_cloud, cloud_pub_2);
      KdTreeNN(nn_dist,
               cloud_2,
               transformed_cloud,
               normal_2,
               normal_1,
               empty,
               empty,
               nearest_neigbors,
               start_points);
      fprintf(stdout, "kdtree run\n");
      VisualizeNN(cloud_2, transformed_cloud, nearest_neigbors, start_points, marker_pub);
      (*trajectory)[i] = pose_1;
     }
  }
}

// Performs Optimization between maps formed from two points sensors (based on correspondences)
void FullMapOptimize(vector<pcl::PointCloud<pcl::PointXYZ> > clouds_1,
                     vector<pcl::PointCloud<pcl::PointXYZ> > clouds_2,
                     pcl::PointCloud<pcl::PointXYZ> map_1,
                     pcl::PointCloud<pcl::PointXYZ> map_2,
                     vector<double*> trajectory_1,
                     vector<double*> trajectory_2,
                     vector<vector<Eigen::Vector3d> > correspondences,
                     vector<vector<double* > > corr_trajectories,
                     double* extrinsic) {
  
    vector<vector<Eigen::Vector3d> > transformed_points;
    vector<Eigen::Vector3d> t_1;
    vector<Eigen::Vector3d> t_2;
    for(size_t j = 0; j < correspondences[0].size(); j++){
      Eigen::Vector3d point_1 = correspondences[0][j];
      Eigen::Vector3d point_2 = correspondences[1][j];
      point_1 = TransformPointQuat(point_1, corr_trajectories[0][j]);
      point_2 = TransformPointQuat(point_2, corr_trajectories[1][j]);
      //       point_1 = TransformPoint(point_1, zero_pose);  
      point_2 = TransformPointQuat(point_2, extrinsic);
      t_1.push_back(point_1);
      t_2.push_back(point_2);
    }
    pcl::PointCloud<pcl::PointXYZ> map_transformed1, map_transformed2;
    map_transformed1 = map_1;
    map_transformed2 = map_2;
    TransformPointCloudQuaternion(map_transformed2, extrinsic);
    transformed_points.push_back(t_1);
    transformed_points.push_back(t_2);
    PublishCloud(map_1, cloud_pub_1);
    PublishCloud(map_transformed2, cloud_pub_2);
    DrawPoints(transformed_points);
    cout << "Original Alignment Drawn" << endl;
    sleep(5);
    ceres::Problem problem;
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
      for(size_t i = 0; i < correspondences[0].size(); i++) {
        double* pose_1 = corr_trajectories[0][i];
        Eigen::Vector3d point_1 = correspondences[0][i];
        double* pose_2 = corr_trajectories[1][i];
        Eigen::Vector3d point_2 = correspondences[1][i];
        ceres::CostFunction* cost_function =
        FeatureErrorCrossMap::Create(point_1,
                                     point_2,
                                     pose_1,
                                     pose_2
        );
        double distance = PointDistance(point_1, point_2);
        cout << distance << endl;
        if(distance < 1) {
        problem.AddResidualBlock(cost_function,
                                new ceres::HuberLoss(0.5),
                                extrinsic
                                );
        }
//         problem.SetParameterBlockConstant(pose_1);
//         problem.SetParameterBlockConstant(pose_2);
         }
       
      // Run Ceres problem
      ceres::Solver::Options options;
      options.num_threads = 12;
      options.num_linear_solver_threads = 12;
      // options.use_explicit_schur_complement = true;
      // options.linear_solver_type = ceres::ITERATIVE_SCHUR;
      //           options.linear_solver_type = ceres::SPARSE_SCHUR;
      options.linear_solver_type = ceres::SPARSE_SCHUR;
      // options.minimizer_progress_to_stdout = true;
      // options.function_tolerance = 1e-10;
      ceres::Solver::Summary summary;
      double cost;
      ceres::Problem::EvaluateOptions evalOptions =
      ceres::Problem::EvaluateOptions();
      problem.Evaluate(evalOptions, &cost, &residuals, NULL, NULL);
      
      cout << setprecision(20) << "RMSE: " << rmse << endl;
      ceres::Solve(options, &problem, &summary);
      rmse =
      sqrt(cost / static_cast<double>(summary.num_residuals));
      rmse =
      sqrt(summary.final_cost / static_cast<double>(summary.num_residuals));
      cout << setprecision(20) << "RMSE: " << rmse << endl;
      t_2.clear();
      t_1.clear();
      transformed_points.clear();
      for(size_t j = 0; j < correspondences[0].size(); j++){
        Eigen::Vector3d point_1 = correspondences[0][j];
        Eigen::Vector3d point_2 = correspondences[1][j];
        point_1 = TransformPointQuat(point_1, corr_trajectories[0][j]);
        point_2 = TransformPointQuat(point_2, corr_trajectories[1][j]);
        //       point_1 = TransformPoint(point_1, zero_pose);  
        point_2 = TransformPointQuat(point_2, extrinsic);
        t_1.push_back(point_1);
        t_2.push_back(point_2);
      }
      map_transformed1 = map_1;
      map_transformed2 = map_2;
      TransformPointCloudQuaternion(map_transformed2, extrinsic);
      transformed_points.push_back(t_1);
      transformed_points.push_back(t_2);
      PublishCloud(map_1, cloud_pub_1);
      PublishCloud(map_transformed2, cloud_pub_2);
      DrawPoints(transformed_points);
      cout << "Current points drawn" << endl;
      sleep(5);
    }
}

// Performs Optimization between maps formed from two points sensors (based on correspondences)
void FullMapOptimizeRefine(vector<pcl::PointCloud<pcl::PointXYZ> > clouds_1,
                     vector<pcl::PointCloud<pcl::PointXYZ> > clouds_2,
                     pcl::PointCloud<pcl::PointXYZ> map_1,
                     pcl::PointCloud<pcl::PointXYZ> map_2,
                     vector<double*> trajectory_1,
                     vector<double*> trajectory_2,
                     vector<vector<Eigen::Vector3d> > correspondences,
                     vector<vector<double* > > corr_trajectories,
                     double* extrinsic) {
  
  vector<vector<Eigen::Vector3d> > transformed_points;
  vector<Eigen::Vector3d> t_1;
  vector<Eigen::Vector3d> t_2;
  for(size_t j = 0; j < correspondences[0].size(); j++){
    Eigen::Vector3d point_1 = correspondences[0][j];
    Eigen::Vector3d point_2 = correspondences[1][j];
    point_1 = TransformPointQuat(point_1, corr_trajectories[0][j]);
    point_2 = TransformPointQuat(point_2, corr_trajectories[1][j]);
    //       point_1 = TransformPoint(point_1, zero_pose);  
    point_2 = TransformPointQuat(point_2, extrinsic);
    t_1.push_back(point_1);
    t_2.push_back(point_2);
  }
  pcl::PointCloud<pcl::PointXYZ> map_transformed1, map_transformed2;
  map_transformed1 = map_1;
  map_transformed2 = map_2;
  TransformPointCloudQuaternion(map_transformed2, extrinsic);
  transformed_points.push_back(t_1);
  transformed_points.push_back(t_2);
  PublishCloud(map_1, cloud_pub_1);
  PublishCloud(map_transformed2, cloud_pub_2);
  DrawPoints(transformed_points);
  cout << "Original Alignment Drawn" << endl;
  sleep(5);
  ceres::Problem problem;
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
    for(size_t i = 0; i < correspondences[0].size(); i++) {
      double* pose_1 = corr_trajectories[0][i];
      Eigen::Vector3d point_1 = correspondences[0][i];
      double* pose_2 = corr_trajectories[1][i];
      Eigen::Vector3d point_2 = correspondences[1][i];
      ceres::CostFunction* cost_function =
      FeatureErrorCrossMapFullRefine::Create(point_1,
                                   point_2
      );
      problem.AddResidualBlock(cost_function,
                               new ceres::HuberLoss(0.5),
                               extrinsic,
                               pose_1,
                               pose_2
      );
      //         problem.SetParameterBlockConstant(pose_1);
      //         problem.SetParameterBlockConstant(pose_2);
    }
    
    // Run Ceres problem
    ceres::Solver::Options options;
    options.num_threads = 12;
    options.num_linear_solver_threads = 12;
    // options.use_explicit_schur_complement = true;
    // options.linear_solver_type = ceres::ITERATIVE_SCHUR;
    //           options.linear_solver_type = ceres::SPARSE_SCHUR;
    options.linear_solver_type = ceres::SPARSE_SCHUR;
    // options.minimizer_progress_to_stdout = true;
    // options.function_tolerance = 1e-10;
    ceres::Solver::Summary summary;
    double cost;
    ceres::Problem::EvaluateOptions evalOptions =
    ceres::Problem::EvaluateOptions();
    problem.Evaluate(evalOptions, &cost, &residuals, NULL, NULL);
    
    cout << setprecision(20) << "RMSE: " << rmse << endl;
    ceres::Solve(options, &problem, &summary);
    rmse =
    sqrt(cost / static_cast<double>(summary.num_residuals));
    rmse =
    sqrt(summary.final_cost / static_cast<double>(summary.num_residuals));
    cout << setprecision(20) << "RMSE: " << rmse << endl;
    t_2.clear();
    t_1.clear();
    transformed_points.clear();
    for(size_t j = 0; j < correspondences[0].size(); j++){
      Eigen::Vector3d point_1 = correspondences[0][j];
      Eigen::Vector3d point_2 = correspondences[1][j];
      point_1 = TransformPointQuat(point_1, corr_trajectories[0][j]);
      point_2 = TransformPointQuat(point_2, corr_trajectories[1][j]);
      //       point_1 = TransformPoint(point_1, zero_pose);  
      point_2 = TransformPointQuat(point_2, extrinsic);
      t_1.push_back(point_1);
      t_2.push_back(point_2);
    }
    map_transformed1 = map_1;
    map_transformed2 = map_2;
    TransformPointCloudQuaternion(map_transformed2, extrinsic);
    transformed_points.push_back(t_1);
    transformed_points.push_back(t_2);
    PublishCloud(map_1, cloud_pub_1);
    PublishCloud(map_transformed2, cloud_pub_2);
    DrawPoints(transformed_points);
    cout << "Current points drawn" << endl;
    sleep(5);
  }
                     }

// Outputs the number of inliers for a given transformed point set
int InlierScore(const vector<vector<Eigen::Vector3d> >& correspondences, const Eigen::Vector3d start_point) {
  int score = 0;
  int count = 0;
  double max_offset = .4;
  
  for(size_t i = 0; i < correspondences[0].size(); i++) {
    double start_distance = PointDistance(correspondences[0][i], start_point);
    double distance = PointDistance(correspondences[0][i], correspondences[1][i]);
//     cout << distance << endl;
    if (start_distance < .05) {
      count += 1;
    
      if (distance < max_offset) {
        score += 1;
      }
    }
  }
  cout << "score calcs" << endl;
  cout << score << endl;
  cout << count << endl;
  if (count > 0) {
  return score / count;
  } else {
    return 0;
  }
}

/* Takes feature correspondences and builds a list of the 3d points which correspond and
 * the poses associated with each of those points (poses in corr_trajectories)
 */
vector<vector<Eigen::Vector3d> > BuildCorrespondenceSet(const vector<pcl::PointCloud<pcl::PointXYZ> >& clouds_1,
                                                        const vector<pcl::PointCloud<pcl::PointXYZ> >& clouds_2,
                                                        const vector<vector<DMatch> >& matches_1,
                                                        const vector<vector<DMatch> >& matches_2,
                                                        const vector<double*>& trajectory_1,
                                                        const vector<double*>& trajectory_2,
                                                        vector<vector<double* > >* corr_trajectories,
                                                        double* extrinsic){
  vector<vector<Eigen::Vector3d> > point_correspondences;
  vector<Eigen::Vector3d> corr_1,corr_2;
  vector<double*> corr_traj_1, corr_traj_2;
  corr_trajectories->clear();
  for(size_t i = 0; i < clouds_1.size(); i++) {
    pcl::PointCloud<pcl::PointXYZ> cloud_1 = clouds_1[i];
    double* pose_1 = trajectory_1[i];
    for(size_t j = 0; j < clouds_2.size(); j++) {
      pcl::PointCloud<pcl::PointXYZ> cloud_2 = clouds_2[j];
      vector<DMatch> current_matches = matches_1[i* (int)clouds_2.size() + j];
      double* pose_2 = trajectory_2[j];
      for(size_t k = 0; k < current_matches.size(); k++) {
        pcl::PointXYZ cloudpt_1 = cloud_1[current_matches[k].queryIdx];
        pcl::PointXYZ cloudpt_2 = cloud_2[current_matches[k].trainIdx];
        Eigen::Vector3d pt_1, pt_2;
        pt_1[0] = cloudpt_1.x;
        pt_1[1] = cloudpt_1.y;
        pt_1[2] = cloudpt_1.z;
        pt_2[0] = cloudpt_2.x;
        pt_2[1] = cloudpt_2.y;
        pt_2[2] = cloudpt_2.z;
        corr_1.push_back(pt_1);
        corr_2.push_back(pt_2);
        corr_traj_1.push_back(pose_1);
        corr_traj_2.push_back(pose_2);
      }
    }
  }
  point_correspondences.push_back(corr_1);
  point_correspondences.push_back(corr_2);
  corr_trajectories->push_back(corr_traj_1);
  corr_trajectories->push_back(corr_traj_2);
  return point_correspondences;
}





vector<vector<Eigen::Vector3d> > BestCorrespondences(int num_iterations,
                                                     pcl::PointCloud<pcl::PointXYZ> map1,
                                                     pcl::PointCloud<pcl::PointXYZ> map2,
                                                     vector<vector<Eigen::Vector3d> > correspondences,
                                                     vector<vector<double* > > corr_trajectories,
                                                     vector<vector<double* > >* final_trajectories,
                                                     double* extrinsics) {

  vector<int> good_points;
  good_points.resize(correspondences[0].size());
  int best_score = 0;
  // For number of ransac iterations
  for (int iter = 0;
       iter < num_iterations;
       ++iter) {
    // Setup the empty transform
    double zero_pose[7];
    vector<double> pose0(7, 0.0);
    std::copy(pose0.begin(), pose0.end(), zero_pose);
    zero_pose[0] = 1;
    vector<int> cur_indexes;
    Eigen::Vector3d point_0;  
    // Generate three random numbers, use those to select three correspondence pairs
    for (int i = 0; i < 3; i++) {
      bool no_point = true;
      
      while(no_point) {
      int v1 = rand() % correspondences[0].size();
      
        /*
        cout << "V1: " << v1 << endl;
        */
      Eigen::Vector3d point_1 = correspondences[0][v1];
//       Eigen::Vector3d point_2 = correspondences[1][cur_indexes[v1]];
      double dist = PointDistance(point_1, point_0);
//       cout << dist << endl;
      if(i == 0 || PointDistance(point_1, point_0) < .05) {
        cur_indexes.push_back(v1);
        no_point = false;
        if(i == 0) {
          point_0 = point_1;
        }
      }
      }
      
    }
    // Tolerance for RMSE.
    static const double kToleranceError = 0.00001;
    // The maximum number of overall iterations.
    static const int kMaxIterations = 80;
    // The maximum number of repeat iterations while the RMSE is unchanged.
    static const int kMaxRepeatIterations = 5;
    double rmse = 1000000;
    double last_rmse = 1000010;
    vector<double> residuals;
    // Until convergence is reached
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
      ceres::Problem problem;
      for (int i = 0; i < 3; i++) {
        Eigen::Vector3d point_1 = correspondences[0][cur_indexes[i]];
        Eigen::Vector3d point_2 = correspondences[1][cur_indexes[i]];

//         point_1 = TransformPoint(point_1, corr_trajectories[0][cur_indexes[i]]);
        //point_2 = TransformPoint(point_2, corr_trajectories[1][v1]);

        //Use these as input to an optimization problem to compute an estimate of the map_1
        const Vector3d in_point_0(point_1[0], point_1[1], point_1[2]);
        const Vector3d in_point_1(point_2[0], point_2[1], point_2[2]);
        //
        ceres::CostFunction* cost_function =
        FeatureError::Create(in_point_0,
                             in_point_1,
                             corr_trajectories[0][cur_indexes[i]]
        );
        problem.AddResidualBlock(cost_function,
                                  new ceres::HuberLoss(0.5),
                                  zero_pose);
      }
      ceres::Solver::Options options;
      options.num_threads = 12;
      options.num_linear_solver_threads = 12;
      options.linear_solver_type = ceres::DENSE_QR;
      ceres::Solver::Summary summary;
      ceres::Solve(options, &problem, &summary);
      rmse =
      sqrt(summary.final_cost / static_cast<double>(summary.num_residuals));
      cout << "RMSE: " << rmse << endl;
    }
    // Transform all of the points, first by their corresponding trajectory, and then by the
    // extrinsic transform calculated from the current set of three poiints
    vector<vector<Eigen::Vector3d> > transformed_points;
    vector<Eigen::Vector3d> t_1;
    vector<Eigen::Vector3d> t_2;
    cout << zero_pose[0] << " " << zero_pose[1] << " " << zero_pose[2] << " " << zero_pose[3] << " " << zero_pose[4] << " " << zero_pose[5] << endl;
    for(size_t j = 0; j < correspondences[0].size(); j++){
      Eigen::Vector3d point_1 = correspondences[0][j];
      Eigen::Vector3d point_2 = correspondences[1][j];
//       Eigen::Vector3d point_1 = correspondences[0][cur_indexes[j]];
//       Eigen::Vector3d point_2 = correspondences[1][cur_indexes[j]];
      
      point_1 = TransformPointQuat(point_1, corr_trajectories[0][j]);
      point_2 = TransformPointQuat(point_2, corr_trajectories[1][j]);
//       point_1 = TransformPoint(point_1, zero_pose);  
      point_2 = TransformPointQuat(point_2, zero_pose);
      t_1.push_back(point_1);
      t_2.push_back(point_2);
    }
    pcl::PointCloud<pcl::PointXYZ> map_transformed1, map_transformed2;
    map_transformed1 = map1;
    map_transformed2 = map2;
    TransformPointCloudQuaternion(map_transformed2, zero_pose);
    transformed_points.push_back(t_1);
    transformed_points.push_back(t_2);
    PublishCloud(map1, cloud_pub_1);
    PublishCloud(map_transformed2, cloud_pub_2);
    DrawPoints(transformed_points);
    // Calcluate the inlier score for the set of points
    int score = InlierScore(transformed_points, transformed_points[0][cur_indexes[0]]);
    // If the inlier score is high enough mark the set of points as good, otherwise throw it out
    
    if (score > 0) {
      fprintf(stdout, "Score: %d out of %d \n", score, (int)correspondences[0].size());
      for(size_t j = 0; j < cur_indexes.size(); j++) {
      good_points[cur_indexes[j]] = 1;
      }
    }
    if(score > best_score) {
      best_score = score;
      std::copy(zero_pose, zero_pose + 6, extrinsics);
    }
  }
    //Build the complete set of points and return this and the corresponding trajectories
    vector<vector<Eigen::Vector3d> > current_points;
    vector<Eigen::Vector3d> points_1;
    vector<Eigen::Vector3d> points_2;
    vector<double*> poses_1, poses_2;
    for(size_t j = 0; j < good_points.size(); j++){
      if(good_points[j] == 1) {
        points_1.push_back(correspondences[0][j]);
        points_2.push_back(correspondences[1][j]);
        poses_1.push_back(corr_trajectories[0][j]);
        poses_2.push_back(corr_trajectories[1][j]);
      }
    }
    final_trajectories->push_back(poses_1);
    final_trajectories->push_back(poses_2);
    current_points.push_back(points_1);
    current_points.push_back(points_2);
    return current_points;
}


/* Performs two tasks. Estimates the initial extrinsic transform between the two maps
 * and selects a set of correspondences from the feature correspondences which are the best in quality
 * (based on which yield the most inliers in map estimate).
*/
void InitialExtrinsicEstimate(vector<pcl::PointCloud<pcl::PointXYZ> > clouds_1,
                              vector<pcl::PointCloud<pcl::PointXYZ> > clouds_2,
                              pcl::PointCloud<pcl::PointXYZ> map1,
                              pcl::PointCloud<pcl::PointXYZ> map2,
                              vector<vector<DMatch> > matches_1,
                              vector<vector<DMatch> > matches_2,
                              vector<double*> trajectory_1,
                              vector<double*> trajectory_2,
                              vector<vector<Eigen::Vector3d> >* final_correlations,
                              vector<vector<double* > >* correlation_trajectories,
                              double* extrinsic) {

  vector<vector<Eigen::Vector3d> > corr_set;
  vector<vector<double* > > corr_trajectories, final_trajectories;
  // Corr_set contains two vectors, where corresponding indexes in each vector are matched points
  // corr_trajectories contains the trajectories which correspond to each of these points
  // The first point is from a cloud in clouds_1, and the second from a cloud in clouds_2
  corr_set = BuildCorrespondenceSet(clouds_1,
                                    clouds_2,
                                    matches_1,
                                    matches_2,
                                    trajectory_1,
                                    trajectory_2,
                                    &corr_trajectories,
                                    extrinsic);
  cout << "Correspondence Set Built" << endl;
  cout << "Number Corrs: " << corr_set[0].size() << endl;
  corr_set = BestCorrespondences(1000, map1, map2, corr_set, corr_trajectories, &final_trajectories, extrinsic);
  cout << "Best Correspondences Selected" << endl;
  cout << "Number Corrs: " << corr_set[0].size() << endl;
  (*final_correlations) = corr_set;
  (*correlation_trajectories) = final_trajectories;
  pcl::PointCloud<pcl::PointXYZ> map_transformed1, map_transformed2;
  map_transformed1 = map1;
  map_transformed2 = map2;
  TransformPointCloudQuaternion(map_transformed2, extrinsic);
//   while(true) {
  PublishCloud(map1, cloud_pub_1);
  PublishCloud(map_transformed2, cloud_pub_2);
  DrawPoints(corr_set);
  sleep(5);
//   }
  }



double* InvertRotation(double* pose1) {

  // For the two poses create a transform
  double* posek = new double[6];
  double* transform = pose1;

  //Create the eigen transform from the first pose
  Eigen::Matrix<double,3,1> axis(transform[0], transform[1], transform[2]);
  const double angle = axis.norm();
  if(angle != 0) {
    axis = axis / angle;
  }
  Eigen::Transform<double, 3, Eigen::Affine> rotation =
  Eigen::Transform<double, 3, Eigen::Affine>(Eigen::AngleAxis<double>(
    angle, axis));

  rotation = rotation.inverse();
  Eigen::AngleAxis<double> angle_axis(rotation.rotation());
  // Get the axis
  Eigen::Vector3d normal_axis = angle_axis.axis();

  // Recompute the rotation angle
  double combined_angle = angle_axis.angle();
  Eigen::Vector3d combined_axis = normal_axis * combined_angle;
  posek[0] = combined_axis[0];
  posek[1] = combined_axis[1];
  posek[2] = combined_axis[2];
  posek[3] = pose1[3];
  posek[4] = pose1[4];
  posek[5] = pose1[5];
  return posek;
}

void WritePoseFile(string filename, vector<double*> transforms) {
  fstream posefile;
  posefile.open(filename.c_str(), std::fstream::out);
  for(size_t i = 0; i < transforms.size(); i++) {
    double* transform = transforms[i];
    for(int j = 0; j < 6; j++) {
      posefile << transform[j] << " ";
    }
    posefile << transform[6] << endl;
  }
}

void ReadFeatures(string filename, 
                         cv::Mat* descriptors) {
  FileStorage fs2(filename, FileStorage::READ);
  cout << filename << endl;
  FileNode descriptor_node = fs2["descriptors"];
  read( descriptor_node, (*descriptors) );
  cout << descriptors->rows << endl;
  fs2.release();
}

// Currently reading in quaternions
void ReadTrajectoryFile(string filename, vector<double*>* trajectory) { 
  std::string line;
  std::ifstream infile(filename.c_str());
  while (std::getline(infile, line)) {
    std::istringstream iss(line);
    double* transform = new double[7];
    
    double time, theta_x, theta_y, theta_z, theta_w, x, y, z;
    if (!(iss >> time >> x >> y >>
      z >> theta_x >> theta_y >> theta_z >> theta_w)) { break; } // error
      cout << time << " " <<  theta_x << " " << theta_y << " " << theta_z << " " << theta_w << " " << x << " " << y << " " << z << endl;
    transform[0] = theta_w;
    transform[1] = theta_x;
    transform[2] = theta_y;
    transform[3] = theta_z;
    transform[4] = x;
    transform[5] = y;
    transform[6] = z;
    trajectory->push_back(transform);
  }
}

void LoadFramesAndFeatures(string filename, 
                           int count,
                           vector<pcl::PointCloud<pcl::PointXYZ> >* clouds,
                           vector<Mat>* descriptors) {
  for(int i = 0; i < count; i++) {
    cout << "Frame: " << i << endl;
    string cloud_name = filename + "/objects/keyframe_" + to_string(i) + ".obj";
    string feature_name = filename + "/features/feature_" + to_string(i) + ".yaml";
    Mat descriptor;
    pcl::PointCloud<pcl::PointXYZ> cloud = CloudFromObj(cloud_name);
    ReadFeatures(feature_name, &descriptor);
    clouds->push_back(cloud);
    descriptors->push_back(descriptor);
  }
}

void SlamCalibrate(string sensor_1_dir,
                   string sensor_2_dir,
                   string bag_name,
                   vector<ros::Publisher> publishers) {

 
  string traj_name_1 = sensor_1_dir + "/KeyframeTrajectory.txt";
  string traj_name_2 = sensor_2_dir + "/KeyframeTrajectory.txt";
  string cloud_name1 = sensor_1_dir + "/objects/FullMap.obj";
  string cloud_name2 = sensor_2_dir + "/objects/FullMap.obj";
  pcl::PointCloud<pcl::PointXYZ> map1 = CloudFromObj(cloud_name1);
  pcl::PointCloud<pcl::PointXYZ> map2 = CloudFromObj(cloud_name2);
  vector<double*> trajectory_1, trajectory_2;
  vector<pcl::PointCloud<pcl::PointXYZ> > clouds_1, clouds_2;
  vector<Mat> descriptors_1, descriptors_2;
  
  ReadTrajectoryFile(traj_name_1, &trajectory_1);
  ReadTrajectoryFile(traj_name_2, &trajectory_2);
  cout << traj_name_1 << endl;
  cout << trajectory_1.size() << endl;
  cout << traj_name_2 << endl;
  cout << trajectory_2.size() << endl;
  LoadFramesAndFeatures(sensor_1_dir, trajectory_1.size(), &clouds_1, &descriptors_1);
  LoadFramesAndFeatures(sensor_2_dir, trajectory_2.size(), &clouds_2, &descriptors_2);
  
  fprintf(stdout, "Number of clouds 1: %d \n Number of Clouds 2: %d \n", (int)clouds_1.size(), (int)clouds_2.size());

  // Compare features between the two maps and select the best features to use for alignment.
   vector<vector<DMatch> > all_matches_1_2, all_matches_1_1, all_matches_2_2;
   vector<vector<DMatch> > all_good_matches_1_2, all_good_matches_1_1, all_good_matches_2_2;
   
   //   Build composite point clouds (maps) from keyframes
     pcl::PointCloud<pcl::PointXYZ> map_1, map_2;
     for(size_t i = 0; i < clouds_1.size(); i++) {
       pcl::PointCloud<pcl::PointXYZ> transformed_cloud = clouds_1[i];
       TransformPointCloudQuaternion(transformed_cloud,trajectory_1[i]);
       map_1 += transformed_cloud;
     }
     for(size_t i = 0; i < clouds_2.size(); i++) {
       pcl::PointCloud<pcl::PointXYZ> transformed_cloud = clouds_2[i];
       TransformPointCloudQuaternion(transformed_cloud, trajectory_2[i]);
       map_2 += transformed_cloud;
     }
//      while(true) {
     PublishCloud(map_1, cloud_pub_1);
     PublishCloud(map_2, cloud_pub_2);
     cout << "Published Clouds " << endl;
     sleep(3);
//      }
//   FeatureMatching(clouds_1,
//                   clouds_1,
//                   descriptors_1,
//                   descriptors_1,
//                   &all_matches_1_1,
//                   &all_good_matches_1_1);
//   
//   cout << "feature matching 1 fin" << endl;
//   
//   FeatureMatching(clouds_2,
//                   clouds_2,
//                   descriptors_2,
//                   descriptors_2,
//                   &all_matches_2_2,
//                   &all_good_matches_2_2);
//    
//   cout << "feature matching 2 fin" << endl;

  FeatureMatching(clouds_1,
                  clouds_2,
                  descriptors_1,
                  descriptors_2,
                  &all_matches_1_2,
                  &all_good_matches_1_2);

  cout << "feature matching Finished" << endl;
//   VisualizeMatches(clouds_1,
//                   clouds_2,
//                   trajectory_1,
//                   trajectory_2,
//                   descriptors_1,
//                   descriptors_2,
//                   all_good_matches_1_2);
  // Align using these selected features only to form an initial guess
  double extrinsic[6];
  vector<double> pose0(6, 0.0);
  std::copy(pose0.begin(), pose0.end(), extrinsic);
  cout << "Array Setup" << endl;
  vector<vector<Eigen::Vector3d> > final_correlations;
  vector<vector<double* > > correlation_trajectories;
  InitialExtrinsicEstimate(clouds_1,
                           clouds_2,
                           map_1,
                           map_2,
                           all_good_matches_1_2,
                           all_matches_1_2,
                           trajectory_1,
                           trajectory_2,
                           &final_correlations,
                           &correlation_trajectories,
                           extrinsic);

  // Create an optimization problem using all trajectories, all feature correspondences and extrinsics to optimize
  FullMapOptimize(clouds_1,
                  clouds_2,
                  map_1,
                  map_2,
                  trajectory_1,
                  trajectory_2,
                  final_correlations,
                  correlation_trajectories,
                  extrinsic);
  
//   FullMapOptimizeRefine(clouds_1,
//                   clouds_2,
//                   map_1,
//                   map_2,
//                   trajectory_1,
//                   trajectory_2,
//                   final_correlations,
//                   correlation_trajectories,
//                   extrinsic);
  // Output final extrinsic calibration to a file
  TransformPointCloudQuaternion(map2, extrinsic);
  vector<double*> final_poses;
  final_poses.push_back(extrinsic);
  WriteToObj("final_map", "final_map", 0,  map2);
  WritePoseFile("slam_calibration.ext", final_poses);
}

void VisualizeTrajectory(string traj_name_1,
                         string traj_name_2,
                         string bag_name,
                         vector<ros::Publisher> publishers) {
  
  // Vector Initilization
  vector<double*> trajectory_1, trajectory_2;
  vector<Mat> images_1, images_2;
  vector<sensor_msgs::Image> depths_1, depths_2;
  vector<pcl::PointCloud<pcl::PointXYZ> > clouds_1, clouds_2;
  vector<double> times_1, times_2;
  vector<vector<int> > cloud_to_depth_1, cloud_to_depth_2;
  vector<std::map<int, int> > depths_to_clouds_1, depths_to_clouds_2;
  // Read Trajectory Files (need to be converted to our transform format first)
  
  std::string line;
  // Duplicated messily (fix this)
  std::ifstream infile(traj_name_1.c_str());
  while (std::getline(infile, line)/* && count2 < 20*/) {
    std::istringstream iss(line);
    double* transform = new double[7];
    
    double time, theta_x, theta_y, theta_z, theta_w, x, y, z;
    if (!(iss >> time >> x >> y >>
      z >> theta_x >> theta_y >> theta_z >> theta_w)) { break; } // error
      cout << time << " " <<  theta_x << " " << theta_y << " " << theta_z << " " << theta_w << " " << x << " " << y << " " << z << endl;
      transform[0] = theta_x;
    transform[1] = theta_y;
    transform[2] = theta_z;
    transform[3] = theta_w;
    //     transform[0] = 0;
    //     transform[1] = 0;
    //     transform[2] = 0;
    //     transform[3] = 0;
    transform[4] = x;
    transform[5] = y;
    transform[6] = z;
    trajectory_1.push_back(transform);
    times_1.push_back(time);
//     if(count % 1 == 0) {
//       trajectory_1.push_back(transform);
//       times_1.push_back(time);
//       count2+=1;
//     }
//     count++;
  }
  
  rosbag::Bag bag;
  bag.open(bag_name, rosbag::bagmode::Read);
  std::vector<std::string> topics_1, topics_2, topics_3, topics_4;
  topics_1.push_back(std::string("/kinect1/depth_registered/image_raw"));
  topics_3.push_back(std::string("/kinect1/rgb/image_raw"));
  topics_2.push_back(std::string("/kinect2/depth/image_raw"));
  topics_4.push_back(std::string("/kinect2/rgb/image_color"));
  
  rosbag::View view_1(bag, rosbag::TopicQuery(topics_1));
  
  rosbag::View view_2(bag, rosbag::TopicQuery(topics_2));
  rosbag::View view_1_rgb(bag, rosbag::TopicQuery(topics_3));
  rosbag::View view_2_rgb(bag, rosbag::TopicQuery(topics_4));
  
  rosbag::View::iterator it_1 = view_1.begin();
  rosbag::View::iterator end_1 = view_1.end();
  rosbag::View::iterator it_2 = view_2.begin();
  rosbag::View::iterator end_2 = view_2.end();
  rosbag::View::iterator it_1_rgb = view_1_rgb.begin();
  rosbag::View::iterator end_1_rgb = view_1_rgb.end();
  rosbag::View::iterator it_2_rgb = view_2_rgb.begin();
  rosbag::View::iterator end_2_rgb = view_2_rgb.end();
  
//   KeyframesFromBag(it_1,
//                    end_1,
//                    it_1_rgb,
//                    end_1_rgb,
//                    trajectory_1,
//                    times_1,
//                    &images_1,
//                    &depths_1,
//                    &cloud_to_depth_1,
//                    &depths_to_clouds_1,
//                    &clouds_1);
  cout << trajectory_1.size() << endl;
  fprintf(stdout, "Number of clouds 1: %d \n Number of Clouds 2: %d \n", (int)clouds_1.size(), (int)clouds_2.size());
  
  geometry_msgs::PoseArray posearray;
  posearray.header.frame_id = "point_cloud";
  pcl::PointCloud<pcl::PointXYZ> combo, combo2;
  pcl::PointCloud<pcl::PointXYZ> map = CloudFromObj(traj_name_1 + "/objects/FullMap.obj");
  vector<pcl::PointCloud<pcl::PointXYZ> > transformed_clouds;
  for(size_t i = 0; i < trajectory_1.size(); i++) {
    pcl::PointCloud<pcl::PointXYZ> cloud = CloudFromObj("kinect2_unmapped/objects/keyframe_" + to_string(i) + ".obj");
    cout << "Publishing Pose: " << i <<  endl;
    double* transform = trajectory_1[i];
    cout << transform[0] << " " << transform[1] << " " << transform[2] << " " << transform[3] << " " << transform[4] << " " << transform[5] << " " << transform[6] << endl;
    geometry_msgs::PoseStamped stamped;
    stamped.header.frame_id = "point_cloud";
    stamped.header.seq = i + 1;
    geometry_msgs::Pose pose;
    geometry_msgs::Quaternion quat;
    geometry_msgs::Point point;
    point.x = transform[4];
    point.y = transform[5];
    point.z = transform[6];
    quat.x = transform[3];
    quat.y = transform[0];
    quat.z = transform[1];
    quat.w = transform[2];
    //     quat.x = 0;
    //     quat.y = 0;
    //     quat.z = 0;
    //     quat.w = 0;
    pose.orientation = quat;
    pose.position = point;
    stamped.pose = pose;
    posearray.poses.push_back(pose);
    poses.publish(posearray);
    pcl::PointCloud<pcl::PointXYZ> transformed = cloud;
    combo2 += transformed;
    TransformPointCloudQuat(transformed, transform);
    combo += transformed;
    PublishCloud(combo, cloud_pub_1);
    PublishCloud(combo2, cloud_pub_2);
    PublishCloud(map, cloud_pub_3);
    transformed_clouds.push_back(transformed);
    sleep(3);
  }
}

int main(int argc, char **argv) {
  signal(SIGINT,HandleStop);
  signal(SIGALRM,HandleStop);

  char* traj_file = (char*)"opposite2_1";
  char* traj_file_2 = (char*)"opposite2_2";
  char* bag_file = (char*)"kinect_l_1.bag";
  bool test_mode = false;
  // Parse arguments.
  static struct poptOption options[] = {
      { "bag-file" , 'B', POPT_ARG_STRING, &bag_file ,0, "Process bag file" ,
        "STR" },
        { "traj-file" , 'C', POPT_ARG_STRING, &traj_file ,0, "Trajectory file" ,
          "STR" },
          { "traj-file2" , 'c', POPT_ARG_STRING, &traj_file_2 ,0, "Trajectory file 2" ,
            "STR" },
        { "test-mode", 'T', POPT_ARG_NONE, &test_mode, 0, "Run simulation test",
          "NONE" },
            POPT_AUTOHELP
            { NULL, 0, 0, NULL, 0, NULL, NULL }
  };

  // parse options
  POpt popt(NULL,argc,(const char**)argv,options,0);
  int c;
  while((c = popt.getNextOpt()) >= 0) {
  }

  // Initialize Ros
  ros::init(argc, argv, "delta_calibration",
            ros::init_options::NoSigintHandler);
  ros::NodeHandle n;
  //----------  Setup Publishers ----------
  cloud_pub_1 = n.advertise<sensor_msgs::PointCloud2> ("cloud_pub_1", 5);
  cloud_pub_2 = n.advertise<sensor_msgs::PointCloud2> ("cloud_pub_2", 5);
  cloud_pub_3 = n.advertise<sensor_msgs::PointCloud2> ("cloud_pub_3", 45);
  cloud_pub_4 = n.advertise<sensor_msgs::PointCloud2> ("cloud_pub_4", 5);
  poses = n.advertise<geometry_msgs::PoseArray>("poses", 4);
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
//   VisualizeTrajectory(traj_file, traj_file_2, bag_file, publishers);
  SlamCalibrate(traj_file, traj_file_2, bag_file, publishers);

  return 0;
}
