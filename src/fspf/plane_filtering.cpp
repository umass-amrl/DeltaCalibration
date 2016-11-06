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
/*!
\file    plane_filtering.cpp
\brief   PlaneFiltering Implementation
\author  Joydeep Biswas, (C) 2011
*/
//========================================================================

#include "plane_filtering.h"

using Eigen::Vector3f;
using Eigen::Matrix3f;
using Eigen::Quaternionf;
using std::size_t;
using std::vector;

//========================================================================
// Implementation: DepthCam
//========================================================================

DepthCam::DepthCam()
{
  width = 640;
  height = 480;
  fovH = RAD(57.00/2);
  fovV = RAD(43.0/2);
}

//========================================================================
// Implementation: KinectRawDepthCam
//========================================================================

KinectRawDepthCam::KinectRawDepthCam()
{
  a = 3.008;
  b = -0.002745;
  /*
  a = 3.008;
  b = -0.002805;
  */
  f = 600.0;
  width = 640;
  height = 480;
  fovH = RAD(57.00);
  fovV = RAD(43.0);
  maxDepthVal = 1500;
  pointCloudLookups.clear();
  initDepthReconstructionLookups();
}

Vector3f KinectRawDepthCam::depthValueTo3D(int row, int col, void* depthValue)
{
  unsigned int ind = row*width+col;
  uint16_t _depth = *((uint16_t*)depthValue);
  return depthLookups[_depth]*pointCloudLookups[ind];
}

float KinectRawDepthCam::rawDepthToMetricDepth(void* depthValue)
{
  uint16_t _depth = *((uint16_t*)depthValue);
  return depthLookups[_depth];
}

Vector3f KinectRawDepthCam::depthPixelTo3D(int ind, void* depthImage)
{
  uint16_t* _depth = (uint16_t*) depthImage;
  return depthLookups[_depth[ind]]*pointCloudLookups[ind];
}

Vector3f KinectRawDepthCam::depthPixelTo3D(int row, int col, void* depthImage)
{
  uint16_t* _depth = (uint16_t*) depthImage;
  unsigned int ind = row*width+col;
  return depthLookups[_depth[ind]]*pointCloudLookups[ind];
}

void KinectRawDepthCam::initDepthReconstructionLookups()
{
  double tanHFovH = tan(fovH*0.5);
  double tanHFovV = tan(fovV*0.5);
  float y,z;
  int ind;

  pointCloudLookups.resize(height*width);

  for(int row = 0; row<height; row++){
    for(int col = 0; col<width; col++){
      ind = row*width + col;
      y = -(float(col)/((float(width)-1.0)*0.5)-1.0)*tanHFovH;
      z = -(float(row)/((float(height)-1.0)*0.5)-1.0)*tanHFovV;
      pointCloudLookups[ind] = Vector3f(1.0f, y, z);
    }
  }

  for(int depth = 0; depth<65536; depth++){
    depthLookups[depth] = 1.0/(a+b*float(depth));
  }
}

bool KinectRawDepthCam::isValidDepth(int row, int col, void* depthImage)
{
  uint16_t* _depth = (uint16_t*) depthImage;
  uint16_t depthVal = _depth[row*width+col];
  return depthVal<=maxDepthVal && depthVal>0;
}

bool KinectRawDepthCam::isValidDepth(int ind, void* depthImage)
{
  uint16_t* _depth = (uint16_t*) depthImage;
  uint16_t depthVal = _depth[ind];
  return depthVal<=maxDepthVal && depthVal>0;
}

//========================================================================
// Implementation: KinectRawDepthCam
//========================================================================

KinectOpenNIDepthCam::KinectOpenNIDepthCam()
{
  f = 600.0;
  width = 640;
  height = 480;
  fovH = RAD(57.00/2);
  fovV = RAD(43.0/2);
  maxDepthVal = 6000;
  pointCloudLookups.clear();
  initDepthReconstructionLookups();
}

Vector3f KinectOpenNIDepthCam::depthValueTo3D(int row, int col, void* depthValue)
{
  unsigned int ind = row*width+col;
  float _depth = 0.001*(*((uint16_t*)depthValue));
  return _depth*pointCloudLookups[ind];
}

float KinectOpenNIDepthCam::rawDepthToMetricDepth(void* depthValue)
{
  return 0.001*(*((uint16_t*) depthValue));
}

Vector3f KinectOpenNIDepthCam::depthPixelTo3D(int row, int col, void* depthImage)
{
  uint16_t* _depth = (uint16_t*) depthImage;
  unsigned int ind = row*width+col;
  return float(_depth[ind])*0.001*pointCloudLookups[ind];
}

Vector3f KinectOpenNIDepthCam::depthPixelTo3D(int ind, void* depthImage)
{
  uint16_t* _depth = (uint16_t*) depthImage;
  return float(_depth[ind])*0.001*pointCloudLookups[ind];
}

bool KinectOpenNIDepthCam::isValidDepth(int row, int col, void* depthImage)
{
  return true;
  uint16_t* _depth = (uint16_t*) depthImage;
  unsigned int ind = row*width+col;
  return _depth[ind]<maxDepthVal;
}

bool KinectOpenNIDepthCam::isValidDepth(int ind, void* depthImage)
{
//   return true;
  uint16_t* _depth = (uint16_t*) depthImage;
  return _depth[ind]<maxDepthVal && _depth[ind] != 0;

}

void KinectOpenNIDepthCam::initDepthReconstructionLookups()
{
  float h = 1.0/f;
  float v = 1.0/f;
  float y,z;
  int ind;

  pointCloudLookups.resize(height*width);

  for(int row = 0; row<height; row++){
    for(int col = 0; col<width; col++){
      ind = row*width + col;
      y = -h*(float(col)-(float(width)-1.0)*0.5);
      z = -v*(float(row)-(float(height)-1.0)*0.5);
      pointCloudLookups[ind] = Vector3f(1.0f, y, z);
    }
  }
}


//========================================================================
// Implementation: PlaneFilter
//========================================================================

PlaneFilter::PlaneFilter()
{
  lastRand = 0;
}

PlaneFilter::PlaneFilter(DepthCam *_depthCam, PlaneFilterParams &_filterParams)
{
  setParameters(_depthCam, _filterParams);
}

void PlaneFilter::clearPerformanceStats()
{
  planeFilteringTimer.clear();
  numSampledLocations = 0;
  numPlanarPoints = 0;
  numNonPlanarPoints = 0;
}

void PlaneFilter::getPerformanceStats(double& planeFilteringTime, int& numSampledLocations, int& numPlanarPoints, int& numNonPlanarPoints)
{
  planeFilteringTime = planeFilteringTimer.time();
  numSampledLocations = this->numSampledLocations;
  numPlanarPoints = this->numPlanarPoints;
  numNonPlanarPoints = this->numNonPlanarPoints;
}


uint32_t PlaneFilter::lcgRand()
{
  lastRand = 1103515245*lastRand+12345;
  return lastRand;
}

inline bool PlaneFilter::sampleLocation(
    void* depthImage, int& index, int& row, int& col, Vector3f& p, int rMin,
    int height, int cMin, int width) {
  unsigned int retries = 0;
  bool valid = false;
  while( !valid && retries<=filterParams.numRetries){
    row = rMin + (lcgRand()%height);
    col = cMin + (lcgRand()%width);
    index = row*depthCam->width + col;
    valid = depthCam->isValidDepth(index,depthImage);
    numSampledLocations++;
    retries++;
  }
  if(valid) p = depthCam->depthPixelTo3D(index,depthImage);
  return valid;
}

void PlaneFilter::GenerateCompletePointCloud(
    void* depthImage, vector<Vector3f>& pointCloud, vector<int>& pixelLocs) {
  const size_t num_pixels = depthCam->height * depthCam->width;
  pointCloud.resize(num_pixels);
  pixelLocs.resize(num_pixels);
  size_t num_points = 0;
  int count = 0;
  for (size_t i = 0; i < num_pixels; ++i) {
    if (!depthCam->isValidDepth(i, depthImage)) {
      count +=1;
      continue;
    }
    pointCloud[num_points] = depthCam->depthPixelTo3D(i,depthImage);
    pixelLocs[num_points] = i;
    ++num_points;
  }
  pointCloud.resize(num_points);
  pixelLocs.resize(num_points);
}

void PlaneFilter::GenerateSampledPointCloud(
    void* depthImage, vector<Vector3f>& pointCloud, unsigned int numPoints) {
  int row, col, ind;
  Vector3f p;
  for (unsigned int i = 0; i < numPoints; i++) {
    if (sampleLocation(depthImage,ind, row, col, p, 0, depthCam->height - 1,
        0, depthCam->width-1)) {
      pointCloud.push_back(p);
    }
  }
}

void PlaneFilter::PointCloudFromRaster(
    void* depth, vector<Vector3f>& pointCloud, unsigned int raster) {
  float row, col;
  pointCloud.clear();
  if (static_cast<int>(raster) > depthCam->height - 1)
    return;
  for(int i=0; i<depthCam->width; i++){
    row = raster;
    col = i;
    int index = row*depthCam->width + col;
    if(!depthCam->isValidDepth(index,depth))
      continue;
    pointCloud.push_back(depthCam->depthPixelTo3D(index, depth));
  }
}

void PlaneFilter::GenerateFilteredPointCloud(
    void* depthImage, vector<Vector3f>& filteredPointCloud,
    vector<vector2i>& pixelLocs, vector<Vector3f>& pointCloudNormals,
    vector<Vector3f>& outlierCloud, vector<PlanePolygon>& polygons) {
  planeFilteringTimer.start();

  polygons.clear();
  filteredPointCloud.clear();
  pixelLocs.clear();
  pointCloudNormals.clear();
  outlierCloud.clear();

  filteredPointCloud.reserve(2*filterParams.maxPoints);
  pixelLocs.reserve(2*filterParams.maxPoints);
  pointCloudNormals.reserve(2*filterParams.maxPoints);
  outlierCloud.reserve(filterParams.numSamples);

  // Derived parameters
  float minInliers =
    filterParams.minInlierFraction * filterParams.numLocalSamples;
  float maxOutliers =
      (1.0 - filterParams.minInlierFraction) * filterParams.numLocalSamples;
  float planeSizeH = 0.5*filterParams.planeSize;
  float planeSize = filterParams.planeSize;
  float w2 = depthCam->width-filterParams.planeSize;
  float h2 = depthCam->height-filterParams.planeSize;
  // Counters
  unsigned int numPlanes = 0;
  unsigned int numPoints = 0;

  // Sample locations
  int ind1,ind2,ind3;

  float sampleRadiusHFactor =
      filterParams.WorldPlaneSize * depthCam->width/(4.0*tan(depthCam->fovH));
  float sampleRadiusVFactor =
      filterParams.WorldPlaneSize * depthCam->height/(4.0*tan(depthCam->fovV));

  Vector3f p1, p2, p3, p;
  vector2i l1, l2, l3, l;

  vector<Vector3f> neighborhoodInliers, neighborhoodOutliers;
  vector<vector2i> neighborhoodPixelLocs;

  float d, meanDepth;
  int sampleRadiusH, sampleRadiusV, rMin, rMax, cMin, cMax, dR, dC;

  for (unsigned int i=0; i < filterParams.numSamples &&
      numPoints<filterParams.maxPoints; i++) {

    //generate random points p1, p2, p3 anywhere on image
    if (!(
        sampleLocation(
            depthImage, ind1, l1.y, l1.x, p1, planeSizeH, h2, planeSizeH, w2) &&
        sampleLocation(
            depthImage, ind2, l2.y, l2.x, p2, l1.y-planeSizeH, planeSize,
            l1.x-planeSizeH, planeSize) &&
        sampleLocation(
            depthImage, ind3, l3.y, l3.x, p3, l1.y-planeSizeH, planeSize,
            l1.x-planeSizeH, planeSize))) {
      continue;
    }

    // Generate Plane normal (n) and distance (d) from origin (distance-normal
    // parameterization of plane)
    Vector3f n = ((p1-p2).cross(p3-p2)).normalized();

    d = p1.dot(n);
    meanDepth = (p1.x()+p2.x()+p3.x())*0.333333333333333333333333;
    sampleRadiusH = ceil(sampleRadiusHFactor/meanDepth*sqrt(1.0-sq(n.y())));
    sampleRadiusV = ceil(sampleRadiusVFactor/meanDepth*sqrt(1.0-sq(n.z())));
    rMin = max(0,l1.y-sampleRadiusV);
    rMax = min(depthCam->height-1,l1.y+sampleRadiusV);
    cMin = max(0,l1.x-sampleRadiusH);
    cMax = min(depthCam->width-1,l1.x+sampleRadiusH);
    dR = rMax-rMin;
    dC = cMax-cMin;

    if(sampleRadiusH<2.0 || sampleRadiusV<2.0){
      continue;
    }

    int inliers = 0, outliers = 0;
    neighborhoodInliers.clear();
    neighborhoodPixelLocs.clear();
    for (unsigned int j = 0; outliers<maxOutliers &&
        j<filterParams.numLocalSamples; j++) {
      //generate random point p within max distance params.planeSize from p1
      int ind = -1;
      if(!sampleLocation(depthImage, ind, l.y, l.x, p, rMin, dR, cMin, dC))
        continue;

      float err = fabs(n.dot(p) - d);
      if (err<filterParams.maxError &&
          p.x()<meanDepth+filterParams.maxDepthDiff &&
          p.x()>meanDepth-filterParams.maxDepthDiff) {
        inliers++;
        neighborhoodInliers.push_back(p);
        neighborhoodPixelLocs.push_back(l);
      }else{
        outliers++;
      }
    }
    if (inliers >= minInliers && inliers > 3) {
      //==Polygonization==
      if(filterParams.runPolygonization){
        PlanePolygon poly(neighborhoodInliers,neighborhoodPixelLocs);
        if(poly.validPolygon)
          polygons.push_back(poly);
        n = poly.normal;
      }
      //This is a local plane
      for(size_t i = 0; i < neighborhoodInliers.size(); i++){
        filteredPointCloud.push_back(neighborhoodInliers[i]);
        pointCloudNormals.push_back(n);
        pixelLocs.push_back(neighborhoodPixelLocs[i]);
      }
      filteredPointCloud.push_back(p1);
      filteredPointCloud.push_back(p2);
      filteredPointCloud.push_back(p3);
      pointCloudNormals.push_back(n);
      pointCloudNormals.push_back(n);
      pointCloudNormals.push_back(n);
      pixelLocs.push_back(l1);
      pixelLocs.push_back(l2);
      pixelLocs.push_back(l3);
      numPoints += neighborhoodInliers.size() + 3;
      numPlanes = numPlanes + 1;
    } else {
      for(size_t i=0; i < neighborhoodInliers.size(); i++) {
        outlierCloud.push_back(neighborhoodInliers[i]);
      }
      outlierCloud.push_back(p1);
      outlierCloud.push_back(p2);
      outlierCloud.push_back(p3);
    }
  }
  if(filterParams.runPolygonization && filterParams.filterOutliers){
    //Remove planar points from outlier cloud
    static const float MaxPlanarDist = filterParams.maxError;
    bool planar = false;
    for(unsigned int i=0; i<outlierCloud.size(); i++){
      planar = false;
      for(unsigned int j=0; !planar && j<polygons.size(); j++){
        const float planar_offset =
            polygons[j].normal.dot(outlierCloud[i])+polygons[j].offset;
        planar = (fabs(planar_offset) < MaxPlanarDist);
      }
      if(planar){
        //Need to remove this point
        outlierCloud.erase(outlierCloud.begin()+i);
        i--;
      }
    }
  }
  numPlanarPoints = filteredPointCloud.size();
  numNonPlanarPoints = outlierCloud.size();
  planeFilteringTimer.stop();
}


vector<PlanePolygon> PlaneFilter::findUniqueDepthPlanes(
    vector<PlanePolygon> planes) {
  static const float MaxCosError = cos(RAD(20.0));
  static const float MaxPlaneDist = 0.02;
  static const float MinPlaneSize = 0.05;
  static const float MinConditionNumber = 0.2;
  vector<PlanePolygon> unique3DPlanes;

  for(unsigned int i=0; i<planes.size(); i++){

    if(planes[i].height<MinPlaneSize || planes[i].width<MinPlaneSize)
      continue;

    if(planes[i].conditionNumber<MinConditionNumber)
      continue;

    bool matchFound = false;
    for(unsigned int j=0; !matchFound && j<unique3DPlanes.size(); j++){
      matchFound =
          fabs(unique3DPlanes[j].normal.dot(planes[i].normal))<MaxCosError &&
          fabs(unique3DPlanes[j].normal.dot(planes[i].p0) +
          unique3DPlanes[j].offset)<MaxPlaneDist;
    }
    if(!matchFound)
      unique3DPlanes.push_back(planes[i]);
  }

  return unique3DPlanes;
}

bool PlaneFilter::pointIsPlanar(
    void* depthImage, vector< PlanePolygon>& planes, int ind, float MaxError) {
  const Vector3f p = depthCam->depthPixelTo3D(ind,depthImage);

  for(unsigned int k=0; k<planes.size(); k++){
    if(fabs(planes[k].normal.dot(p)+planes[k].offset)<MaxError)
      return true;
  }
  return false;
}