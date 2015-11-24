/*
* Filip Srajer
* filip.srajer (at) fel.cvut.cz
* Center for Machine Perception
* Czech Technical University in Prague
*
* This software is under construction.
* 02/2015
*/

#pragma once

#include <string>

// VS2013 compiler gave many warnings about:
// 1) conversion size_t <-> int
// 2) no appropriate delete operator to overloaded new
// These are in FLANN by design, therefore, we ignore the warnings.
#pragma warning(push) 
#pragma warning(disable : 4267 4291)
#include "FLANN/flann.hpp"
#pragma warning(pop)
#include "ceres/ceres.h"

#include "defines.h"

using std::string;

namespace yasfm
{

class OptionsSIFT
{
public:
  YASFM_API OptionsSIFT();

  YASFM_API bool isSetMaxWorkingDimension() const;
  YASFM_API bool isSetMaxOctaves() const;
  YASFM_API bool isSetDogLevelsInAnOctave() const;
  YASFM_API bool isSetDogThresh() const;
  YASFM_API bool isSetEdgeThresh() const;

  // Images larger than this will be downsampled. 
  // Negative value selects default: 3200.
  int maxWorkingDimension_;
  // Min is -1. Default: -1 (TODO: automatic estimation).
  int firstOctave_;
  // Negative value selects default: no limit.
  int maxOctaves_;
  // Negative value selects default: 3.
  int dogLevelsInAnOctave_;
  // Negative value selects default: 0.02/3.
  float dogThresh_;
  // Negative value selects default: 10.
  float edgeThresh_;
  // Only one fixed orientation per keypoint location. Default: false.
  bool detectUprightSIFT_;
};

class OptionsFLANN
{
public:
  YASFM_API OptionsFLANN();
  YASFM_API bool filterByRatio() const;
  // What method to use for searching. See FLANN for more details.
  // Default is 4 randomized kd-trees.
  flann::IndexParams indexParams_;
  // Search parameters, eg. num of checks. See FLANN for more details.
  flann::SearchParams searchParams_;
  // Threshold of the ratio d1/d2, where di is distance to the i-th nearest neighbor
  // Negative values disable this filter. Default: 0.8.
  float ratioThresh_;
  // Discards non-unique matches, i.e., those for which two or more 
  // different features in feats1 matched to the same feature in feats2
  // Default is true.
  bool onlyUniques_;
};

class OptionsRANSAC
{
public:
  YASFM_API OptionsRANSAC(int ransacRounds, double errorThresh,
    int minInliers);
  YASFM_API OptionsRANSAC(int ransacRounds,double errorThresh,
    int minInliers,double inliersEnough);
  int ransacRounds_;
  double errorThresh_;
  int minInliers_;
  // inliersEnough is the amount of inliers which, if found, 
  // stops the algorithm immediately. The values are from range [0,1].
  // Default: 1.0
  double confidence_;
};

typedef struct YASFM_API OptionsBundleAdjustment
{
  OptionsBundleAdjustment()
  {
    solverOptions.max_num_iterations = 10;
    solverOptions.num_threads = 1;
    solverOptions.function_tolerance = 1e-3;
    solverOptions.parameter_tolerance = 1e-3;
    solverOptions.gradient_tolerance = 1e-3;
    robustify = false;
  }
  // Some of the interesting options: max_num_iterations, num_threads
  // function_tolerance, parameter_tolerance, gradient_tolerance,
  // minimizer_progress_to_stdout
  ceres::Solver::Options solverOptions;
  // If tru, uses Hubers loss function instead
  // of L2 norm, more robust to outliers.
  // Default: false.
  bool robustify;
}OptionsBundleAdjustment;

class Options
{
public:
  YASFM_API Options();
  string ccdDBFilename_;
  OptionsSIFT sift_;
  OptionsFLANN matchingFLANN_;
  // Even though geometric verification uses relative pose algorithms,
  // this options overrides the relativePose_
  // The error is sampson distance. Units are pixels.
  // Defaults: 
  //   ransacRounds_: 2048
  //   errorThresh_: 4.0
  //   minInliers_: 16
  OptionsRANSAC geometricVerification_;
  // The error is reprojection error. Units are pixels.
  // Defaults: 
  //   ransacRounds_: 4096
  //   errorThresh_: 4.0
  //   minInliers_: 16
  OptionsRANSAC absolutePose_;
  // Units of the error are pixels.
  // Defaults: 
  //   ransacRounds_: 512
  //   errorThresh_: 1.25
  //   minInliers_: 10
  OptionsRANSAC relativePose_;
  OptionsRANSAC homography_;
  // Min number of matches defining a poorly matched pair. Default: 16.
  int minNumMatches_;
  // chooseWellMatchedCameras finds the camera with most matches, say N
  // and then finds all cameras with N*wellMatchedCamsFactor_ matches. Default: 0.75
  double wellMatchedCamsFactor_;
  // Default: 16.
  int minNumCam2SceneMatches_;
  OptionsBundleAdjustment ba_;
  // Default: 16.
  double pointsReprojErrorThresh_;
  // Consider a ray from a camera center through a keypoint.
  // Consider next, the largest angle between all such rays
  // corresponding to one track/3d point.
  // This threshold is used for that angle.
  // Use degrees.
  // Default: 2.0.
  double rayAngleThresh_;
  // Up to 4. Default is 1.
  int verbosityLevel_;
  // Default: 0.5
  double minInitPairHomographyProportion_;
};

}