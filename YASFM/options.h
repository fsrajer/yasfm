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

struct OptionsSIFTGPU
{
  YASFM_API OptionsSIFTGPU()
  {
    firstOctave = -1;
    detectUprightSIFT = false;

    // let SIFTGPU autoselect these
    maxWorkingDimension = -1;
    maxOctaves = -1;
    dogLevelsInAnOctave = -1;
    dogThresh = -1;
    edgeThresh = -1;
  }

  bool isSetMaxWorkingDimension() const;
  bool isSetMaxOctaves() const;
  bool isSetDogLevelsInAnOctave() const;
  bool isSetDogThresh() const;
  bool isSetEdgeThresh() const;

  // Images larger than this will be downsampled. 
  // Negative value selects default: 3200.
  int maxWorkingDimension;
  // Min is -1. Default: -1 (TODO: automatic estimation).
  int firstOctave;
  // Negative value selects default: no limit.
  int maxOctaves;
  // Negative value selects default: 3.
  int dogLevelsInAnOctave;
  // Negative value selects default: 0.02/3.
  float dogThresh;
  // Negative value selects default: 10.
  float edgeThresh;
  // Only one fixed orientation per keypoint location. Default: false.
  bool detectUprightSIFT;
};

struct OptionsFLANN
{
  YASFM_API OptionsFLANN()
  {
    indexParams = flann::KDTreeIndexParams();
    searchParams = flann::SearchParams();
    ratioThresh = 0.8f;
    onlyUniques = true;
  }
  bool filterByRatio() const;
  // What method to use for searching. See FLANN for more details.
  // Default is 4 randomized kd-trees.
  flann::IndexParams indexParams;
  // Search parameters, eg. num of checks. See FLANN for more details.
  flann::SearchParams searchParams;
  // Threshold of the ratio d1/d2, where di is distance to the i-th nearest neighbor
  // Negative values disable this filter and only the nearest neighbor is
  // searched for. Default: 0.8.
  float ratioThresh;
  // Discards non-unique matches, i.e., those for which two or more 
  // different features in feats1 matched to the same feature in feats2
  // Default is true.
  bool onlyUniques;
};

struct OptionsRANSAC
{
  YASFM_API OptionsRANSAC(int maxRounds,double errorThresh,
    int minInliers) 
    : maxRounds(maxRounds),errorThresh(errorThresh),
    minInliers(minInliers)
  {
    confidence = 0.95;
  }
  YASFM_API OptionsRANSAC(int maxRounds,double errorThresh,
    int minInliers,double confidence)
    : maxRounds(maxRounds),errorThresh(errorThresh),
    minInliers(minInliers),confidence(confidence)
  {
  }
  // Maximum number of iterations.
  int maxRounds;
  double errorThresh;
  int minInliers;
  // Finds a good hypothesis with probability confidence. 
  // The values are from range [0,1].
  // Default: 0.95
  double confidence;
};

struct YASFM_API OptionsBundleAdjustment
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
};

}