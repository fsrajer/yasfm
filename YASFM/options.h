//----------------------------------------------------------------------------------------
/**
* \file       options.h
* \brief      Options structs.
*
*  Options structs.
*
*/
//----------------------------------------------------------------------------------------

#pragma once

#include <string>
#include <iostream>

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

using std::ostream;
using std::string;

namespace yasfm
{

/// Options for detecting SIFT using SIFTGPU.
struct OptionsSIFTGPU
{

  /// Constructor setting defaults.
  YASFM_API OptionsSIFTGPU()
  {
    firstOctave = -1;
    detectUprightSIFT = false;
    verbosityLevel = 1;

    // let SIFTGPU autoselect these
    maxWorkingDimension = -1;
    maxOctaves = -1;
    dogLevelsInAnOctave = -1;
    dogThresh = -1;
    edgeThresh = -1;
  }

  /// /return True if maxWorkingDimension was set.
  bool isSetMaxWorkingDimension() const;

  /// /return True if maxOctaves was set.
  bool isSetMaxOctaves() const;

  /// /return True if dogLevelsInAnOctave was set.
  bool isSetDogLevelsInAnOctave() const;

  /// /return True if dogThresh was set.
  bool isSetDogThresh() const;

  /// /return True if edgeThresh was set.
  bool isSetEdgeThresh() const;

  /// Write to a file to record which parameters were used.
  /// \param[in,out] file Opened output file.
  YASFM_API void write(ostream& file) const;

  /// Images larger than this will be downsampled. Negative value selects 
  /// SIFTGPU default: 3200.
  int maxWorkingDimension;

  /// The smaller the more features will get detected. Min is -1.
  int firstOctave;

  /// The smaller the less features will get detected. Negative value selects 
  /// SIFTGPU default: no limit.
  int maxOctaves;

  /// DOG levels in an octave. Can affect the number of extracted features. 
  /// Negative value selects SIFTGPU default: 3.
  int dogLevelsInAnOctave;

  /// DOG threshold. Negative value selects SIFTGPU default: 0.02/3.
  float dogThresh;
  
  /// Edge threshold. Decrease to eliminate more keypoints. Negative value 
  /// selects default: 10.
  float edgeThresh;

  /// Only one fixed orientation per keypoint location.
  bool detectUprightSIFT;

  /// Verbosity:
  /**
  0:   no output at all,except errors 
  1:   print out over all timing and features numbers 
  2:   print out timing for each steps 
  3/4: print out timing for each octaves/ levels
  */
  int verbosityLevel;
};

/// Options for matching features using FLANN.
struct OptionsFLANN
{
  /// Constructor setting defaults.
  YASFM_API OptionsFLANN()
  {
    indexParams = flann::KDTreeIndexParams();
    searchParams = flann::SearchParams();
    ratioThresh = 0.6f;
    onlyUniques = true;
    verbose = true;
  }

  /// \return True if the matches should be filtered by ratio of distances of 
  /// the first over the second nearest neighbors.
  bool filterByRatio() const;

  /// Write to a file to record which parameters were used.
  /// \param[in,out] file Opened output file.
  YASFM_API void write(ostream& file) const;

  /// What method to use for matching. See FLANN for more details.
  flann::IndexParams indexParams;

  /// Nearest neighbor search parameters, eg. num of checks. See FLANN for more details.
  flann::SearchParams searchParams;

  /// Threshold for the ratio d1/d2, where di is distance to the i-th nearest neighbor
  /// Negative values disable this filter and only the the nearest is searched.
  float ratioThresh;
  
  /// Discards non-unique matches, i.e., those for which two or more 
  /// different features in feats1 matched to the same feature in feats2
  bool onlyUniques;

  /// Verbosity.
  bool verbose;
};

/// Options for running RANSAC like algorithms.
struct OptionsRANSAC
{
  /// Constructor.
  YASFM_API OptionsRANSAC(int maxRounds,double errorThresh,
    int minInliers) 
    : maxRounds(maxRounds),errorThresh(errorThresh),
    minInliers(minInliers)
  {
    confidence = 0.95;
    refineTolerance = 1e-12;
  }

  /// Constructor.
  YASFM_API OptionsRANSAC(int maxRounds,double errorThresh,
    int minInliers,double confidence)
    : maxRounds(maxRounds),errorThresh(errorThresh),
    minInliers(minInliers),confidence(confidence)
  {
    refineTolerance = 1e-12;
  }

  /// Write to a file to record which parameters were used.
  /// \param[in,out] file Opened output file.
  YASFM_API void write(ostream& file) const;

  /// Maximum number of iterations.
  int maxRounds;

  /// Error threshold.
  double errorThresh;

  /// Minimum number of inliers.
  int minInliers;

  /// Find a good hypothesis with this confidence. The values are from range [0,1].
  double confidence;

  /// The tolerance for refining the result on inliers. Used for example to terminate
  // optimization of non-linear function using LM method. 
  // (Not all ransac mediators implement refine phase.)
  double refineTolerance;
};

/// Options for runnig bundle adjustment.
struct YASFM_API OptionsBundleAdjustment
{
  /// Constructor setting defaults.
  OptionsBundleAdjustment()
  {
    solverOptions.max_num_iterations = 10;
    solverOptions.num_threads = 1;
    solverOptions.function_tolerance = 1e-3;
    solverOptions.parameter_tolerance = 1e-3;
    solverOptions.gradient_tolerance = 1e-3;
    robustify = false;
  }

  /// Write to a file to record which parameters were used.
  /// \param[in,out] file Opened output file.
  void write(ostream& file) const;

  /// Ceres solver options. Some of the interesting are:
  /// max_num_iterations, num_threads, function_tolerance, parameter_tolerance, 
  /// gradient_tolerance, minimizer_progress_to_stdout
  ceres::Solver::Options solverOptions;

  /// Uses Hubers loss function instead of L2 norm, more robust to outliers, 
  /// if set to true.
  bool robustify;
};

/// Options for geometric verification
struct OptionsGeometricVerification
{
  YASFM_API OptionsGeometricVerification()
  {
    similarityThresh = 20;
    affinityThresh = 10;
    homographyThresh = 5;
    minInliersPerTransform = 10;
    maxTransforms = 7;
    nRefineIterations = 8;
    minInliersToRefine = 4;
    stopInlierFraction = 0.7;
  }

  /// Threshold defining inliers for estimated similarity.
  double similarityThresh;

  /// Threshold defining inliers for estimated affinity.
  double affinityThresh;

  /// Threshold defining inliers for estimated homography.
  double homographyThresh;

  /// Minimum number of inliers to one transformation.
  int minInliersPerTransform;

  /// Maximum number of transformations that should be estimated.
  int maxTransforms;

  /// Number of refine iterations that should be done to a transformation.
  /// 1st iteration is estimated using key coordinates, scale and orientation.
  /// 2-4th iterations are affinities
  /// 5+th iterations are homographies
  int nRefineIterations;

  /// Minimum number of inliers to continue in further refining.
  int minInliersToRefine;

  /// Value in [0,1]. If there is this fraction of inliers found the 
  /// refine phase is terminated.
  double stopInlierFraction;

  /// Write to a file to record which parameters were used.
  /// \param[in,out] file Opened output file.
  YASFM_API void write(ostream& file) const;
};

}