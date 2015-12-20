//----------------------------------------------------------------------------------------
/**
* \file       bundle_adjust.h
* \brief      Bundle adjustment.
*
*  Bundle adjustment.
*
*/
//----------------------------------------------------------------------------------------

#pragma once

#include <vector>

#include "defines.h"
#include "sfm_data.h"

using std::vector;

namespace yasfm
{

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

/// Run bundle adjustement on all the cameras and all the points.
/**
\param[in] opt Options.
\param[in,out] cams Cameras.
\param[in,out] pts Points.
*/
YASFM_API void bundleAdjust(const OptionsBundleAdjustment& opt,
  ptr_vector<Camera> *cams,Points *pts);

/// Run bundle adjustement on all the cameras and keep the points fixed.
/**
\param[in] opt Options.
\param[in,out] cams Cameras.
\param[in,out] pts Points.
*/
YASFM_API void bundleAdjustCams(const OptionsBundleAdjustment& opt,
  ptr_vector<Camera> *cams,Points *pts);

/// Run bundle adjustement on and all the points and keep the cameras fixed.
/**
\param[in] opt Options.
\param[in,out] cams Cameras.
\param[in,out] pts Points.
*/
YASFM_API void bundleAdjustPoints(const OptionsBundleAdjustment& opt,
  ptr_vector<Camera> *cams,Points *pts);

/// Run bundle adjustment.
/**
\param[in] opt Options.
\param[in] constantCams Which cameras should be kept constant.
\param[in] constantPoints Which points should be kept constant.
\param[in,out] cams Cameras.
\param[in,out] pts Points.
*/
YASFM_API void bundleAdjust(const OptionsBundleAdjustment& opt,
  const vector<bool>& constantCams,const vector<bool>& constantPoints,
  ptr_vector<Camera> *cams,Points *pts);

/// Run bundle adjustment of one camera and keep all the points fixed.
/**
\param[in] opt Options.
\param[in] camIdx Camera index.
\param[in,out] cam Camera.
\param[in] pts Points.
*/
YASFM_API void bundleAdjustOneCam(const OptionsBundleAdjustment& opt,int camIdx,
  Camera *cam,Points *pts);

/// Run bundle adjustment of one camera.
/**
\param[in] opt Options.
\param[in] camIdx Camera index.
\param[in] constantPoints Which points should be kept constant.
\param[in,out] cam Camera.
\param[in,out] pts Points.
*/
YASFM_API void bundleAdjustOneCam(const OptionsBundleAdjustment& opt,
  int camIdx,const vector<bool>& constantPoints,Camera *cam,Points *pts);

} // namespace yasfm