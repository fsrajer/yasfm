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
#include <memory>

#include "ceres/ceres.h"

#include "defines.h"
#include "sfm_data.h"
#include "options_types.h"

using std::vector;
using std::make_unique;

namespace yasfm
{

/// Options for runnig bundle adjustment.
/**
Fields:
/// Ceres solver options. Some of the interesting are:
/// max_num_iterations, num_threads, function_tolerance, parameter_tolerance, 
/// gradient_tolerance, minimizer_progress_to_stdout
ceres::Solver::Options solverOptions;

/// Uses Hubers loss function instead of L2 norm, more robust to outliers, 
/// if set to true.
bool robustify;
*/
class OptionsBundleAdjustment : public OptionsWrapper
{
public:
  /// Constructor setting defaults.
  YASFM_API OptionsBundleAdjustment()
  {
    opt.emplace("solverOptions",make_unique<OptTypeWithVal<ceres::Solver::Options>>());
    opt.emplace("robustify",make_unique<OptTypeWithVal<bool>>(false));

    auto& solverOptions = get<ceres::Solver::Options>("solverOptions");
    solverOptions.max_num_iterations = 10;
    solverOptions.num_threads = 8;
    solverOptions.function_tolerance = 1e-3;
    solverOptions.parameter_tolerance = 1e-3;
    solverOptions.gradient_tolerance = 1e-3;
  }
};

/// Run bundle adjustement on all the cameras and all the points.
/**
\param[in] opt Options.
\param[in,out] cams Cameras.
\param[in,out] pts Points.
*/
YASFM_API void bundleAdjust(const OptionsBundleAdjustment& opt,
  ptr_vector<Camera> *cams,vector<Point> *pts);

/// Run bundle adjustement on all the cameras and keep the points fixed.
/**
\param[in] opt Options.
\param[in,out] cams Cameras.
\param[in,out] pts Points.
*/
YASFM_API void bundleAdjustCams(const OptionsBundleAdjustment& opt,
  ptr_vector<Camera> *cams,vector<Point> *pts);

/// Run bundle adjustement on and all the points and keep the cameras fixed.
/**
\param[in] opt Options.
\param[in,out] cams Cameras.
\param[in,out] pts Points.
*/
YASFM_API void bundleAdjustPoints(const OptionsBundleAdjustment& opt,
  ptr_vector<Camera> *cams,vector<Point> *pts);

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
  ptr_vector<Camera> *cams,vector<Point> *pts);

/// Run bundle adjustment of one camera and keep all the points fixed.
/**
\param[in] opt Options.
\param[in] camIdx Camera index.
\param[in,out] cam Camera.
\param[in] pts Points.
*/
YASFM_API void bundleAdjustOneCam(const OptionsBundleAdjustment& opt,int camIdx,
  Camera *cam,vector<Point> *pts);

/// Run bundle adjustment of one camera.
/**
\param[in] opt Options.
\param[in] camIdx Camera index.
\param[in] constantPoints Which points should be kept constant.
\param[in,out] cam Camera.
\param[in,out] pts Points.
*/
YASFM_API void bundleAdjustOneCam(const OptionsBundleAdjustment& opt,
  int camIdx,const vector<bool>& constantPoints,Camera *cam,vector<Point> *pts);

} // namespace yasfm