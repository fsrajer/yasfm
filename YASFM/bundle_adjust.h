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
#include "options.h"
#include "sfm_data.h"

using std::vector;

namespace yasfm
{

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