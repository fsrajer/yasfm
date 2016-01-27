//----------------------------------------------------------------------------------------
/**
* \file       features.h
* \brief      Functions for feature detection and description.
*
*  Functions for feature detection and description.
*
*/
//----------------------------------------------------------------------------------------

#pragma once

#include <cstdlib>
#include <memory>

#include "SiftGPU/SiftGPU.h"

#include "defines.h"
#include "sfm_data.h"
#include "options_types.h"

using std::make_unique;

namespace yasfm
{

/// Options for SIFTGPU
/**
Fields:
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

/// Soft max features per image (see SiftGPU documentation for tc3 option).
/// It will typically detect somewhat more features than this limit.
int softmaxFeatures;

/// Verbosity:
/// 0:   no output at all,except errors
/// 1:   print out over all timing and features numbers
/// 2:   print out timing for each steps
/// 3/4: print out timing for each octaves/ levels
int verbosityLevel;
*/
class OptionsSIFTGPU : public OptionsWrapper
{
public:
  /// Constructor setting defaults.
  YASFM_API OptionsSIFTGPU()
  {
    opt.emplace("firstOctave",make_unique<OptTypeWithVal<int>>(-1));
    opt.emplace("detectUprightSIFT",make_unique<OptTypeWithVal<bool>>(false));
    opt.emplace("verbosityLevel",make_unique<OptTypeWithVal<int>>(1));
    opt.emplace("softmaxFeatures",make_unique<OptTypeWithVal<int>>(20000));

    // let SIFTGPU autoselect these
    opt.emplace("maxWorkingDimension",make_unique<OptTypeWithVal<int>>(-1));
    opt.emplace("maxOctaves",make_unique<OptTypeWithVal<int>>(-1));
    opt.emplace("dogLevelsInAnOctave",make_unique<OptTypeWithVal<int>>(-1));
    opt.emplace("dogThresh",make_unique<OptTypeWithVal<int>>(-1));
    opt.emplace("edgeThresh",make_unique<OptTypeWithVal<int>>(-1));
  }

  bool useSIFTGPUDefaultForIntField(const string& fieldName) const
  {
    return (get<int>(fieldName) < 0);
  }
};

/// Callback function for progress notifying
/**
\param[in,out] object void pointer to the callee object
\param[in] camId index of the last processed camera
*/
typedef void(*DetectSiftCallbackFunctionPtr)(void *object, int camId);

/// Detect SIFT for all cameras using SIFTGPU (descriptors normalized to unit length).
/**
Calls overloaded function so that SIFT would be detected for all cameras.

\param[in] opt Options.
\param[in,out] cams Cameras. Image dimensions and image filename have to be set
in order to detect SIFT.
\param[out] callbackFunction Optional. Function to be called after finishing
detection in one image
\param[out] callbackObjectPtr Optional. Object to be passed to callbackFunction.
*/
YASFM_API void detectSiftGPU(const OptionsSIFTGPU& opt,ptr_vector<Camera> *cams,
  DetectSiftCallbackFunctionPtr callbackFunction = NULL,void * callbackObjectPtr = NULL);

/// Detect SIFT for all cameras using SIFTGPU (descriptors normalized to unit length).
/**
\param[in] opt Options.
\param[in] camsToUse On which cams should SIFT be detected?
\param[in,out] cams Cameras. Image dimensions and image filename have to be set
in order to detect SIFT.
\param[out] callbackFunction Optional. Function to be called after finishing
detection in one image
\param[out] callbackObjectPtr Optional. Object to be passed to callbackFunction.
*/
YASFM_API void detectSiftGPU(const OptionsSIFTGPU& opt,const vector<int>& camsToUse,
  ptr_vector<Camera> *cams,
	DetectSiftCallbackFunctionPtr callbackFunction = NULL, void * callbackObjectPtr = NULL);

} // namespace yasfm

namespace
{

/// Detect SIFT on one image using loaded SiftGPU.
/**
\param[in] sift Initialized SIFTGPU object.
\param[in,out] cam Camera, which has to have image filename set.
*/
void detectSiftGPU(SiftGPU *sift,Camera *cam);

/// Convert from OptionsSIFTGPU to options in SIFTGPU object.
/// \param[in] opt Options.
/// \param[out] sift SIFT object.
void setParamsSiftGPU(const OptionsSIFTGPU& opt,SiftGPU *sift);

/// Initialize sift object.
/**
Initialize context and allocate pyramid.

\param[in] opt Options.
\param[in] maxWidth Maximum width of all images that will be used.
\param[in] maxHeight Maximum height of all images that will be used.
\param[out] sift SIFT object to initialize.
\return True if successfully initialized.
*/
bool initializeSiftGPU(const OptionsSIFTGPU& opt,int maxWidth,int maxHeight,
  SiftGPU *sift);

} // namespace