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

#include "SiftGPU/SiftGPU.h"

#include "defines.h"
#include "sfm_data.h"

// loading dll at runtime, see SiftGPU SimpleSIFT example for 
// changing to dynamic linking
#ifdef _WIN32
#define WIN32_LEAN_AND_MEAN
#include <windows.h>
#define YASFM_FREE_LIB FreeLibrary
#define YASFM_GET_PROC GetProcAddress
#else
#include <dlfcn.h>
#define YASFM_FREE_LIB dlclose
#define YASFM_GET_PROC dlsym
#endif

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

/// Detect SIFT for all cameras using SIFTGPU.
/**
\param[in] opt Options.
\param[in,out] cams Cameras. Image dimensions and image filename have to be set
in order to detect SIFT.
*/
YASFM_API void detectSiftGPU(const OptionsSIFTGPU& opt,ptr_vector<Camera> *cams);

/// Detect SIFT using SIFTGPU.
/**
\param[in] opt Options.
\param[in,out] cam Camera. Image dimensions and image filename have to be set
in order to detect SIFT.
*/
YASFM_API void detectSiftGPU(const OptionsSIFTGPU& opt,Camera *cam);

} // namespace yasfm

namespace
{

/// Helper class for automatically releasing SIFTGPU resources.
/**
After construction, one should check that isLoadedDLL(), run initialize() and 
verifie that true was returned by initialize().
*/
class SiftGPUAutoMemRelease
{
public:
  /// Constructor. Automatically loads SIFTGPU dll.
  SiftGPUAutoMemRelease();

  /// Destructor.
  ~SiftGPUAutoMemRelease();

  /// \return True if the dll was successfully loaded.
  bool isLoadedDLL() const;

  /// Initialize sift object.
  /**
  Initialize context and allocate pyramid.

  \param[in] opt Options.
  \param[in] maxWidth Maximum width of all images that will be used.
  \param[in] maxHeight Maximum height of all images that will be used.
  \return True if successfully initialized.
  */
  bool initialize(const OptionsSIFTGPU& opt,int maxWidth,int maxHeight);

  SiftGPU* sift; ///< SIFTGPU object.

private:
  /// Convert from OptionsSIFTGPU to options in SIFTGPU object.
  /// \param[in] opt Options.
  void setParams(const OptionsSIFTGPU& opt);

#ifdef _WIN32
  HMODULE siftgpuHandle; ///< nullptr value means that loading was unsuccessful
#else
  void *siftgpuHandle ///< nullptr value means that loading was unsuccessful
#endif
};

/// Detect SIFT on one image using loaded SiftGPU.
/**
\param[in] siftHandle Initialized object.
\param[in,out] cam Camera, which has to have image filename set.
*/
void detectSiftGPU(const SiftGPUAutoMemRelease& siftHandle,Camera *cam);

} // namespace