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
#include "options.h"
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