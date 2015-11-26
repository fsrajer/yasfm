/*
* Filip Srajer
* filip.srajer (at) fel.cvut.cz
* Center for Machine Perception
* Czech Technical University in Prague
*
* This software is under construction.
* 05/2015
*/

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

// Loads input from images as well as saves output into them.
YASFM_API void detectSiftGPU(const OptionsSIFTGPU& opt,ptr_vector<Camera> *cams);

} // namespace yasfm

namespace
{

// Automatically loads SiftGPU dll.
// 1) isLoadedDLL and 2) initialize should be called and 
// verified that true was returned.
class SiftGPUAutoMemRelease
{
public:
  SiftGPUAutoMemRelease();
  ~SiftGPUAutoMemRelease();

  bool isLoadedDLL() const;
  bool initialize(const OptionsSIFTGPU& opt,int maxWidth,int maxHeight);

  SiftGPU* sift;
private:
  void setParams(const OptionsSIFTGPU& opt);

#ifdef _WIN32
  HMODULE siftgpuHandle; // nullptr value means that loading was unsuccessful
#else
  void *siftgpuHandle // nullptr value means that loading was unsuccessful
#endif
};

// Detect on one image using loaded SiftGPU. image serves as input and output.
void detectSiftGPU(const SiftGPUAutoMemRelease& siftHandle,Camera *cam);

} // namespace