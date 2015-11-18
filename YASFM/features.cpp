/*
* Filip Srajer
* filip.srajer (at) fel.cvut.cz
* Center for Machine Perception
* Czech Technical University in Prague
*
* This software is under construction.
* 05/2015
*/

#include "features.h"

#include <cstdlib>
#include <algorithm>
#include <iostream>
#include <string>
#include <vector>
#include "SiftGPU/SiftGPU.h"

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

#include "defines.h"
#include "options.h"
#include "utils.h"

using Eigen::Vector3d;
using Eigen::Vector4d;
using Eigen::Matrix3d;
using Eigen::Matrix4d;
using namespace yasfm;
using std::cout;
using std::cerr;
using std::endl;
using std::string;
using std::vector;
using std::to_string;

namespace yasfm
{

void detectSiftGPU(const OptionsSIFT& opt,ptr_vector<Camera> *cams)
{
  int maxWidth = -1;
  int maxHeight = -1;
  for(const auto& pcam : (*cams))
  {
    maxWidth = std::max(maxWidth,pcam->imgWidth());
    maxHeight = std::max(maxHeight,pcam->imgHeight());
  }

  SiftGPUAutoMemRelease siftHandle;
  if(!siftHandle.isLoadedDLL())
  { return; }

  bool success = siftHandle.initialize(opt,maxWidth,maxHeight);
  if(!success)
  { return; }

  for(auto& pcam : (*cams))
  {
    detectSiftGPU(siftHandle,&(*pcam));
  }
}

} // namespace yasfm


namespace
{

void detectSiftGPU(const SiftGPUAutoMemRelease& siftHandle,Camera *cam)
{
  if(siftHandle.sift->RunSIFT(cam->imgFilename().c_str()))
  {
    int num = siftHandle.sift->GetFeatureNum();
    auto *keys = new SiftGPU::SiftKeypoint[num];
    auto *descr = new float[128 * num];
    siftHandle.sift->GetFeatureVector(keys,descr);

    cam->reserveFeatures(num,128);
    for(int i = 0; i < num; i++)
    {
      if (keys[i].x >= 0 && keys[i].y >= 0) // bug check - this happens rarely but it happens
      {
        cam->addFeature(keys[i].x, keys[i].y, descr + i * 128);
      }
    }
    delete[] keys;
    delete[] descr;
  }
}

SiftGPUAutoMemRelease::SiftGPUAutoMemRelease() : sift(nullptr),siftgpuHandle(nullptr)
{
#ifdef _WIN32
#ifdef _DEBUG
  siftgpuHandle = LoadLibrary("SiftGPU64_d.dll");
#else
  siftgpuHandle = LoadLibrary("SiftGPU64.dll");
#endif
#else
  siftgpuHandle = dlopen("libsiftgpu.so",RTLD_LAZY);
#endif

  if(siftgpuHandle == nullptr)
  {
    cerr << "detectSiftGPU: could not load SiftGPU library" << endl;
    return;
  }

  SiftGPU* (*pCreateNewSiftGPU)(int) = nullptr;
  pCreateNewSiftGPU = (SiftGPU* (*) (int)) YASFM_GET_PROC(siftgpuHandle,"CreateNewSiftGPU");
  sift = pCreateNewSiftGPU(1);
}

bool SiftGPUAutoMemRelease::isLoadedDLL() const
{
  return (siftgpuHandle != nullptr && sift != nullptr);
}

void SiftGPUAutoMemRelease::setParams(const OptionsSIFT& opt)
{
  vector<string> opts;
  opts.push_back("-fo");
  opts.push_back(to_string(opt.firstOctave_));
  if(opt.isSetMaxWorkingDimension())
  {
    opts.push_back("-maxd");
    opts.push_back(to_string(opt.maxWorkingDimension_));
  }
  if(opt.isSetMaxOctaves())
  {
    opts.push_back("-no");
    opts.push_back(to_string(opt.maxOctaves_));
  }
  if(opt.isSetDogLevelsInAnOctave())
  {
    opts.push_back("-d");
    opts.push_back(to_string(opt.dogLevelsInAnOctave_));
  }
  if(opt.isSetDogThresh())
  {
    opts.push_back("-t");
    opts.push_back(to_string(opt.dogThresh_));
  }
  if(opt.isSetEdgeThresh())
  {
    opts.push_back("-e");
    opts.push_back(to_string(opt.edgeThresh_));
  }
  if(opt.detectUprightSIFT_)
  {
    // fix orientation
    opts.push_back("-ofix");
    // max 1 orientation per feature
    opts.push_back("-m");
    opts.push_back("-mo");
    opts.push_back("1");
  }
  opts.push_back("-v");
  opts.push_back(to_string(1)); // verbosity

  char **argv = new char*[opts.size()];
  for(size_t i = 0; i < opts.size(); i++)
  {
    argv[i] = new char[opts[i].length() + 1];
    strcpy(argv[i],opts[i].c_str());
  }
  int argc = (int)opts.size();
  sift->ParseParam(argc,argv);

  for(int i = 0; i < argc; i++)
  {
    delete[] argv[i];
  }
  delete[] argv;
}

bool SiftGPUAutoMemRelease::initialize(const OptionsSIFT& opt,int maxWidth,int maxHeight)
{
  setParams(opt);

  if(sift->CreateContextGL() != SiftGPU::SIFTGPU_FULL_SUPPORTED)
  {
    cerr << "detectSiftGPU: could not create OpenGL context" << endl;
    return false;
  }

  sift->AllocatePyramid(maxWidth,maxHeight);
  return true;
}

SiftGPUAutoMemRelease::~SiftGPUAutoMemRelease()
{
  if(sift)
    delete sift;
  if(siftgpuHandle)
    YASFM_FREE_LIB(siftgpuHandle);
}

} // namespace