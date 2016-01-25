#include "features.h"

#include <iostream>
#include <string>
#include <vector>
#include <memory>

using std::cerr;
using std::cout;
using std::string;
using std::to_string;
using std::vector;
using std::unique_ptr;
using namespace yasfm;

namespace yasfm
{

void detectSiftGPU(const OptionsSIFTGPU& opt,ptr_vector<Camera> *cams,
  DetectSiftCallbackFunctionPtr callbackFunction,void * callbackObjectPtr)
{
  int nCams = static_cast<int>(cams->size());
  vector<int> camsToUse(nCams);
  for(int i = 0; i < nCams; i++)
    camsToUse[i] = i;
  detectSiftGPU(opt,camsToUse,cams,callbackFunction,callbackObjectPtr);
}

void detectSiftGPU(const OptionsSIFTGPU& opt,const vector<int>& camsToUse,
  ptr_vector<Camera> *pcams,
	DetectSiftCallbackFunctionPtr callbackFunction, void * callbackObjectPtr)
{
  auto& cams = *pcams;
  int maxWidth = -1;
  int maxHeight = -1;
  for(size_t i = 0; i < camsToUse.size(); i++)
  {
    const auto& cam = *cams[camsToUse[i]];
    maxWidth = std::max(maxWidth,cam.imgWidth());
    maxHeight = std::max(maxHeight,cam.imgHeight());
  }

  unique_ptr<SiftGPU> sift(CreateNewSiftGPU(1));

  bool success = initializeSiftGPU(opt,maxWidth,maxHeight,sift.get());
  if(!success)
  { return; }
  
  for(size_t i = 0; i < camsToUse.size(); i++)
  {
    int camIdx = camsToUse[i];
	::detectSiftGPU(sift.get(), cams[camIdx].get());
    if(callbackFunction != NULL&&callbackObjectPtr != NULL)
    {
      callbackFunction(callbackObjectPtr,camIdx);
    }
  }
}

} // namespace yasfm


namespace
{

void detectSiftGPU(SiftGPU *sift,Camera *cam)
{
  if(sift->RunSIFT(cam->imgFilename().c_str()))
  {
    int num = sift->GetFeatureNum();
    auto *keys = new SiftGPU::SiftKeypoint[num];
    auto *descr = new float[128 * num];
    sift->GetFeatureVector(keys,descr);

    cam->resizeFeatures(num,128);
    for(int i = 0; i < num; i++)
    {
      cam->setFeature(i,keys[i].x,keys[i].y,keys[i].s,keys[i].o,descr + i * 128);
    }
    delete[] keys;
    delete[] descr;
    cam->writeFeatures();
  }
}

void setParamsSiftGPU(const OptionsSIFTGPU& opt,SiftGPU *sift)
{
  vector<string> opts;
  opts.push_back("-fo");
  opts.push_back(to_string(opt.get<int>("firstOctave")));
  if(!opt.useSIFTGPUDefaultForIntField("maxWorkingDimension"))
  {
    opts.push_back("-maxd");
    opts.push_back(to_string(opt.get<int>("maxWorkingDimension")));
  }
  if(!opt.useSIFTGPUDefaultForIntField("maxOctaves"))
  {
    opts.push_back("-no");
    opts.push_back(to_string(opt.get<int>("maxOctaves")));
  }
  if(!opt.useSIFTGPUDefaultForIntField("dogLevelsInAnOctave"))
  {
    opts.push_back("-d");
    opts.push_back(to_string(opt.get<int>("dogLevelsInAnOctave")));
  }
  if(!opt.useSIFTGPUDefaultForIntField("dogThresh"))
  {
    opts.push_back("-t");
    opts.push_back(to_string(opt.get<int>("dogThresh")));
  }
  if(!opt.useSIFTGPUDefaultForIntField("edgeThresh"))
  {
    opts.push_back("-e");
    opts.push_back(to_string(opt.get<int>("edgeThresh")));
  }
  if(opt.get<bool>("detectUprightSIFT"))
  {
    // fix orientation
    opts.push_back("-ofix");
    // max 1 orientation per feature
    opts.push_back("-m");
    opts.push_back("-mo");
    opts.push_back("1");
  }
  opts.push_back("-v");
  opts.push_back(to_string(opt.get<int>("verbosityLevel")));

  opts.push_back("-tc3");
  opts.push_back(to_string(opt.get<int>("softmaxFeatures")));

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

bool initializeSiftGPU(const OptionsSIFTGPU& opt,int maxWidth,int maxHeight,
  SiftGPU *sift)
{
  setParamsSiftGPU(opt,sift);

  if(sift->CreateContextGL() != SiftGPU::SIFTGPU_FULL_SUPPORTED)
  {
    cerr << "ERROR: SiftGPUAutoMemRelease::initialize: Could not create OpenGL context.\n";
    return false;
  }

  sift->AllocatePyramid(maxWidth,maxHeight);
  return true;
}

} // namespace