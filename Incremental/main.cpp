/*
* Filip Srajer
* filip.srajer (at) fel.cvut.cz
* Center for Machine Perception
* Czech Technical University in Prague
*
* This software is under construction.
* 02/2015
*/

#include <cstdlib>
#include <ctime>

#include <iostream>
#include <fstream>
#include <string>
#include <unordered_set>
#include <vector>

#include "YASFM\features.h"
#include "YASFM\bundle_adjust.h"
#include "YASFM\matching.h"
#include "YASFM\options.h"
#include "YASFM\sfm_data.h"
#include "YASFM\points.h"
#include "YASFM\absolute_pose.h"
#include "YASFM\relative_pose.h"
#include "YASFM\utils.h"
#include "YASFM\utils_io.h"
#include "Eigen\Dense"

using namespace yasfm;
using std::cin;
using std::cout;
using std::endl;
using std::ifstream;
using std::string;
using std::unordered_set;
using std::vector;

int main(int argc, const char* argv[])
{
  Options opt;
  opt.sift_.firstOctave_ = 0;
  if(argc >= 4)
    opt.sift_.firstOctave_ = atoi(argv[3]);
  opt.ba_.solverOptions.num_threads = 8;
  double focalConstraintWeight = 0.0001;
  double radialConstraint = 0.;
  double radialConstraintWeight = 100.;

  string dir(argv[1]);
  string imgsSubdir(argv[2]);
  
  Dataset data(dir);
  data.addCameras<StandardCameraRadial>(imgsSubdir);
  // -> the principal point is always set to the
  // image center in StandardCamera

  // Initialize calibration for every camera
  vector<double> focals(data.cams().size());
  findFocalLengthInEXIF(opt.ccdDBFilename_,data.cams(),&focals);
  for(int i = 0; i < data.numCams(); i++)
  {
    StandardCameraRadial *cam = static_cast<StandardCameraRadial *>(&data.cam(i));
    vector<double> radConstraints(2,radialConstraint),radWeights(2,radialConstraintWeight);
    cam->constrainRadial(&radConstraints[0],&radWeights[0]);
    if(focals[i] > 0.)
    {
      data.cam(i).setFocal(focals[i]);
      cam->constrainFocal(focals[i],focalConstraintWeight);
    }
  }

  detectSiftGPU(opt.sift_,&data.cams());

  matchFeatFLANN(opt.matchingFLANN_,data.cams(),&data.pairs());
  removePoorlyMatchedPairs(opt.minNumMatches_,&data.pairs());
  data.clearDescriptors();

  verifyMatchesGeometrically(opt,data.cams(),&data.pairs());
  removePoorlyMatchedPairs(opt.minNumMatches_,&data.pairs());

  twoViewMatchesToNViewMatches(data.cams(),data.pairs(),&data.points().matchesToReconstruct());

  vector<int> camsPriority(data.numCams(),0);
  for(size_t i = 0; i < camsPriority.size(); i++)
    if(focals[i] > 0.)
      camsPriority[i] = 1;
  IntPair initPair = chooseInitialCameraPair(opt.minNumMatches_,
    data.points().matchesToReconstruct(),camsPriority);
  if(initPair.first < 0 || initPair.second < 0)
    return EXIT_FAILURE;

  if(focals[initPair.first] > 0. && focals[initPair.second] > 0.)
  {
    initReconstructionFromCalibratedCamPair(opt,initPair,&data);
  } else
  {
    initReconstructionFromCamPair(opt,initPair,&data);
  }

  bundleAdjust(opt.ba_,&data.cams(),&data.points());
  writeSFMBundlerFormat(joinPaths(data.dir(), "bundle2.out"), data);

  uset<int> exploredCams;
  exploredCams.insert(initPair.first);
  exploredCams.insert(initPair.second);
  while(data.cams().size() > exploredCams.size())
  {
    vector<vector<IntPair>> camToSceneMatches;
    findCamToSceneMatches(exploredCams,data.numCams(),data.points(),&camToSceneMatches);

    uset<int> wellMatchedCams;
    chooseWellMatchedCameras(opt.minNumCam2SceneMatches_,opt.wellMatchedCamsFactor_,
      camToSceneMatches,&wellMatchedCams);

    if(wellMatchedCams.empty())
      break;

    for(int camIdx : wellMatchedCams)
    {
      vector<int> inliers;
      cout << "trying to resect camera " << camIdx << " using " << 
        camToSceneMatches[camIdx].size() << " matches ... ";
      bool success = resectCamera6ptRANSAC(opt.absolutePose_,camToSceneMatches[camIdx],
        data.points().ptCoord(),&data.cam(camIdx),&inliers);

      exploredCams.insert(camIdx);
      if(success)
      {
        cout << "camera successfully added.\n";
        vector<int> ptIdxs;
        unzipPairsVectorSecond(camToSceneMatches[camIdx],&ptIdxs);
        data.markCamAsReconstructed(camIdx,ptIdxs,inliers);

        bundleAdjustOneCam(opt.ba_,camIdx,&data.cam(camIdx),&data.points());
      } else
      {
        cout << "camera could not be added.\n";
      }
    }

    int minObservingCams = 2;
    vector<SplitNViewMatch> matchesToReconstructNow;
    extractCandidateNewPoints(minObservingCams,opt.rayAngleThresh_,
      data.reconstructedCams(),data.cams(),
      &data.points().matchesToReconstruct(),&matchesToReconstructNow);

    reconstructPoints(data.cams(),matchesToReconstructNow,&data.points());
    removeHighReprojErrorPoints(opt.pointsReprojErrorThresh_,data.cams(),&data.points());

    bundleAdjust(opt.ba_,&data.cams(),&data.points());
    removeHighReprojErrorPoints(opt.pointsReprojErrorThresh_,data.cams(),&data.points());
    removeIllConditionedPoints(0.5*opt.rayAngleThresh_,data.cams(),&data.points());

    writeSFMBundlerFormat(joinPaths(data.dir(),"bundle" + 
      std::to_string(exploredCams.size()) + ".out"),data);
  }

  writeSFMBundlerFormat(joinPaths(data.dir(),"bundle_final.out"),data);
}