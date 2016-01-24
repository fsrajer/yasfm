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
#include <direct.h>

#include <algorithm>
#include <iostream>
#include <fstream>
#include <string>
#include <unordered_set>
#include <vector>

#include "YASFM/standard_camera_radial.h"
#include "YASFM/features.h"
#include "YASFM/bundle_adjust.h"
#include "YASFM/matching.h"
#include "YASFM/sfm_data.h"
#include "YASFM/points.h"
#include "YASFM/absolute_pose.h"
#include "YASFM/relative_pose.h"
#include "YASFM/utils.h"
#include "YASFM/utils_io.h"
#include "YASFM/image_similarity.h"
#include "Eigen/Dense"

using namespace yasfm;
using std::cin;
using std::cout;
using std::endl;
using std::ifstream;
using std::ofstream;
using std::string;
using std::unordered_set;
using std::vector;

struct Options
{
  Options()
    :
    ccdDBFilename("../resources/camera_ccd_widths.txt"),
    sift(),
    maxVocabularySize(15000),
    nSimilarCamerasToMatch(20),
    matchingFLANN(),
    minNumPairwiseMatches(16),
    geometricVerification(),
    epipolarVerification(2048,sqrt(5.),minNumPairwiseMatches),
    initialPairRelativePose(512,1.25,10),
    homography(512,5.,10),
    minInitPairHomographyProportion(0.5),
    absolutePose(4096,4.,16,0.999999),
    minNumCamToSceneMatches(minNumPairwiseMatches),
    wellMatchedCamsFactor(0.75),
    bundleAdjust(),
    pointsReprojErrorThresh(8.),
    rayAngleThresh(2.),
    focalConstraintWeight(0.0001),
    radialConstraint(0.),
    radialConstraintWeight(100.),
    defaultFocalDividedBySensorSize(1.083)  // assume angle of view 55 degrees
  {
  }

  string ccdDBFilename;
  OptionsSIFTGPU sift;
  int maxVocabularySize;
  int nSimilarCamerasToMatch;
  OptionsFLANN matchingFLANN;
  // Min number of matches defining a poorly matched pair. Default: 16.
  int minNumPairwiseMatches;
  OptionsGeometricVerification geometricVerification;
  // The error is symmetric distance. Units are pixels.
  OptionsRANSAC epipolarVerification;
  // Units of the error are pixels.
  OptionsRANSAC initialPairRelativePose;
  // Units of the error are pixels.
  OptionsRANSAC homography;
  double minInitPairHomographyProportion;
  // The error is reprojection error. Units are pixels.
  OptionsRANSAC absolutePose;
  int minNumCamToSceneMatches;
  // chooseWellMatchedCameras finds the camera with most matches, say N
  // and then finds all cameras with N*wellMatchedCamsFactor matches. Default: 0.75
  double wellMatchedCamsFactor;
  OptionsBundleAdjustment bundleAdjust;
  double pointsReprojErrorThresh;
  // Consider a ray from a camera center through a keypoint.
  // Consider next, the largest angle between all such rays
  // corresponding to one track/3d point.
  // This threshold is used for that angle.
  // Use degrees.
  double rayAngleThresh;
  double focalConstraintWeight;
  double radialConstraint;
  double radialConstraintWeight;
  // Used in case that the focals of initial cams were not found.
  // Formula for angle of view alpha:
  // defaultFocalDividedBySensorSize = 1/(2*sin(0.5*alpha))
  double defaultFocalDividedBySensorSize;

  void write(const string& filename) const;
};

void runSFM(const Options& opt,const string& outDir,
  const vector<bool>& isCalibrated,const ArrayXXd& homographyScores,
  const uset<int>& camsToIgnoreForInitialization,uset<int> *pexploredCams,
  Dataset *pdata);

int main(int argc,const char* argv[])
{
  Options opt;
  opt.bundleAdjust.solverOptions.num_threads = 8;
  if(argc >= 4)
    opt.sift.firstOctave = atoi(argv[3]);

  if(argc >= 5)
    opt.ccdDBFilename = argv[4];

  string dir(argv[1]);
  string imgsSubdir(argv[2]);
  _mkdir(dir.c_str());
  string outDir = joinPaths(dir,"models");
  _mkdir(outDir.c_str());

  opt.write(joinPaths(dir,"options.txt"));

  Dataset data(dir);

  data.addCameras<StandardCameraRadial>(imgsSubdir);
  // -> the principal point is always set to the
  // image center in StandardCamera

  // Initialize calibration for every camera
  vector<double> focals(data.cams().size());
  findFocalLengthInEXIF(opt.ccdDBFilename,data.cams(),&focals);
  for(int i = 0; i < data.numCams(); i++)
  {
    StandardCameraRadial *cam = static_cast<StandardCameraRadial *>(&data.cam(i));
    vector<double> radConstraints(2,opt.radialConstraint),
      radWeights(2,opt.radialConstraintWeight);
    cam->constrainRadial(&radConstraints[0],&radWeights[0]);
    if(focals[i] > 0.)
    {
      data.cam(i).setFocal(focals[i]);
      cam->constrainFocal(focals[i],opt.focalConstraintWeight);
    }
  }

  detectSiftGPU(opt.sift,&data.cams());
  data.readKeysColors();
  data.writeASCII("init.txt");
  
  cout << "Looking for similar camera pairs.\n";
  vector<set<int>> queries;
  bool verbose = true;
  findSimilarCameraPairs(data.cams(),opt.maxVocabularySize,
    opt.nSimilarCamerasToMatch,verbose,&queries);

  matchFeatFLANN(opt.matchingFLANN,data.cams(),queries,&data.pairs());
  //matchFeatFLANN(opt.matchingFLANN,data.cams(),&data.pairs());
  removePoorlyMatchedPairs(opt.minNumPairwiseMatches,&data.pairs());
  data.clearDescriptors();

  data.writeASCII("tentatively_matched.txt");
  //data.readASCII("tentatively_matched.txt",Camera::ReadNoDescriptors);

  //verifyMatchesGeometrically(opt.geometricVerification,data.cams(),&data.pairs());
  verifyMatchesEpipolar(opt.epipolarVerification,data.cams(),&data.pairs());
  
  data.writeASCII("matched.txt");
  //data.readASCII("matched.txt",Camera::ReadNoDescriptors);

  cout << "Computing homographies of verified pairs.\n";
  ArrayXXd homographyProportion;
  computeHomographyInliersProportion(opt.homography,data.cams(),data.pairs(),
    &homographyProportion);

  cout << "Searching for N view matches ... ";
  twoViewMatchesToNViewMatches(data.cams(),data.pairs(),
    &data.points().matchesToReconstruct());
  cout << "found " << data.points().matchesToReconstruct().size() << "\n";
  data.pairs().clear(); // No need for 2 view matches anymore.

  vector<bool> isCalibrated(data.numCams(),false);
  for(int i = 0; i < data.numCams(); i++)
  {
    StandardCamera *cam = static_cast<StandardCamera *>(&data.cam(i));
    isCalibrated[i] = cam->f() > 0.;
  }

  ArrayXXd homographyScores(homographyProportion.rows(),homographyProportion.cols());
  for(int c = 0; c < homographyProportion.cols(); c++)
  {
    for(int r = 0; r < homographyProportion.rows(); r++)
    {
      if(homographyProportion(r,c) == 0.)
        homographyScores(r,c) = DBL_MAX;
      else
        homographyScores(r,c) = 1. / homographyProportion(r,c);
    }
  }

  int modelId = 0;
  uset<int> exploredCams;
  while(data.cams().size() - exploredCams.size() >= 2)
  {
    size_t nExploredPrev = exploredCams.size();
    string appendix = "model" + std::to_string(modelId);
    string currOutDir = joinPaths(outDir,appendix);
    _mkdir(currOutDir.c_str());

    Dataset currData = data;
    uset<int> exploredCamsCurr;
    runSFM(opt,currOutDir,isCalibrated,homographyScores,exploredCams,
      &exploredCamsCurr,&currData);
    exploredCams.insert(exploredCamsCurr.begin(),exploredCamsCurr.end());

    if(nExploredPrev >= exploredCams.size())
      break;

    modelId++;

    if(currData.reconstructedCams().size() > 3)
    {
      writeSFMBundlerFormat(joinPaths(currData.dir(),"bundle_final_"
        + appendix + ".out"),currData);
      currData.writeASCII("final_" + appendix + ".txt");
    }
  }

  cout << "\n"
    << "Final report:\n"
    << "  " << modelId << " models reconstructed.\n"
    << "  " << data.cams().size() - exploredCams.size()
    << " out of " << data.cams().size() << " cameras left unexplored.\n";
}

void runSFM(const Options& opt,const string& outDir,
  const vector<bool>& isCalibrated,const ArrayXXd& homographyScores,
  const uset<int>& camsToIgnoreForInitialization,uset<int> *pexploredCams,
  Dataset *pdata)
{
  auto& exploredCams = *pexploredCams;
  auto& data = *pdata;

  double minPairScore = 1. / opt.minInitPairHomographyProportion;

  cout << "Choosing initial pair ... ";
  IntPair initPair = chooseInitialCameraPair(opt.minNumPairwiseMatches,
    minPairScore,isCalibrated,camsToIgnoreForInitialization,
    data.points().matchesToReconstruct(),homographyScores);
  cout << "[" << initPair.first << "," << initPair.second << "]\n";

  if(initPair.first < 0 || initPair.second < 0)
  {
    cerr << "runSFM: No good pairs for initialization\n";
    return;
  }

  if(!isCalibrated[initPair.first])
  {
    auto& cam = data.cam(initPair.first);
    double maxDim = std::max(cam.imgWidth(),cam.imgHeight());
    double focalPx = opt.defaultFocalDividedBySensorSize * maxDim;
    cam.setFocal(focalPx);
    cout << "Initial focal of the first camera assumed to be " 
      << focalPx << " pixels\n";
  }

  if(!isCalibrated[initPair.second])
  {
    auto& cam = data.cam(initPair.second);
    double maxDim = std::max(cam.imgWidth(),cam.imgHeight());
    double focalPx = opt.defaultFocalDividedBySensorSize * maxDim;
    cam.setFocal(focalPx);
    cout << "Initial focal of the second camera assumed to be "
      << focalPx << " pixels\n";
  }

  initReconstructionFromCalibratedCamPair(opt.initialPairRelativePose,
    opt.pointsReprojErrorThresh,initPair,&data);

  bundleAdjust(opt.bundleAdjust,&data.cams(),&data.points());

  exploredCams.insert(initPair.first);
  exploredCams.insert(initPair.second);
  while(data.cams().size() > exploredCams.size())
  {
    vector<vector<IntPair>> camToSceneMatches;
    findCamToSceneMatches(exploredCams,data.numCams(),data.points(),&camToSceneMatches);

    uset<int> wellMatchedCams;
    chooseWellMatchedCameras(opt.minNumCamToSceneMatches,opt.wellMatchedCamsFactor,
      camToSceneMatches,&wellMatchedCams);

    if(wellMatchedCams.empty())
      break;

    for(int camIdx : wellMatchedCams)
    {
      exploredCams.insert(camIdx);
      vector<int> inliers;
      cout << "Trying to resect camera " << camIdx << " using " <<
        camToSceneMatches[camIdx].size() << " matches ... ";
      //bool success = resectCamera5AndHalfPtRANSAC(opt.absolutePose_,camToSceneMatches[camIdx],
      //  data.points().ptCoord(),&data.cam(camIdx),&inliers);
      bool success = resectCamera6ptLSRANSAC(opt.absolutePose,camToSceneMatches[camIdx],
        data.points().ptCoord(),&data.cam(camIdx),&inliers);


      StandardCamera *cam = static_cast<StandardCamera *>(&data.cam(camIdx));
      int maxDim = std::max(cam->imgWidth(),cam->imgHeight());
      if(success && (cam->f() > 0.1*maxDim))
      {
        cout << "camera successfully added.\n";
        vector<int> ptIdxs;
        unzipPairsVectorSecond(camToSceneMatches[camIdx],&ptIdxs);
        data.markCamAsReconstructed(camIdx,ptIdxs,inliers);

        cout << "Bundle adjusting the new camera\n";
        bundleAdjustOneCam(opt.bundleAdjust,camIdx,&data.cam(camIdx),&data.points());
      } else
      {
        cout << "camera could not be added.\n";
      }
    }

    int minObservingCams = 2;
    vector<SplitNViewMatch> matchesToReconstructNow;
    extractCandidateNewPoints(minObservingCams,opt.rayAngleThresh,
      data.reconstructedCams(),data.cams(),
      &data.points().matchesToReconstruct(),&matchesToReconstructNow);

    cout << "Reconstructing " << matchesToReconstructNow.size() << " points\n";
    reconstructPoints(matchesToReconstructNow,&data.cams(),&data.points());
    int prevPts = data.points().numPtsAlive();
    removeHighReprojErrorPoints(opt.pointsReprojErrorThresh,&data.cams(),&data.points());
    cout << "Removing " << prevPts-data.points().numPtsAlive()
      << " points with high reprojection error\n";

    do
    {
      prevPts = data.points().numPtsAlive();
      cout << "Running bundle adjustment with: \n"
        << "  " << data.reconstructedCams().size() << " cams\n"
        << "  " << prevPts << " points\n"
        << "  " << data.countReconstructedObservations() << " observations\n";
      bundleAdjust(opt.bundleAdjust,&data.cams(),&data.points());
      removeHighReprojErrorPoints(opt.pointsReprojErrorThresh,&data.cams(),&data.points());
      cout << "Removing " << prevPts-data.points().numPtsAlive()
        << " points with high reprojection error\n";
    } while(prevPts > data.points().numPtsAlive());

    removeIllConditionedPoints(0.5*opt.rayAngleThresh,&data.cams(),&data.points());
    cout << "Removing " << prevPts-data.points().numPtsAlive() 
      << " ill conditioned points\n";

    writeSFMBundlerFormat(joinPaths(outDir,"bundle" +
      std::to_string(data.reconstructedCams().size()) + ".out"),data);
  }
}

void Options::write(const string& filename) const
{
  ofstream file(filename);
  if(!file.is_open())
    cerr << "Options::write: error: could not open " << filename << " for writing\n";
  file << "ccdDBFilename:\n " << ccdDBFilename << "\n";
  file << "sift:\n";
  sift.write(file);
  file << "maxVocabularySize:\n " << maxVocabularySize << "\n";
  file << "nSimilarCamerasToMatch:\n " << nSimilarCamerasToMatch << "\n";
  file << "matchingFLANN:\n";
  matchingFLANN.write(file);
  file << "minNumPairwiseMatches:\n " << minNumPairwiseMatches << "\n";
  file << "geometricVerification:\n";
  geometricVerification.write(file);
  file << "epipolarVerification:\n";
  epipolarVerification.write(file);
  file << "initialPairRelativePose:\n";
  initialPairRelativePose.write(file);
  file << "homography:\n";
  homography.write(file);
  file << "minInitPairHomographyProportion:\n " << minInitPairHomographyProportion << "\n";
  file << "absolutePose:\n";
  absolutePose.write(file);
  file << "minNumCamToSceneMatches:\n " << minNumCamToSceneMatches << "\n";
  file << "wellMatchedCamsFactor:\n " << wellMatchedCamsFactor << "\n";
  file << "bundleAdjust:\n";
  bundleAdjust.write(file);
  file << "pointsReprojErrorThresh:\n " << pointsReprojErrorThresh << "\n";
  file << "rayAngleThresh:\n " << rayAngleThresh << "\n";
  file << "focalConstraintWeight:\n " << focalConstraintWeight << "\n";
  file << "radialConstraint:\n " << radialConstraint << "\n";
  file << "radialConstraintWeight:\n " << radialConstraintWeight << "\n";
  file << "defaultFocalDividedBySensorSize:\n " << defaultFocalDividedBySensorSize << "\n";
  file.close();
}