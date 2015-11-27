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

struct Options
{
  Options()
    :
    ccdDBFilename("../resources/camera_ccd_widths.txt"),
    sift(),
    matchingFLANN(),
    minNumPairwiseMatches(16),
    geometricVerification(2048,sqrt(5.),minNumPairwiseMatches),
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
    radialConstraintWeight(100.)
  {
  }

  string ccdDBFilename;
  OptionsSIFTGPU sift;
  OptionsFLANN matchingFLANN;
  // Min number of matches defining a poorly matched pair. Default: 16.
  int minNumPairwiseMatches;
  // The error is symmetric distance. Units are pixels.
  OptionsRANSAC geometricVerification;
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

  void write(const string& filename) const;
};

int main(int argc, const char* argv[])
{
  Options opt;
  opt.bundleAdjust.solverOptions.num_threads = 8;
  if(argc >= 4)
    opt.sift.firstOctave = atoi(argv[3]);  

  string dir(argv[1]);
  string imgsSubdir(argv[2]);

  opt.write(joinPaths(dir,"options.txt"));

  Dataset data(dir);
  /*_mkdir(joinPaths(dir,"yasfm").c_str());
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
  data.writeASCII("init.txt",Camera::WriteAll | Camera::WriteConvertNormalizedSIFTToUint);

  matchFeatFLANN(opt.matchingFLANN,data.cams(),&data.pairs());
  removePoorlyMatchedPairs(opt.minNumPairwiseMatches,&data.pairs());

  verifyMatchesGeometrically(opt.geometricVerification,data.cams(),&data.pairs());
  removePoorlyMatchedPairs(opt.minNumPairwiseMatches,&data.pairs());
  
  data.writeASCII("matched.txt",Camera::WriteNoFeatures);
  data.clearDescriptors();
  */

  data.readASCII("matched.txt",Camera::ReadNoDescriptors);

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

  double minPairScore = 1. / opt.minInitPairHomographyProportion;
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

  cout << "Choosing initial pair ... ";
  IntPair initPair = chooseInitialCameraPair(opt.minNumPairwiseMatches,minPairScore,
    isCalibrated,data.points().matchesToReconstruct(),homographyScores);
  cout << "[" << initPair.first << "," << initPair.second << "]\n";
    
  if(initPair.first < 0 || initPair.second < 0)
    return EXIT_FAILURE;
    
  if(isCalibrated[initPair.first] && isCalibrated[initPair.second])
  {
    initReconstructionFromCalibratedCamPair(opt.initialPairRelativePose,
      opt.pointsReprojErrorThresh,initPair,&data);
  } else
  {
    initReconstructionFromCamPair(opt.initialPairRelativePose,
      opt.pointsReprojErrorThresh,initPair,&data);
  }

  bundleAdjust(opt.bundleAdjust,&data.cams(),&data.points());

  uset<int> exploredCams;
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
    reconstructPoints(data.cams(),matchesToReconstructNow,&data.points());
    int prevPts = data.points().numPts();
    removeHighReprojErrorPoints(opt.pointsReprojErrorThresh,data.cams(),&data.points());
    cout << "Removing " << prevPts-data.points().numPts()
      << " points with high reprojection error\n";

    do
    {
      prevPts = data.points().numPts();
      cout << "Running bundle adjustment with: \n"
        << "  " << data.reconstructedCams().size() << " cams\n"
        << "  " << data.points().numPts() << " points\n"
        << "  " << data.countReconstructedObservations() << " observations\n";
      bundleAdjust(opt.bundleAdjust,&data.cams(),&data.points());
      removeHighReprojErrorPoints(opt.pointsReprojErrorThresh,data.cams(),&data.points());
      cout << "Removing " << prevPts-data.points().numPts() 
        << " points with high reprojection error\n";
    } while(prevPts > data.points().numPts());

    removeIllConditionedPoints(0.5*opt.rayAngleThresh,data.cams(),&data.points());
    cout << "Removing " << prevPts-data.points().numPts() << " ill conditioned points\n";

    writeSFMBundlerFormat(joinPaths(data.dir(),"yasfm/bundle" + 
      std::to_string(exploredCams.size()) + ".out"),data);
  }

  writeSFMBundlerFormat(joinPaths(data.dir(),"bundle_final.out"),data);
  data.writeASCII("out.txt",Camera::WriteNoFeatures);
}

void Options::write(const string& filename) const
{
  ofstream file(filename);
  if(!file.is_open())
    cerr << "Options::write: error: could not open " << filename << " for writing\n";
  file << "ccdDBFilename:\n " << ccdDBFilename << "\n";
  file << "sift:\n";
  sift.write(file);
  file << "matchingFLANN:\n";
  matchingFLANN.write(file);
  file << "minNumPairwiseMatches:\n " << minNumPairwiseMatches << "\n";
  file << "geometricVerification:\n";
  geometricVerification.write(file);
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
  file.close();
}