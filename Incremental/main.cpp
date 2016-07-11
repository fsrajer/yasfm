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
#include <memory>
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
#include "YASFM/options_types.h"
#include "YASFM/utils_io.h"
#include "YASFM/image_similarity.h"
#include "Eigen/Dense"
#include "ceres/ceres.h"

using namespace yasfm;
using std::cin;
using std::cout;
using std::endl;
using std::ifstream;
using std::ofstream;
using std::string;
using std::unordered_set;
using std::make_shared;
using std::vector;
using std::string;

/// All options.
/*
Fields:
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
*/
class IncrementalOptions : public OptionsWrapper
{
public:
  IncrementalOptions()
  {
    opt.emplace("ccdDBFilename",
      make_unique<OptTypeWithVal<string>>("../resources/camera_ccd_widths.txt"));
    
    OptionsWrapperPtr sift = make_shared<OptionsSIFTGPU>();
    opt.emplace("sift",make_unique<OptTypeWithVal<OptionsWrapperPtr>>(sift));

    opt.emplace("maxVocabularySize",make_unique<OptTypeWithVal<int>>(15000));
    opt.emplace("nSimilarCamerasToMatch",make_unique<OptTypeWithVal<int>>(20));

    int minNumPairwiseMatches = 16;
    OptionsWrapperPtr matchingFLANN = make_shared<OptionsFLANN>();
    opt.emplace("matchingFLANN",
      make_unique<OptTypeWithVal<OptionsWrapperPtr>>(matchingFLANN));
    opt.emplace("minNumPairwiseMatches",
      make_unique<OptTypeWithVal<int>>(minNumPairwiseMatches));

    OptionsWrapperPtr geometricVerification = make_shared<OptionsGeometricVerification>();
    opt.emplace("geometricVerification",
      make_unique<OptTypeWithVal<OptionsWrapperPtr>>(geometricVerification));

    OptionsWrapperPtr epipolarVerification = 
      make_shared<OptionsRANSAC>(2048,sqrt(5.),minNumPairwiseMatches);
    opt.emplace("epipolarVerification",
      make_unique<OptTypeWithVal<OptionsWrapperPtr>>(epipolarVerification));

    OptionsWrapperPtr initialPairRelativePose = make_shared<OptionsRANSAC>(512,1.25,10);
    opt.emplace("initialPairRelativePose",
      make_unique<OptTypeWithVal<OptionsWrapperPtr>>(initialPairRelativePose));

    OptionsWrapperPtr homography = make_shared<OptionsRANSAC>(512,5.,10);
    opt.emplace("homography",
      make_unique<OptTypeWithVal<OptionsWrapperPtr>>(homography));
    opt.emplace("minInitPairHomographyProportion",make_unique<OptTypeWithVal<double>>(0.5));

    OptionsWrapperPtr absolutePose = make_shared<OptionsRANSAC>(4096,4.,16,0.999999);
    opt.emplace("absolutePose",
      make_unique<OptTypeWithVal<OptionsWrapperPtr>>(absolutePose));

    opt.emplace("minNumCamToSceneMatches",
      make_unique<OptTypeWithVal<int>>(minNumPairwiseMatches));
    opt.emplace("wellMatchedCamsFactor",make_unique<OptTypeWithVal<double>>(0.75));

    OptionsWrapperPtr bundleAdjust = make_shared<OptionsBundleAdjustment>();
    opt.emplace("bundleAdjust",
      make_unique<OptTypeWithVal<OptionsWrapperPtr>>(bundleAdjust));
      
    opt.emplace("pointsReprojErrorThresh",make_unique<OptTypeWithVal<double>>(8.));
    opt.emplace("rayAngleThresh",make_unique<OptTypeWithVal<double>>(2.));
    
    opt.emplace("focalConstraintWeight",make_unique<OptTypeWithVal<double>>(0.0001));
    opt.emplace("radialConstraint",make_unique<OptTypeWithVal<double>>(0.));
    opt.emplace("radialConstraintWeight",make_unique<OptTypeWithVal<double>>(100.));
    opt.emplace("defaultFocalDividedBySensorSize",
      make_unique<OptTypeWithVal<double>>(1.083)); // assume angle of view 55 degrees
  }

  template<class T>
  const T& getOpt(const string& name) const
  {
    return *static_cast<T *>(&(*get<OptionsWrapperPtr>(name)));
  }

  template<class T>
  T& getOpt(const string& name)
  {
    return *static_cast<T *>(&(*get<OptionsWrapperPtr>(name)));
  }

  void write(const string& filename) const;
};

void runSFM(const IncrementalOptions& opt,const string& outDir,
  const vector<bool>& isCalibrated,const ArrayXXd& homographyScores,
  const uset<int>& camsToIgnoreForInitialization,uset<int> *pexploredCams,
  Dataset *pdata);

void readPairsGV(const string& fn,Dataset *pdata)
{
  auto& data = *pdata;
  ifstream file(fn);
  if(!file.is_open())
  {
    return;
  }
  string fn1,fn2;
  int iCam = 0;
  while(!file.eof())
  {
    file >> fn1 >> fn2;
    string dummy;
    std::getline(file,dummy);
    std::getline(file,dummy);
    data.addCamera<StandardCameraRadial>(fn1);
    data.addCamera<StandardCameraRadial>(fn2);
    data.queries().emplace_back();
    data.queries().emplace_back();
    data.queries().back().insert(iCam);
    iCam += 2;
  }
  file.close();
}

int main(int argc,const char* argv[])
{
#define PRINT_ROC_DATA
#ifdef PRINT_ROC_DATA
  string dir("c:/Users/Uzivatel/Dropbox/cvpr17/0.8/Hs");
  Dataset data(dir);
  string dataName = "matched_2";
  data.readASCII(dataName + ".txt");

  IncrementalOptions opt;
  //string name = "thesis";
  //string name = "decomp-avgall";
  //string name = "decomp-opt-avgall";
  //string name = "decomp-avgboth";
  string name = "decomp-opt-avgboth";
  string outFn = "c:/Users/Uzivatel/Dropbox/cvpr17/0.8/Hs/" +
    dataName + "-labels-" + name + ".txt";
  FILE *file = fopen(outFn.c_str(),"w");
  
  for(const auto& entry : data.pairs())
  {
    int iCam = entry.first.first;
    int jCam = entry.first.second;
    const auto& pair = entry.second;
    const auto& groups = pair.groups;

    vector<vector<int>> groupsMatches(groups.size());
    int iMatch = 0;
    for(size_t ig = 0; ig < groups.size(); ig++)
      for(int i = 0; i < groups[ig].size; i++)
        groupsMatches[ig].push_back(iMatch++);

    for(int ig = 0; ig < int(groups.size()); ig++)
    {
      for(int jg = ig+1; jg < int(groups.size()); jg++)
      {
        /*const auto& H1 = groups[ig].T;
        const auto& H2 = groups[jg].T;
        double eigScore = -computePairwiseEigScore(H1,H2);
        double egScore = computePairwiseEGScore(
          opt.getOpt<OptionsGeometricVerification>("geometricVerification"),
          data.cam(iCam).keys(),data.cam(jCam).keys(),
          pair.matches,groupsMatches[ig],groupsMatches[jg]);
        
        double score = -egScore/eigScore;*/

        vector<IntPair> matches1,matches2;
        Matrix3d H1 = groups[ig].T,
          H2 = groups[jg].T;
        for(int idx : groupsMatches[ig])
          matches1.push_back(pair.matches[idx]);
        for(int idx : groupsMatches[jg])
          matches2.push_back(pair.matches[idx]);
        double score = -computeHomologyDataFitScore(data.cam(iCam),data.cam(jCam),
          matches1,matches2,OptionsRANSAC(2048,3.,10),&H1,&H2);

        fprintf(file,"%i %i %i %i %.100e\n",iCam,jCam,ig,jg,score);
      }
    }
  }
  fclose(file);
  
#else
  // ======================================
  // See the description of this variable.
  // Camera::maxDescrInMemoryTotal_ = 5000000;
  // ======================================

  IncrementalOptions opt;
  if(argc >= 4)
    opt.getOpt<OptionsSIFTGPU>("sift").get<int>("firstOctave") = atoi(argv[3]);

  if(argc >= 5)
    opt.get<string>("ccdDBFilename") = argv[4];

  string dir(argv[1]);
  string imgsSubdir(argv[2]);
  makeDirRecursive(dir);
  string outDir = joinPaths(dir,"models");
  makeDirRecursive(outDir);

  //opt.write(joinPaths(dir,"options.txt"));

  Dataset data(dir);
  /*readPairsGV(joinPaths(dir,"pairs.txt"),&data);
  data.addCameras<StandardCameraRadial>(imgsSubdir);
  // -> the principal point is always set to the
  // image center in StandardCamera

  // Initialize calibration for every camera
  vector<double> focals(data.cams().size());
  findFocalLengthInEXIF(opt.get<string>("ccdDBFilename"),data.cams(),&focals);
  for(int i = 0; i < data.numCams(); i++)
  {
    StandardCameraRadial *cam = static_cast<StandardCameraRadial *>(&data.cam(i));
    vector<double> radConstraints(2,opt.get<double>("radialConstraint")),
      radWeights(2,opt.get<double>("radialConstraintWeight"));
    cam->constrainRadial(&radConstraints[0],&radWeights[0]);
    if(focals[i] > 0.)
    {
      data.cam(i).setFocal(focals[i]);
      cam->constrainFocal(focals[i],opt.get<double>("focalConstraintWeight"));
    }
  }

  detectSiftGPU(opt.getOpt<OptionsSIFTGPU>("sift"),&data.cams());
  data.readKeysColors();
  data.writeASCII("init.txt");
  //data.readASCII("init.txt");
  
  cout << "Looking for similar camera pairs.\n";
  bool verbose = true;
  findSimilarCameraPairs(data.cams(),opt.get<int>("maxVocabularySize"),
    opt.get<int>("nSimilarCamerasToMatch"),verbose,&data.queries());

  data.writeASCII("similar.txt");
  //data.readASCII("similar.txt");

  matchFeatFLANN(opt.getOpt<OptionsFLANN>("matchingFLANN"),data.cams(),
    data.queries(),&data.pairs());
  //matchFeatFLANN(opt.getOpt<OptionsFLANN>("matchingFLANN"),data.cams(),&data.pairs());
  removePoorlyMatchedPairs(opt.get<int>("minNumPairwiseMatches"),&data.pairs());
  data.clearDescriptors();

  data.writeASCII("tentatively_matched_all.txt");*/
  data.readASCII("0.8/tentatively_matched_all.txt");
  //data.readASCII("0.8/tentatively_matched_all_with_gt.txt");

  for(int i = 0; i < data.numCams(); i++)
  {
    auto& cam = data.cam(i);
    double maxDim = std::max(cam.imgWidth(),cam.imgHeight());
    double focalPx = opt.get<double>("defaultFocalDividedBySensorSize") * maxDim;
    cam.setFocal(focalPx);
  }
  bool useCalibratedEpipolarVerif = true;

//#define DDEBUG

  float& ratioThresh = opt.getOpt<OptionsFLANN>("matchingFLANN").get<float>("ratioThresh");
#ifdef DDEBUG
  ratioThresh = 0.7f;
  int nSteps = 1;
#else
  ratioThresh = 0.6f;
  int nSteps = 3;
#endif
  float stepSize = 0.1f;
  string name = "0.8/prosac-F";
  _mkdir(joinPaths(dir,name).c_str());
  pair_umap<CameraPair> allPairs = data.pairs();
  
#ifdef DDEBUG
  /*IntPair curr(76,77);
  allPairs.clear();
  allPairs[curr] = data.pairs()[curr];
  data.pairs() = allPairs;*/
#endif
  
  for(int i = 0; i < nSteps; i++,ratioThresh += stepSize)
  {
    cout << "Processing " << i << "-th out of " << nSteps << "\n";
    for(auto& entry : data.pairs())
    {
      auto& pair = entry.second;
      vector<bool> keep(pair.matches.size());
      for(size_t i = 0; i < pair.dists.size(); i++)
      {
        keep[i] = pair.dists[i] < ratioThresh;
      }
      filterVector(keep,&pair.matches);
      filterVector(keep,&pair.dists);

      keep.clear();
      size_t numKeys2 = data.cam(entry.first.second).keys().size();
      findUniqueMatches(pair.matches,numKeys2,&keep);
      filterVector(keep,&pair.matches);
      filterVector(keep,&pair.dists);
    }
    verifyMatchesGeometrically(opt.getOpt<OptionsGeometricVerification>("geometricVerification"),
      data.cams(),&data.pairs());
    //verifyMatchesEpipolar(opt.getOpt<OptionsRANSAC>("epipolarVerification"),
    //  useCalibratedEpipolarVerif,data.cams(),&data.pairs());

#ifndef DDEBUG
    data.writeASCII(name + "/matched_" + std::to_string(i+1) + ".txt");
    opt.write(joinPaths(dir,name + "/options_" + std::to_string(i+1) + ".txt"));
#endif
    data.pairs() = allPairs;
  }

  /*verifyMatchesGeometrically(opt.getOpt<OptionsGeometricVerification>("geometricVerification"),
    data.cams(),&data.pairs());
  //verifyMatchesEpipolar(opt.getOpt<OptionsRANSAC>("epipolarVerification"),
  //  data.cams(),&data.pairs());
  
  data.writeASCII("matched.txt");*/
  //data.readASCII("matched.txt");

  /*cout << "Computing homographies of verified pairs.\n";
  ArrayXXd homographyProportion;
  computeHomographyInliersProportion(opt.getOpt<OptionsRANSAC>("homography"),
    data.cams(),data.pairs(),
    &homographyProportion);

  cout << "Searching for N view matches ... ";
  twoViewMatchesToNViewMatches(data.cams(),data.pairs(),
    &data.nViewMatches());
  cout << "found " << data.nViewMatches().size() << "\n";
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
    makeDirRecursive(currOutDir);

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
      currData.nViewMatches().clear(); // We don't need unused n-view matches
      writeSFMBundlerFormat(joinPaths(currData.dir(),"bundle_final_"
        + appendix + ".out"),currData);
      currData.writeASCII("final_" + appendix + ".txt");
    }
  }

  cout << "\n"
    << "Final report:\n"
    << "  " << modelId << " models reconstructed.\n"
    << "  " << data.cams().size() - exploredCams.size()
    << " out of " << data.cams().size() << " cameras left unexplored.\n";*/
#endif
}

void runSFM(const IncrementalOptions& opt,const string& outDir,
  const vector<bool>& isCalibrated,const ArrayXXd& homographyScores,
  const uset<int>& camsToIgnoreForInitialization,uset<int> *pexploredCams,
  Dataset *pdata)
{
  auto& exploredCams = *pexploredCams;
  auto& data = *pdata;
  const auto& baOpt = opt.getOpt<OptionsBundleAdjustment>("bundleAdjust");

  double minPairScore = 1. / opt.get<double>("minInitPairHomographyProportion");

  cout << "Choosing initial pair ... ";
  IntPair initPair = chooseInitialCameraPair(opt.get<int>("minNumPairwiseMatches"),
    minPairScore,isCalibrated,camsToIgnoreForInitialization,
    data.nViewMatches(),homographyScores);
  cout << "[" << initPair.first << "," << initPair.second << "]\n";

  if(initPair.first < 0 || initPair.second < 0)
  {
    cout << __func__ << ": No good pairs for initialization\n";
    return;
  }

  if(!isCalibrated[initPair.first])
  {
    auto& cam = data.cam(initPair.first);
    double maxDim = std::max(cam.imgWidth(),cam.imgHeight());
    double focalPx = opt.get<double>("defaultFocalDividedBySensorSize") * maxDim;
    cam.setFocal(focalPx);
    cout << "Initial focal of the first camera assumed to be " 
      << focalPx << " pixels\n";
  }

  if(!isCalibrated[initPair.second])
  {
    auto& cam = data.cam(initPair.second);
    double maxDim = std::max(cam.imgWidth(),cam.imgHeight());
    double focalPx = opt.get<double>("defaultFocalDividedBySensorSize") * maxDim;
    cam.setFocal(focalPx);
    cout << "Initial focal of the second camera assumed to be "
      << focalPx << " pixels\n";
  }

  initReconstructionFromCalibratedCamPair(
    opt.getOpt<OptionsRANSAC>("initialPairRelativePose"),
    opt.get<double>("pointsReprojErrorThresh"),initPair,&data);

  bundleAdjust(baOpt,&data.cams(),&data.pts());

  exploredCams.insert(initPair.first);
  exploredCams.insert(initPair.second);
  while(data.cams().size() > exploredCams.size())
  {
    vector<vector<IntPair>> camToSceneMatches;
    findCamToSceneMatches(exploredCams,data.numCams(),data.pts(),&camToSceneMatches);

    uset<int> wellMatchedCams;
    chooseWellMatchedCameras(opt.get<int>("minNumCamToSceneMatches"),
      opt.get<double>("wellMatchedCamsFactor"),camToSceneMatches,&wellMatchedCams);

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
      bool success = resectCamera6ptLSRANSAC(
        opt.getOpt<OptionsRANSAC>("absolutePose"),camToSceneMatches[camIdx],
        data.pts(),&data.cam(camIdx),&inliers);


      StandardCamera *cam = static_cast<StandardCamera *>(&data.cam(camIdx));
      int maxDim = std::max(cam->imgWidth(),cam->imgHeight());
      if(success && (cam->f() > 0.1*maxDim))
      {
        cout << "camera successfully added.\n";
        vector<int> ptIdxs;
        unzipPairsVectorSecond(camToSceneMatches[camIdx],&ptIdxs);
        data.markCamAsReconstructed(camIdx,ptIdxs,inliers);

        cout << "Bundle adjusting the new camera\n";
        bundleAdjustOneCam(baOpt,camIdx,&data.cam(camIdx),&data.pts());
      } else
      {
        cout << "camera could not be added.\n";
      }
    }

    int minObservingCams = 2;
    vector<SplitNViewMatch> matchesToReconstructNow;
    extractCandidateNewPoints(minObservingCams,opt.get<double>("rayAngleThresh"),
      data.reconstructedCams(),data.cams(),
      &data.nViewMatches(),&matchesToReconstructNow);

    cout << "Reconstructing " << matchesToReconstructNow.size() << " points\n";
    reconstructPoints(matchesToReconstructNow,&data.cams(),&data.pts());
    int nPtsRemoved = removeHighReprojErrorPoints(
      opt.get<double>("pointsReprojErrorThresh"),&data.cams(),&data.pts());
    cout << "Removing " << nPtsRemoved << " points with high reprojection error\n";

    do
    {
      cout << "Running bundle adjustment with: \n"
        << "  " << data.reconstructedCams().size() << " cams\n"
        << "  " << data.countPtsAlive() << " points\n"
        << "  " << data.countReconstructedObservations() << " observations\n";
      bundleAdjust(baOpt,&data.cams(),&data.pts());
      nPtsRemoved = removeHighReprojErrorPoints(
        opt.get<double>("pointsReprojErrorThresh"),&data.cams(),&data.pts());
      cout << "Removing " << nPtsRemoved << " points with high reprojection error\n";
    } while(nPtsRemoved > 0);

    nPtsRemoved = removeIllConditionedPoints(0.5*opt.get<double>("rayAngleThresh"),
      &data.cams(),&data.pts());
    cout << "Removing " << nPtsRemoved << " ill conditioned points\n";

    writeSFMBundlerFormat(joinPaths(outDir,"bundle" +
      std::to_string(data.reconstructedCams().size()) + ".out"),data);
  }
}

void IncrementalOptions::write(const string& filename) const
{
  ofstream file(filename);
  if(!file.is_open())
  {
    YASFM_PRINT_ERROR_FILE_OPEN(filename);
    return;
  }
  OptionsWrapper::write(file);
  file.close();
}