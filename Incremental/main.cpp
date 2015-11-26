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

};

int main(int argc, const char* argv[])
{
  Options opt;
  opt.bundleAdjust.solverOptions.num_threads = 8;
  if(argc >= 4)
    opt.sift.firstOctave = atoi(argv[3]);  

  string dir(argv[1]);
  string imgsSubdir(argv[2]);

  Dataset data(dir);
  /*data.addCameras<StandardCameraRadial>(imgsSubdir);
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

  twoViewMatchesToNViewMatches(data.cams(),data.pairs(),
    &data.points().matchesToReconstruct());
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
  IntPair initPair = chooseInitialCameraPair(opt.minNumPairwiseMatches,minPairScore,
    isCalibrated,data.points().matchesToReconstruct(),homographyScores);
    

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
      cout << "trying to resect camera " << camIdx << " using " << 
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

    reconstructPoints(data.cams(),matchesToReconstructNow,&data.points());
    removeHighReprojErrorPoints(opt.pointsReprojErrorThresh,data.cams(),&data.points());

    int prevPts;
    do
    {
      prevPts = data.points().numPts();
      bundleAdjust(opt.bundleAdjust,&data.cams(),&data.points());
      removeHighReprojErrorPoints(opt.pointsReprojErrorThresh,data.cams(),&data.points());
    } while(prevPts > data.points().numPts());

    removeIllConditionedPoints(0.5*opt.rayAngleThresh,data.cams(),&data.points());

    writeSFMBundlerFormat(joinPaths(data.dir(),"yasfm/bundle" + 
      std::to_string(exploredCams.size()) + ".out"),data);
  }

  writeSFMBundlerFormat(joinPaths(data.dir(),"bundle_final.out"),data);
  data.writeASCII("out.txt",Camera::WriteNoFeatures);
}

/*



#include <fstream>
using std::ifstream;
using std::ofstream;

void readFeaturesASCII(const string& fn,Camera *cam)
{
ifstream in(fn);
if(!in.is_open())
{
cerr << "readFeaturesASCII: unable to open: " << fn << " for reading\n";
return;
}
int nFeats,descrDim;
in >> nFeats >> descrDim;
cam->reserveFeatures(nFeats,descrDim);
float *descr = new float[descrDim];
string s;
for(int i = 0; i < nFeats; i++)
{
double x,y,dummy;
in >> y >> x >> dummy >> dummy;
for(int i = 0; i < 8; i++)
{
std::getline(in,s);
}
cam->addFeature(x,y,descr);
}
delete[] descr;
in.close();
cam->clearDescriptors();
}

void readTracksASCII(const string& filename,vector<NViewMatch> *ptracks)
{
auto& tracks = *ptracks;
ifstream in(filename);
if(!in.is_open())
{
cerr << "readTracksASCII: unable to open: " << filename << " for reading" << "\n";
return;
}
int nTracks;
in >> nTracks;
tracks.resize(nTracks);
for(int i = 0; i < nTracks; i++)
{
int trackSize;
in >> trackSize;
tracks[i].reserve(trackSize);
for(int j = 0; j < trackSize; j++)
{
int camIdx,keyIdx;
in >> camIdx >> keyIdx;
tracks[i][camIdx] = keyIdx;
}
}
in.close();
}

void readMatchesASCII(const string& filename,pair_umap<CameraPair> *ppairs)
{
auto& pairs = *ppairs;
ifstream in(filename);
if(!in.is_open())
{
cerr << "readMatchesASCII: unable to open: " << filename << " for reading" << "\n";
return;
}
int dummy;
in >> dummy;
string dummy2;
while(!in.eof())
{
int img1,img2,nMatches;
in >> img1 >> img2 >> nMatches;
auto& pair = pairs[IntPair(img1,img2)];
pair.matches.reserve(nMatches);
pair.dists.reserve(nMatches);
for(int iMatch = 0; iMatch < nMatches; iMatch++)
{
int i,j;
double dist;
in >> i >> j >> dist;
pair.matches.emplace_back(i,j);
pair.dists.push_back(dist);
}
//std::getline(in,dummy2);
std::getline(in,dummy2);
}
in.close();
}

void readCMPSFMPrestate(const Options& opt,Dataset *pdata)
{
auto& data = *pdata;
string dir = "C:/Users/Filip/Workspace/cmp/Data/daliborka";
//string listTracks = joinPaths(dir,"tracks-.txt");
//string listMatches = joinPaths(dir,"matches.eg.txt");
string listMatches = joinPaths(dir,"matches.init.txt");
string listImgs = joinPaths(dir,"list_imgs.txt");


double focalConstraintWeight = 0.0001;
double radialConstraint = 0.;
double radialConstraintWeight = 100.;

data = Dataset(dir);
data.addCameras<StandardCameraRadial>("imgs");
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

for(int i = 0; i < data.numCams(); i++)
{
string fn = data.cam(i).imgFilename();
fn[fn.size()-3] = 'k';
fn[fn.size()-2] = 'e';
fn[fn.size()-1] = 'y';
readFeaturesASCII(fn,&data.cam(i));
}
readMatchesASCII(listMatches,&data.pairs());
//readTracksASCII(listTracks,&data.points().matchesToReconstruct());
}

void writeMatchesASCII(const string& filename,const pair_umap<CameraPair>& pairs)
{
ofstream out(filename);
if(!out.is_open())
{
cerr << "writeMatchesASCII: unable to open: " << filename << " for writing" << "\n";
return;
}
out.flags(std::ios::fixed);
out << pairs.size() << "\n";
for(const auto& entry : pairs)
{
IntPair idx = entry.first;
const auto& pair = entry.second;
out << idx.first << ' ' << idx.second << "\n";
out << pair.matches.size() << "\n";
for(int i = 0; i < pair.matches.size(); i++)
{
out << pair.matches[i].first << ' ' <<
pair.matches[i].second << ' ' <<
std::setprecision(3) << pair.dists[i] << "\n";
}
out << Eigen::Matrix3d::Zero() << "\n";
}
out.close();
}
*/