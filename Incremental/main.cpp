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
  /*data.addCameras<StandardCameraRadial>(imgsSubdir);
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

  verifyMatchesGeometrically(opt,data.cams(),&data.pairs());
  removePoorlyMatchedPairs(opt.minNumMatches_,&data.pairs());

  data.writeASCII("matched.txt",Camera::WriteMode::ConvertNormalizedSIFTToUint);
  data.clearDescriptors();
  */
  data.readASCII("matched.txt",Camera::ReadMode::SkipDescriptors);

  ArrayXXd homographyProportion;
  computeHomographyInliersProportion(opt.homography_,data.cams(),data.pairs(),
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

  double minPairScore = 1. / opt.minInitPairHomographyProportion_;
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
  IntPair initPair = chooseInitialCameraPair(opt.minNumMatches_,minPairScore,
    isCalibrated,data.points().matchesToReconstruct(),homographyScores);
    

  if(initPair.first < 0 || initPair.second < 0)
    return EXIT_FAILURE;
    
  if(isCalibrated[initPair.first] && isCalibrated[initPair.second])
  {
    initReconstructionFromCalibratedCamPair(opt,initPair,&data);
  } else
  {
    initReconstructionFromCamPair(opt,initPair,&data);
  }

  bundleAdjust(opt.ba_,&data.cams(),&data.points());

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
      exploredCams.insert(camIdx);
      vector<int> inliers;
      cout << "trying to resect camera " << camIdx << " using " << 
        camToSceneMatches[camIdx].size() << " matches ... ";
      //bool success = resectCamera5AndHalfPtRANSAC(opt.absolutePose_,camToSceneMatches[camIdx],
      //  data.points().ptCoord(),&data.cam(camIdx),&inliers);
      bool success = resectCamera6ptLSRANSAC(opt.absolutePose_,camToSceneMatches[camIdx],
        data.points().ptCoord(),&data.cam(camIdx),&inliers);


      StandardCamera *cam = static_cast<StandardCamera *>(&data.cam(camIdx));
      int maxDim = std::max(cam->imgWidth(),cam->imgHeight());
      if(success && (cam->f() > 0.1*maxDim))
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

    int prevPts;
    do
    {
      prevPts = data.points().numPts();
      bundleAdjust(opt.ba_,&data.cams(),&data.points());
      removeHighReprojErrorPoints(opt.pointsReprojErrorThresh_,data.cams(),&data.points());
    } while(prevPts > data.points().numPts());

    removeIllConditionedPoints(0.5*opt.rayAngleThresh_,data.cams(),&data.points());

    writeSFMBundlerFormat(joinPaths(data.dir(),"yasfm/bundle" + 
      std::to_string(exploredCams.size()) + ".out"),data);
  }

  writeSFMBundlerFormat(joinPaths(data.dir(),"bundle_final.out"),data);
  data.writeASCII("out.txt",Camera::NoFeatures);
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