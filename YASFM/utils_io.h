/*
* Filip Srajer
* filip.srajer (at) fel.cvut.cz
* Center for Machine Perception
* Czech Technical University in Prague
*
* This software is under construction.
* 03/2015
*/

#pragma once

#include <fstream>
#include <unordered_set>
#include <string>
#include <vector>

#include "defines.h"
#include "options.h"
#include "sfm_data.h"

using namespace yasfm;
using std::ifstream;
using std::string;
using std::unordered_set;
using std::vector;

namespace yasfm
{

// Forward declarations
class Camera;
class Points;
class Dataset;

// List all image files with supported extensions in a directory.
YASFM_API void listImgFilenames(const string& dir,vector<string> *filenames);

// Runs overloaded function with filenameExtensions being an empty vector.
YASFM_API void listFilenames(const string& dir,vector<string> *filenames);

// List all files with given permitted extensions.
// An empty extensions vector means that all files will be listed.
// On fail, no filenames are returned.
YASFM_API void listFilenames(const string& dir,
  const vector<string>& filenameExtensions,vector<string> *filenames);

YASFM_API void getImgDims(const string& filename,int *width,int *height);

/*
// Creates a file, where every line contains
// one image filename. This is particularly useful
// when you want to fix the order of images.
YASFM_API void writeImgFnsList(const string& listFilename,const ptr_vector<ICamera>& cams);
// This function is just a "header" function taking IDataset.
// It only calls overloaded function taking full spectrum of parameters.
YASFM_API void writeImgFnsList(const string& listFilename,const IDataset& dts);

// Creates a file, where every line contains
// one feats filename. 
YASFM_API void writeFeatsFnsList(const string& listFilename,const ptr_vector<ICamera>& cams);
// This function is just a "header" function taking IDataset.
// It only calls overloaded function taking full spectrum of parameters.
YASFM_API void writeFeatsFnsList(const string& listFilename,const IDataset& dts);
*/

/*
// Reads colors for features and optinally also returns image dimensions
// IMPORTANT: This function requires features to return coordinates in 
// the original image coordinate system.
YASFM_API void readFeatureColors(const string& dir,ICamera& cam,
  int *width = nullptr,int *height = nullptr);
// This function is just a "header" function taking IDataset.
// It only calls overloaded function taking full spectrum of parameters.
// IMPORTANT: This function requires features to return coordinates in 
// the original image coordinate system.
YASFM_API void readFeatureColors(IDataset& dts);
*/

// focals are 0 when they were not found
YASFM_API void findFocalLengthInEXIF(const string& ccdDBFilename,const ptr_vector<Camera>& cams,
  vector<double> *focals);
// returns 0 when focal not found
YASFM_API double findFocalLengthInEXIF(const string& ccdDBFilename,const Camera& cam);
// returns 0 when focal not found
YASFM_API double findFocalLengthInEXIF(const string& ccdDBFilename,const string& imgFilename,
  int maxImgDim);

/*
// Saves features for every image
// Saves features in format:
// nFeatures descriptorDimension
// feat0
// feat1
// ...
// where feati is:
// y x
// d0 d2 d3 ... d19
// d20 d21 ... d39
// ...
YASFM_API void writeFeaturesASCII(const IDataset& dts,bool convertDescrToUint);
YASFM_API void writeFeaturesASCII(const ptr_vector<ICamera>& cams,const string& dir,bool convertDescrToUint);
YASFM_API void writeFeaturesASCII(const string& dir,const ICamera& cam,bool convertDescrToUint);
YASFM_API void readFeaturesASCII(IDataset& dts);
YASFM_API void readFeaturesASCII(ptr_vector<ICamera>& cams,const string& dir);
YASFM_API void readFeaturesASCII(const string& dir,ICamera& cam);

// Saves keypoints in format:
// nKeypoints
// key0
// key1
// ...
// where keyi is:
// y x
YASFM_API void writeKeysASCII(const IDataset& dts);
YASFM_API void writeKeysASCII(const ptr_vector<ICamera>& cams,const string& dir);
YASFM_API void writeKeysASCII(const string& dir,const ICamera& cam);
YASFM_API void readKeysASCII(IDataset& dts);
YASFM_API void readKeysASCII(ptr_vector<ICamera>& cams,const string& dir);
YASFM_API void readKeysASCII(const string& dir,ICamera& cam);

YASFM_API void writeFeaturesBinary(const IDataset& dts);
YASFM_API void writeFeaturesBinary(const ptr_vector<ICamera>& cams,const string& dir);
YASFM_API void writeFeaturesBinary(const string& dir,const ICamera& cam);
YASFM_API void readFeaturesBinary(IDataset& dts);
YASFM_API void readFeaturesBinary(ptr_vector<ICamera>& cams,const string& dir);
YASFM_API void readFeaturesBinary(const string& dir,ICamera& cam);

// Saves matches of all pairs in format:
// nPairs
// pair0
// pair1
// ...
// where pairi:
// imgIdxFrom imgIdxTo
// nMatches
// keyIdxFrom0 keyIdxTo0 minScore0
// keyIdxFrom1 keyIdxTo1 minScore1
// ...
YASFM_API void writeMatchesASCII(const string& filename,const IDataset& dts);
YASFM_API void writeMatchesASCII(const string& filename,const pair_ptr_unordered_map<ICameraPair>& pairs);
template<typename CameraPairDerived>
void readMatchesASCII(const string& filename,IDataset& dts);
template<typename CameraPairDerived>
void readMatchesASCII(const string& filename,pair_ptr_unordered_map<ICameraPair>& pairs);

// Saves transformations between pairs in format:
// nPairs
// pair0
// pair1
// ...
// where pairi:
// imgIdxFrom imgIdxTo
// F
YASFM_API void writeTransformsASCII(const string& filename,const IDataset& dts);
YASFM_API void writeTransformsASCII(const string& filename,const pair_ptr_unordered_map<ICameraPair>& pairs);
template<typename CameraPairDerived>
void readTransformsASCII(const string& filename,IDataset& dts);
template<typename CameraPairDerived>
void readTransformsASCII(const string& filename,pair_ptr_unordered_map<ICameraPair>& pairs);

// Writes tracks in format:
// nTracks
// track0
// track1
// ...
// where tracki:
// numViews cam0 key0 cam1 key1 ...
YASFM_API void writeTracksASCII(const string& filename,const vector<NViewMatch>& tracks);
YASFM_API void readTracksASCII(const string& filename,IDataset& dts);
YASFM_API void readTracksASCII(const string& filename,vector<NViewMatch>& tracks,
  vector<int> *tracks2points);
*/

// Writes data into Bundler's Bundle format
// see: http://www.cs.cornell.edu/~snavely/bundler/bundler-v0.4-manual.html
// The format requires writing features' coordinates 
// in a coordinate system where image center is the origin
// and the y axis goes from bottom to the top. In addition
// -z should be the forward axis.
// IMPORTANT: This function assumes +z to be the forward axis.
// Also, to get radial parameters, this function tries to cast into 
// StandardCameraRadial.
YASFM_API void writeSFMBundlerFormat(const string& filename,const uset<int>& reconstructedCams,
  const ptr_vector<Camera>& cams,const Points& points);
// This function is just a "header" function taking Dataset.
// It only calls overloaded function taking full spectrum of parameters.
YASFM_API void writeSFMBundlerFormat(const string& filename,const Dataset& data);

/*
YASFM_API void writeSFMPLYFormat(const string& filename,const unordered_set<int>& addedCams,
  const ptr_vector<ICamera>& cams,const vector<Vector3d>& points,const vector<bool>& pointsMask,
  const vector<Vector3uc> *pointColors = nullptr);
// This function is just a "header" function taking IDataset.
// It only calls overloaded function taking full spectrum of parameters.
YASFM_API void writeSFMPLYFormat(const string& filename,const IDataset& dts,
  const vector<Vector3uc> *pointColors = nullptr);
*/

} // namespace yasfm

namespace
{

class BundlerPointView
{
public:
  BundlerPointView(int _camIdx,int _keyIdx,double _x,double _y)
    : camIdx(_camIdx),keyIdx(_keyIdx),x(_x),y(_y) {}
  int camIdx,keyIdx;
  double x,y;
};

bool hasExtension(const string& filename,const string& extension);
bool hasExtension(const string& filename,const vector<string>& allowedExtensions);

// Returns 0. when the appropriate entry could be found.
double findCCDWidthInDB(const string& dbFilename,const string& cameraMake,const string& cameraModel);
bool readCameraMakeModelFromDBEntry(const string& entry,string& makeModel);
// Returns 0. when the appropriate entry could be found.
double readCCDWidthFromDBEntry(const string& entry);

} // namespace

///////////////////////////////////////////////////////////////////////

namespace yasfm
{

/*
template<typename CameraPairDerived>
void readMatchesASCII(const string& filename,IDataset& dts)
{
  readMatchesASCII<CameraPairDerived>(filename,dts.pairs());
}

template<typename CameraPairDerived>
void readMatchesASCII(const string& filename,pair_ptr_unordered_map<ICameraPair>& pairs)
{
  ifstream in(filename);
  if(!in.is_open())
  {
    cerr << "readMatchesASCII: unable to open: " << filename << " for reading" << endl;
    return;
  }
  int nPairs;
  in >> nPairs;
  pairs.reserve(nPairs);
  for(int i = 0; i < nPairs; i++)
  {
    int cam1,cam2,nMatches;
    in >> cam1 >> cam2 >> nMatches;
    IntPair pairIdx(cam1,cam2);
    if(pairs.count(pairIdx) == 0)
    {
      pairs.emplace(pairIdx,std::unique_ptr<ICameraPair>(new CameraPairDerived));
    }
    auto& pair = pairs[pairIdx];
    for(int j = 0; j < nMatches; j++)
    {
      IntPair match;
      double dist;
      in >> match.first >> match.second >> dist;
      pair->addMatch(match,dist);
    }
  }
  in.close();
}

template<typename CameraPairDerived>
void readTransformsASCII(const string& filename,IDataset& dts)
{
  readTransformsASCII<CameraPairDerived>(filename,dts.pairs());
}

template<typename CameraPairDerived>
void readTransformsASCII(const string& filename,pair_ptr_unordered_map<ICameraPair>& pairs)
{
  ifstream in(filename);
  if(!in.is_open())
  {
    cerr << "readTransformsASCII: unable to open: " << filename << " for reading" << endl;
    return;
  }
  int nPairs;
  in >> nPairs;
  for(int i = 0; i < nPairs; i++)
  {
    int cam1,cam2;
    in >> cam1 >> cam2;
    IntPair pairIdx(cam1,cam2);
    if(pairs.count(pairIdx) == 0)
    {
      pairs.emplace(pairIdx,std::unique_ptr<ICameraPair>(new CameraPairDerived));
    }
    auto& pair = pairs[pairIdx];
    Matrix3d F;
    in >> F(0,0) >> F(0,1) >> F(0,2)
      >> F(1,0) >> F(1,1) >> F(1,2)
      >> F(2,0) >> F(2,1) >> F(2,2);
    pair->setF(F);
  }
  in.close();
}
*/

} // namespace yasfm
