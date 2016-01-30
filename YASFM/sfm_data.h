//----------------------------------------------------------------------------------------
/**
* \file       sfm_data.h
* \brief      Classes for storing sfm results (except camera classes).
*
*  Classes for storing sfm results (except camera classes).
*
*/
//----------------------------------------------------------------------------------------

#pragma once

#include <memory>
#include <string>
#include <vector>
#include <istream>
#include <ostream>
#include <set>

#include "Eigen/Dense"

#include "defines.h"
#include "camera.h"
#include "utils.h"
#include "utils_io.h"

using Eigen::Vector3d;
using std::string;
using std::vector;
using std::ostream;
using std::istream;
using std::set;
using std::make_unique;

namespace yasfm
{

////////////////////////////////////////////////////
///////////////   Declarations   ///////////////////
////////////////////////////////////////////////////

/// Structure for storing camera pair information.
typedef struct CameraPair
{
  /// Matches of features. The first entry in the IntPair is an index of the
  /// key in the first image and in the same manner the second.
  vector<IntPair> matches; 
  /// This can be any score. The smaller the better.
  vector<double> dists;
  /// For geometric verification. Numbers of inliers to individual transformations.
  vector<int> supportSizes;
} CameraPair;

/// Main class for storing results of the reconstruction.
/**
Class used for storing results of the reconstruction. Able to copy itself using
copy constructor or assignment operator, write itself into file and read itself
again.
*/
class Dataset
{
public:
  /// Constructor.
  /**
  Does nothing except setting the dir.

  \param[in] dir Working directory.
  */
  YASFM_API Dataset(const string& dir);
  
  /// Copy constructor. Copies all the data.
  /// \param[in] o Other dataset.
  YASFM_API Dataset(const Dataset& o);

  /// Assignment operator. Copies all the data.
  /// \param[in] o Other dataset.
  /// \return Reference to this.
  YASFM_API Dataset& operator=(const Dataset& o);
  
  /// Add one camera to the dataset.
  /**
  \param[in] filename Path to an image file.
  \param[in] isSubdir true if the filename is relative to the working dir.
  */
  template<class T>
  void addCamera(const string& filename,bool isSubdir = true);

  /// Add all images in a directory to the dataset as cameras.
  /**
  \param[in] imgsDir Directory with the images.
  \param[in] isSubdir true if the imgsDir is relative to the working dir.
  */
  template<class T>
  void addCameras(const string& imgsDir,bool isSubdir = true);

  /// Erase all the descriptors to release memory.
  YASFM_API void clearDescriptors();

  /// \return Number of cameras.
  YASFM_API int numCams() const;

  /// Marks cam as reconstructed and updates points views and viewsToAdd.
  /// \param[in] camIdx Index of the reconstructed camera.
  YASFM_API void markCamAsReconstructed(int camIdx);

  /// Marks cam as reconstructed and updates points views and viewsToAdd.
  /**
  \param[in] camIdx Index of the reconstructed camera.
  \param[in] correspondingPoints Indices of the relevant points.
  \param[in] correspondingPointsInliers Indices to correspondingPoints. Indicates
  inliers to the reconstructed camera.
  */
  YASFM_API void markCamAsReconstructed(int camIdx,
    const vector<int>& correspondingPoints,
    const vector<int>& correspondingPointsInliers);

  /// Go through all the points and count number of reconstructed views/observations.
  /// \return Total number of observations/views.
  YASFM_API int countReconstructedObservations() const;
  
  /// Count points that have some views.
  /// \return Number of points that have non-empty views.
  YASFM_API int countPtsAlive() const;

  /// Write the dataset to a file (except features).
  /**
  \param[in] filename Path to output file relative to the working directory.
  */
  YASFM_API void writeASCII(const string& filename) const;

  /// For all cameras reads keys colors.
  YASFM_API void readKeysColors();

  /// Reads the dataset from a file (features get read from separate files).
  /**
  \param[in] filename Path to input file relative to the working directory.
  */
  YASFM_API void readASCII(const string& filename);

  /// \return Working directory.
  YASFM_API const string& dir() const;

  /// \return Working directory.
  YASFM_API string& dir();

  /// \param[in] idx Camera index.
  /// \return The camera.
  YASFM_API const Camera& cam(int idx) const;

  /// \param[in] idx Camera index.
  /// \return The camera.
  YASFM_API Camera& cam(int idx);

  /// \param[in] idx Camera index.
  /// \return The camera.
  YASFM_API const Camera& cam(size_t idx) const;

  /// \param[in] idx Camera index.
  /// \return The camera.
  YASFM_API Camera& cam(size_t idx);

  /// \return Cameras.
  YASFM_API const ptr_vector<Camera>& cams() const;

  /// \return Cameras.
  YASFM_API ptr_vector<Camera>& cams();

  /// \return Camera pairs.
  YASFM_API const pair_umap<CameraPair>& pairs() const;

  /// \return Camera pairs.
  YASFM_API pair_umap<CameraPair>& pairs();

  /// \return Reconstructed cameras.
  YASFM_API const uset<int>& reconstructedCams() const;

  YASFM_API const vector<NViewMatch>& nViewMatches() const;
  YASFM_API vector<NViewMatch>& nViewMatches();

  /// \return Points.
  YASFM_API const vector<Point>& pts() const;

  /// \return Points.
  YASFM_API vector<Point>& pts();

  YASFM_API const vector<set<int>>& queries() const;
  YASFM_API vector<set<int>>& queries();

  /// \return Features directory.
  YASFM_API string featsDir() const;

private:
  /// Copy dataset from another one.
  /**
  This is used in copy constructor and assignment operator. It is implemented 
  so that ptr_vector<Camera> would correctly copy (unique_ptr<Base> has to be 
  copied using clone()).
  !!! Update this whenever a new member variable is added.
  */
  void copyIn(const Dataset& o);

  void readPts(istream& file);
  void readMatches(istream& file);

  // Keep this for a while and remove when that format is not used anywhere.
  void readPointsOld(istream& file);

  string dir_; ///< Working directory.
  /// The cameras stored as pointers to Camera.
  ptr_vector<Camera> cams_;
  vector<set<int>> queries_; ///< Which pairs should be matched. 
  pair_umap<CameraPair> pairs_;
  uset<int> reconstructedCams_;
  vector<NViewMatch> nViewMatches_; ///< Only matches not converted to points yet.
  vector<Point> pts_;
};

////////////////////////////////////////////////////
///////////////   Definitions   ////////////////////
////////////////////////////////////////////////////

template<class T>
void Dataset::addCamera(const string& filename,bool isSubdir)
{
  string fnAbs;
  if(isSubdir)
    fnAbs = joinPaths(dir_,filename);
  else
    fnAbs = filename;

  cams_.push_back(make_unique<T>(fnAbs,featsDir()));
}
template<class T>
void Dataset::addCameras(const string& imgsDir,bool isSubdir)
{
  string imgsDirAbs;
  if(isSubdir)
    imgsDirAbs = joinPaths(dir(),imgsDir);
  else
    imgsDirAbs = imgsDir;

  vector<string> filenames;
  listImgFilenames(imgsDirAbs,&filenames);

  size_t nImgs = filenames.size();
  cams_.reserve(cams_.capacity() + nImgs);
  for(size_t i = 0; i < nImgs; i++)
  {
    cams_.push_back(make_unique<T>(joinPaths(imgsDirAbs,filenames[i]),featsDir()));
  }
}

} // namespace yasfm