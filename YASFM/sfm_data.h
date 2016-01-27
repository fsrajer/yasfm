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
} CameraPair;

/// The class for storing points and n-view matches.
/**
Class including point data as well as n-view matches which have not been reconstructed 
yet. For adding points, prefer member functions which handle removal of corresponding
n-view matches.
*/
class Points
{
public:
  /// Data for one point except coordinates.
  typedef struct PointData
  {
    NViewMatch reconstructed; ///< Points views in images that have been reconstucted.
    NViewMatch toReconstruct; ///< Points views in images that have not been reconstucted.
    Vector3uc color;          ///< Point color.
  } PointData;

  /// Constructor
  YASFM_API Points();

  /// Add new points reconstructed from two views.
  /**
  Add new points created from corresponding matchesToReconstructIdxs.
  The corresponding matchesToReconstruct are erased so NOTE that indices
  to matchesToReconstruct you might have get invalidated.
  All the inputs except the first one have to be the same size.

  \param[in] camsIdxs Indices of the two cameras used for reconstuction of the points.
  \param[in] matchesToReconstructIdxs Indices of matchesToReconstruct from which
  the points have been reconstructed. The corresponding matchesToReconstruct 
  will be deleted.
  \param[in] coord New points' coordinates.
  \param[in] colors New points' colors.
  */
  YASFM_API void addPoints(const IntPair& camsIdxs,
    const vector<int>& matchesToReconstructIdxs,const vector<Vector3d>& coord,
    const vector<Vector3uc>& colors);

  /// Add new points reconstructed from n views. 
  /**
  All the inputs have to be the same size.

  \param[in] pointCoord New points' coordinates.
  \param[in] colors New points' colors.
  \param[in] pointViews View of the new points in cameras that have been reconstructed
  and in cameras that haven't been reconstucted yet.
  */
  YASFM_API void addPoints(const vector<Vector3d>& pointCoord,
    const vector<Vector3uc>& colors,const vector<SplitNViewMatch>& pointViews);

  /// Remove points views (disables the points).
  /**
  This clears .reconstructed and .toReconstruct in order to disable the points.
  This is used instead of directly removing the points in order to keep the indices
  to points, which someone might have, valid.

  \param[in] keep Indication which points to keep.
  */
  YASFM_API void removePointsViews(const vector<bool>& keep);
  
  /// \return Number of reconstucted points including those with no views.
  YASFM_API int numPtsAll() const;

  /// \return Number of reconstucted points that were not removed using 
  /// removePointsProjections.
  YASFM_API int numPtsAlive() const;

  /// Update points' views.
  /**
  Goes through pointData and updates reconstructed and toReconstruct by moving 
  the cam for all corresponding points from toReconstruct to reconstructed.

  \param[in] camIdx Index of the camera which got reconstructed
  */
  YASFM_API void markCamAsReconstructed(int camIdx);

  /// Update points' views.
  /**
  Goes through pointData and updates reconstructed and toReconstruct by moving 
  the cam for all correspondingPoints[correspondingPointsInliers[i]] from 
  toReconstruct to reconstructed and removing the cam from toReconstruct for all 
  non-inliers. When correspondingPointsInliers is null then all correspondingPoints 
  are considered to be inliers.

  \param[in] camIdx Index of the camera which got reconstructed.
  \param[in] correspondingPoints Indices of the relevant points.
  \param[in] correspondingPointsInliers Indices to correspondingPoints. Indicates
  inliers to the reconstructed camera.
  */
  YASFM_API void markCamAsReconstructed(int camIdx,
    const vector<int>& correspondingPoints,
    const vector<int>& correspondingPointsInliers);

  /// Write all the point data including matchesToReconstruct.
  /**
  IMPORTANT: Only alive points (with .reconstructed.size() > 0) get written out.

  \param[in,out] file Opened output file.
  */
  YASFM_API void writeASCII(ostream& file) const;
  
  /// Read points from a file.
  /// \param[in,out] file Opened input file.
  YASFM_API void readASCII(istream& file);

  /// \return N-View matches which have not been yet reconstructed.
  YASFM_API const vector<NViewMatch>& matchesToReconstruct() const;

  /// \return N-View matches which have not been yet reconstructed.
  YASFM_API vector<NViewMatch>& matchesToReconstruct();

  /// \return Points' coordinates.
  YASFM_API const vector<Vector3d>& ptCoord() const;

  /**
  Points are stored in a contiguous block so getting pointer to first points 
  also gives you all the points (including those with no projection).

  \param[in] ptIdx Index of the point.
  \return Point coordinates.
  */
  YASFM_API double* ptCoord(int ptIdx);

  /// \return Points' data.
  YASFM_API const vector<PointData>& ptData() const;

private:
  /// N-View matches which have not been yet reconstructed.
  vector<NViewMatch> matchesToReconstruct_; 
  vector<Vector3d> ptCoord_;  ///< Points' coordinates.
  vector<PointData> ptData_;  ///< Points' data.
  /// Number of reconstucted points that were not removed using removePointsProjections.
  int nPtsAlive_; 
};

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

  /// Marks cam as reconstructed and calls the same method in points.
  /// \param[in] camIdx Index of the reconstructed camera.
  YASFM_API void markCamAsReconstructed(int camIdx);

  /// Marks cam as reconstructed and calls the same method in points.
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

  /// \return Points and unreconstructed n-view matches.
  YASFM_API const Points& points() const;

  /// \return Points and unreconstructed n-view matches.
  YASFM_API Points& points();

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

  string dir_; ///< Working directory.
  /// The cameras stored as pointers to Camera.
  ptr_vector<Camera> cams_;
  pair_umap<CameraPair> pairs_;
  uset<int> reconstructedCams_;
  Points points_;
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