//----------------------------------------------------------------------------------------
/**
* \file       utils_io.h
* \brief      Various utility functions for input/output.
*
*  Various utility functions for input/output.
*
*/
//----------------------------------------------------------------------------------------

#pragma once

#include <fstream>
#include <string>
#include <vector>

#include "Eigen/Dense"

#include "defines.h"
#include "camera.h"
#include "sfm_data.h"

using Eigen::Vector2d;
using std::ifstream;
using std::istream;
using std::ostream;
using std::string;
using std::vector;
using namespace yasfm;

namespace yasfm
{

// Forward declarations
class Points;
class Dataset;

/// List all image files with supported extensions in a directory.
/**
\param[in] dir Directory.
\param[out] filenames Filenames (not paths).
*/
YASFM_API void listImgFilenames(const string& dir,vector<string> *filenames);

/// List all files
/**
\param[in] dir Directory.
\param[out] filenames Filenames (not paths).
*/
YASFM_API void listFilenames(const string& dir,vector<string> *filenames);

/// List all files with given extensions.
/**
\param[in] dir Directory.
\param[in] filenameExtensions Allowed file extensions. An empty extensions vector 
means that all files will be listed.
\param[out] filenames Filenames (not paths).
*/
YASFM_API void listFilenames(const string& dir,
  const vector<string>& filenameExtensions,vector<string> *filenames);

/// Load image and read dimensions.
/**
\param[in] filename Image filename.
\param[out] width Image width.
\param[out] height Image height.
*/
YASFM_API void getImgDims(const string& filename,int *width,int *height);

/// Read colors of specified locations.
/**
\param[in] filename Image filename.
\param[in] coord Coordinater where to read colors (gets rounded to closest integer).
\param[out] colors Colors.
*/
YASFM_API void readColors(const string& filename,const vector<Vector2d>& coord,
  vector<Vector3uc> *colors);

/// Finds focals in EXIF and using sensor size converts to pixels (is verbose).
/**
\param[in] ccdDBFilename Database with camera sensor sizes.
\param[in] cams Cameras with filenames and image dimensions known.
\param[out] focals Focal lengths of cameras in pixels. 0 if a focal could not be
computed.
*/
YASFM_API void findFocalLengthInEXIF(const string& ccdDBFilename,
  const ptr_vector<Camera>& cams,vector<double> *focals);

/// Finds focals in EXIF and using sensor size converts to pixels.
/**
\param[in] ccdDBFilename Database with camera sensor sizes.
\param[in] cams Cameras with filenames and image dimensions known.
\param[in] verbose Print status?
\param[out] focals Focal lengths of cameras in pixels. 0 if a focal could not be
computed.
*/
YASFM_API void findFocalLengthInEXIF(const string& ccdDBFilename,
  const ptr_vector<Camera>& cams,bool verbose,vector<double> *focals);

/// Finds focal in EXIF and using sensor size converts to pixels.
/**
\param[in] ccdDBFilename Database with camera sensor sizes.
\param[in] cam Camera with filename and image dimensions known.
\param[in] verbose Print status?
\return Focal length. 0 if a focal could not be computed.
*/
YASFM_API double findFocalLengthInEXIF(const string& ccdDBFilename,const Camera& cam,
  bool verbose);

/// Finds focal in EXIF and using sensor size converts to pixels.
/**
\param[in] ccdDBFilename Database with camera sensor sizes.
\param[in] imgFilename Image filename.
\param[in] maxImgDim Maximum of image width and height.
\param[in] verbose Print status?
\return Focal length. 0 if a focal could not be computed.
*/
YASFM_API double findFocalLengthInEXIF(const string& ccdDBFilename,
  const string& imgFilename,int maxImgDim,bool verbose);

/// Writes data into Bundler's Bundle format
/**
See: http://www.cs.cornell.edu/~snavely/bundler/bundler-v0.4-manual.html
The format requires writing features' coordinates in a coordinate system where image 
center is the origin and the y axis goes from bottom to the top. In addition -z should 
be the forward axis.
IMPORTANT: This function assumes +z to be the forward axis. Also, to get radial 
parameters, this function tries to cast into StandardCameraRadial.
IMPORTANT: Only alive points (with .reconstructed.size() > 0) get written out.

\param[in] filename Output filename.
\param[in] reconstructedCams Cameras that were reconstructed.
\param[in] cams Cameras.
\param[in] points Points.
*/
YASFM_API void writeSFMBundlerFormat(const string& filename,
  const uset<int>& reconstructedCams,const ptr_vector<Camera>& cams,
  const Points& points);

/// Calls overloaded fuction.
YASFM_API void writeSFMBundlerFormat(const string& filename,const Dataset& data);

YASFM_API ostream& operator<<(ostream& file,const NViewMatch& m);
YASFM_API istream& operator>>(istream& file,NViewMatch& m);

} // namespace yasfm

namespace
{

/// Helper struct for output.
class BundlerPointView
{
public:
  BundlerPointView(int _camIdx,int _keyIdx,double _x,double _y)
    : camIdx(_camIdx),keyIdx(_keyIdx),x(_x),y(_y) {}
  int camIdx,keyIdx;
  double x,y;
};

/// Check if the filename has the extension.
bool hasExtension(const string& filename,const string& extension);

/// Check if the filename has any of the extensions.
bool hasExtension(const string& filename,const vector<string>& allowedExtensions);

/// Find entry in CCD Database.
/**
\param[in] dbFilename Filename with database.
\param[in] cameraMake Camera make (as in EXIF).
\param[in] cameraModel Camera model (as in EXIF).
\return CCD sensor width. Returns 0 when the appropriate entry could be found.
*/
double findCCDWidthInDB(const string& dbFilename,const string& cameraMake,
  const string& cameraModel);

/// Read CCD Database entry name.
/**
\param[in] entry Database entry.
\param[in] makeModel Camera make + camera model.
\return Successfully found?
*/
bool readCameraMakeModelFromDBEntry(const string& entry,string *makeModel);

/// Read CCD Database entry value.
/**
\param[in] entry Database entry.
\return CCD sensor width. Returns 0 when unsuccessful.
*/
double readCCDWidthFromDBEntry(const string& entry);

} // namespace



/*
YASFM_API void writeFeaturesBinary(const IDataset& dts);
YASFM_API void writeFeaturesBinary(const ptr_vector<ICamera>& cams,const string& dir);
YASFM_API void writeFeaturesBinary(const string& dir,const ICamera& cam);
YASFM_API void readFeaturesBinary(IDataset& dts);
YASFM_API void readFeaturesBinary(ptr_vector<ICamera>& cams,const string& dir);
YASFM_API void readFeaturesBinary(const string& dir,ICamera& cam);


YASFM_API void writeSFMPLYFormat(const string& filename,const unordered_set<int>& addedCams,
const ptr_vector<ICamera>& cams,const vector<Vector3d>& points,const vector<bool>& pointsMask,
const vector<Vector3uc> *pointColors = nullptr);
// This function is just a "header" function taking IDataset.
// It only calls overloaded function taking full spectrum of parameters.
YASFM_API void writeSFMPLYFormat(const string& filename,const IDataset& dts,
const vector<Vector3uc> *pointColors = nullptr);
*/