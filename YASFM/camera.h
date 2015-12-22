//----------------------------------------------------------------------------------------
/**
* \file       camera.h
* \brief      Base class for all cameras.
*
*  The base class for all cameras.
*
*/
//----------------------------------------------------------------------------------------

#pragma once

#include <memory>
#include <istream>
#include <ostream>
#include <string>
#include <vector>

#include "ceres/ceres.h"
#include "Eigen/Dense"

#include "defines.h"

using Eigen::ArrayXXf;
using Eigen::Matrix3d;
using Eigen::Vector2d;
using Eigen::Vector3d;
using std::ostream;
using std::istream;
using std::string;
using std::unique_ptr;
using std::vector;

namespace yasfm
{

/// The base class for all cameras.
/**
This class is the base class for all cameras. It handles image access
and is able to store keypoints (coordinates, scales and orientations), 
their colors and descriptors. Examine all the functions to get an idea 
which do you need to override.
NOTE that every DerivedCamera should have a member:
static CameraRegister<DerivedCamera> reg_;
and implement it in the following way:
CameraRegister<DerivedCamera> DerivedCamera::reg_("DerivedCamera");
The registered names are used to determine which derived camera should
be created.
*/
class Camera
{
public:
  /// Constructor. (empty)
  YASFM_API Camera();
  /// Constructor. (Reads image dimensions.)
  /**
  Opens the image file in order to get image dimensions.

  \param[in] imgFilename Path to the image file.
  */
  YASFM_API Camera(const string& imgFilename);

  /// Constructor. (Reads everything from the file.)
  /**
  Reads saved camera and features from files.

  \param[in,out] file Opened file containg main camera information as written
  by writeASCII().
  \param[in] readMode Given by WriteMode.
  \param[in] featuresDir Directory containg a file with features.
  */
  YASFM_API Camera(istream& file,int readMode,const string& featuresDir);

  /// Destructor.
  /** 
  Seems like the destructor has to be virtual so that the destruction
  of the pointer to the base class would destroy all the data that are 
  owned by the derived class.
  */
  YASFM_API virtual ~Camera();


  //////////////////////////////////////////
  //////////////////////////////////////////
  // TO BE IMPLEMENTED BY DERIVED CLASSES //
  //////////////////////////////////////////
  //////////////////////////////////////////

  /// Cloning. !! HAS TO BE OVERRIDEN BY EVERY DERIVED CLASS IN HIERARCHY !!
  /**
  This is used for cloning when one has only pointer to the Camera.
  DerivedCamera has to implement clone() like this:
  { return make_unique<DerivedCamera>(*this); }

  \return Base class pointer to the clone of the DerivedCamera.
  */
  YASFM_API virtual unique_ptr<Camera> clone() const = 0;

  /// Class name. !! HAS TO BE OVERRIDEN BY EVERY DERIVED CLASS IN HIERARCHY !!
  /// \return "DerivedCamera" for DerivedCamera.
  YASFM_API virtual string className() const = 0;

  /// Project a 3d point.
  /**
  \param[in] pt The 3d point.
  \return A projection of the 3d point.
  */
  YASFM_API virtual Vector2d project(const Vector3d& pt) const = 0;
  
  /// Generate ceres cost function.
  /**
  This cost should accept a camera parameters as the first parameter and
  a point as the second parameter. The residual should be of size 2 for
  x and y coordinates.

  \param[in] keyIdx Index of a key/observation to which does this cost 
  function relates to.
  \return Pointer to the cost function. Deletion is assumed to be handled by ceres.
  */
  YASFM_API virtual ceres::CostFunction* costFunction(int keyIdx) const = 0;
  
  /// Generate ceres parameters cost function.
  /**
  This cost function should accept camera parameters and return the same 
  number of residuals.

  \return Pointer to the cost function. Deletion is assumed to be handled by ceres.
  */
  YASFM_API virtual ceres::CostFunction* constraintsCostFunction() const = 0;

  /// Export parameters.
  /**
  \param[out] params Current camera parameters.
  */
  YASFM_API virtual void params(vector<double> *params) const = 0;
  
  /// \param[in] i Key index.
  /// \return Key with undone calibration.
  YASFM_API virtual Vector2d keyNormalized(int i) const = 0;

  /// \return Projection matrix.
  YASFM_API virtual Matrix34d P() const = 0;

  /// \return Calibration matrix.
  YASFM_API virtual Matrix3d K() const = 0;

  /// \return Camera pose. That is [R t]. Where R is rotation and t translation.
  YASFM_API virtual Matrix34d pose() const = 0;

  /// \return Rotation matrix.
  YASFM_API virtual Matrix3d R() const = 0;

  /// \return Camera center.
  YASFM_API virtual Vector3d C() const = 0;

  /// Set parameters from exported ones (same format as params()).
  /// \param[in] params New camera parameters.
  YASFM_API virtual void setParams(const vector<double>& params) = 0;

  /// Set parameters from projection matrix.
  /// \param[in] P Projection matrix.
  YASFM_API virtual void setParams(const Matrix34d& P) = 0;
  
  /// Set rotation from rotation matrix.
  /// \param[in] R Rotation matrix.
  YASFM_API virtual void setRotation(const Matrix3d& R) = 0;
  
  /// Set camera center.
  /// \param[in] C Camera center.
  YASFM_API virtual void setC(const Vector3d& C) = 0;
  
  /// Set focal length.
  /// \param[in] f Focal length.
  YASFM_API virtual void setFocal(double f) = 0;

  /// Set parameters constraints and their weights (same ordering as params()).
  /**
  \param[in] constraints Constraints with the same ordering as parameters
  given by params().
  \param[in] weights Constraints weights with the same ordering as parameters
  given by params().
  */
  YASFM_API virtual void setParamsConstraints(const vector<double>& constraints,
    const vector<double>& weights) = 0;

protected:
  /// Write the camera parameters (not features).
  /**
  Write out camera parameters. This can be done by first calling
  BaseCamera::writeASCII(file);
  and then writing the members contained in the DerivedCamera.

  \param[in,out] file Opened output file.
  */
  virtual void writeASCII(ostream& file) const;

private:
  /*
  !! HAS TO BE FOR EVERY DERIVED CLASS IN HIERARCHY !!
  NOTE that every DerivedCamera should have a member:
  static CameraRegister<DerivedCamera> reg_;
  and implement it in the following way :
  CameraRegister<DerivedCamera> DerivedCamera::reg_("DerivedCamera");
  The registered names are used to determine which derived camera should
  be created when reading from a file.
  */



  ///////////////////////////////////
  ///////////////////////////////////
  // ALREADY IMPLEMENTED BY CAMERA //
  ///////////////////////////////////
  ///////////////////////////////////

public:
  /// Enum for indicating how should features be written.
  /**
  Bits information:
  000 = 0: no features,
  001 = 1: keys,
  010 = 2: descriptors,
  110 = 6: descriptors + conversion of descriptors.
  */
  enum WriteMode
  {
    WriteNoFeatures = 0,  ///< Don't write features at all.
    WriteKeys = 1,        ///< Writes keypoints (coordinates, scales and orientations)
    WriteDescriptors = 2, ///< Write descriptors.
    WriteAll = 3,         ///< Write both, keys and descriptors.
    /// Convert descriptors to unsigned int before writing.
    /// (Does not change actual data. Just writes converted.)
    WriteConvertNormalizedSIFTToUint = 4
  };

  /// Enum for indicating what features information should be read.
  enum ReadMode
  {
    /// Read only keys (coordinates, scales and orientations) and no descriptors
    ReadNoDescriptors = 0,
    ReadAll = 1            ///< Read keys and descriptors.
  };

  /// Allocate storage for keys and descriptors.
  /**
  Does not allocate colors. Those should be read using readKeysColors()
  after you have all the keys.

  \param[in] num Number of features.
  \param[in] dim Dimension of feature descriptors.
  */
  YASFM_API virtual void resizeFeatures(int num,int dim);
  
  /// Set one feature.
  /**
  Before calling this, the feature should be allocated with resizeFeatures().

  \param[in] idx Index of the feature to set.
  \param[in] x x coordinate of the feature/key.
  \param[in] y y coordinate of the feature/key.
  \param[in] scale Scale.
  \param[in] orientation Orientation (angle in radians).
  \param[in] descr Feature descriptor.
  */
  YASFM_API virtual void setFeature(int idx,double x,double y,double scale,
    double orientation,const float* const descr);

  /// Read keys colors from the image file.
  /**
  Allocate and read color of every key from the image file.
  */
  YASFM_API virtual void readKeysColors();

  /// Erase all descriptors to release memory.
  YASFM_API virtual void clearDescriptors();
  
  /// Set the image.
  /**
  \param[in] filename Path to the image.
  \param[in] width Image width.
  \param[in] height Image height
  */
  YASFM_API virtual void setImage(const string& filename,int width,int height);

  /// \return Reference to the path to the image file.
  YASFM_API const string& imgFilename() const;

  /// \return Image width.
  YASFM_API int imgWidth() const;

  /// \return Image height.
  YASFM_API int imgHeight() const;

  /// \return Const reference to keys.
  YASFM_API const vector<Vector2d>& keys() const;

  /**
  \param[in] i Index of the key.
  \return Const reference to one key.
  */
  YASFM_API const Vector2d& key(int i) const;

  /**
  \param[in] i Index of the key.
  \return reference to one key.
  */
  YASFM_API Vector2d& key(int i);
  
  /// \return Const reference to keys scales.
  YASFM_API const vector<double>& keysScales() const;

  /// \return Const reference to keys orientations in radians.
  YASFM_API const vector<double>& keysOrientations() const;

  /// \return Const reference to keys colors. May be empty.
  YASFM_API const vector<Vector3uc>& keysColors() const;

  /** 
  \param[in] i Index of the key.
  \return Const reference to color of one key.
  */
  YASFM_API const Vector3uc& keyColor(int i) const;
  
  /// \return Const reference to all descriptors (one column is one descriptor).
  YASFM_API const ArrayXXf& descr() const;

  /// \return Indices of points visible in this camera in ascending order.
  YASFM_API const vector<int>& visiblePoints() const;

  /// \return Indices of points visible in this camera in ascending order.
  YASFM_API vector<int>& visiblePoints();
  
  /// Call overloaded function.
  /**
  Calls overloaded function with featuresDir set to the "d/keys", where
  d is the directory where the image is stored.

  \param[in,out] file Opened output file.
  \param[in] writeMode Defined by WriteMode.
  */
  YASFM_API void writeASCII(ostream& file,int writeMode) const;

  /// Write the class into files.
  /**
  Most of the data will be stored in the file but keys and descriptors will
  be stored separately in "featuresDir/imgFilename.key"

  \param[in,out] file Opened output file.
  \param[in] writeMode Defined by WriteMode.
  \param[in] featuresDir Directory where to write a file with features.
  */
  YASFM_API void writeASCII(ostream& file,int writeMode,
    const string& featuresDir) const;

protected:
  /// Generate path to file with features.
  /**
  \param[in] featuresDir Directory where the file should reside.
  */
  virtual string featuresFilename(const string& featuresDir) const;

private:
  string imgFilename_; ///< Path to image file.
  int imgWidth_;       ///< Image width.
  int imgHeight_;      ///< Image height.

  vector<Vector2d> keys_;           ///< Keys (features coordinates).
  vector<double> keysScales_;       ///< Keys scales
  vector<double> keysOrientations_; ///< Orientation (angle in radians).
  vector<Vector3uc> keysColors_;    ///< Keys colors.
  ArrayXXf descr_;     ///< Descriptors (one column is one descriptor).
  /// Indices of points visible in this camera in ascending order.
  vector<int> visiblePoints_;       
};

} // namespace yasfm