//----------------------------------------------------------------------------------------
/**
* \file       standard_camera.h
* \brief      Standard camera class.
*
*  Standard camera with square pixel and principal point in the middle of the image.
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
#include "ceres/rotation.h"
#include "Eigen/Dense"

#include "defines.h"
#include "camera.h"
#include "camera_factory.h"

using Eigen::AngleAxisd;
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

////////////////////////////////////////////////////
///////////////   Declarations   ///////////////////
////////////////////////////////////////////////////

/// The standard camera with square pixel and principal point in the middle of the image.
/**
This camera has the following parameters (exports in the given order):
3 for rotation in angle-axis (internally stored as angle and normalized axis).
3 for camera center.
1 for focal length.
*/
class StandardCamera : public Camera
{
public:

  /// Constructor.
  /**
  Empty constructor setting values to zero.
  */
  YASFM_API StandardCamera();

  /// Constructor.
  /**
  Calls base class constructor and sets principal point to be the middle of the image.

  \param[in] imgFilename Path to the image file.
  */
  YASFM_API StandardCamera(const string& imgFilename);

  /// Constructor. (Reads everything from the file.)
  /**
  Reads saved camera and features from files. (Starts by calling base class constructor.)

  \param[in,out] file Opened file containg main camera information as written
  by writeASCII().
  \param[in] readMode Given by WriteMode.
  \param[in] featuresDir Directory containg a file with features.
  */
  YASFM_API StandardCamera(istream& file,int readMode,const string& featuresDir);

  /// Destructor.
  /**
  Seems like the destructor has to be virtual so that the destruction
  of the pointer to the base class would destroy all the data that are
  owned by the derived class.
  */
  YASFM_API virtual ~StandardCamera();


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
  YASFM_API virtual unique_ptr<Camera> clone() const;

  /// Class name. !! HAS TO BE OVERRIDEN BY EVERY DERIVED CLASS IN HIERARCHY !!
  /// \return "DerivedCamera" for DerivedCamera.
  YASFM_API virtual string className() const;


  ///////////////////////////////////////////////
  ///////////////////////////////////////////////
  // OPTIONALLY IMPLEMENTED BY DERIVED CLASSES //
  ///////////////////////////////////////////////
  ///////////////////////////////////////////////

  /// Project a 3d point.
  /**
  \param[in] pt The 3d point.
  \return A projection of the 3d point.
  */
  YASFM_API virtual Vector2d project(const Vector3d& pt) const;

  /// Generate ceres cost function.
  /**
  This cost accepts a camera parameters as the first parameter and
  a point as the second parameter. The residual is of size 2 for
  x and y coordinates.

  \param[in] keyIdx Index of a key/observation to which does this cost
  function relates to.
  \return Pointer to the cost function. Deletion is assumed to be handled by ceres.
  */
  YASFM_API virtual ceres::CostFunction* costFunction(int keyIdx) const;

  /// Generate ceres parameters cost function.
  /**
  This cost function accepts camera parameters and returns the same
  number of residuals.

  \return Pointer to the cost function. Deletion is assumed to be handled by ceres.
  */
  YASFM_API virtual ceres::CostFunction* constraintsCostFunction() const;

  /// Export parameters (rotation, camera center, focal).
  /**
  \param[out] params Current camera parameters.
  */
  YASFM_API virtual void params(vector<double> *params) const;

  /// \param[in] i Key index.
  /// \return Key with undone calibration.
  YASFM_API virtual Vector2d keyNormalized(int i) const;

  /// \return Projection matrix.
  YASFM_API virtual Matrix34d P() const;

  /// \return Calibration matrix.
  YASFM_API virtual Matrix3d K() const;

  /// \return Camera pose. That is [R t]. Where R is rotation and t translation.
  YASFM_API virtual Matrix34d pose() const;

  /// \return Rotation matrix.
  YASFM_API virtual Matrix3d R() const;

  /// \return Camera center.
  YASFM_API virtual Vector3d C() const;

  /// \return Const reference to the rotation.
  YASFM_API virtual const AngleAxisd& rot() const;

  /// \return Focal length.
  YASFM_API virtual double f() const;

  /// \return Const reference to the principal point.
  YASFM_API const Vector2d& x0() const;

  /// Set parameters from exported ones (same format as params()).
  /// \param[in] params New camera parameters.
  YASFM_API virtual void setParams(const vector<double>& params);

  /// Set parameters from projection matrix.
  /** 
  Note that from the recovered calibration matrix, only focal will be used.

  \param[in] P Projection matrix.
  */
  YASFM_API virtual void setParams(const Matrix34d& P);

  /// Set rotation from rotation matrix.
  /// \param[in] R Rotation matrix.
  YASFM_API virtual void setRotation(const Matrix3d& R);

  /// Set camera center.
  /// \param[in] C Camera center.
  YASFM_API virtual void setC(const Vector3d& C);

  /// Set focal length.
  /// \param[in] f Focal length.
  YASFM_API virtual void setFocal(double f);

  /// Set parameters constraints and their weights (same ordering as params()).
  /**
  \param[in] constraints Constraints with the same ordering as parameters
  given by params().
  \param[in] weights Constraints weights with the same ordering as parameters
  given by params().
  */
  YASFM_API virtual void setParamsConstraints(const vector<double>& constraints,
    const vector<double>& weights);

  /// Constraint focal length
  /**
  \param[in] constraint Reference focal length.
  \param[in] weight Constraint weight (try 0.0001).
  */
  YASFM_API virtual void constrainFocal(double constraint,double weight);

protected:

  /// Write the camera parameters (not features).
  /**
  Write out camera parameters. This can is done by first calling
  Camera::writeASCII(file);
  and then writing the members contained in here.

  \param[in,out] file Opened output file.
  */
  virtual void writeASCII(ostream& file) const;

  AngleAxisd rot_; ///< Rotation.
  Vector3d C_;     ///< Camera center.
  double f_;       ///< Focal length.
  Vector2d x0_;    ///< Principal point.
  vector<double> paramsConstraints_;        ///< Parameters' constraints.
  vector<double> paramsConstraintsWeights_; ///< Weights of parameters' constraints.

  /// Index of the first rotation parameter in the export order.
  static const int rotIdx_ = 0; 
  /// Index of the first camera center parameter in the export order.
  static const int CIdx_ = 3;
  /// Index of the focal length in the export ordex.
  static const int fIdx_ = 6;

private:

  static const int nParams_ = 7; ///< Total number of parameters.

  /// Projection function with externally specified camera parameters.
  /**
  \param[in] camera Camera parameters as given by params().
  \param[in] point 3D point to be projected.
  \param[out] projection 2D projected point.
  */
  template<typename T>
  void projectWithExternalParams(const T* const camera,
    const T* const point,T *projection) const;
  
  /// Functor for computing reprojection error used to create ceres cost function. 
  struct ReprojectionErrorFunctor
  {
    /// Constructor
    /**
    \param[in] keyX Key/observation x coordinate.
    \param[in] keyY Key/observation y coordinate.
    \param[in] cam Camera used for projecting.
    */
    ReprojectionErrorFunctor(double keyX,double keyY,const StandardCamera& cam);

    /**
    Project a point and then substract the key/observation.

    \param[in] camera Camera parameters in as given by params().
    \param[in] point 3D point to be projected.
    \param[out] residuals 2D residuals for x and y coordinates.
    */
    template <typename T>
    bool operator()(const T* const camera,
      const T* const point,
      T* residuals) const;

    const StandardCamera& cam_; ///< Camera used for projecting.
    double keyX_; ///< Key/observation x coordinate.
    double keyY_; ///< Key/observation y coordinate.
  };

  /// Register entry of this class in the factory.
  /*
  NOTE that every DerivedCamera should have a member:
  static CameraRegister<DerivedCamera> reg_;
  and implement it in the following way :
  CameraRegister<DerivedCamera> DerivedCamera::reg_("DerivedCamera");
  The registered names are used to determine which derived camera should
  be created when reading from a file.
  */
  static CameraRegister<StandardCamera> reg_;
};

////////////////////////////////////////////////////
///////////////   Definitions   ////////////////////
////////////////////////////////////////////////////

template<typename T>
void StandardCamera::projectWithExternalParams(const T* const camera,
  const T* const point,T *projection) const
{
  // Subtract camera center.
  T p[3];
  p[0] = point[0] - camera[CIdx_ + 0];
  p[1] = point[1] - camera[CIdx_ + 1];
  p[2] = point[2] - camera[CIdx_ + 2];

  // Rotate.
  T pCam[3];
  ceres::AngleAxisRotatePoint(&camera[rotIdx_],p,pCam);

  T xp = pCam[0] / pCam[2];
  T yp = pCam[1] / pCam[2];

  // Multiply by calibration matrix, i.e., apply focal and principal point.
  projection[0] = camera[fIdx_] * xp + T(x0_(0));
  projection[1] = camera[fIdx_] * yp + T(x0_(1));
}

template<typename T>
bool StandardCamera::ReprojectionErrorFunctor::operator()(const T* const camera,
  const T* const point,T* residuals) const
{
  T projection[2];
  cam_.projectWithExternalParams(camera,point,projection);

  residuals[0] = projection[0] - T(keyX_);
  residuals[1] = projection[1] - T(keyY_);

  return true;
}

} // namespace yasfm