/*
* Filip Srajer
* filip.srajer (at) fel.cvut.cz
* Center for Machine Perception
* Czech Technical University in Prague
*
* This software is under construction.
* 10/2015
*/

#pragma once

#include <array>
#include <memory>
#include <string>
#include <vector>

#include "ceres/ceres.h"
#include "ceres/rotation.h"
#include "Eigen/Dense"

#include "defines.h"
#include "utils.h"
#include "utils_io.h"

using Eigen::AngleAxisd;
using Eigen::ArrayXXf;
using Eigen::Matrix3d;
using Eigen::Vector2d;
using Eigen::Vector3d;
using std::array;
using std::make_unique;
using std::string;
using std::unique_ptr;
using std::vector;

namespace yasfm
{

////////////////////////////////////////////////////
///////////////   Declarations   ///////////////////
////////////////////////////////////////////////////

// Base class for all cameras. This class implements handling of 
// image data as well as features. All of its functions involving
// camera parameters do nothing and are supposed to be implemented
// derived classes.
class Camera
{
public:
  YASFM_API Camera(const string& imgFilename);
  // Destructor has to be virtual (even if it was empty) 
  // because of use of ptr_vector. It ensures that all the items
  // will be correctly deleted.
  YASFM_API virtual ~Camera();
  // !! derived classes need to implement this as follows:
  // { return make_unique<DerivedCamera>(*this); }
  YASFM_API virtual unique_ptr<Camera> clone() const;

  YASFM_API virtual void reserveFeatures(int num,int dim);
  YASFM_API virtual void addFeature(double x,double y,const float* const descr);
  // remove descriptors from memory
  YASFM_API virtual void clearDescriptors();

  // accessors
  YASFM_API virtual const string& imgFilename() const;
  YASFM_API virtual int imgWidth() const;
  YASFM_API virtual int imgHeight() const;
  YASFM_API virtual const vector<Vector2d>& keys() const;
  YASFM_API virtual const Vector2d& key(int i) const;
  // One column is one descriptor.
  YASFM_API virtual const ArrayXXf& descr() const;

  // to be implemented by derived classes
  YASFM_API virtual Vector2d project(const Vector3d& pt) const { return Vector2d::Zero(); }
  YASFM_API virtual ceres::CostFunction* costFunction(int keyIdx) const { return nullptr; }
  YASFM_API virtual ceres::CostFunction* constraintsCostFunction() const { return nullptr; }
  YASFM_API virtual void params(vector<double> *params) const {}
  // projection matrix
  YASFM_API virtual Matrix34d P() const { return Matrix34d::Zero(); }
  // Return key with undone calibration parameters.
  YASFM_API virtual Vector2d keyNormalized(int i) const { return Vector2d::Zero(); }
  // calibration matrix
  YASFM_API virtual Matrix3d K() const { return Matrix3d::Zero(); }
  // camera pose
  YASFM_API virtual Matrix34d pose() const { return Matrix34d::Zero(); }
  // rotation matrix
  YASFM_API virtual Matrix3d R() const  { return Matrix3d::Zero(); }
  // camera center
  YASFM_API virtual Vector3d C() const  { return Vector3d::Zero(); }
  YASFM_API virtual void setParams(const vector<double>& params)  {}
  // from projection matrix
  YASFM_API virtual void setParams(const Matrix34d& P)  {}
  YASFM_API virtual void setRotation(const Matrix3d& R)  {}
  // camera center
  YASFM_API virtual void setC(const Vector3d& C)  {}
  YASFM_API virtual void setFocal(double f) {}
  YASFM_API virtual void setParamsConstraints(const vector<double>& constraints,
    const vector<double>& weights) {}

private:
  string imgFilename_;
  int imgWidth_,imgHeight_;

  vector<Vector2d> keys_; // keypoints' coordinates
  ArrayXXf descr_; // descriptors; column is one descriptor
};

// Standard camera with parameters the following parameters:
// 3 rotation in angle-axis
// 3 camera center
// 1 focal length
class StandardCamera : public Camera
{
public:
  YASFM_API StandardCamera(const string& imgFilename);
  // Destructor has to be virtual (even if it was empty) 
  // because of use of ptr_vector. It ensures that all the items
  // will be correctly deleted.
  YASFM_API virtual ~StandardCamera();
  // !! derived classes need to implement this as follows:
  // { return make_unique<DerivedCamera>(*this); }
  YASFM_API virtual unique_ptr<Camera> clone() const;

  YASFM_API virtual Vector2d project(const Vector3d& pt) const;
  YASFM_API virtual ceres::CostFunction* costFunction(int keyIdx) const;
  YASFM_API virtual ceres::CostFunction* constraintsCostFunction() const;
  // 3 rotation, 3 translation, 1 focal length
  YASFM_API virtual void setParams(const vector<double>& params);
  // from projection matrix
  YASFM_API virtual void setParams(const Matrix34d& P);
  YASFM_API virtual void setRotation(const Matrix3d& R);
  YASFM_API virtual void setFocal(double f);
  // camera center
  YASFM_API virtual void setC(const Vector3d& C);
  // try weight 0.0001 for focal constraint
  YASFM_API virtual void constrainFocal(double constraint,double weigtht);
  YASFM_API virtual void setParamsConstraints(const vector<double>& constraints,
    const vector<double>& weights);

  // accessors
  // 3 rotation, 3 translation, 1 focal length
  YASFM_API virtual void params(vector<double> *params) const;
  // projection matrix
  YASFM_API virtual Matrix34d P() const;
  // Return key with undone calibration parameters.
  YASFM_API virtual Vector2d keyNormalized(int i) const;
  // calibration matrix
  YASFM_API virtual Matrix3d K() const;
  // camera pose
  YASFM_API virtual Matrix34d pose() const;
  // rotation matrix
  YASFM_API virtual Matrix3d R() const;
  // camera center
  YASFM_API virtual Vector3d C() const;
  YASFM_API virtual const AngleAxisd& rot() const;
  YASFM_API virtual double f() const;
  YASFM_API virtual const Vector2d& x0() const;

protected:
  AngleAxisd rot_;
  Vector3d C_;
  double f_;
  Vector2d x0_; // principal point
  vector<double> paramsConstraints_,paramsConstraintsWeights_;
  static const int rotIdx_ = 0;
  static const int CIdx_ = 3;
  static const int fIdx_ = 6;

private:
  template<typename T>
  void projectWithExternalParams(const T* const camera,
    const T* const point,T *projection) const;

  struct ReprojectionErrorFunctor
  {
    ReprojectionErrorFunctor(double keyX,double keyY,const StandardCamera& cam);

    template <typename T>
    bool operator()(const T* const camera,
      const T* const point,
      T* residuals) const;

    const StandardCamera& cam_;
    double keyX_,keyY_;
  };

  static const int nParams_ = 7;
};

// Standard camera with parameters the following parameters:
// 3 rotation in angle-axis
// 3 camera center
// 1 focal length
// 2 radial parameters
class StandardCameraRadial : public StandardCamera
{
public:
  YASFM_API StandardCameraRadial(const string& imgFilename);
  // Destructor has to be virtual (even if it was empty) 
  // because of use of ptr_vector. It ensures that all the items
  // will be correctly deleted.
  YASFM_API virtual ~StandardCameraRadial();
  // !! derived classes need to implement this as follows:
  // { return make_unique<DerivedCamera>(*this); }
  YASFM_API virtual unique_ptr<Camera> clone() const;

  YASFM_API virtual Vector2d project(const Vector3d& pt) const;
  YASFM_API virtual ceres::CostFunction* costFunction(int keyIdx) const;
  YASFM_API virtual ceres::CostFunction* constraintsCostFunction() const;
  // Return key with undone calibration parameters.
  YASFM_API virtual Vector2d keyNormalized(int i) const;
  // 3 rotation, 3 translation, 1 focal length, 2 radial distortion
  YASFM_API virtual void setParams(const vector<double>& params);
  // from projection matrix
  YASFM_API virtual void setParams(const Matrix34d& P);
  // try constraints 0 and weights 100
  YASFM_API virtual void constrainRadial(double *constraints,double *weigthts);

  // accessors
  // 3 rotation, 3 translation, 1 focal length, 2 radial distortion
  YASFM_API virtual void params(vector<double> *params) const;
  YASFM_API virtual const double* const radParams() const;

protected:
  array<double,2> radParams_;
  array<double,4> invRadParams_;
  static const int nParams_ = 9;
  static const int radIdx_ = 7;

private:
  template<typename T>
  void projectWithExternalParams(const T* const camera,
    const T* const point,T *projection) const;

  struct ReprojectionErrorFunctor
  {
    ReprojectionErrorFunctor(double keyX,double keyY,const StandardCameraRadial& cam);

    template <typename T>
    bool operator()(const T* const camera,
      const T* const point,
      T* residuals) const;

    const StandardCameraRadial& cam_;
    double keyX_,keyY_;
  };
};

typedef struct
{
  vector<IntPair> matches;
  vector<double> dists; // can be any score - the smaller the better
} CameraPair;

// Class including point data as well as n-view matches which
// have not been reconstructed yet. For adding points, prefer
// member functions which handle removal of corresponding
// matchesToReconstruct.
class Points
{
public:
  typedef struct
  {
    NViewMatch reconstructed;
    NViewMatch toReconstruct;
  } PointData;

  // Add new points created from corresponding matchesToReconstructIdxs.
  // The corresponding matchesToReconstruct are erased so NOTE that indices
  // to matchesToReconstruct you might have get invalidated.
  YASFM_API void addPoints(const IntPair& camsIdxs,const vector<int>& matchesToReconstructIdxs,
    const vector<Vector3d>& coord);
  // Add new points. pointCoord and pointViews should have the same size.
  YASFM_API void addPoints(const vector<Vector3d>& pointCoord,
    const vector<SplitNViewMatch>& pointViews);
  // This invalidates the indices to points you might have.
  YASFM_API void removePoints(const vector<bool>& keep);
  YASFM_API int numPts() const;
  // Goes through pointData and updates reconstructed and toReconstruct
  // by moving the cam for all corresponding points from toReconstruct
  // to reconstructed.
  YASFM_API void markCamAsReconstructed(int camIdx);
  // Goes through pointData and updates reconstructed and toReconstruct
  // by moving the cam for all correspondingPoints[correspondingPointsInliers[i]] 
  // from toReconstruct to reconstructed and removing the cam from toReconstruct
  // for all non-inliers. When correspondingPointsInliers is null then
  // all correspondingPoints are considered to be inliers.
  YASFM_API void markCamAsReconstructed(int camIdx,
    const vector<int>& correspondingPoints,
    const vector<int>& correspondingPointsInliers);

  // Accessors
  YASFM_API const vector<NViewMatch>& matchesToReconstruct() const;
  YASFM_API vector<NViewMatch>& matchesToReconstruct();
  YASFM_API const vector<Vector3d>& ptCoord() const;
  // Points are stored in a contiguous block so getting pointer to first
  // points also gives you all the points.
  YASFM_API double* ptCoord(int ptIdx);
  YASFM_API const vector<PointData>& ptData() const;
private:
  vector<NViewMatch> matchesToReconstruct_;
  vector<Vector3d> ptCoord_;
  vector<PointData> ptData_;
};

class Dataset
{
public:
  YASFM_API Dataset(const string& dir);
  YASFM_API Dataset(const Dataset& o);
  YASFM_API Dataset& operator=(const Dataset& o);
  template<class T>
  void addCamera(const string& filename,bool isSubdir = true);
  template<class T>
  void addCameras(const string& imgsDir,bool isSubdir = true);
  // remove descriptors from memory
  YASFM_API void clearDescriptors();
  YASFM_API int numCams() const;
  // This also calls markCamAsReconstructed in points.
  YASFM_API void markCamAsReconstructed(int camIdx);
  // This also calls markCamAsReconstructed in points. When 
  // correspondingPointsInliers is null then all correspondingPoints 
  // are considered to be inliers.
  YASFM_API void markCamAsReconstructed(int camIdx,
    const vector<int>& correspondingPoints,
    const vector<int>& correspondingPointsInliers);

  // accessors
  YASFM_API const string& dir() const;
  YASFM_API const Camera& cam(int idx) const;
  YASFM_API Camera& cam(int idx);
  YASFM_API const Camera& cam(size_t idx) const;
  YASFM_API Camera& cam(size_t idx);
  YASFM_API const ptr_vector<Camera>& cams() const;
  YASFM_API ptr_vector<Camera>& cams();
  YASFM_API const pair_umap<CameraPair>& pairs() const;
  YASFM_API pair_umap<CameraPair>& pairs();
  YASFM_API const uset<int>& reconstructedCams() const;
  YASFM_API const Points& points() const;
  YASFM_API Points& points();
private:
  // copyIn is used in copy constructor and assignment operator.
  // This is implemented so that ptr_vector<Camera> would correctly 
  // copy (unique_ptr<Base> has to be copied using clone()).
  // !!! Update this whenever you add a new member variable.
  void copyIn(const Dataset& o);
  string dir_;
  ptr_vector<Camera> cams_;
  pair_umap<CameraPair> pairs_;
  uset<int> reconstructedCams_;
  Points points_;
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

template<typename T>
void StandardCameraRadial::projectWithExternalParams(const T* const camera,
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

  T r2 = xp*xp + yp*yp;
  T distortion = T(1.) + r2 * (camera[radIdx_ + 0] + r2 * camera[radIdx_ + 1]);

  // Multiply by calibration matrix, i.e., apply focal and principal point.
  projection[0] = camera[fIdx_] * distortion * xp + T(x0_(0));
  projection[1] = camera[fIdx_] * distortion * yp + T(x0_(1));
}

template<typename T>
bool StandardCameraRadial::ReprojectionErrorFunctor::operator()(const T* const camera,
  const T* const point,T* residuals) const
{
  T projection[2];
  cam_.projectWithExternalParams(camera,point,projection);

  residuals[0] = projection[0] - T(keyX_);
  residuals[1] = projection[1] - T(keyY_);

  return true;
}

template<class T>
void Dataset::addCamera(const string& filename,bool isSubdir)
{
  string fnAbs;
  if(isSubdir)
    fnAbs = joinPaths(dir_,filename);
  else
    fnAbs = filename;

  cams_.push_back(make_unique<T>(fnAbs));
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
    cams_.push_back(make_unique<T>(joinPaths(imgsDirAbs,filenames[i])));
  }
}

} // namespace yasfm