#include "standard_camera.h"

#include "utils.h"

using Eigen::Map;
using std::make_unique;

namespace yasfm
{

string StandardCamera::className() const
{
  return "StandardCamera";
}

CameraRegister<StandardCamera> StandardCamera::reg_("StandardCamera");

StandardCamera::StandardCamera(const string& imgFilename)
  : Camera(imgFilename),rot_(0.,Vector3d::UnitX()),C_(Vector3d::Zero()),
  f_(0),paramsConstraints_(nParams_,0.),paramsConstraintsWeights_(nParams_,0.)
{
  // assume the image center to be the principal point
  x0_(0) = 0.5 * (imgWidth() - 1);
  x0_(1) = 0.5 * (imgHeight() - 1);
}

StandardCamera::StandardCamera(istream& file,int readMode,const string& featuresDir)
  : Camera(file,readMode,featuresDir)
{
  file >> rot_.angle() >> rot_.axis()(0) >> rot_.axis()(1) >> rot_.axis()(2);
  file >> C_(0) >> C_(1) >> C_(2);
  file >> f_;
  file >> x0_(0) >> x0_(1);
  int nConstraints;
  file >> nConstraints;
  paramsConstraints_.resize(nConstraints);
  paramsConstraintsWeights_.resize(nConstraints);
  for(int i = 0; i < nConstraints; i++)
  {
    file >> paramsConstraints_[i] >> paramsConstraintsWeights_[i];
  }
}

StandardCamera::~StandardCamera()
{

}

unique_ptr<Camera> StandardCamera::clone() const
{
  return make_unique<StandardCamera>(*this);
}

Vector2d StandardCamera::project(const Vector3d& pt) const
{
  Vector2d ptCam = (rot_ * (pt - C_)).hnormalized();
  return f_ * ptCam + x0_;
}

ceres::CostFunction* StandardCamera::costFunction(int keyIdx) const
{
  const auto& k = key(keyIdx);
  return (new ceres::AutoDiffCostFunction<ReprojectionErrorFunctor,2,nParams_,3>(
    new ReprojectionErrorFunctor(k(0),k(1),*this)));
}

ceres::CostFunction* StandardCamera::constraintsCostFunction() const
{
  return generateConstraintsCostFunction<nParams_>(&paramsConstraints_[0],
    &paramsConstraintsWeights_[0]);
}

void StandardCamera::params(vector<double> *pparams) const
{
  auto& params = *pparams;
  params.resize(nParams_);
  Map<Vector3d> rot(&params[rotIdx_]),C(&params[CIdx_]);
  rot = rot_.angle() * rot_.axis();
  C = C_;
  params[fIdx_] = f_;
}

Vector2d StandardCamera::keyNormalized(int i) const
{
  return (key(i) - x0_) / f_;
}

Matrix34d StandardCamera::P() const
{
  Matrix34d out(Matrix34d::Identity());
  out.rightCols(1) = -C_;
  out = K()*rot_.toRotationMatrix()*out;
  return out;
}

Matrix3d StandardCamera::K() const
{
  Matrix3d K(Matrix3d::Identity());
  K(0,0) = f();
  K(1,1) = f();
  K.block(0,2,2,1) = x0_;
  return K;
}

Matrix34d StandardCamera::pose() const
{
  Matrix34d tmp(Matrix34d::Identity());
  tmp.col(3) = -C_;
  return R() * tmp;
}

Matrix3d StandardCamera::R() const
{
  return rot_.toRotationMatrix();
}

Vector3d StandardCamera::C() const 
{ 
  return C_; 
}

const AngleAxisd& StandardCamera::rot() const { return rot_; }
double StandardCamera::f() const { return f_; }
const Vector2d& StandardCamera::x0() const { return x0_; }

void StandardCamera::setParams(const vector<double>& params)
{
  Map<const Vector3d> rot(&params[rotIdx_]),C(&params[CIdx_]);
  double angle = rot.squaredNorm();
  Vector3d axis;
  if(angle == 0.)
    axis = Vector3d::UnitX();
  else
  {
    angle = sqrt(angle);
    axis = rot / angle;
  }
  rot_ = AngleAxisd(angle,axis);
  C_ = C;
  f_ = params[fIdx_];
}

void StandardCamera::setParams(const Matrix34d& P)
{
  Matrix3d K,R;
  Vector3d Ctmp;
  P2KRC(P,&K,&R,&Ctmp);

  setFocal(0.5*(K(0,0)+K(1,1)));
  C_ = Ctmp;
  rot_.fromRotationMatrix(R);
}

void StandardCamera::setRotation(const Matrix3d& R)
{
  rot_.fromRotationMatrix(R);
}

void StandardCamera::setC(const Vector3d& C)
{
  C_ = C;
}

void StandardCamera::setFocal(double f)
{
  f_ = f;
}

void StandardCamera::setParamsConstraints(const vector<double>& constraints,
  const vector<double>& weights)
{
  for(size_t i = 0; i < paramsConstraints_.size(); i++)
  {
    paramsConstraints_[i] = constraints[i];
    paramsConstraintsWeights_[i] = weights[i];
  }
}

void StandardCamera::constrainFocal(double constraint,double weigtht)
{
  paramsConstraints_[fIdx_] = constraint;
  paramsConstraintsWeights_[fIdx_] = weigtht;
}

void StandardCamera::writeASCII(ostream& file) const
{
  Camera::writeASCII(file);
  file << rot_.angle() << " "
    << rot_.axis()(0) << " "
    << rot_.axis()(1) << " "
    << rot_.axis()(2) << "\n";
  file << C_(0) << " " << C_(1) << " " << C_(2) << "\n";
  file << f_ << "\n";
  file << x0_(0) << " " << x0_(1) << "\n";
  file << paramsConstraints_.size();
  for(size_t i = 0; i < paramsConstraints_.size(); i++)
  {
    file << " " << paramsConstraints_[i] << " " << paramsConstraintsWeights_[i];
  }
  file << "\n";
}

StandardCamera::ReprojectionErrorFunctor::ReprojectionErrorFunctor(double keyX,
  double keyY,const StandardCamera& cam)
  : keyX_(keyX),keyY_(keyY),cam_(cam)
{
}

} // namespace yasfm