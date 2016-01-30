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

StandardCamera::StandardCamera()
	: Camera(),params_(nParams_,0.),paramsConstraints_(nParams_, 0.), 
  paramsConstraintsWeights_(nParams_, 0.)
{
	x0_(0) = -1;
	x0_(1) = -1;
}

StandardCamera::StandardCamera(const string& imgFilename,const string& featuresDir)
  : Camera(imgFilename,featuresDir),params_(nParams_,0.),
  paramsConstraints_(nParams_,0.),paramsConstraintsWeights_(nParams_,0.)
{
  // assume the image center to be the principal point
  x0_(0) = 0.5 * (imgWidth() - 1);
  x0_(1) = 0.5 * (imgHeight() - 1);
}

StandardCamera::StandardCamera(istream& file)
  : Camera(file)
{
  int n;
  file >> n;
  params_.resize(n);
  paramsConstraints_.resize(n);
  paramsConstraintsWeights_.resize(n);
  for(int i = 0; i < n; i++)
  {
    file >> params_[i] >> paramsConstraints_[i] >> paramsConstraintsWeights_[i];
  }

  file >> x0_(0) >> x0_(1);
}

StandardCamera::~StandardCamera()
{

}

unique_ptr<Camera> StandardCamera::clone() const
{
  return make_unique<StandardCamera>(*this);
}

Vector2d StandardCamera::project(const Point& pt) const
{
  Vector2d proj;
  projectWithExternalParams(&params_[0],&pt.coord(0),&proj(0));
  return proj;
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
  *pparams = params_;
}

Vector2d StandardCamera::keyNormalized(int i) const
{
  return (key(i) - x0_) / f();
}

Matrix34d StandardCamera::P() const
{
  return K()*pose();
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
  Map<const Vector3d> C_(&params_[CIdx_]);
  Matrix34d tmp(Matrix34d::Identity());
  tmp.col(3) = -C_;
  return R() * tmp;
}

Matrix3d StandardCamera::R() const
{
  Map<const Vector3d> angleaxis(&params_[rotIdx_]);
  double angle = angleaxis.squaredNorm();
  Vector3d axis;
  if(angle == 0.)
    axis = Vector3d::UnitX();
  else
  {
    angle = sqrt(angle);
    axis = angleaxis / angle;
  }
  AngleAxisd rot(angle,axis);

  return rot.toRotationMatrix();
}

Vector3d StandardCamera::C() const 
{ 
  return Map<const Vector3d>(&params_[CIdx_]);
}

double StandardCamera::f() const { return params_[fIdx_]; }
const Vector2d& StandardCamera::x0() const { return x0_; }

void StandardCamera::setImage(const string& filename,int width,int height)
{
  Camera::setImage(filename,width,height);
  // assume the image center to be the principal point
  x0_(0) = 0.5 * (imgWidth() - 1);
  x0_(1) = 0.5 * (imgHeight() - 1);
}

void StandardCamera::setParams(const vector<double>& params)
{
  params_[rotIdx_ + 0] = params[rotIdx_ + 0];
  params_[rotIdx_ + 1] = params[rotIdx_ + 1];
  params_[rotIdx_ + 2] = params[rotIdx_ + 2];
  params_[CIdx_ + 0] = params[CIdx_ + 0];
  params_[CIdx_ + 1] = params[CIdx_ + 1];
  params_[CIdx_ + 2] = params[CIdx_ + 2];
  params_[fIdx_] = params[fIdx_];
}

void StandardCamera::setParams(const Matrix34d& P)
{
  Matrix3d K,R;
  Vector3d Ctmp;
  P2KRC(P,&K,&R,&Ctmp);

  setFocal(0.5*(K(0,0)+K(1,1)));
  setC(Ctmp);
  setRotation(R);
}

void StandardCamera::setRotation(const Matrix3d& R)
{
  AngleAxisd aa;
  aa.fromRotationMatrix(R);

  Map<Vector3d> aa_(&params_[rotIdx_]);
  aa_ = aa.angle() * aa.axis();
}

void StandardCamera::setC(const Vector3d& C)
{
  Map<Vector3d> C_(&params_[CIdx_]);
  C_ = C;
}

void StandardCamera::setFocal(double f)
{
  params_[fIdx_] = f;
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

  file << params_.size() << "\n";
  for(size_t i = 0; i < params_.size(); i++)
  {
    file << params_[i] << " " 
      << paramsConstraints_[i] << " " 
      << paramsConstraintsWeights_[i] << "\n";
  }

  file << x0_(0) << " " << x0_(1) << "\n";
}

StandardCamera::ReprojectionErrorFunctor::ReprojectionErrorFunctor(double keyX,
  double keyY,const StandardCamera& cam)
  : keyX_(keyX),keyY_(keyY),cam_(cam)
{
}

} // namespace yasfm