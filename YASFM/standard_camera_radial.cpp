#include "standard_camera_radial.h"

#include "utils.h"

using std::make_unique;

namespace yasfm
{

string StandardCameraRadial::className() const
{
  return "StandardCameraRadial";
}

CameraRegister<StandardCameraRadial> StandardCameraRadial::reg_("StandardCameraRadial");

StandardCameraRadial::StandardCameraRadial()
	: StandardCamera()
{
	paramsConstraints_.push_back(0.);
	paramsConstraints_.push_back(0.);
	paramsConstraintsWeights_.push_back(0.);
	paramsConstraintsWeights_.push_back(0.);
	radParams_[0] = 0.;
	radParams_[1] = 0.;
	invRadParams_.fill(0.);
}

StandardCameraRadial::StandardCameraRadial(const string& imgFilename)
  : StandardCamera(imgFilename)
{
  paramsConstraints_.push_back(0.);
  paramsConstraints_.push_back(0.);
  paramsConstraintsWeights_.push_back(0.);
  paramsConstraintsWeights_.push_back(0.);
  radParams_[0] = 0.;
  radParams_[1] = 0.;
  invRadParams_.fill(0.);
}

StandardCameraRadial::StandardCameraRadial(istream& file,int readMode,
  const string& featuresDir)
  : StandardCamera(file,readMode,featuresDir)
{
  int n;
  file >> n >> radParams_[0] >> radParams_[1];
  file >> n >> invRadParams_[0] >> invRadParams_[1] >> invRadParams_[2]
    >> invRadParams_[3];
}

StandardCameraRadial::~StandardCameraRadial()
{
}

unique_ptr<Camera> StandardCameraRadial::clone() const
{
  return make_unique<StandardCameraRadial>(*this);
}

Vector2d StandardCameraRadial::project(const Vector3d& pt) const
{
  Vector2d ptCam = (rot_ * (pt - C_)).hnormalized();
  double r2 = ptCam.squaredNorm();
  double distortion = 1. + r2 * (radParams_[0] + r2 * radParams_[1]);
  return f_ * distortion * ptCam + x0_;
}

ceres::CostFunction* StandardCameraRadial::costFunction(int keyIdx) const
{
  const auto& k = key(keyIdx);
  return (new ceres::AutoDiffCostFunction<ReprojectionErrorFunctor,2,nParams_,3>(
    new ReprojectionErrorFunctor(k(0),k(1),*this)));
}

ceres::CostFunction* StandardCameraRadial::constraintsCostFunction() const
{
  return generateConstraintsCostFunction<nParams_>(&paramsConstraints_[0],
    &paramsConstraintsWeights_[0]);
}

void StandardCameraRadial::params(vector<double> *pparams) const
{
  auto& params = *pparams;
  StandardCamera::params(pparams);
  params.push_back(radParams_[0]);
  params.push_back(radParams_[1]);
}

Vector2d StandardCameraRadial::keyNormalized(int i) const
{
  Vector2d distorted = StandardCamera::keyNormalized(i);
  double radius = distorted.norm();
  double undistortFactor = 1. + radius *
    (invRadParams_[0] + radius *
    (invRadParams_[1] + radius *
    (invRadParams_[2] + radius * invRadParams_[3])));
  return undistortFactor*distorted;
}

void StandardCameraRadial::setParams(const vector<double>& params)
{
  StandardCamera::setParams(params);
  radParams_[0] = params[radIdx_ + 0];
  radParams_[1] = params[radIdx_ + 1];

  // update inverse parameters
  array<double,4> radParamsFull = {0.,radParams_[0],0.,radParams_[1]};
  double xMax = imgWidth() - x0_(0);
  double yMax = imgHeight() - x0_(1);
  double maxRadius = sqrt(xMax*xMax + yMax*yMax);
  int nForward = static_cast<int>(radParamsFull.size());
  int nInverse = static_cast<int>(invRadParams_.size());
  approximateInverseRadialDistortion(nForward,nInverse,maxRadius,&radParamsFull[0],
    &invRadParams_[0]);
}

void StandardCameraRadial::setParams(const Matrix34d& P)
{
  StandardCamera::setParams(P);
  radParams_[0] = 0.;
  radParams_[1] = 0.;
  invRadParams_.fill(0.);
}

void StandardCameraRadial::constrainRadial(double *constraints,double *weights)
{
  paramsConstraints_[radIdx_ + 0] = constraints[0];
  paramsConstraints_[radIdx_ + 1] = constraints[1];
  paramsConstraintsWeights_[radIdx_ + 0] = weights[0];
  paramsConstraintsWeights_[radIdx_ + 1] = weights[1];
}

const double* StandardCameraRadial::radParams() const { return &radParams_[0]; }

void StandardCameraRadial::writeASCII(ostream& file) const
{
  StandardCamera::writeASCII(file);
  file << "2 " << radParams_[0] << " " << radParams_[1] << "\n";
  file << "4 " << invRadParams_[0] << " "
    << invRadParams_[1] << " "
    << invRadParams_[2] << " "
    << invRadParams_[3] << "\n";
}

StandardCameraRadial::ReprojectionErrorFunctor::ReprojectionErrorFunctor(double keyX,
  double keyY,const StandardCameraRadial& cam)
  : keyX_(keyX),keyY_(keyY),cam_(cam)
{
}

} // namespace yasfm