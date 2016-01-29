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
  for(int i = 0; i < 2; i++)
  {
    params_.push_back(0.);
    paramsConstraints_.push_back(0.);
    paramsConstraintsWeights_.push_back(0.);
  }
	invRadParams_.fill(0.);
}

StandardCameraRadial::StandardCameraRadial(const string& imgFilename,
  const string& featuresDir)
  : StandardCamera(imgFilename,featuresDir)
{
  for(int i = 0; i < 2; i++)
  {
    params_.push_back(0.);
    paramsConstraints_.push_back(0.);
    paramsConstraintsWeights_.push_back(0.);
  }
  invRadParams_.fill(0.);
}

StandardCameraRadial::StandardCameraRadial(istream& file)
  : StandardCamera(file)
{  
  file >> invRadParams_[0] 
    >> invRadParams_[1] 
    >> invRadParams_[2]
    >> invRadParams_[3];
}

StandardCameraRadial::~StandardCameraRadial()
{
}

unique_ptr<Camera> StandardCameraRadial::clone() const
{
  return make_unique<StandardCameraRadial>(*this);
}

Vector2d StandardCameraRadial::project(const Point& pt) const
{
  Vector2d proj;
  projectWithExternalParams(&params_[0],&pt.coord(0),&proj(0));
  return proj;
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
  params_[radIdx_ + 0] = params[radIdx_ + 0];
  params_[radIdx_ + 1] = params[radIdx_ + 1];

  // update inverse parameters
  array<double,4> radParamsFull = {0.,params_[radIdx_ + 0],0.,params_[radIdx_ + 1]};
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
  params_[radIdx_ + 0] = 0.;
  params_[radIdx_ + 1] = 0.;
  invRadParams_.fill(0.);
}

void StandardCameraRadial::constrainRadial(double *constraints,double *weights)
{
  paramsConstraints_[radIdx_ + 0] = constraints[0];
  paramsConstraints_[radIdx_ + 1] = constraints[1];
  paramsConstraintsWeights_[radIdx_ + 0] = weights[0];
  paramsConstraintsWeights_[radIdx_ + 1] = weights[1];
}

const double* StandardCameraRadial::radParams() const { return &params_[radIdx_]; }

void StandardCameraRadial::writeASCII(ostream& file) const
{
  StandardCamera::writeASCII(file);
  file << invRadParams_[0] << " "
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