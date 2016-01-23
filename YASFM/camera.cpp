#include "camera.h"

#include <direct.h>
#include <iostream>
#include <fstream>
#include <iomanip>

#include "utils_io.h"

using Eigen::VectorXf;
using Eigen::Map;
using std::cerr;
using std::cout;
using std::setprecision;
using std::ofstream;

namespace yasfm
{

Camera::Camera()
{
	imgWidth_ = -1;
	imgHeight_ = -1;
}

Camera::Camera(const string& imgFilename,const string& featuresDir)
  : imgFilename_(imgFilename),imgWidth_(-1),imgHeight_(-1)
{
  string fn = extractFilename(imgFilename);
  size_t dotPos = fn.find_last_of(".");
  featsFilename_ = joinPaths(featuresDir,fn.substr(0,dotPos) + ".key");

  getImgDims(imgFilename,&imgWidth_,&imgHeight_);
}

Camera::Camera(istream& file,int mode)
{
  file >> imgFilename_;
  file >> imgWidth_ >> imgHeight_;
  file >> featsFilename_;

  readFeatures(mode);

  readKeysColors();
}

Camera::~Camera()
{
}

void Camera::resizeFeatures(int num,int dim)
{
  keys_.resize(num);
  keysScales_.resize(num);
  keysOrientations_.resize(num);
  descr_.resize(dim,num);
}

void Camera::setFeature(int idx,double x,double y,double scale,double orientation,
  const float* const descr)
{
  keys_[idx](0) = x;
  keys_[idx](1) = y;
  keysScales_[idx] = scale;
  keysOrientations_[idx] = orientation;
  Map<const VectorXf> descrMapped(descr,descr_.rows());
  descr_.col(idx) = descrMapped;
}

void Camera::readKeysColors()
{
  readColors(imgFilename(),keys_,&keysColors_);
}

void Camera::clearDescriptors()
{
  descr_.resize(0,0);
}

void Camera::setImage(const string& filename,int width,int height)
{
  imgFilename_ = filename;
  imgWidth_ = width;
  imgHeight_ = height;
}

const string& Camera::imgFilename() const { return imgFilename_; }
int Camera::imgWidth() const { return imgWidth_; }
int Camera::imgHeight() const { return imgHeight_; }
const vector<Vector2d>& Camera::keys() const { return keys_; }
const Vector2d& Camera::key(int i) const { return keys_[i]; }
Vector2d& Camera::key(int i)  { return keys_[i]; }
const vector<double>& Camera::keysScales() const { return keysScales_; }
const vector<double>& Camera::keysOrientations() const { return keysOrientations_; }
const MatrixXf& Camera::descr() const { return descr_; }
const vector<Vector3uc>& Camera::keysColors() const { return keysColors_; }
const Vector3uc& Camera::keyColor(int i) const { return keysColors_[i]; }
const vector<int>& Camera::visiblePoints() const { return visiblePoints_; }
vector<int>& Camera::visiblePoints() { return visiblePoints_; }

void Camera::writeASCII(ostream& file) const
{
  file << imgFilename_ << "\n";
  file << imgWidth_ << " " << imgHeight_ << "\n";
  file << featsFilename_ << "\n";
}

void Camera::writeFeatures(bool convertNormalizedToUInt) const
{
  ofstream featuresFile(featsFilename_);
  if(!featuresFile.is_open())
  {
    cerr << "ERROR: Camera::writeFeatures: unable to open: " << featsFilename_
      << " for writing\n";
    return;
  }
  featuresFile.flags(std::ios::fixed);
  featuresFile << keys_.size() << " " << descr_.rows() << "\n";
  for(size_t i = 0; i < keys_.size(); i++)
  {
    // save in format y, x, 0, 0
    featuresFile << setprecision(2) << keys_[i](1) << " " << keys_[i](0) << " "
      << keysScales_[i] << " " << keysOrientations_[i] << "\n";

    featuresFile << setprecision(8);
    for(int r = 0; r < descr_.rows(); r++)
    {
      if(convertNormalizedToUInt)
        featuresFile << " " << ((unsigned int)floor(0.5+512.0f*descr_(r,i)));
      else
        featuresFile << " " << descr_(r,i);

      if((r+1) % 20 == 0)
        featuresFile << "\n";
    }
    featuresFile << "\n";
  }
  featuresFile.close();
}

void Camera::readFeatures(int mode)
{
  ifstream featuresFile(featsFilename_);
  if(!featuresFile.is_open())
  {
    cerr << "ERROR: Camera::readFeatures: unable to open: " << featsFilename_
      << " for reading\n";
    return;
  }
  int nKeys,descrDim;
  featuresFile >> nKeys >> descrDim;

  if(mode & ReadKeys)
  {
    keys_.resize(nKeys);
    keysScales_.resize(nKeys);
    keysOrientations_.resize(nKeys);
  }
  if(mode & ReadDescriptors)
    descr_.resize(descrDim,nKeys);

  for(int i = 0; i < nKeys; i++)
  {
    if(mode & ReadKeys)
    {
      featuresFile >> keys_[i](1) >> keys_[i](0) >> keysScales_[i] >> keysOrientations_[i];
    } else
    {
      double dummy;
      featuresFile >> dummy >> dummy >> dummy >> dummy;
    }
    if(mode & ReadDescriptors)
    {
      for(int d = 0; d < descrDim; d++)
      {
        featuresFile >> descr_(d,i);
      }
    } else
    {
      int nLines = static_cast<int>(ceil(descrDim / 20.)) + 1;
      for(int j = 0; j < nLines; j++)
      {
        string s;
        getline(featuresFile,s);
      }
    }
  }
  featuresFile.close();

  if(mode & ReadDescriptors)
    descr_.colwise().normalize();
}

} // namespace yasfm