#include "camera.h"

#include <direct.h>
#include <iostream>
#include <fstream>
#include <iomanip>

#include "utils_io.h"

using Eigen::ArrayXf;
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

Camera::Camera(const string& imgFilename)
  : imgWidth_(-1),imgHeight_(-1),imgFilename_(imgFilename)
{
  imgFilename_ = imgFilename;
  getImgDims(imgFilename,&imgWidth_,&imgHeight_);
}

Camera::Camera(istream& file,int mode,const string& featuresDir)
{
  file >> imgFilename_;
  file >> imgWidth_ >> imgHeight_;

  string fn = featuresFilename(featuresDir);
  ifstream featuresFile(fn);
  if(!featuresFile.is_open())
  {
    cerr << "ERROR: Camera::Camera: unable to open: " << fn << " for reading\n";
    return;
  }
  int nKeys,descrDim;
  featuresFile >> nKeys >> descrDim;

  keys_.resize(nKeys);
  keysScales_.resize(nKeys);
  keysOrientations_.resize(nKeys);
  if(mode & ReadAll)
    descr_.resize(descrDim,nKeys);
  else
    descr_.resize(0,0);

  for(int i = 0; i < nKeys; i++)
  {
    featuresFile >> keys_[i](1) >> keys_[i](0) >> keysScales_[i] >> keysOrientations_[i];
    if(mode & ReadAll)
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

  if(mode & ReadAll)
    descr_.matrix().colwise().normalize();

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
  Map<const ArrayXf> descrMapped(descr,descr_.rows());
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
const ArrayXXf& Camera::descr() const { return descr_; }
const vector<Vector3uc>& Camera::keysColors() const { return keysColors_; }
const Vector3uc& Camera::keyColor(int i) const { return keysColors_[i]; }
const vector<int>& Camera::visiblePoints() const { return visiblePoints_; }
vector<int>& Camera::visiblePoints() { return visiblePoints_; }

void Camera::writeASCII(ostream& file,int writeMode) const
{
  string featuresDir = joinPaths(extractPath(imgFilename()),"keys");
  _mkdir(featuresDir.c_str());
  writeASCII(file,writeMode,featuresDir);
}

void Camera::writeASCII(ostream& file) const
{
  file << imgFilename_ << "\n";
  file << imgWidth_ << " " << imgHeight_ << "\n";
}

void Camera::writeASCII(ostream& file,int mode,
  const string& featuresDir) const
{
  writeASCII(file);
  if(mode & WriteAll)
  {
    string fn = featuresFilename(featuresDir);
    ofstream featuresFile(fn);
    if(!featuresFile.is_open())
    {
      cerr << "ERROR: Camera::writeASCII: unable to open: " << fn << " for writing\n";
      return;
    }
    featuresFile.flags(std::ios::fixed);
    featuresFile << keys_.size() << " ";
    if(mode & WriteDescriptors)
      featuresFile << descr_.rows() << "\n";
    else
      featuresFile << "0\n";
    for(size_t i = 0; i < keys_.size(); i++)
    {
      // save in format y, x, 0, 0
      featuresFile << setprecision(2) << keys_[i](1) << " " << keys_[i](0) << " " 
        << keysScales_[i] << " " << keysOrientations_[i] << "\n";
      if((mode & WriteDescriptors) && descr_.cols() != 0)
      {
        featuresFile << setprecision(8);
        for(int r = 0; r < descr_.rows(); r++)
        {
          if(mode & WriteConvertNormalizedSIFTToUint)
            featuresFile << " " << ((unsigned int)floor(0.5+512.0f*descr_(r,i)));
          else
            featuresFile << " " << descr_(r,i);

          if((r+1) % 20 == 0)
            featuresFile << "\n";
        }
        featuresFile << "\n";
      }
    }
    featuresFile.close();
  }
}

string Camera::featuresFilename(const string& featuresDir) const
{
  string fn = joinPaths(featuresDir,extractFilename(imgFilename()));
  if(fn.size() >= 3)
  {
    fn[fn.size() - 3] = 'k';
    fn[fn.size() - 2] = 'e';
    fn[fn.size() - 1] = 'y';
  }
  return fn;
}

} // namespace yasfm