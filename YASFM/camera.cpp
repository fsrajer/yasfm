#include "camera.h"

#include <direct.h>
#include <iostream>
#include <fstream>
#include <iomanip>
#include <mutex>

#include "utils_io.h"

using Eigen::VectorXf;
using Eigen::Map;
using std::cerr;
using std::cout;
using std::setprecision;
using std::ofstream;
using std::recursive_mutex;
using std::unique_lock;

namespace yasfm
{

recursive_mutex mtx; ///< For locking the static variables.
size_t Camera::nDescrInMemoryTotal_ = 0;
list<Camera *> Camera::camsWithLoadedDescr_;
size_t Camera::maxDescrInMemoryTotal_ = 5000000;

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
  featsFilename_ = joinPaths(featuresDir,fn.substr(0,dotPos) + ".sft");

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

Camera::Camera(const Camera& o)
{
  copyIn(o);
}

Camera::~Camera()
{
  // This has to be called in order to mark the descriptors as released.
  clearDescriptors();
}

Camera& Camera::operator=(const Camera& o)
{
  copyIn(o);
  return *this;
}

void Camera::copyIn(const Camera& o)
{
  imgFilename_ = o.imgFilename_;
  imgWidth_ = o.imgWidth_;
  imgHeight_ = o.imgHeight_;
  featsFilename_ = o.featsFilename_;
  keys_ = o.keys_;
  keysScales_ = o.keysScales_;
  keysOrientations_ = o.keysOrientations_;
  keysColors_ = o.keysColors_;
  if(o.descr_.cols() > 0)
    allocAndRegisterDescr(int(o.descr_.cols()),int(o.descr_.rows()));
  descr_ = o.descr_;
  visiblePoints_ = o.visiblePoints_;
}

void Camera::resizeFeatures(int num,int dim)
{
  keys_.resize(num);
  keysScales_.resize(num);
  keysOrientations_.resize(num);
  allocAndRegisterDescr(num,dim);
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
  unique_lock<recursive_mutex> lck(mtx);
  nDescrInMemoryTotal_ -= descr_.cols();
  camsWithLoadedDescr_.remove(this);
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
const vector<Vector3uc>& Camera::keysColors() const { return keysColors_; }
const Vector3uc& Camera::keyColor(int i) const { return keysColors_[i]; }
const vector<int>& Camera::visiblePoints() const { return visiblePoints_; }
vector<int>& Camera::visiblePoints() { return visiblePoints_; }

const MatrixXf& Camera::descr() 
{ 
  if(descr_.cols() == 0)
    readFeatures(ReadDescriptors);

  return descr_; 
}
void Camera::writeASCII(ostream& file) const
{
  file << imgFilename_ << "\n";
  file << imgWidth_ << " " << imgHeight_ << "\n";
  file << featsFilename_ << "\n";
}

void Camera::writeFeatures() const
{
  ofstream featuresFile(featsFilename_,std::ios::binary);
  if(!featuresFile.is_open())
  {
    cerr << "ERROR: Camera::writeFeatures: unable to open: " << featsFilename_
      << " for writing\n";
    return;
  }

  int nKeys = static_cast<int>(keys_.size());
  int dim = static_cast<int>(descr_.rows());
  featuresFile.write((char*)(&nKeys),sizeof(int));
  featuresFile.write((char*)(&dim),sizeof(int));

  for(size_t i = 0; i < keys_.size(); i++)
  {
    float x = float(keys_[i](0));
    float y = float(keys_[i](1));
    float scale = float(keysScales_[i]);
    float ori = float(keysOrientations_[i]);

    featuresFile.write((char*)(&y),sizeof(float));
    featuresFile.write((char*)(&x),sizeof(float));
    featuresFile.write((char*)(&scale),sizeof(float));
    featuresFile.write((char*)(&ori),sizeof(float));
  }

  for(size_t i = 0; i < keys_.size(); i++)
  {
    featuresFile.write((char*)(&descr_(0,i)),dim*sizeof(float));
  }

  featuresFile.close();
}

void Camera::readFeatures(int mode)
{
  ifstream featuresFile(featsFilename_,std::ios::binary);
  if(!featuresFile.is_open())
  {
    cerr << "ERROR: Camera::readFeatures: unable to open: " << featsFilename_
      << " for reading\n";
    return;
  }

  int nKeys,descrDim;
  featuresFile.read((char*)(&nKeys),sizeof(int));
  featuresFile.read((char*)(&descrDim),sizeof(int));

  if(mode & ReadKeys)
  {
    keys_.resize(nKeys);
    keysScales_.resize(nKeys);
    keysOrientations_.resize(nKeys);
  }
  if(mode & ReadDescriptors)
    allocAndRegisterDescr(nKeys,descrDim); 

  float tmp[4];
  if(mode & ReadKeys)
  {
    for(int i = 0; i < nKeys; i++)
    {
      featuresFile.read((char*)(&tmp[0]),4*sizeof(float));
      keys_[i](0) = tmp[1];
      keys_[i](1) = tmp[0];
      keysScales_[i] = tmp[2];
      keysOrientations_[i] = tmp[3];
    }
  } else
  {
    for(int i = 0; i < nKeys; i++)
      featuresFile.read((char*)(&tmp[0]),4*sizeof(float));
  }

  if(mode & ReadDescriptors)
  {
    for(int i = 0; i < nKeys; i++)
      featuresFile.read((char*)(&descr_(0,i)),descrDim*sizeof(float));
  }

  featuresFile.close();
}

void Camera::allocAndRegisterDescr(int num,int dim)
{
  unique_lock<recursive_mutex> lck(mtx);
  while(nDescrInMemoryTotal_ > maxDescrInMemoryTotal_)
  {
    camsWithLoadedDescr_.front()->clearDescriptors();
  }
  nDescrInMemoryTotal_ += num;
  camsWithLoadedDescr_.push_back(this);
  descr_.resize(dim,num);
}

} // namespace yasfm