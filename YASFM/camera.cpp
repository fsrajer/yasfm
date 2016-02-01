#include "camera.h"

#include <direct.h>
#include <iostream>
#include <fstream>
#include <iomanip>
#include <mutex>

#include "zlib/zlib.h"

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
size_t Camera::maxDescrInMemoryTotal_ = 4000000;

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
  featsFilename_ = joinPaths(featuresDir,fn.substr(0,dotPos) + ".feat.gz");

  getImgDims(imgFilename,&imgWidth_,&imgHeight_);
}

Camera::Camera(istream& file)
{
  file >> imgFilename_;
  file >> imgWidth_ >> imgHeight_;
  file >> featsFilename_;

  readFeatures(Camera::ReadKeys);

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

void Camera::setFeaturesFilename(const string& filename,bool readKeys)
{
  keys_.clear();
  keysScales_.clear();
  keysOrientations_.clear();
  clearDescriptors();
  keysColors_.clear();

  featsFilename_ = filename;
  if(readKeys)
  {
    readFeatures(ReadMode::ReadKeys);
    readKeysColors();
  }
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
  gzFile file = gzopen(featsFilename_.c_str(),"wb"); // wb = write binary

  if(!file)
  {
    YASFM_PRINT_ERROR_FILE_OPEN(featsFilename_);
    return;
  }

  int nKeys = static_cast<int>(keys_.size());
  int dim = static_cast<int>(descr_.rows());
  gzwrite(file,(void*)(&nKeys),sizeof(int));
  gzwrite(file,(void*)(&dim),sizeof(int));

  for(size_t i = 0; i < keys_.size(); i++)
  {
    float x = float(keys_[i](0));
    float y = float(keys_[i](1));
    float scale = float(keysScales_[i]);
    float ori = float(keysOrientations_[i]);

    gzwrite(file,(void*)(&y),sizeof(float));
    gzwrite(file,(void*)(&x),sizeof(float));
    gzwrite(file,(void*)(&scale),sizeof(float));
    gzwrite(file,(void*)(&ori),sizeof(float));
  }

  for(int i = 0; i < nKeys; i++)
  {
    for(int j = 0; j < dim; j++)
    {
      unsigned int tmpui = (unsigned int)floor(0.5+512.0*descr_(j,i));
      unsigned char tmpuc = (unsigned char)std::min((unsigned int)UCHAR_MAX,tmpui);
      gzwrite(file,(void*)(&tmpuc),sizeof(unsigned char));
    }
  }

  gzclose(file);
}

void Camera::readFeatures(int mode)
{
  // Format is:
  // 1) compressed in .feat.gz 
  // 2) old full binary in .sft
  // 3) classic ASCII .key
  if(hasExtension(featsFilename_,".feat.gz"))
  {
    readFeaturesFeatGz(mode);
  } else if(hasExtension(featsFilename_,".sft"))
  {
    readFeaturesSft(mode);
  } else if(hasExtension(featsFilename_,".key"))
  {
    readFeaturesKey(mode);
  } else
  {
    YASFM_PRINT_ERROR("Unsupported features file format:\n" << featsFilename_);
  }
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

void Camera::readFeaturesFeatGz(int mode)
{
  gzFile file = gzopen(featsFilename_.c_str(),"rb"); // rb = read binary

  if(!file)
  {
    YASFM_PRINT_ERROR_FILE_OPEN(featsFilename_);
    return;
  }

  int nKeys,dim;
  gzread(file,(void*)(&nKeys),sizeof(int));
  gzread(file,(void*)(&dim),sizeof(int));

  if(mode & ReadKeys)
  {
    keys_.resize(nKeys);
    keysScales_.resize(nKeys);
    keysOrientations_.resize(nKeys);
  }

  if(mode & ReadDescriptors)
    allocAndRegisterDescr(nKeys,dim);

  float tmp[4];
  if(mode & ReadKeys)
  {
    for(int i = 0; i < nKeys; i++)
    {
      gzread(file,(void*)(&tmp[0]),4*sizeof(float));
      keys_[i](0) = tmp[1];
      keys_[i](1) = tmp[0];
      keysScales_[i] = tmp[2];
      keysOrientations_[i] = tmp[3];
    }
  } else
  {
    for(int i = 0; i < nKeys; i++)
      gzread(file,(void*)(&tmp[0]),4*sizeof(float));
  }

  if(mode & ReadDescriptors)
  {
    for(int i = 0; i < nKeys; i++)
    {
      for(int j = 0; j < dim; j++)
      {
        unsigned char tmp;
        gzread(file,(void*)(&tmp),sizeof(unsigned char));
        descr_(j,i) = tmp;
      }
    }
    descr_.colwise().normalize();
  }

  gzclose(file);
}

void Camera::readFeaturesSft(int mode)
{
  ifstream featuresFile(featsFilename_,std::ios::binary);
  if(!featuresFile.is_open())
  {
    YASFM_PRINT_ERROR_FILE_OPEN(featsFilename_);
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

void Camera::readFeaturesKey(int mode)
{
  ifstream file(featsFilename_);
  if(!file.is_open())
  {
    YASFM_PRINT_ERROR_FILE_OPEN(featsFilename_);
    return;
  }

  int nKeys,descrDim;
  file >> nKeys >> descrDim;

  if(mode & ReadKeys)
  {
    keys_.resize(nKeys);
    keysScales_.resize(nKeys);
    keysOrientations_.resize(nKeys);
  }
  if(mode & ReadDescriptors)
    allocAndRegisterDescr(nKeys,descrDim);

  double dummy;
  for(int i = 0; i < nKeys; i++)
  {
    if(mode & ReadKeys)
      file >> keys_[i](1) >> keys_[i](0) >> keysScales_[i] >> keysOrientations_[i];
    else
      file >> dummy >> dummy >> dummy >> dummy;

    if(mode & ReadDescriptors)
    {
      for(int d = 0; d < descrDim; d++)
        file >> descr_(d,i);
    } else
    {
      int nLines = static_cast<int>(ceil(descrDim/20.)) + 1;
      for(int j = 0; j < nLines; j++)
      {
        string s;
        std::getline(file,s);
      }
    }
  }

  if(mode & ReadDescriptors)
    descr_.colwise().normalize();

  file.close();
}

} // namespace yasfm