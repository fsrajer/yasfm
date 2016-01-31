#include "utils_io.h"

#include <climits>
#include <algorithm>
#include <memory>
#include <iostream>
#include <iomanip>
#include <fstream>
#include <sstream>

#include "IL\il.h"
#ifdef _WIN32
#include "dirent\dirent.h"
#else
#include "dirent.h"
#endif
extern "C"
{
  namespace jhead
  {
  #include "jhead\jhead.h"
  extern ImageInfo_t ImageInfo;
  }
}
#include "utils.h"
#include "standard_camera_radial.h"

using std::cout;
using std::cerr;
using std::ifstream;
using std::ofstream;
using std::setprecision;
using std::getline;
using std::istringstream;

namespace yasfm
{

bool hasExtension(const string& filename,const string& extension)
{
  if(filename.length() < extension.length())
    return false;
  string filenameExt = filename.substr(filename.length() - extension.length());
  std::transform(filenameExt.begin(),filenameExt.end(),filenameExt.begin(),::tolower);
  return (filenameExt.compare(extension) == 0);
}

bool hasExtension(const string& filename,const vector<string>& allowedExtensions)
{
  for(const string& ext : allowedExtensions)
  {
    if(hasExtension(filename,ext))
    {
      return true;
    }
  }
  return false;
}

void listImgFilenames(const string& dir,vector<string> *filenames)
{
  vector<string> extensions{"jpg","jpeg","jpe","jp2","png","pgm","gif","bmp","raw",
    "tif","tiff","ico","pbm","cut","dds","doom","exr","hdr","lbm","mdl","mng","pal",
    "pcd","pcx","pic","ppm","psd","psp","sgi","tga"};
  listFilenames(dir,extensions,filenames);
}

void listFilenames(const string& dir,vector<string> *filenames)
{
  listFilenames(dir,vector<string>(),filenames);
}

void listFilenames(const string& dir,
  const vector<string>& filenameExtensions,vector<string> *filenames)
{
  DIR *dirReadable = opendir(dir.c_str());
  if(dirReadable != NULL)
  {
    struct dirent *entry;
    while((entry = readdir(dirReadable)) != NULL)
    {
      switch(entry->d_type)
      {
      case DT_REG:  // file
        string filename(entry->d_name);
        if(filenameExtensions.empty() || hasExtension(filename,filenameExtensions))
        {
          filenames->push_back(filename);
        }
      }
    }
    closedir(dirReadable);
  } else
  {
    cerr << "ERROR: listFilenames: Could not open dir: " << dir << "\n";
  }
}

void getImgDims(const string& filename,int *width,int *height)
{
  if(hasExtension(filename,"jpg") || hasExtension(filename,"jpeg"))
    getImgDimsJPG(filename,width,height);
  else
    getImgDimsAny(filename,width,height);
}

void readColors(const string& filename,const vector<Vector2d>& coord,
  vector<Vector3uc> *pcolors)
{
  auto& colors = *pcolors;

  initDevIL();

  ILuint imId; // will be used to store image name
  ilGenImages(1,&imId); //this is what DevIL uses to keep track of image object
  ilBindImage(imId); // setting the current working image

  ILboolean success = ilLoadImage((const ILstring)filename.c_str());
  if(success)
  {
    int width = ilGetInteger(IL_IMAGE_WIDTH);
    int height = ilGetInteger(IL_IMAGE_HEIGHT);

    colors.resize(coord.size());
    for(size_t i = 0; i < coord.size(); i++)
    {
      int x = static_cast<int>(std::round(coord[i](0)));
      int y = static_cast<int>(std::round(coord[i](1)));
      x = std::min(width-1,std::max(0,x));
      y = std::min(height-1,std::max(0,y));
      ilCopyPixels(
        x,y,0, // returned block offset
        1,1,1, // returned block size -> just 1 pixel
        IL_RGB,IL_UNSIGNED_BYTE,&colors[i](0));
    }
  } else
  {
    colors.clear();
    cerr << "ERROR: readColors: could not load image: " << filename << "\n";
  }
  ilDeleteImages(1,&imId);
}

void findFocalLengthInEXIF(const string& ccdDBFilename,const ptr_vector<Camera>& cams,
  vector<double> *focals)
{
  findFocalLengthInEXIF(ccdDBFilename,cams,true,focals);
}

void findFocalLengthInEXIF(const string& ccdDBFilename,const ptr_vector<Camera>& cams,
  bool verbose,vector<double> *focals)
{
  focals->resize(cams.size());
  for(size_t i = 0; i < cams.size(); i++)
  {
    if(verbose)
      cout << "searching for focal of img " << i << "\n";
    (*focals)[i] = findFocalLengthInEXIF(ccdDBFilename,*cams[i],verbose);
  }
}
double findFocalLengthInEXIF(const string& ccdDBFilename,const Camera& cam,bool verbose)
{
  return findFocalLengthInEXIF(ccdDBFilename,cam.imgFilename(),
    std::max<int>(cam.imgWidth(),cam.imgHeight()),verbose);
}
double findFocalLengthInEXIF(const string& ccdDBFilename,const string& imgFilename,
  int maxImgDim,bool verbose)
{
  jhead::ResetJpgfile();
  jhead::ImageInfo = jhead::ImageInfo_t();
  jhead::ReadJpegFile(imgFilename.c_str(),jhead::ReadMode_t::READ_METADATA);

  if(jhead::ImageInfo.FocalLength == 0.f)
  {
    if(verbose)
      cout << "  focal [mm] not found in EXIF\n";
    return 0.;
  } else
  {
    double focalMM = jhead::ImageInfo.FocalLength;
    if(verbose)
      cout << "  focal [mm] found in EXIF\n";

    string cameraMake(jhead::ImageInfo.CameraMake);
    string cameraModel(jhead::ImageInfo.CameraModel);
    double CCDWidth = findCCDWidthInDB(ccdDBFilename,cameraMake,cameraModel);
    if(CCDWidth == 0.)
    {
      CCDWidth = jhead::ImageInfo.CCDWidth;
      if(verbose)
      {
        cout << "  ccd width not found in DB\n";
        if(CCDWidth > 0.)
          cout << "  ccd width computed from EXIF\n";
      }
    } else if(verbose)
    {
      cout << "  ccd width found in DB\n";
    }

    if(CCDWidth == 0.)
    {
      if(verbose)
      {
        cout << "  ccd width not computed from EXIF\n";
        cout << "  unable to compute focal [px]\n";
      }
      return 0.;
    } else
    {
      double focalPX = ((double)maxImgDim) * (focalMM / CCDWidth);
      if(verbose)
        cout << "  focal [px] computed: " << focalPX << "\n";
      return focalPX;
    }
  }
}

void readCMPSFMFormat(double focalConstraintWeight,double radConstraint,
  double radConstraintWeight,Dataset *pdata,ArrayXXd *homographyProportion)
{
  auto& data = *pdata;
  string listImgs = joinPaths(data.dir(),"list_imgs.txt");
  string focalEstimates = joinPaths(data.dir(),"focal_estimates.txt");
  string listKeys = joinPaths(data.dir(),"list_keys.txt");
  string matchesInit = joinPaths(data.dir(),"matches.init.txt");
  string matchesEG = joinPaths(data.dir(),"matches.eg.txt");
  string transforms = joinPaths(data.dir(),"transforms.txt");
  string tracks = joinPaths(data.dir(),"tracks.txt");
  readCMPSFMImageList(listImgs,radConstraint,radConstraintWeight,&data);
  readCMPSFMFocalEstimates(focalEstimates,focalConstraintWeight,&data);
  readCMPSFMKeys(listKeys,&data);
  //readCMPSFMMatches(matchesInit,false,&data.pairs());
  readCMPSFMMatches(matchesEG,true,&data.pairs());
  readCMPSFMTransforms(transforms,homographyProportion);
  readCMPSFMTracks(tracks,&data.nViewMatches());
}

void readCMPSFMImageList(const string& imgListFn,double radConstraint,
  double radConstraintWeight,Dataset *data)
{
  ifstream file(imgListFn);
  if(!file.is_open())
  {
    cerr << "ERROR: readCMPSFMImageList: unable to open: " << imgListFn << "\n";
    return;
  }
  data->cams().clear();
  string fn;
  while(!file.eof())
  {
    getline(file,fn);
    if(fn.empty())
      continue;
    fn = joinPaths(data->dir(),fn);
    data->cams().push_back(
      std::make_unique<StandardCameraRadial>(fn,data->featsDir()));

    StandardCameraRadial *cam = 
      static_cast<StandardCameraRadial *>(&(*data->cams().back()));
    vector<double> radConstraints(2,radConstraint),radWeights(2,radConstraintWeight);
    cam->constrainRadial(&radConstraints[0],&radWeights[0]);
  }
  file.close();
}

void readCMPSFMFocalEstimates(const string& focalsFn,double focalConstraintWeight,
  Dataset *data)
{
  ifstream file(focalsFn);
  if(!file.is_open())
  {
    cerr << "ERROR: readCMPSFMFocalEstimates: unable to open: " << focalsFn << "\n";
    return;
  }
  string endline;
  int i = 0;
  while(!file.eof())
  {
    double focalEstimate;
    file >> focalEstimate;
    getline(file,endline);
    if(endline.empty())
      continue;

    StandardCamera *cam = static_cast<StandardCamera *>(&data->cam(i));
    if(focalEstimate > 0.)
    {
      cam->setFocal(focalEstimate);
      cam->constrainFocal(focalEstimate,focalConstraintWeight);
    }

    i++;
  }
  file.close();
}

void readCMPSFMKeys(const string& keysListFn,Dataset *data)
{
  ifstream file(keysListFn);
  if(!file.is_open())
  {
    cerr << "ERROR: readCMPSFMKeys: unable to open: " << keysListFn << "\n";
    return;
  }
  string fn;
  int i = 0;
  while(!file.eof())
  {
    getline(file,fn);
    if(fn.empty())
      continue;
    
    data->cam(i).setFeaturesFilename(joinPaths(data->dir(),fn),true);

    i++;
  }
  file.close();
}

void readCMPSFMMatches(const string& matchesFn,
  bool isMatchesEG,pair_umap<CameraPair> *ppairs)
{
  auto& pairs = *ppairs;
  ifstream file(matchesFn);
  if(!file.is_open())
  {
    cerr << "ERROR: readCMPSFMMatches: unable to open: " << matchesFn << "\n";
    return;
  }
  int version;
  file >> version;
  string line;
  while(!file.eof())
  {
    getline(file,line);
    if(line.empty())
      continue;

    istringstream ss(line);
    int i,j;
    ss >> i >> j;

    auto& pair = pairs[IntPair(i,j)];
    int n;
    file >> n;
    pair.matches.resize(n);
    pair.dists.resize(n);
    for(int i = 0; i < n; i++)
      file >> pair.matches[i].first >> pair.matches[i].second >> pair.dists[i];
    
    if(isMatchesEG)
      getline(file,line);
    getline(file,line);
  }
  file.close();
}

void readCMPSFMTransforms(const string& transformsFn,
  ArrayXXd *phomographyProportion)
{
  auto& prop = *phomographyProportion;
  ifstream file(transformsFn);
  if(!file.is_open())
  {
    cerr << "ERROR: readCMPSFMTransforms: unable to open: " << transformsFn << "\n";
    return;
  }
  int nImgs;
  file >> nImgs;
  prop.resize(nImgs,nImgs);
  string line;
  while(!file.eof())
  {
    getline(file,line);
    if(line.empty())
      continue;

    istringstream ss(line);
    int i,j;
    ss >> i >> j;

    getline(file,line);
    getline(file,line);
    file >> prop(i,j);
    prop(j,i) = prop(i,j);
    getline(file,line);
    getline(file,line);
  }
  file.close();
}

void readCMPSFMTracks(const string& tracksFn,
  vector<NViewMatch> *ptracks)
{
  auto& tracks = *ptracks;
  ifstream file(tracksFn);
  if(!file.is_open())
  {
    cerr << "ERROR: readCMPSFMTransforms: unable to open: " << tracksFn << "\n";
    return;
  }
  string line;
  while(!file.eof())
  {
    getline(file,line);
    if(line.empty())
      continue;

    tracks.emplace_back();
    auto& track = tracks.back();
    
    istringstream ss(line);
    int size,camIdx,keyIdx;
    ss >> size;
    track.reserve(size);

    for(int i = 0; i < size; i++)
    {
      ss >> camIdx >> keyIdx;
      track.emplace(camIdx,keyIdx);
    }
  }
  file.close();
}

void writeSFMBundlerFormat(const string& filename,const uset<int>& reconstructedCams,
  const ptr_vector<Camera>& cams,const vector<Point>& pts)
{
  ofstream out(filename);
  if(!out.is_open())
  {
    cerr << "ERROR: writeSFMBundlerFormat: unable to open: " << filename << " for writing\n";
    return;
  }
  out << "# Bundle file v0.3\n";
  int nPtsAlive = 0;
  for(const auto& pt : pts)
    nPtsAlive += (!pt.views.empty());
  out << cams.size() << " " << nPtsAlive << "\n";

  vector<Vector2d> x0(cams.size());
  out.flags(std::ios::scientific);
  for(int i = 0; i < static_cast<int>(cams.size()); i++)
  {
    if(reconstructedCams.count(i) == 0)
    {
      for(size_t i = 0; i < 5; i++)
        out << "0 0 0\n";
    } else
    {
      Matrix3d K = cams[i]->K();
      double f = 0.5 * (K(0,0) + K(1,1));
      double rad[2];
      if(StandardCameraRadial *radCam =
        dynamic_cast<StandardCameraRadial *>(&(*cams[i])))
      {
        rad[0] = radCam->radParams()[0];
        rad[1] = radCam->radParams()[1];
      } else
      {
        rad[0] = 0.;
        rad[1] = 0.;
      }
      x0[i] = K.block(0,2,2,1);
      // f k1 k2
      out << f << " " << rad[0] << " " << rad[1] << "\n";
      // R
      Matrix3d R = cams[i]->R();
      Matrix3d T = -Matrix3d::Identity();
      T(0,0) = 1.;
      R = T*R; // Transform into Bundler format
      for(size_t j = 0; j < 3; j++)
        out << R(j,0) << " " << R(j,1) << " " << R(j,2) << "\n";
      // t
      Vector3d C = cams[i]->C();
      Vector3d t = -R*C;
      out << t(0) << " " << t(1) << " " << t(2) << "\n";
    }
  }
  for(const auto& pt : pts)
  {
    if(pt.views.empty())
      continue;

    out.flags(std::ios::scientific);
    // coordinates
    out << pt.coord(0) << " " << pt.coord(1) << " " << pt.coord(2) << "\n";
    out.flags(std::ios::fixed);
    // color
    out << pt.color.cast<int>().transpose() << "\n";
    // views
    vector<BundlerPointView> views;
    for(const auto& camKey : pt.views)
    {
      if(reconstructedCams.count(camKey.first) > 0)
      {
        const auto& key = cams[camKey.first]->key(camKey.second);
        Vector2d keyCentered = key - x0[camKey.first];
        views.emplace_back(camKey.first,camKey.second,
          keyCentered(0),keyCentered(1));
      }
    }
    out << views.size();
    for(const auto& view : views)
    {
      out << " " << view.camIdx << " " << view.keyIdx << " " <<
        view.x << " " << view.y;
    }
    out << "\n";
  }
  out.close();
}

void writeSFMBundlerFormat(const string& filename,const Dataset& data)
{
  writeSFMBundlerFormat(filename,data.reconstructedCams(),data.cams(),data.pts());
}

ostream& operator<<(ostream& file,const NViewMatch& m)
{
  file << m.size();
  for(const auto& entry : m)
    file << " " << entry.first << " " << entry.second;
  return file;
}

istream& operator>>(istream& file,NViewMatch& m)
{
  int n;
  file >> n;
  m.clear();
  m.reserve(n);
  int key;
  for(int i = 0; i < n; i++)
  {
    file >> key;
    file >> m[key];
  }
  return file;
}

} // namespace yasfm

namespace
{

void getImgDimsJPG(const string& filename,int *width,int *height)
{
  jhead::ResetJpgfile();
  jhead::ImageInfo = jhead::ImageInfo_t();
  jhead::ReadJpegFile(filename.c_str(),jhead::ReadMode_t::READ_METADATA);

  if(width)
    *width = jhead::ImageInfo.Width;

  if(height)
    *height = jhead::ImageInfo.Height;

  // Precaution - maybe useless
  if((width && *width <= 0) || (height && *height <= 0))
    getImgDimsAny(filename,width,height);
}


void initDevIL()
{
  static bool devilLoaded = false;
  if(!devilLoaded)
  {
    ilInit();
    ilOriginFunc(IL_ORIGIN_UPPER_LEFT);
    ilEnable(IL_ORIGIN_SET);
    devilLoaded = true;
  }
}

void getImgDimsAny(const string& filename,int *width,int *height)
{
  initDevIL();

  ILuint imId; // will be used to store image name
  ilGenImages(1,&imId); //this is what DevIL uses to keep track of image object
  ilBindImage(imId); // setting the current working image

  ILboolean success = ilLoadImage((const ILstring)filename.c_str());
  if(success)
  {
    if(width)
      *width = ilGetInteger(IL_IMAGE_WIDTH);

    if(height)
      *height = ilGetInteger(IL_IMAGE_HEIGHT);
  } else
  {
    cerr << "ERROR: getImgDimsAny: Could not load image: " << filename << "\n";
  }
  ilDeleteImages(1,&imId);
}

double findCCDWidthInDB(const string& dbFilename,const string& cameraMake,const string& cameraModel)
{
  ifstream file(dbFilename);
  if(!file.is_open())
  {
    cerr << "ERROR: could not open file: " << dbFilename << "\n";
    return 0.;
  }
  string makeModel = cameraMake + " " + cameraModel;
  string line;
  while(getline(file,line))
  {
    string readMakeModel("");
    if(readCameraMakeModelFromDBEntry(line,&readMakeModel))
    {
      if(readMakeModel.compare(makeModel) == 0)
      {
        file.close();
        return readCCDWidthFromDBEntry(line);
      }
    }
  }
  file.close();
  return 0.;
}

bool readCameraMakeModelFromDBEntry(const string& entry,string *pmakeModel)
{
  auto& makeModel = *pmakeModel;
  size_t idx1 = entry.find_first_of("\"");
  size_t idx2 = entry.find_first_of("\"",idx1 + 1);
  if(idx1 == string::npos || idx2 == string::npos)
  {
    makeModel = "";
    return false;
  }

  size_t idxCheck = entry.find("=>");
  if(idx1 > idxCheck || idx2 > idxCheck)
  {
    makeModel = "";
    return false;
  }

  makeModel = entry.substr(idx1 + 1,idx2 - idx1 - 1);
  return true;
}

double readCCDWidthFromDBEntry(const string& entry)
{
  size_t idx1 = entry.find("=>");
  size_t idx2 = entry.find_first_of(',',idx1 + 1);
  if(idx1 == string::npos || idx2 == string::npos)
  {
    return 0.;
  }
  string out = entry.substr(idx1 + 2,idx2 - idx1 - 2);
  idx1 = out.find_last_of(" \t");
  out = out.substr(idx1 + 1,idx2 - idx1);

  try
  {
    return std::stod(out);
  } catch(const std::invalid_argument&)
  {
    return 0.;
  }
}

} // namespace


/*
void writeFeaturesBinary(const IDataset& dts)
{
  writeFeaturesBinary(dts.cams(),dts.dir());
}

void writeFeaturesBinary(const ptr_vector<ICamera>& cams,const string& dir)
{
  for(const auto& cam : cams)
    writeFeaturesBinary(dir,*cam);
}

void writeFeaturesBinary(const string& dir,const ICamera& cam)
{
  ofstream out(joinPaths(dir,cam.featsFilename()),std::ios::binary);
  if(!out.is_open())
  {
    cerr << "writeFeaturesBinary: unable to open: " << cam.featsFilename() << " for binary writing" << "\n";
    return;
  }
  int nFeatures = cam.nFeatures();
  out.write((char*)(&nFeatures),sizeof(nFeatures));
  if(nFeatures > 0)
  {
    int descrDim = cam.feature(0).descrDim();
    out.write((char*)(&descrDim),sizeof(descrDim));
    float *descr = new float[descrDim];
    for(int i = 0; i < nFeatures; i++)
    {
      const IFeature& feat = cam.feature(i);
      double x = feat.x();
      double y = feat.y();
      out.write((char*)(&y),sizeof(y));
      out.write((char*)(&x),sizeof(x));
      feat.descriptor(descr);
      out.write((char*)(descr),descrDim * sizeof(*(descr)));
    }
    delete[] descr;
  }
  out.close();
}

void readFeaturesBinary(IDataset& dts)
{
  readFeaturesBinary(dts.cams(),dts.dir());
}

void readFeaturesBinary(ptr_vector<ICamera>& cams,const string& dir)
{
  for(auto& cam : cams)
    readFeaturesBinary(dir,*cam);
}

void readFeaturesBinary(const string& dir,ICamera& cam)
{
  ifstream in(joinPaths(dir,cam.featsFilename()),std::ios::binary);
  if(!in.is_open())
  {
    cerr << "readFeaturesBinary: unable to open: " << cam.featsFilename()<< " for binary reading" << "\n";
    return;
  }
  int nFeatures,descrDim;
  in.read((char*)(&nFeatures),sizeof(nFeatures));
  cam.reserveFeatures(nFeatures);
  if(nFeatures > 0)
  {
    in.read((char*)(&descrDim),sizeof(descrDim));
    float *descr = new float[descrDim];
    for(int i = 0; i < nFeatures; i++)
    {
      double x,y;
      in.read((char*)(&y),sizeof(y));
      in.read((char*)(&x),sizeof(x));
      in.read((char*)(descr),descrDim * sizeof(*(descr)));
      cam.addFeature(x,y,descrDim,descr,nullptr);
    }
    delete[] descr;
  }
  in.close();
}

void writeSFMPLYFormat(const string& filename,const unordered_set<int>& addedCams,
  const ptr_vector<ICamera>& cams,const vector<Vector3d>& points,const vector<bool>& pointsMask,
  const vector<Vector3uc> *pointColors)
{
  ofstream out(filename);
  if(!out.is_open())
  {
    cerr << "writeSFMPLYFormat: unable to open: " << filename << " for writing" << "\n";
    return;
  }
  out << "ply" << "\n"
    << "format ascii 1.0" << "\n"
    << "element vertex " << (points.size() + 2 * cams.size()) << "\n"
    << "property float x" << "\n"
    << "property float y" << "\n"
    << "property float z" << "\n"
    << "property uchar diffuse_red" << "\n"
    << "property uchar diffuse_green" << "\n"
    << "property uchar diffuse_blue" << "\n"
    << "end_header" << "\n";

  out.flags(std::ios::scientific);
  for(size_t i = 0; i < points.size(); i++)
  {
    if(pointsMask[i])
    {
      out << points[i](0) << " " << points[i](1) << " " << points[i](2) << " ";
      if(pointColors)
      {
        out << (unsigned int)(*pointColors)[i](0) << " " <<
          (unsigned int)(*pointColors)[i](1) << " " <<
          (unsigned int)(*pointColors)[i](2);
      } else
      {
        out << "0 0 0";
      }
      out << "\n";
    }
  }
  Vector3d ptCam; // this point is for showing symbolic direction
  ptCam << 0,0,0.05;
  for(const auto& cam : cams)
  {
    const Vector3d& C = cam->C();
    Vector3d ptInFront = cam->R().transpose() * ptCam + C;

    out << C(0) << " " << C(1) << " " << C(2) << " 0 255 0" << "\n";
    out << ptInFront(0) << " " << ptInFront(1) << " " << ptInFront(2) << " 255 255 0" << "\n";
  }
  out.close();
}

void writeSFMPLYFormat(const string& filename,const IDataset& dts,
  const vector<Vector3uc> *pointColors)
{
  writeSFMPLYFormat(filename,dts.addedCams(),dts.cams(),dts.points(),
    dts.pointsMask(),pointColors);
}
*/
