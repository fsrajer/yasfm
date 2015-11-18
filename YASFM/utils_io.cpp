/*
* Filip Srajer
* filip.srajer (at) fel.cvut.cz
* Center for Machine Perception
* Czech Technical University in Prague
*
* This software is under construction.
* 03/2015
*/

#include "utils_io.h"

#include <climits>
#include <algorithm>
#include <iostream>
#include <iomanip>
#include <fstream>

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

using std::cout;
using std::cerr;
using std::ifstream;
using std::ofstream;
using std::setprecision;

namespace yasfm
{

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
    cerr << "listFilenames: could not open dir: " << dir << "\n";
  }
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

void getImgDims(const string& filename,int *width,int *height)
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
    cerr << "getImgDims: could not load image: " << filename << "\n";
  }
  ilDeleteImages(1,&imId);
}

/*
void readFeatureColors(const string& dir,ICamera& cam,int *width,int *height)
{
  initDevIL();

  ILuint imId; // will be used to store image name
  ilGenImages(1,&imId); //this is what DevIL uses to keep track of image object
  ilBindImage(imId); // setting the current working image

  ILboolean success = ilLoadImage((const ILstring)(joinPaths(dir,cam.imgFilename())).c_str());
  if(success)
  {
    if(width)
      *width = ilGetInteger(IL_IMAGE_WIDTH);

    if(height)
      *height = ilGetInteger(IL_IMAGE_HEIGHT);

    Vector3uc col;
    for(int i = 0; i < cam.nFeatures(); i++)
    {
      int x = static_cast<int>(std::round(cam.feature(i).x()));
      int y = static_cast<int>(std::round(cam.feature(i).y()));
      ilCopyPixels(
        x,y,0, // returned block offset
        1,1,1, // returned block size -> just 1 pixel
        IL_RGB,IL_UNSIGNED_BYTE,col.data());

      cam.feature(i).setColor(col);
    }
  } else
  {
    cerr << "readFeatureColors: could not load image: " << cam.imgFilename() << "\n";
  }
  ilDeleteImages(1,&imId);
}

void readFeatureColors(IDataset& dts)
{
  for(size_t i = 0; i < dts.cams().size(); i++)
  {
    yasfm::readFeatureColors(dts.dir(),*(dts.cams()[i]));
  }
}
*/

/*
void writeImgFnsList(const string& listFilename,const ptr_vector<ICamera>& cams)
{
  ofstream out(listFilename);
  if(!out.is_open())
  {
    cerr << "writeImgFnsList: unable to open: " << listFilename << " for writing\n";
    return;
  }
  for(const auto& cam : cams)
  {
    out << cam->imgFilename() << "\n";
  }
  out.close();
}

void writeImgFnsList(const string& listFilename,const IDataset& dts)
{
  writeImgFnsList(listFilename,dts.cams());
}

void writeFeatsFnsList(const string& listFilename,const ptr_vector<ICamera>& cams)
{
  ofstream out(listFilename);
  if(!out.is_open())
  {
    cerr << "writeFeatsFnsList: unable to open: " << listFilename << " for writing\n";
    return;
  }
  for(const auto& cam : cams)
  {
    out << cam->featsFilename() << "\n";
  }
  out.close();
}

void writeFeatsFnsList(const string& listFilename,const IDataset& dts)
{
  writeFeatsFnsList(listFilename,dts.cams());
}
*/

void findFocalLengthInEXIF(const string& ccdDBFilename,const ptr_vector<Camera>& cams,
  vector<double> *focals)
{
  focals->resize(cams.size());
  for(size_t i = 0; i < cams.size(); i++)
  {
    cout << "searching for focal of img " << i << "\n";
    (*focals)[i] = findFocalLengthInEXIF(ccdDBFilename,*cams[i]);
  }
}
double findFocalLengthInEXIF(const string& ccdDBFilename,const Camera& cam)
{
  return findFocalLengthInEXIF(ccdDBFilename,cam.imgFilename(),
    std::max<int>(cam.imgWidth(),cam.imgHeight()));
}
double findFocalLengthInEXIF(const string& ccdDBFilename,const string& imgFilename,
  int maxImgDim)
{
  jhead::ResetJpgfile();
  jhead::ImageInfo = jhead::ImageInfo_t();
  jhead::ReadJpegFile(imgFilename.c_str(),jhead::ReadMode_t::READ_METADATA);

  if(jhead::ImageInfo.FocalLength == 0.f)
  {
    cout << "  focal [mm] not found in EXIF\n";
    return 0.;
  } else
  {
    double focalMM = jhead::ImageInfo.FocalLength;
    cout << "  focal [mm] found in EXIF\n";

    string cameraMake(jhead::ImageInfo.CameraMake);
    string cameraModel(jhead::ImageInfo.CameraModel);
    double CCDWidth = findCCDWidthInDB(ccdDBFilename,cameraMake,cameraModel);
    if(CCDWidth == 0.)
    {
      CCDWidth = jhead::ImageInfo.CCDWidth;
      cout << "  ccd width not found in DB\n";
      if(CCDWidth > 0.)
        cout << "  ccd width computed from EXIF\n";
    } else
    {
      cout << "  ccd width found in DB\n";
    }

    if(CCDWidth == 0.)
    {
      cout << "  ccd width not computed from EXIF\n";
      cout << "  unable to compute focal [px]\n";
      return 0.;
    } else
    {
      double focalPX = ((double)maxImgDim) * (focalMM / CCDWidth);
      cout << "  focal [px] computed: " << focalPX << "\n";
      return focalPX;
    }
  }
}

/*
void writeFeaturesASCII(const IDataset& dts,bool convertDescrToUint)
{
  writeFeaturesASCII(dts.cams(),dts.dir(),convertDescrToUint);
}

void writeFeaturesASCII(const ptr_vector<ICamera>& cams,const string& dir,bool convertDescrToUint)
{
  for(const auto& cam : cams)
    writeFeaturesASCII(dir,*cam,convertDescrToUint);
}

void writeFeaturesASCII(const string& dir,const ICamera& cam,bool convertDescrToUint)
{
  ofstream out(joinPaths(dir,cam.featsFilename()));
  if(!out.is_open())
  {
    cerr << "writeFeaturesASCII: unable to open: " << cam.featsFilename() << " for writing\n";
    return;
  }
  out.flags(std::ios::fixed);
  out << cam.nFeatures();
  if(cam.nFeatures() > 0)
  {
    out << ' ' << cam.feature(0).descrDim() << "\n";
    for(int i = 0; i < cam.nFeatures(); i++)
    {
      const IFeature& feat = cam.feature(i);
      out << setprecision(2) << feat.y() << ' ' << setprecision(2) << feat.x();
      for(int j = 0; j < feat.descrDim(); j++)
      {
        if((j % 20) == 0)
        {
          out << "\n";
        }
        if(convertDescrToUint)
        {
          out << ((unsigned int)floor(0.5 + 512.0f*feat.descriptor(j))) << ' ';
        } else
        {
          out << setprecision(8) << feat.descriptor(j) << ' ';
        }
      }
      out << "\n";
    }
  }
  out.close();
}

void readFeaturesASCII(IDataset& dts)
{
  readFeaturesASCII(dts.cams(),dts.dir());
}

void readFeaturesASCII(ptr_vector<ICamera>& cams,const string& dir)
{
  for(auto& cam : cams)
    readFeaturesASCII(dir,*cam);
}

void readFeaturesASCII(const string& dir,ICamera& cam)
{
  ifstream in(joinPaths(dir,cam.featsFilename()));
  if(!in.is_open())
  {
    cerr << "readFeaturesASCII: unable to open: " << cam.featsFilename() << " for reading\n";
    return;
  }
  int nFeats,descrDim;
  in >> nFeats >> descrDim;
  cam.reserveFeatures(nFeats);
  float *descr = new float[descrDim];
  for(int i = 0; i < nFeats; i++)
  {
    double x,y;
    in >> y >> x;
    for(int j = 0; j < descrDim; j++)
    {
      in >> *(descr+j);
    }
    cam.addFeature(x,y,descrDim,descr,nullptr);
  }
  delete[] descr;
  in.close();
}

void writeKeysASCII(const IDataset& dts)
{
  writeKeysASCII(dts.cams(),dts.dir());
}

void writeKeysASCII(const ptr_vector<ICamera>& cams,const string& dir)
{
  for(const auto& cam : cams)
    writeKeysASCII(dir,*cam);
}

void writeKeysASCII(const string& dir,const ICamera& cam)
{
  ofstream out(joinPaths(dir,cam.featsFilename()));
  if(!out.is_open())
  {
    cerr << "writeKeysASCII: unable to open: " << cam.featsFilename() << " for writing\n";
    return;
  }
  out.flags(std::ios::fixed);
  out << cam.nFeatures() << "\n";
  for(int i = 0; i < cam.nFeatures(); i++)
  {
    out << setprecision(2) << cam.feature(i).y() << ' ' << setprecision(2) << cam.feature(i).x() << "\n";
  }
  out.close();
}

void readKeysASCII(IDataset& dts)
{
  readKeysASCII(dts.cams(),dts.dir());
}

void readKeysASCII(ptr_vector<ICamera>& cams,const string& dir)
{
  for(auto& cam : cams)
    readKeysASCII(dir,*cam);
}

void readKeysASCII(const string& dir,ICamera& cam)
{
  ifstream in(joinPaths(dir,cam.featsFilename()));
  if(!in.is_open())
  {
    cerr << "readKeysASCII: unable to open: " << cam.featsFilename() << " for reading" << "\n";
    return;
  }
  int nKeys;
  in >> nKeys;
  cam.reserveFeatures(nKeys);
  for(size_t i = 0; i < nKeys; i++)
  {
    double x,y;
    in >> y >> x;
    cam.addFeature(x,y);
  }
  in.close();
}

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

void writeMatchesASCII(const string& filename,const IDataset& dts)
{
  writeMatchesASCII(filename,dts.pairs());
}

void writeMatchesASCII(const string& filename,const pair_ptr_unordered_map<ICameraPair>& pairs)
{
  ofstream out(filename);
  if(!out.is_open())
  {
    cerr << "writeMatchesASCII: unable to open: " << filename << " for writing" << "\n";
    return;
  }
  out.flags(std::ios::fixed);
  out << pairs.size() << "\n";
  for(const auto& entry : pairs)
  {
    IntPair idx = entry.first;
    const auto& pair = *(entry.second);
    out << idx.first << ' ' << idx.second << "\n";
    out << pair.nMatches() << "\n";
    for(int i = 0; i < pair.nMatches(); i++)
    {
      out << pair.match(i).first << ' ' <<
        pair.match(i).second << ' ' <<
        setprecision(3) << pair.minScore(i) << "\n";
    }
  }
  out.close();
}

void writeTransformsASCII(const string& filename,const IDataset& dts)
{
  writeTransformsASCII(filename,dts.pairs());
}

void writeTransformsASCII(const string& filename,const pair_ptr_unordered_map<ICameraPair>& pairs)
{
  ofstream out(filename);
  if(!out.is_open())
  {
    cerr << "writeTransformsASCII: unable to open: " << filename << " for writing" << "\n";
    return;
  }
  out << pairs.size() << "\n";
  for(const auto& entry : pairs)
  {
    IntPair idx = entry.first;
    const auto& pair = *(entry.second);
    out << idx.first << ' ' << idx.second << "\n";
    out << pair.F() << "\n";
  }
  out.close();
}

void writeTracksASCII(const string& filename,const vector<NViewMatch>& tracks)
{
  ofstream out(filename);
  if(!out.is_open())
  {
    cerr << "writeTracksASCII: unable to open: " << filename << " for writing" << "\n";
    return;
  }
  out << tracks.size() << "\n";
  for(const auto& track : tracks)
  {
    out << track.size();
    for(const auto& camKey : track)
    {
      out << " " << camKey.first << " " << camKey.second;
    }
    out << "\n";
  }
  out.close();
}

void readTracksASCII(const string& filename,IDataset& dts)
{
  readTracksASCII(filename,dts.tracks(),&(dts.tracks2points()));
}

void readTracksASCII(const string& filename,vector<NViewMatch>& tracks,
  vector<int> *tracks2points)
{
  ifstream in(filename);
  if(!in.is_open())
  {
    cerr << "readTracksASCII: unable to open: " << filename << " for reading" << "\n";
    return;
  }
  int nTracks;
  in >> nTracks;
  tracks.resize(nTracks);
  for(int i = 0; i < nTracks; i++)
  {
    int trackSize;
    in >> trackSize;
    tracks[i].reserve(trackSize);
    for(int j = 0; j < trackSize; j++)
    {
      int camIdx,keyIdx;
      in >> camIdx >> keyIdx;
      tracks[i][camIdx] = keyIdx;
    }
  }
  in.close();
  if(tracks2points)
    tracks2points->resize(tracks.size(),-1);
}
*/

void writeSFMBundlerFormat(const string& filename,const uset<int>& reconstructedCams,
  const ptr_vector<Camera>& cams,const Points& points)
{
  ofstream out(filename);
  if(!out.is_open())
  {
    cerr << "writeSFMBundlerFormat: unable to open: " << filename << " for writing\n";
    return;
  }
  out << "# Bundle file v0.3\n";
  out << cams.size() << " " << points.numPts() << "\n";
  
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
      R.bottomRows(1) *= -1; // make the -z axis the viewing direction
      for(size_t j = 0; j < 3; j++)
        out << R(j,0) << " " << R(j,1) << " " << R(j,2) << "\n";
      // t
      Vector3d C = cams[i]->C();
      Vector3d t = -R*C;
      out << t(0) << " " << t(1) << " " << t(2) << "\n";
    }
  }
  for(int i = 0; i < points.numPts(); i++)
  {
    out.flags(std::ios::scientific);
    // coordinates
    const auto& coord = points.ptCoord()[i];
    out << coord(0) << " " << coord(1) << " " << coord(2) << "\n";
    out.flags(std::ios::fixed);
    // color
    out << "0 0 0\n";
    // views
    vector<BundlerPointView> views;
    for(const auto& camKey : points.ptData()[i].reconstructed)
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

void writeSFMBundlerFormat(const string& filename, const Dataset& data)
{
  writeSFMBundlerFormat(filename,data.reconstructedCams(),data.cams(),data.points());
}

/*
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

} // namespace yasfm

namespace
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
    { return true; }
  }
  return false;
}

double findCCDWidthInDB(const string& dbFilename,const string& cameraMake,const string& cameraModel)
{
  ifstream file(dbFilename);
  if(!file.is_open())
  {
    cout << "error: could not open file: " << dbFilename << "\n";
    return 0.;
  }
  string makeModel = cameraMake + " " + cameraModel;
  string line;
  while(getline(file,line))
  {
    string readMakeModel("");
    if(readCameraMakeModelFromDBEntry(line,readMakeModel))
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

bool readCameraMakeModelFromDBEntry(const string& entry,string& makeModel)
{
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
  }catch(const std::invalid_argument&)
  {
    return 0.;
  }
}

} // namespace
