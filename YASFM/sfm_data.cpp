#include "sfm_data.h"

#include <iostream>
#include <fstream>
#include <direct.h>

#include "camera_factory.h"

using Eigen::ArrayXf;
using Eigen::Map;
using std::cerr;
using std::cout;
using std::setprecision;
using std::getline;
using std::ifstream;
using std::stoi;
using std::ofstream;

namespace yasfm
{

void Points::addPoints(const IntPair& camsIdxs,const vector<int>& matchesToReconstructIdxs,
  const vector<Vector3d>& coord,const vector<Vector3uc>& colors)
{
  size_t sz = ptCoord_.size() + coord.size();
  ptCoord_.reserve(sz);
  ptData_.reserve(sz);

  for(size_t i = 0; i < coord.size(); i++)
  {
    ptCoord_.push_back(coord[i]);
    ptData_.emplace_back();
    const auto& nViewMatch = matchesToReconstruct_[matchesToReconstructIdxs[i]];

    auto& reconstructed = ptData_.back().reconstructed;
    reconstructed[camsIdxs.first] = nViewMatch.at(camsIdxs.first);
    reconstructed[camsIdxs.second] = nViewMatch.at(camsIdxs.second);
    auto& toReconstruct = ptData_.back().toReconstruct;
    toReconstruct = nViewMatch;
    toReconstruct.erase(camsIdxs.first);
    toReconstruct.erase(camsIdxs.second);
    ptData_.back().color = colors[i];
  }
  filterOutOutliers(matchesToReconstructIdxs,&matchesToReconstruct_);
}
void Points::addPoints(const vector<Vector3d>& pointCoord,const vector<Vector3uc>& colors,
  const vector<SplitNViewMatch>& pointViews)
{
  ptCoord_.reserve(numPts() + pointCoord.size());
  ptData_.reserve(numPts() + pointCoord.size());
  for(size_t i = 0; i < pointCoord.size(); i++)
  {
    ptCoord_.push_back(pointCoord[i]);
    ptData_.emplace_back();
    ptData_.back().reconstructed = pointViews[i].observedPart;
    ptData_.back().toReconstruct = pointViews[i].unobservedPart;
    ptData_.back().color = colors[i];
  }
}

void Points::removePoints(const vector<bool>& keep)
{
  filterVector(keep,&ptCoord_);
  filterVector(keep,&ptData_);
}
int Points::numPts() const 
{
  return static_cast<int>(ptCoord().size());
}
void Points::markCamAsReconstructed(int camIdx)
{
  for(auto& entry : ptData_)
  {
    entry.reconstructed.emplace(camIdx,entry.toReconstruct.at(camIdx));
    entry.toReconstruct.erase(camIdx);
  }
}
void Points::markCamAsReconstructed(int camIdx,
  const vector<int>& correspondingPoints,
  const vector<int>& correspondingPointsInliers)
{
  for(int inlierIdx : correspondingPointsInliers)
  {
    auto& entry = ptData_[correspondingPoints[inlierIdx]];
    entry.reconstructed.emplace(camIdx,entry.toReconstruct.at(camIdx));
  }
  for(int ptIdx : correspondingPoints)
  {
    auto& entry = ptData_[ptIdx];
    entry.toReconstruct.erase(camIdx);
  }
}

void Points::writeASCII(ostream& file) const
{
  const int nFields = 3;
  const int nFieldsPointData = 3;
  
  file << nFields << "\n";
  file << "matchesToReconstruct_ " << matchesToReconstruct_.size() << "\n";
  for(const auto& match : matchesToReconstruct_)
    file << match << "\n";
  file << "ptCoord_ " << ptCoord_.size() << "\n";
  for(const auto& coord : ptCoord_)
    file << coord(0) << " " << coord(1) << " " << coord(2) << "\n";
  file << "ptData_ " << ptData_.size() << " " << nFieldsPointData << "\n";
  file << "reconstructed\n";
  file << "toReconstruct\n";
  file << "color\n";
  for(const auto& data : ptData_)
  {
    Eigen::Vector3i color = data.color.cast<int>();
    file << data.reconstructed << " " << data.toReconstruct
      << " " << color(0) << " " << color(1) << " " << color(1) << "\n";
  }
}

void Points::readASCII(istream& file)
{
  int nFields;
  file >> nFields;
  string s;
  for(int iField = 0; iField < nFields; iField++)
  {
    file >> s;
    if(s == "matchesToReconstruct_")
    {
      int nMatches;
      file >> nMatches;
      matchesToReconstruct_.resize(nMatches);
      for(auto& match : matchesToReconstruct_)
        file >> match;
    } else if(s == "ptCoord_")
    {
      int n;
      file >> n;
      ptCoord_.resize(n);
      for(auto& coord : ptCoord_)
        file >> coord(0) >> coord(1) >> coord(2);
    } else if(s == "ptData_")
    {
      int n,nFieldsPointData;
      file >> n >> nFieldsPointData;
      int format = 0;
      for(int i = 0; i < nFieldsPointData; i++)
      {
        file >> s;
        if(s == "reconstructed")
          format |= 1;
        else if(s == "toReconstruct")
          format |= 2;
        else if(s == "color")
          format |= 4;
      }
      ptData_.resize(n);
      for(auto& data: ptData_)
      {
        if(format & 1)
          file >> data.reconstructed;
        if(format & 2)
          file >> data.toReconstruct;
        if(format & 4)
          file >> data.color(0) >> data.color(1) >> data.color(2);
      }
    }
  }
}

const vector<NViewMatch>& Points::matchesToReconstruct() const { return matchesToReconstruct_; }
vector<NViewMatch>& Points::matchesToReconstruct() { return matchesToReconstruct_; }
const vector<Vector3d>& Points::ptCoord() const { return ptCoord_; }
double* Points::ptCoord(int ptIdx)
{
  return &ptCoord_[ptIdx](0);
}
const vector<Points::PointData>& Points::ptData() const { return ptData_; }

Dataset::Dataset(const string& dir)
  : dir_(dir)
{
}

Dataset::Dataset(const Dataset& o)
{
  copyIn(o);
}

Dataset& Dataset::operator = (const Dataset& o)
{
  copyIn(o);
  return *this;
}
void Dataset::copyIn(const Dataset& o)
{
  dir_ = o.dir_;
  cams_.reserve(o.cams_.size());
  for(const auto& cam : o.cams_)
  {
    cams_.push_back(cam->clone());
  }
  pairs_ = o.pairs_;
  reconstructedCams_ = o.reconstructedCams_;
  points_ = o.points_;
}
void Dataset::clearDescriptors()
{
  for(auto& cam : cams_)
  {
    cam->clearDescriptors();
  }
}
void Dataset::markCamAsReconstructed(int camIdx)
{
  reconstructedCams_.insert(camIdx);
  points_.markCamAsReconstructed(camIdx);
}
void Dataset::markCamAsReconstructed(int camIdx,
  const vector<int>& correspondingPoints,
  const vector<int>& correspondingPointsInliers)
{
  reconstructedCams_.insert(camIdx);
  points_.markCamAsReconstructed(camIdx,correspondingPoints,
    correspondingPointsInliers);
}

int Dataset::numCams() const
{
  return static_cast<int>(cams_.size());
}

void Dataset::writeASCII(const string& filename,int camWriteMode) const
{
  string featuresDir = joinPaths(dir_,"keys");
  _mkdir(featuresDir.c_str());
  writeASCII(filename,camWriteMode,featuresDir);
}

void Dataset::writeASCII(const string& filename,int camWriteMode,
  const string& featuresDir) const
{
  string fn = joinPaths(dir(),filename);
  ofstream file(fn);
  if(!file.is_open())
  {
    cerr << "ERROR: Dataset::writeASCII: unable to open: " << fn << " for writing\n";
    return;
  }

  const int nFields = 5;
  const int nFieldsCameraPair = 2;

  file << "########### INFO ###########\n"
    << "# num cams: " << numCams() << "\n"
    << "# num reconstructed cams: " << reconstructedCams_.size() << "\n"
    << "# num points: " << points_.numPts() << "\n"
    << "# num unreconstructed n-view matches: "
      << points_.matchesToReconstruct().size() << "\n"
    << "############################\n";
  file << nFields << "\n";

  file << "dir_\n" << dir_ << "\n";

  file << "reconstructedCams_ " << reconstructedCams_.size() << "\n";
  for(int camIdx : reconstructedCams_)
    file << " " << camIdx;
  file << "\n";

  file << "cams_ " << cams_.size() << "\n";
  for(int i = 0; i < numCams(); i++)
  {
    file << cam(i).className() << " id: " << i << "\n";
    cam(i).writeASCII(file,camWriteMode,featuresDir);
  }

  file << "points_\n";
  points_.writeASCII(file);

  file << "pairs_ " << pairs_.size() << " " << nFieldsCameraPair << "\n";
  file << "matches\n";
  file << "dists\n";
  for(const auto& entry : pairs_)
  {
    IntPair idxs = entry.first;
    const auto& pair = entry.second;
    file << idxs.first << " " << idxs.second << "\n";
    file << pair.matches.size() << "\n";
    for(IntPair match : pair.matches)
      file << match.first << " " << match.second << "\n";
    file << pair.dists.size() << "\n";
    for(double dist : pair.dists)
      file << dist << "\n";
  }

  file.close();
}

void Dataset::readASCII(const string& filename,int camReadMode)
{
  string featuresDir = joinPaths(dir_,"keys");
  readASCII(filename,camReadMode,featuresDir);
}

void Dataset::readASCII(const string& filename,int camReadMode,
  const string& featuresDir)
{
  string fn = joinPaths(dir(),filename);
  ifstream file(fn);
  if(!file.is_open())
  {
    cerr << "ERROR: Dataset::readASCII: unable to open: " << fn << " for reading\n";
    return;
  }
  while(!file.eof())
  {
    string s;
    file >> s;
    if(s.empty())
    {
      continue;
    } else if(s[0] == '#')
    {
      getline(file,s);
      continue;
    } else
    {
      int nFields = stoi(s);
      for(int iField = 0; iField < nFields; iField++)
      {
        file >> s;
        if(s == "dir_")
        {
          file >> dir_;
        } else if(s == "reconstructedCams_")
        {
          int n,idx;
          file >> n;
          reconstructedCams_.clear();
          reconstructedCams_.reserve(n);
          for(int i = 0; i < n; i++)
          {
            file >> idx;
            reconstructedCams_.insert(idx);
          }
        } else if(s == "cams_")
        {
          int n;
          file >> n;
          cams_.clear();
          cams_.reserve(n);
          for(int i = 0; i < n; i++)
          {
            string className;
            file >> className;
            getline(file,s);
            cams_.push_back(CameraFactory::createInstance(className,file,camReadMode,
              featuresDir));
          }
        } else if(s == "points_")
        {
          points_.readASCII(file);
        } else if(s == "pairs_")
        {
          int nPairs,nFieldsCameraPair;
          file >> nPairs >> nFieldsCameraPair;
          int format = 0;
          for(int i = 0; i < nFieldsCameraPair; i++)
          {
            file >> s;
            if(s == "matches")
              format |= 1;
            else if(s == "dists")
              format |= 2;
          }
          pairs_.clear();
          pairs_.reserve(nPairs);
          for(int iPair = 0; iPair < nPairs; iPair++)
          {
            IntPair idx;
            file >> idx.first >> idx.second;
            auto& pair = pairs_[idx];
            if(format & 1)
            {
              int nMatches;
              file >> nMatches;
              pair.matches.resize(nMatches);
              for(int iMatch = 0; iMatch < nMatches; iMatch++)
              {
                auto& match = pair.matches[iMatch];
                file >> match.first >> match.second;
              }
            }
            if(format & 2)
            {
              int nDists;
              file >> nDists;
              pair.dists.resize(nDists);
              for(int iDist = 0; iDist < nDists; iDist++)
              {
                file >> pair.dists[iDist];
              }
            }
          }
        }
      }
    }
  }
  file.close();
}

void Dataset::readKeysColors()
{
  for(const auto& cam : cams_)
    cam->readKeysColors();
}

int Dataset::countReconstructedObservations() const
{
  size_t nObs = 0;
  for(int iPt = 0; iPt < points().numPts(); iPt++)
  {
    nObs += points().ptData()[iPt].reconstructed.size();
  }
  return static_cast<int>(nObs);
}

const string& Dataset::dir() const { return dir_; }
const Camera& Dataset::cam(int idx) const { return *cams_[idx]; }
const Camera& Dataset::cam(size_t idx) const { return *cams_[idx]; }
Camera& Dataset::cam(int idx) { return *cams_[idx]; }
Camera& Dataset::cam(size_t idx) { return *cams_[idx]; }
const ptr_vector<Camera>& Dataset::cams() const { return cams_; }
ptr_vector<Camera>& Dataset::cams() { return cams_; }
const pair_umap<CameraPair>& Dataset::pairs() const { return pairs_; }
pair_umap<CameraPair>& Dataset::pairs() { return pairs_; }
const uset<int>& Dataset::reconstructedCams() const { return reconstructedCams_; }
const Points& Dataset::points() const { return points_; }
Points& Dataset::points() { return points_; }


} // namespace yasfm