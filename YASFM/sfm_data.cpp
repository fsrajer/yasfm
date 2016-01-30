#include "sfm_data.h"

#include <iostream>
#include <fstream>
#include <direct.h>
#include <iomanip>

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
Dataset::Dataset(const string& dir)
  : dir_(dir)
{
  _mkdir(featsDir().c_str());
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
  queries_ = o.queries_;
  pairs_ = o.pairs_;
  reconstructedCams_ = o.reconstructedCams_;
  nViewMatches_ = o.nViewMatches_;
  pts_ = o.pts_;
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

  for(auto& pt : pts_)
  {
    if(pt.viewsToAdd.count(camIdx) > 0)
    {
      pt.views.emplace(camIdx,pt.viewsToAdd.at(camIdx));
      pt.viewsToAdd.erase(camIdx);
    }
  }
}
void Dataset::markCamAsReconstructed(int camIdx,
  const vector<int>& correspondingPoints,
  const vector<int>& correspondingPointsInliers)
{
  reconstructedCams_.insert(camIdx);
  
  for(int inlierIdx : correspondingPointsInliers)
  {
    auto& pt = pts_[correspondingPoints[inlierIdx]];
    if(pt.viewsToAdd.count(camIdx) > 0)
    {
      pt.views.emplace(camIdx,pt.viewsToAdd.at(camIdx));
    }
  }
  for(int ptIdx : correspondingPoints)
    pts_[ptIdx].viewsToAdd.erase(camIdx);
}

int Dataset::numCams() const
{
  return static_cast<int>(cams_.size());
}

void Dataset::writeASCII(const string& filename) const
{
  string fn = joinPaths(dir(),filename);
  ofstream file(fn);
  if(!file.is_open())
  {
    cerr << "ERROR: Dataset::writeASCII: unable to open: " << fn << " for writing\n";
    return;
  }

  const int nFields = 7;
  int nPtsAlive = countPtsAlive();

  file << "########### INFO ###########\n"
    << "# num cams: " << numCams() << "\n"
    << "# num reconstructed cams: " << reconstructedCams_.size() << "\n"
    << "# num points: " << nPtsAlive << "\n"
    << "# num unreconstructed n-view matches: " << nViewMatches_.size() << "\n"
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
    cam(i).writeASCII(file);
  }

  file << "nViewMatches_ " << nViewMatches_.size() << "\n";
  for(const auto& match : nViewMatches_)
    file << match << "\n";

  const int nFieldsPoints = 4;
  file << "pts_ " << nPtsAlive << " " << nFieldsPoints << "\n";
  file << "coord\n";
  file << "views\n";
  file << "viewsToAdd\n";
  file << "color\n";
  for(const auto& pt : pts_)
  {
    if(!pt.views.empty())
    {
      file << pt.coord(0) << " " << pt.coord(1) << " " << pt.coord(2) << " "
        << pt.views << " "
        << pt.viewsToAdd << " "
        << pt.color.cast<int>().transpose() << "\n";
    }
  }

  file << "queries_ " << queries_.size() <<"\n";
  for(size_t i = 0; i < queries_.size(); i++)
  {
    file << queries_[i].size();
    for(int idx : queries_[i])
      file << " " << idx;
    file << "\n";
  }

  const int nFieldsCameraPair = 3;
  file << "pairs_ " << pairs_.size() << " " << nFieldsCameraPair << "\n";
  file << "matches\n";
  file << "dists\n";
  file << "supportSizes\n";
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
    file << pair.supportSizes.size() << "\n";
    for(int supportSize : pair.supportSizes)
      file << " " << supportSize;
    file << "\n";
  }

  file.close();
}

void Dataset::readASCII(const string& filename)
{
  string fn = joinPaths(dir(),filename);
  dir_.clear();
  cams_.clear();
  queries_.clear();
  pairs_.clear();
  reconstructedCams_.clear();
  nViewMatches_.clear();
  pts_.clear();
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
            cams_.push_back(CameraFactory::createInstance(className,file));
          }
        } else if(s == "nViewMatches_")
        {
          size_t n;
          file >> n;
          nViewMatches_.resize(n);
          for(auto& match : nViewMatches_)
            file >> match;

        } else if(s == "pts_")
        {
          readPts(file);
        } else if(s == "points_")
        {
          // Keep this for a while and remove when that format is not used anywhere.
          readPointsOld(file);
        } else if(s == "queries_")
        {
          size_t nQueries;
          file >> nQueries;
          queries_.resize(nQueries);
          for(size_t i = 0; i < nQueries; i++)
          {
            size_t n;
            file >> n;
            for(size_t j = 0; j < n; j++)
            {
              int idx;
              file >> idx;
              queries_[i].insert(idx);
            }
          }
        } else if(s == "pairs_")
        {
          readMatches(file);
        }
      }
    }
  }
  file.close();
  int nPts = static_cast<int>(pts_.size());
  for(int iPt = 0; iPt < nPts; iPt++)
  {
    for(const auto& camKey : pts_[iPt].views)
      cams_[camKey.first]->visiblePoints().push_back(iPt);

    for(const auto& camKey : pts_[iPt].viewsToAdd)
      cams_[camKey.first]->visiblePoints().push_back(iPt);
  }
}
void Dataset::readPts(istream& file)
{
  string s;
  int nPts,nFields;
  file >> nPts >> nFields;
  int format = 0;
  for(int i = 0; i < nFields; i++)
  {
    file >> s;
    if(s == "coord")
      format |= 1;
    else if(s == "views")
      format |= 2;
    else if(s == "viewsToAdd")
      format |= 4;
    else if(s == "color")
      format |= 8;
  }
  pts_.resize(nPts);
  for(auto& pt : pts_)
  {
    if(format & 1)
      file >> pt.coord(0) >> pt.coord(1) >> pt.coord(2);
    if(format & 2)
      file >> pt.views;
    if(format & 4)
      file >> pt.viewsToAdd;
    if(format & 8)
    {
      for(int i = 0; i < 3; i++)
      {
        int tmp;
        file >> tmp;
        pt.color(i) = tmp;
      }
    }
  }
}
void Dataset::readMatches(istream& file)
{
  string s;
  int nPairs,nFields;
  file >> nPairs >> nFields;
  int format = 0;
  for(int i = 0; i < nFields; i++)
  {
    file >> s;
    if(s == "matches")
      format |= 1;
    else if(s == "dists")
      format |= 2;
    else if(s == "supportSizes")
      format |= 4;
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
    if(format & 4)
    {
      int n;
      file >> n;
      pair.supportSizes.resize(n); 
      for(int& sz : pair.supportSizes)
        file >> sz;
    }
  }
}
void Dataset::readPointsOld(istream& file)
{
  int nFields;
  file >> nFields;
  string s;
  for(int iField = 0; iField < nFields; iField++)
  {
    file >> s;
    if(s == "matchesToReconstruct_")
    {
      size_t nMatches;
      file >> nMatches;
      nViewMatches_.resize(nMatches);
      for(auto& match : nViewMatches_)
        file >> match;
    } else if(s == "ptCoord_")
    {
      size_t n;
      file >> n;
      if(pts_.size() != n)
        pts_.resize(n);
      for(auto& pt : pts_)
        file >> pt.coord(0) >> pt.coord(1) >> pt.coord(2);
    } else if(s == "ptData_")
    {
      size_t n,nFieldsPointData;
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
      if(pts_.size() != n)
        pts_.resize(n);
      for(auto& pt : pts_)
      {
        if(format & 1)
          file >> pt.views;
        if(format & 2)
          file >> pt.viewsToAdd;
        if(format & 4)
        {
          for(int i = 0; i < 3; i++)
          {
            int tmp;
            file >> tmp;
            pt.color(i) = tmp;
          }
        }
      }
    }
  }
}

void Dataset::readKeysColors()
{
  for(const auto& cam : cams_)
    cam->readKeysColors();
}

int Dataset::countReconstructedObservations() const
{
  size_t nObs = 0;
  for(const auto& pt : pts_)
    nObs += pt.views.size();
  return static_cast<int>(nObs);
}

int Dataset::countPtsAlive() const
{
  int n = 0;
  for(const auto& pt : pts_)
    n += (!pt.views.empty());
  return n;
}

const string& Dataset::dir() const { return dir_; }
string& Dataset::dir() { return dir_; }
const Camera& Dataset::cam(int idx) const { return *cams_[idx]; }
const Camera& Dataset::cam(size_t idx) const { return *cams_[idx]; }
Camera& Dataset::cam(int idx) { return *cams_[idx]; }
Camera& Dataset::cam(size_t idx) { return *cams_[idx]; }
const ptr_vector<Camera>& Dataset::cams() const { return cams_; }
ptr_vector<Camera>& Dataset::cams() { return cams_; }
const pair_umap<CameraPair>& Dataset::pairs() const { return pairs_; }
pair_umap<CameraPair>& Dataset::pairs() { return pairs_; }
const uset<int>& Dataset::reconstructedCams() const { return reconstructedCams_; }
const vector<set<int>>& Dataset::queries() const { return queries_; }
vector<set<int>>& Dataset::queries() { return queries_; }
const vector<NViewMatch>& Dataset::nViewMatches() const { return nViewMatches_; }
vector<NViewMatch>& Dataset::nViewMatches() { return nViewMatches_; }
const vector<Point>& Dataset::pts() const { return pts_; }
vector<Point>& Dataset::pts() { return pts_; }

string Dataset::featsDir() const
{
  return joinPaths(dir_,"keys");
}

} // namespace yasfm