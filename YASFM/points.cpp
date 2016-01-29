#include "points.h"

#include <algorithm>
#include <iostream>
#include <queue>

#include "utils.h"

using Eigen::JacobiSVD;
using Eigen::Matrix4d;
using Eigen::MatrixXd;
using Eigen::Vector4d;
using Eigen::VectorXd;
using std::cerr;
using std::cout;
using std::queue;

namespace yasfm
{

void twoViewMatchesToNViewMatches(const ptr_vector<Camera>& cams,
  const pair_umap<CameraPair>& pairs,
  vector<NViewMatch> *nViewMatches, 
  FindNVMCallbackFunctionPtr callbackFunction, void * callbackObjectPtr)
{
  pair_umap<vector<int>> matches;
  vector<uset<int>> matchedCams;
  convertMatchesToLocalRepresentation(cams,pairs,&matchedCams,&matches);

  vector<vector<bool>> visitedFeats;
  visitedFeats.resize(cams.size());
  for(size_t i = 0; i < cams.size(); i++)
  {
    visitedFeats[i].resize(cams[i]->keys().size(),false);
  }

  int nCams = static_cast<int>(cams.size());
  for(int camIdx = 0; camIdx < nCams; camIdx++)
  {
    int nFeatures = static_cast<int>(cams[camIdx]->keys().size());
    for(int featIdx = 0; featIdx < nFeatures; featIdx++)
    {
      if(!visitedFeats[camIdx][featIdx])
      {
        findNViewMatch(matchedCams,matches,camIdx,featIdx,&visitedFeats,nViewMatches);
		if (callbackFunction != NULL&&callbackObjectPtr != NULL){
			callbackFunction(callbackObjectPtr, camIdx / double(nCams));
		}
      }
    }
  }
}

void nViewMatchesToTwoViewMatches(const vector<NViewMatch>& nViewMatches,
  IntPair pair,vector<IntPair> *twoViewMatches,vector<int> *nViewMatchesIdxs)
{
  int camIdxFrom = pair.first;
  int camIdxTo = pair.second;
  int numNView = static_cast<int>(nViewMatches.size());
  for(int iNView = 0; iNView < numNView; iNView++)
  {
    const auto& nViewMatch = nViewMatches[iNView];
    if(nViewMatch.count(camIdxFrom) > 0 && nViewMatch.count(camIdxTo) > 0)
    {
      twoViewMatches->emplace_back(nViewMatch.at(camIdxFrom),nViewMatch.at(camIdxTo));
      nViewMatchesIdxs->push_back(iNView);
    }
  }
}

int reconstructPoints(const vector<NViewMatch>& nViewMatches,
  const vector<int>& nViewMatchIdxs,const IntPair& camsIdxs,
  Camera* pcam1,Camera* pcam2,vector<Point> *pts)
{
  auto& cam1 = *pcam1;
  auto& cam2 = *pcam2;
  Matrix34d Rt1 = cam1.pose();
  Matrix34d Rt2 = cam2.pose();

  int nPtsBefore = static_cast<int>(pts->size());
  pts->reserve(pts->size() + nViewMatchIdxs.size());

  for(size_t i = 0; i < nViewMatchIdxs.size(); i++)
  {
    const auto& match = nViewMatches[nViewMatchIdxs[i]];
    int key1Idx = match.at(camsIdxs.first);
    int key2Idx = match.at(camsIdxs.second);
    Vector2d key1 = cam1.keyNormalized(key1Idx);
    Vector2d key2 = cam2.keyNormalized(key2Idx);

    Point pt;
    triangulate(Rt1,Rt2,key1,key2,&pt.coord);

    bool isInFrontOfBoth = 
      isInFrontNormalizedP(Rt1,pt.coord) && isInFrontNormalizedP(Rt2,pt.coord);

    if(isInFrontOfBoth)
    {
      pt.views.emplace(camsIdxs.first,key1Idx);
      pt.views.emplace(camsIdxs.second,key2Idx);
      for(const auto& camKey : match)
      {
        if(camKey.first != camsIdxs.first && camKey.first != camsIdxs.second)
          pt.viewsToAdd.insert(camKey);
      }

      if(!cam1.keysColors().empty())
        pt.color = cam1.keyColor(key1Idx);

      pts->push_back(pt);
    }
  }

  int nPtsNew = static_cast<int>(pts->size()) - nPtsBefore;
  for(int iNew = 0; iNew < nPtsNew; iNew++)
  {
    cam1.visiblePoints().push_back(nPtsBefore + iNew);
    cam2.visiblePoints().push_back(nPtsBefore + iNew);
  }
  return nPtsNew;
}

int reconstructPoints(const vector<SplitNViewMatch>& matchesToReconstruct,
  ptr_vector<Camera>* pcams,vector<Point> *ppts)
{
  auto& cams = *pcams;
  auto& pts = *ppts;
  vector<Matrix34d> Rts(cams.size());
  vector<bool> RtsValid(cams.size(),false);

  int nPtsBefore = static_cast<int>(pts.size());
  pts.reserve(pts.size() + matchesToReconstruct.size());

  for(const auto& match : matchesToReconstruct)
  {
    vector<Vector2d> keys;
    vector<int> camIdxs;
    keys.reserve(match.observedPart.size());
    camIdxs.reserve(match.observedPart.size());
    for(const auto& camKey : match.observedPart)
    {
      int camIdx = camKey.first;
      camIdxs.push_back(camIdx);
      if(!RtsValid[camIdx])
      {
        Rts[camIdx] = cams[camIdx]->pose();
        RtsValid[camIdx] = true;
      }
      keys.emplace_back(cams[camIdx]->keyNormalized(camKey.second));
    }

    Point pt;
    if(match.observedPart.size() == 2)
      triangulate(Rts[camIdxs[0]],Rts[camIdxs[1]],keys[0],keys[1],&pt.coord);
    else
      triangulate(Rts,camIdxs,keys,&pt.coord);

    bool isInFrontOfAll = true;
    for(size_t iProj = 0; iProj < camIdxs.size(); iProj++)
    {
      if(!isInFrontNormalizedP(Rts[camIdxs[iProj]],pt.coord))
      {
        isInFrontOfAll = false;
        break;
      }
    }

    if(isInFrontOfAll)
    {
      pt.views = match.observedPart;
      pt.viewsToAdd = match.unobservedPart;

      int cam0 = camIdxs[0];
      if(!cams[cam0]->keysColors().empty())
        pt.color = cams[cam0]->keyColor(match.observedPart.at(cam0));

      pts.push_back(pt);
    }
  }

  int nPts = static_cast<int>(pts.size());
  for(int iPt = nPtsBefore; iPt < nPts; iPt++)
  {
    for(const auto& camKey : pts[iPt].views)
      cams[camKey.first]->visiblePoints().push_back(iPt);

    for(const auto& camKey : pts[iPt].viewsToAdd)
      cams[camKey.first]->visiblePoints().push_back(iPt);
  }
  return nPts-nPtsBefore;
}

void triangulate(const Matrix34d& P1,const Matrix34d& P2,const vector<Vector2d>& keys1,
  const vector<Vector2d>& keys2,const vector<IntPair>& matches,vector<Vector3d> *ppts)
{
  auto& pts = *ppts;
  pts.resize(matches.size());
  for(size_t i = 0; i < pts.size(); i++)
  {
    triangulate(P1,P2,keys1[matches[i].first],keys2[matches[i].second],&pts[i]);
  }
}

void triangulate(const Matrix34d& P1,const Matrix34d& P2,const Vector2d& key1,
  const Vector2d& key2,Vector3d *pt)
{
  Matrix4d D,S;
  D.row(0) = key1(0) * P1.row(2) - P1.row(0);
  D.row(1) = key1(1) * P1.row(2) - P1.row(1);
  D.row(2) = key2(0) * P2.row(2) - P2.row(0);
  D.row(3) = key2(1) * P2.row(2) - P2.row(1);

  // S for numerical conditioning
  Vector4d maxCols = D.cwiseAbs().colwise().maxCoeff();
  S = maxCols.cwiseInverse().asDiagonal();

  JacobiSVD<Matrix4d> svd(D*S,Eigen::ComputeFullV);

  Vector4d ptHom;
  ptHom.noalias() = S*(svd.matrixV().rightCols(1));
  *pt = ptHom.hnormalized();
}

void triangulate(const vector<Matrix34d>& Ps,const vector<int>& PsToUse,
  const vector<Vector2d>& keys,Vector3d *pt)
{
  MatrixXd A;
  VectorXd b;
  A.resize(2 * PsToUse.size(),3);
  b.resize(2 * PsToUse.size());
  for(size_t i = 0; i < PsToUse.size(); i++)
  {
    const auto& P = Ps[PsToUse[i]];
    const auto& key = keys[i];
    A.row(2 * i) = key(0) * P.block(2,0,1,3) - P.block(0,0,1,3);
    A.row(2 * i + 1) = key(1) * P.block(2,0,1,3) - P.block(1,0,1,3);
    b(2 * i) = P(0,3) - key(0) * P(2,3);
    b(2 * i + 1) = P(1,3) - key(1) * P(2,3);
  }

  *pt = A.jacobiSvd(Eigen::ComputeThinU | Eigen::ComputeThinV).solve(b);
}

bool isInFront(const Matrix34d& P,const Vector3d& point)
{
  double projCoord2 = P.row(2).dot(point.homogeneous());
  return (projCoord2*P.leftCols(3).determinant()) > 0.;
}

bool isInFrontNormalizedP(const Matrix34d& P,const Vector3d& point)
{
  double projCoord2 = P.row(2).dot(point.homogeneous());
  return projCoord2 > 0.;
}

void extractCandidateNewPoints(int minObservingCams,double rayAngleThresh,
  const uset<int>& reconstructedCams,const ptr_vector<Camera>& cams,
  vector<NViewMatch> *pnViewMatches,vector<SplitNViewMatch> *pextracted)
{
  auto& matches = *pnViewMatches;
  auto& extracted = *pextracted;
  vector<int> toReconstructIdxs;
  extracted.emplace_back();
  int nMatches = static_cast<int>(matches.size());
  for(int iMatch = 0; iMatch < nMatches; iMatch++)
  {
    auto& curr = extracted.back();
    for(const auto& camKey : matches[iMatch])
    {
      int camIdx = camKey.first;
      if(reconstructedCams.count(camIdx) > 0)
      {
        curr.observedPart.emplace(camIdx,camKey.second);
      } else
      {
        curr.unobservedPart.emplace(camIdx,camKey.second);
      }
    }
    if(curr.observedPart.size() >= minObservingCams &&
      isWellConditioned(rayAngleThresh,cams,curr.observedPart))
    {
      toReconstructIdxs.push_back(iMatch);
      extracted.emplace_back();
    } else
    {
      curr.observedPart.clear();
      curr.unobservedPart.clear();
    }
  }
  extracted.pop_back();
  filterOutOutliers(toReconstructIdxs,&matches);
}

void findCamToSceneMatches(const uset<int>& camsToIgnore,int numCams,
  const vector<Point>& pts,vector<vector<IntPair>> *pmatches)
{
  auto &matches = *pmatches;
  matches.resize(numCams);
  for(int ptIdx = 0; ptIdx < static_cast<int>(pts.size()); ptIdx++)
  {
    for(const auto& camKey : pts[ptIdx].viewsToAdd)
    {
      if(camsToIgnore.count(camKey.first) == 0)
      {
        matches[camKey.first].emplace_back(camKey.second,ptIdx);
      }
    }
  }
}

bool isWellConditioned(double rayAngleThresh,const ptr_vector<Camera>& cams,
  const NViewMatch& pointViews)
{
  auto camKey1 = pointViews.begin();
  double rayAngleThreshRadians = deg2Rad(rayAngleThresh);
  for(; camKey1 != pointViews.end(); ++camKey1)
  {
    const auto& cam1 = *cams[camKey1->first];
    auto camKey2 = camKey1;
    ++camKey2;
    for(; camKey2 != pointViews.end(); ++camKey2)
    {
      const auto& cam2 = *cams[camKey2->first];
      double angle = computeRayAngle(cam1,camKey1->second,
        cam2,camKey2->second);
      if(angle > rayAngleThreshRadians)
        return true;
    }
  }
  return false;
}

double computeRayAngle(const Camera& cam1,int key1Idx,
  const Camera& cam2,int key2Idx)
{
  Vector2d key1Norm = cam1.keyNormalized(key1Idx);
  Vector2d key2Norm = cam2.keyNormalized(key2Idx);
  Vector3d ray1 = cam1.R().transpose() * key1Norm.homogeneous();
  Vector3d ray2 = cam2.R().transpose() * key2Norm.homogeneous();
  return acos(ray1.dot(ray2) / (ray1.norm() * ray2.norm()));
}

int removeIllConditionedPoints(double rayAngleThresh,
  ptr_vector<Camera> *pcams,vector<Point> *ppts)
{
  auto& cams = *pcams;
  auto& pts = *ppts;
  int nPts = static_cast<int>(pts.size());
  int nPtsRemoved = 0;
  for(int ptIdx = 0; ptIdx < nPts; ptIdx++)
  {
    bool wellConditioned = false;
    auto& pt = pts[ptIdx];
    auto camKey1 = pt.views.begin();
    for(; camKey1 != pt.views.end() && !wellConditioned; ++camKey1)
    {
      Vector3d ray1 = pt.coord - cams[camKey1->first]->C();
      ray1.normalize();
      auto camKey2 = camKey1;
      ++camKey2;
      for(; camKey2 != pt.views.end() && !wellConditioned; ++camKey2)
      {
        Vector3d ray2 = pt.coord - cams[camKey2->first]->C();
        ray2.normalize();
        double angle = rad2Deg(acos(ray1.dot(ray2)));
        wellConditioned = angle > rayAngleThresh;
      }
    }
    if(!wellConditioned && !pt.views.empty())
    {
      nPtsRemoved++;
      removePointViews(ptIdx,&pt,&cams);
    }
  }
  return nPtsRemoved;
}

int removeHighReprojErrorPoints(double avgReprojErrThresh,ptr_vector<Camera> *pcams,
  vector<Point> *ppts)
{
  auto& cams = *pcams;
  auto& pts = *ppts;
  int nPts = static_cast<int>(pts.size());
  int nPtsRemoved = 0;
  for(int ptIdx = 0; ptIdx < nPts; ptIdx++)
  {
    auto& pt = pts[ptIdx];
    double err = 0;
    for(const auto& camKey : pt.views)
    {
      const auto& cam = *cams[camKey.first];
      const auto& key = cam.key(camKey.second);
      Vector2d proj = cam.project(pt.coord);
      err += (proj - key).norm();
    }
    if(!pt.views.empty())
      err /= pt.views.size();
    if(err > avgReprojErrThresh)
    {
      nPtsRemoved++;
      removePointViews(ptIdx,&pt,&cams);
    }
  }
  return nPtsRemoved;
}

void removePointViews(int ptIdx,Point *pt,ptr_vector<Camera> *pcams)
{
  auto& cams = *pcams;
  for(const auto& camKey : pt->views)
  {
    auto& visPts = cams[camKey.first]->visiblePoints();
    auto elem = std::lower_bound(visPts.begin(),visPts.end(),ptIdx);
    if(elem != visPts.end())
      visPts.erase(elem);
  }
  for(const auto& camKey : pt->viewsToAdd)
  {
    auto& visPts = cams[camKey.first]->visiblePoints();
    auto elem = std::lower_bound(visPts.begin(),visPts.end(),ptIdx);
    if(elem != visPts.end())
      visPts.erase(elem);
  }
  pt->views.clear();
  pt->viewsToAdd.clear();
}

} // namespace yasfm

namespace
{

void findNViewMatch(const vector<uset<int>>& matchedCams,
  const pair_umap<vector<int>>& matches,int startCamIdx,int startFeatIdx,
  vector<vector<bool>> *pvisitedFeats,vector<NViewMatch> *nViewMatches)
{
  auto& visitedFeats = *pvisitedFeats;
  bool isConsistent = true; // no two different features in the same image
  nViewMatches->emplace_back();
  auto& nViewMatch = nViewMatches->back();

  // the .first symbolizes camIdx and .second respective featIdx
  queue<IntPair> q;

  q.emplace(startCamIdx,startFeatIdx);
  visitedFeats[startCamIdx][startFeatIdx] = true;

  while(!q.empty())
  {
    int cam1 = q.front().first;
    int feat1 = q.front().second;
    q.pop();

    if(isConsistent && (nViewMatch.count(cam1) == 0))
    {
      nViewMatch[cam1] = feat1;
    } else
    {
      isConsistent = false;
    }

    for(int cam2 : matchedCams[cam1])
    {
      int feat2 = matches.at(IntPair(cam1,cam2))[feat1];
      if(feat2 >= 0 && !visitedFeats[cam2][feat2])
      {
        q.emplace(cam2,feat2);
        visitedFeats[cam2][feat2] = true;
      }
    }
  }
  if(!isConsistent || nViewMatch.size() < 2)
  {
    nViewMatches->pop_back();
  }
}

void convertMatchesToLocalRepresentation(const ptr_vector<Camera>& cams,
  const pair_umap<CameraPair>& pairs,vector<uset<int>> *pmatchedCams,
  pair_umap<vector<int>> *pmatches)
{
  auto& matchedCams = *pmatchedCams;
  auto& matches = *pmatches;
  matchedCams.resize(cams.size());
  for(const auto& pair : pairs)
  {
    IntPair idx = pair.first;
    int cam1 = idx.first;
    int cam2 = idx.second;
    IntPair reversedIdx(cam2,cam1);

    matchedCams[cam1].insert(cam2);
    matchedCams[cam2].insert(cam1);
    matches[idx].resize(cams[cam1]->keys().size(),-1);
    matches[reversedIdx].resize(cams[cam2]->keys().size(),-1);

    for(const auto& origMatch : pair.second.matches)
    {
      matches[idx][origMatch.first] = origMatch.second;
      matches[reversedIdx][origMatch.second] = origMatch.first;
    }
  }
}

} // namespace