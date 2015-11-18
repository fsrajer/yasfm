/*
* Filip Srajer
* filip.srajer (at) fel.cvut.cz
* Center for Machine Perception
* Czech Technical University in Prague
*
* This software is under construction.
* 05/2015
*/

#include "absolute_pose.h"

#include <cassert>
#include <iostream>
#include <vector>
#include <unordered_set>
#include "Eigen\Dense"
#include "Eigen\SVD"

#include "defines.h"
#include "options.h"
#include "ransac.h"
#include "utils.h"
#include "sfm_data.h"

using Eigen::Vector2d;
using Eigen::Vector3d;
using Eigen::Vector4d;
using Eigen::VectorXd;
using Eigen::Matrix3d;
using Eigen::Matrix4d;
using Eigen::MatrixXd;
using Eigen::Matrix;
using Eigen::JacobiSVD;
using namespace yasfm;
using std::vector;
using std::unordered_set;
using std::cout;
using std::cerr;
using std::endl;

namespace yasfm
{

bool resectCamera6ptRANSAC(const OptionsRANSAC& opt,
  const vector<IntPair>& camToSceneMatches,const vector<Vector3d>& points,
  Camera *cam,vector<int> *inliers)
{
  Matrix34d P;
  bool success = resectCamera6ptRANSAC(opt,camToSceneMatches,cam->keys(),points,&P,inliers);
  if(success)
    cam->setParams(P);
  return success;
}

bool resectCamera6ptRANSAC(const OptionsRANSAC& opt,
  const vector<IntPair>& camToSceneMatches,const vector<Vector2d>& keys,
  const vector<Vector3d>& points,
  Matrix34d *P,vector<int> *inliers)
{
  MediatorResectioning6ptRANSAC m(keys,points,camToSceneMatches);
  int nInliers = estimateTransformRANSAC(m,opt,P,inliers);
  return (nInliers > 0);
}

void resectCamera6ptMinimal(const vector<Vector2d>& keys,const vector<Vector3d>& points,
  const vector<IntPair>& camToSceneMatches,vector<Matrix34d> *Ps)
{
  /*const size_t minPts = 6;
  assert(cam2SceneMatches.size() >= minPts);

  MatrixXd A,b;
  A.resize(2 * cam2SceneMatches.size(), 11);
  A.setZero();
  b.resize(2 * cam2SceneMatches.size(),1);
  for(size_t i = 0; i < cam2SceneMatches.size(); i++)
  {
    int ptIdx = cam2SceneMatches[i].second;
    const auto& pt = points[ptIdx].transpose();
    int keyIdx = cam2SceneMatches[i].first;
    const auto& key = cam.feature(keyIdx);

    A.block(2 * i,    0,1,4) = pt.homogeneous();
    A.block(2 * i,    8,1,3) = -key.x() * pt;
    A.block(2 * i + 1,4,1,4) = pt.homogeneous();
    A.block(2 * i + 1,8,1,3) = -key.y() * pt;

    b(2 * i) = key.x();
    b(2 * i + 1) = key.y();
  }

  MatrixXd X = A.jacobiSvd(Eigen::ComputeThinU | Eigen::ComputeThinV).solve(b);

  Ps.resize(1);
  Ps[0].row(0) = X.topRows(4).transpose();
  Ps[0].row(1) = X.block(4,0,4,1).transpose();
  Ps[0].row(2).leftCols(3) = X.bottomRows(3).transpose();
  Ps[0](2,3) = 1.;*/

  const size_t minPts = 6;
  if(camToSceneMatches.size() < minPts)
    return;

  MatrixXd Ms[2];
  Ms[0].resize(11,12);
  Ms[0].setZero();
  for(size_t i = 0; i < 6; i++)
  {
    const auto& key = keys[camToSceneMatches[i].first];
    int ptIdx = camToSceneMatches[i].second;
    const auto& pt = points[ptIdx].homogeneous().transpose();

    Ms[0].block(2 * i,0,1,4) = pt;
    Ms[0].block(2 * i,8,1,4) = -key.x() * pt;

    if(i != 5)
    {
      Ms[0].block(2 * i + 1,4,1,4) = pt;
      Ms[0].block(2 * i + 1,8,1,4) = -key.y() * pt;
    } else
    {
      Ms[1] = Ms[0];
      Ms[1].block(2 * i,4,1,4) = pt;
      Ms[1].block(2 * i,8,1,4) = -key.y() * pt;
    }
  }

  Ps->resize(2);
  for(size_t i = 0; i < 2; i++)
  {
    JacobiSVD<MatrixXd> svd(Ms[i],Eigen::ComputeFullV);

    const auto& p = svd.matrixV().rightCols(1);
    for(size_t j = 0; j < 3; j++)
    {
      (*Ps)[i].row(j) = p.block(j * 4,0,4,1).transpose();
    }
  }
}

} // namespace yasfm

namespace
{

MediatorResectioning6ptRANSAC::MediatorResectioning6ptRANSAC(const vector<Vector2d>& keys,
  const vector<Vector3d>& points,const vector<IntPair>& camToSceneMatches)
  : minMatches_(6),keys_(keys),points_(points),camToSceneMatches_(camToSceneMatches)
{
}

int MediatorResectioning6ptRANSAC::numMatches() const
{
  return static_cast<int>(camToSceneMatches_.size());
}

int MediatorResectioning6ptRANSAC::minMatches() const
{ 
  return minMatches_; 
}

void MediatorResectioning6ptRANSAC::computeTransformation(const vector<int>& idxs,
  vector<Matrix34d> *Ps) const
{
  vector<IntPair> selectedMatches;
  selectedMatches.reserve(minMatches_);
  for(int idx : idxs)
    selectedMatches.push_back(camToSceneMatches_[idx]);
  resectCamera6ptMinimal(keys_,points_,selectedMatches,Ps);
}

double MediatorResectioning6ptRANSAC::computeSquaredError(const Matrix34d& P,int matchIdx) const
{
  const auto& pt = points_[camToSceneMatches_[matchIdx].second];
  const auto& key = keys_[camToSceneMatches_[matchIdx].first];

  Vector3d proj = (P*pt.homogeneous());
  return (key - proj.hnormalized()).squaredNorm();
}

void MediatorResectioning6ptRANSAC::refine(const vector<int>& inliers,Matrix34d *P) const
{
  // TODO: Find LM lib for non-linear minimization
  /*vector<IntPair> selectedMatches;
  selectedMatches.reserve(inliers.size());
  for(int idx : inliers)
    selectedMatches.push_back(cam2SceneMatches_[idx]);
  vector<Matrix34d> tmp;
  yasfm::resectCameraMinimal6pt(cam_,points_,selectedMatches,tmp);
  P = tmp[0];*/
}

bool MediatorResectioning6ptRANSAC::isPermittedSelection(const vector<int>& idxs) const
{
  // TODO: Should we exclude some cases? (twisted cubic and plane)
  return true;
}

} // namespace