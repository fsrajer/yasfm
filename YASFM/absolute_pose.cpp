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

#include <iostream>

using Eigen::JacobiSVD;
using Eigen::MatrixXd;
using Eigen::VectorXd;

using std::cerr;
using std::cout;

namespace yasfm
{

bool resectCamera5AndHalfPtRANSAC(const OptionsRANSAC& opt,
  const vector<IntPair>& camToSceneMatches,const vector<Vector3d>& points,
  Camera *cam,vector<int> *inliers)
{
  Matrix34d P;
  bool success = resectCamera5AndHalfPtRANSAC(opt,camToSceneMatches,cam->keys(),points,&P,inliers);
  if(success)
    cam->setParams(P);
  return success;
}

bool resectCamera5AndHalfPtRANSAC(const OptionsRANSAC& opt,
  const vector<IntPair>& camToSceneMatches,const vector<Vector2d>& keys,
  const vector<Vector3d>& points,
  Matrix34d *P,vector<int> *inliers)
{
  MediatorResectioning5AndHalfPtRANSAC m(keys,points,camToSceneMatches);
  int nInliers = estimateTransformRANSAC(m,opt,P,inliers);
  return (nInliers > 0);
}

void resectCamera5AndHalfPt(const vector<Vector2d>& keys,const vector<Vector3d>& points,
  const vector<IntPair>& camToSceneMatches,vector<Matrix34d> *Ps)
{
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

bool resectCamera6ptLSRANSAC(const OptionsRANSAC& opt,
  const vector<IntPair>& camToSceneMatches,const vector<Vector3d>& points,
  Camera *cam,vector<int> *inliers)
{
  Matrix34d P;
  bool success = resectCamera6ptLSRANSAC(opt,camToSceneMatches,cam->keys(),points,&P,inliers);
  if(success)
    cam->setParams(P);
  return success;
}

bool resectCamera6ptLSRANSAC(const OptionsRANSAC& opt,
  const vector<IntPair>& camToSceneMatches,const vector<Vector2d>& keys,
  const vector<Vector3d>& points,
  Matrix34d *P,vector<int> *inliers)
{
  MediatorResectioning6ptLSRANSAC m(keys,points,camToSceneMatches);
  int nInliers = estimateTransformRANSAC(m,opt,P,inliers);
  return (nInliers > 0);
}

void resectCameraLS(const vector<Vector2d>& keys,const vector<Vector3d>& points,
  const vector<IntPair>& camToSceneMatches,Matrix34d *pP)
{
  const size_t minPts = 6;
  if(camToSceneMatches.size() < minPts)
    return;

  MatrixXd A(MatrixXd::Zero(2 * camToSceneMatches.size(),11));
  VectorXd b(VectorXd::Zero(2 * camToSceneMatches.size()));

  for(size_t i = 0; i < camToSceneMatches.size(); i++)
  {
    int keyIdx = camToSceneMatches[i].first;
    const auto& key = keys[keyIdx];
    int ptIdx = camToSceneMatches[i].second;
    const auto& pt = points[ptIdx];

    A.block(2*i,0,1,4) = pt.homogeneous().transpose();
    A.block(2*i,8,1,3) = -key.x() * pt.transpose();
    A.block(2*i+1,4,1,4) = pt.homogeneous().transpose();
    A.block(2*i+1,8,1,3) = -key.y() * pt.transpose();

    b(2*i) = key.x();
    b(2*i+1) = key.y();
  }

  MatrixXd X = A.jacobiSvd(Eigen::ComputeThinU | Eigen::ComputeThinV).solve(b);

  auto& P = (*pP);
  P.row(0) = X.topRows(4).transpose();
  P.row(1) = X.middleRows(4,4).transpose();
  P.row(2).leftCols(3) = X.bottomRows(3).transpose();
  P(2,3) = 1.;
}

} // namespace yasfm

namespace
{

MediatorResectioningRANSAC::MediatorResectioningRANSAC(int minMatches,
  const vector<Vector2d>& keys,const vector<Vector3d>& points,
  const vector<IntPair>& camToSceneMatches)
  : minMatches_(minMatches),keys_(keys),points_(points),
  camToSceneMatches_(camToSceneMatches)
{
}

int MediatorResectioningRANSAC::numMatches() const
{
  return static_cast<int>(camToSceneMatches_.size());
}

int MediatorResectioningRANSAC::minMatches() const
{ 
  return minMatches_; 
}

double MediatorResectioningRANSAC::computeSquaredError(const Matrix34d& P,int matchIdx) const
{
  const auto& pt = points_[camToSceneMatches_[matchIdx].second];
  const auto& key = keys_[camToSceneMatches_[matchIdx].first];

  Vector3d proj = (P*pt.homogeneous());
  return (key - proj.hnormalized()).squaredNorm();
}

void MediatorResectioningRANSAC::refine(const vector<int>& inliers,Matrix34d *P) const
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

bool MediatorResectioningRANSAC::isPermittedSelection(const vector<int>& idxs) const
{
  // TODO: Should we exclude some cases? (twisted cubic and plane)
  return true;
}

MediatorResectioning5AndHalfPtRANSAC::MediatorResectioning5AndHalfPtRANSAC(
  const vector<Vector2d>& keys,const vector<Vector3d>& points,
  const vector<IntPair>& camToSceneMatches)
  : MediatorResectioningRANSAC(6,keys,points,camToSceneMatches)
{
}

void MediatorResectioning5AndHalfPtRANSAC::computeTransformation(const vector<int>& idxs,
  vector<Matrix34d> *Ps) const
{
  vector<IntPair> selectedMatches;
  selectedMatches.reserve(minMatches_);
  for(int idx : idxs)
    selectedMatches.push_back(camToSceneMatches_[idx]);
  resectCamera5AndHalfPt(keys_,points_,selectedMatches,Ps);
}

MediatorResectioning6ptLSRANSAC::MediatorResectioning6ptLSRANSAC(
  const vector<Vector2d>& keys,const vector<Vector3d>& points,
  const vector<IntPair>& camToSceneMatches)
  : MediatorResectioningRANSAC(6,keys,points,camToSceneMatches)
{
}

void MediatorResectioning6ptLSRANSAC::computeTransformation(const vector<int>& idxs,
  vector<Matrix34d> *Ps) const
{
  vector<IntPair> selectedMatches;
  selectedMatches.reserve(minMatches_);
  for(int idx : idxs)
    selectedMatches.push_back(camToSceneMatches_[idx]);
  Ps->resize(1);
  resectCameraLS(keys_,points_,selectedMatches,&(*Ps)[0]);
}

} // namespace