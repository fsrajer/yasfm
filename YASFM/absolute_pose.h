/*
* Filip Srajer
* filip.srajer (at) fel.cvut.cz
* Center for Machine Perception
* Czech Technical University in Prague
*
* This software is under construction.
* 05/2015
*/

#pragma once

#include <vector>

#include "Eigen\Dense"

#include "defines.h"
#include "options.h"
#include "ransac.h"
#include "sfm_data.h"

using Eigen::Vector2d;
using Eigen::Vector3d;
using std::vector;
using namespace yasfm;

namespace yasfm
{

// Returns false if the estimated hypothesis was not supported by
// enough inliers.
YASFM_API bool resectCamera5AndHalfPtRANSAC(const OptionsRANSAC& opt,
  const vector<IntPair>& camToSceneMatches,const vector<Vector3d>& points,
  Camera *cam,vector<int> *inliers = nullptr);

// Returns false if the estimated hypothesis was not supported by
// enough inliers.
YASFM_API bool resectCamera5AndHalfPtRANSAC(const OptionsRANSAC& opt,
  const vector<IntPair>& camToSceneMatches,const vector<Vector2d>& keys,
  const vector<Vector3d>& points,
  Matrix34d *P,vector<int> *inliers = nullptr);

// Minimal solver. 6 points give 12 equations. P matrix has 11 degrees of freedom. 
// This solver takes 5 points for 10 equations and 6th point is used twice 
// for the 11-th equation. This gives 2 solutions.
YASFM_API void resectCamera5AndHalfPt(const vector<Vector2d>& keys,
  const vector<Vector3d>& points,const vector<IntPair>& camToSceneMatches,
  vector<Matrix34d> *Ps);

// Returns false if the estimated hypothesis was not supported by
// enough inliers.
YASFM_API bool resectCamera6ptLSRANSAC(const OptionsRANSAC& opt,
  const vector<IntPair>& camToSceneMatches,const vector<Vector3d>& points,
  Camera *cam,vector<int> *inliers = nullptr);

// Returns false if the estimated hypothesis was not supported by
// enough inliers.
YASFM_API bool resectCamera6ptLSRANSAC(const OptionsRANSAC& opt,
  const vector<IntPair>& camToSceneMatches,const vector<Vector2d>& keys,
  const vector<Vector3d>& points,
  Matrix34d *P,vector<int> *inliers = nullptr);

// Least squares solver for projection matrix.
YASFM_API void resectCameraLS(const vector<Vector2d>& keys,
  const vector<Vector3d>& points,const vector<IntPair>& camToSceneMatches,
  Matrix34d *P);

} // namespace yasfm

namespace
{

class MediatorResectioningRANSAC: public MediatorRANSAC<Matrix34d>
{
public:
  // Mind the order of points. We estimate such F that pts2'*F*pts1 = 0.
  MediatorResectioningRANSAC(int minMatches,const vector<Vector2d>& keys,
    const vector<Vector3d>& points,const vector<IntPair>& camToSceneMatches);
  virtual int numMatches() const;
  virtual int minMatches() const;
  virtual double computeSquaredError(const Matrix34d& P,int matchIdx) const;
  virtual void refine(const vector<int>& inliers, Matrix34d *P) const;
  virtual bool isPermittedSelection(const vector<int>& idxs) const;

protected:
  const int minMatches_;
  const vector<Vector2d>& keys_;
  const vector<Vector3d>& points_;
  const vector<IntPair>& camToSceneMatches_;
};

class MediatorResectioning5AndHalfPtRANSAC : public MediatorResectioningRANSAC
{
public:
  // Mind the order of points. We estimate such F that pts2'*F*pts1 = 0.
  MediatorResectioning5AndHalfPtRANSAC(const vector<Vector2d>& keys,
    const vector<Vector3d>& points,const vector<IntPair>& camToSceneMatches);
  virtual void computeTransformation(const vector<int>& idxs,vector<Matrix34d> *Ps) const;
};

class MediatorResectioning6ptLSRANSAC : public MediatorResectioningRANSAC
{
public:
  // Mind the order of points. We estimate such F that pts2'*F*pts1 = 0.
  MediatorResectioning6ptLSRANSAC(const vector<Vector2d>& keys,
    const vector<Vector3d>& points,const vector<IntPair>& camToSceneMatches);
  virtual void computeTransformation(const vector<int>& idxs,vector<Matrix34d> *Ps) const;
};

} // namespace