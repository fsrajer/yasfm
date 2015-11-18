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

using Eigen::ArrayXXi;
using Eigen::Matrix3d;
using Eigen::Vector2d;
using Eigen::Vector3d;
using Eigen::Vector4d;
using Eigen::VectorXd;
using std::vector;
using namespace yasfm;

namespace yasfm
{

// All cameras with the same priority. Returns (-1,-1) when there are no good matches.
YASFM_API IntPair chooseInitialCameraPair(int minMatches,
  const vector<NViewMatch>& nViewMatches,int numCams);
// A priority level is chosen such that at least one pair has more than
// minMatches. A pair having the most matches among pairs with the chosen
// priority is chosen. Choose priority in interval [0,10].
// Returns (-1,-1) when there are no good matches.
YASFM_API IntPair chooseInitialCameraPair(int minMatches,
  const vector<NViewMatch>& nViewMatches,const vector<int>& camsPriority);
// A priority level is chosen such that at least one pair has more than
// minMatches. A pair having the most matches among pairs with the chosen
// priority is chosen. Choose priority in interval [0,10].
// numMatchesOfPairs should be symmetric. Returns (-1,-1) when there are 
// no good matches.
YASFM_API IntPair chooseInitialCameraPair(int minMatches,
  const ArrayXXi& numMatchesOfPairs,const vector<int>& camsPriority);

// 1) Initialize their cameras based on their 
// relative position estimated from common n-view matches.
// Use 7-pt algorithm.
// 2) Triangulate the common n-view matches.
YASFM_API void initReconstructionFromCamPair(const Options& opt,
  const IntPair& initPair,Dataset *data);
// 1) Initialize thei cameras based on their 
// relative position estimated from common n-view matches.
// Use 5-pt algorithm.
// 2) Triangulate the common n-view matches.
YASFM_API void initReconstructionFromCalibratedCamPair(const Options& opt,
  const IntPair& initPair,Dataset *data);

// Initializes projection matrices of a pair of cameras
// from their fundamental matrix. The first camera P1 is assumed to be:
// P1 = [1 0 0 0]
//      [0 1 0 0]
//      [0 0 1 0]
// and P2 = [[e]_x*F | e] where e is the epipole (e*F = 0)
YASFM_API void F2Ps(const Matrix3d& F,Matrix34d *P2);

// Creates from equation E = K2^T * F * K1 and by forcing
// both singular values of E to be equal (E(1,1) <- E(0,0)).
// Note that this makes the original F inconsistent with E.
YASFM_API void FK2E(const Matrix3d& F, const Matrix3d& K1, const Matrix3d& K2, Matrix3d *E);

// Decomposes E. First camera is assumed to have R identity and C origin.
// Second camera's R and C is returned.
YASFM_API void E2RC(const Matrix3d& E,const Matrix3d& K1,const Matrix3d& K2,
  const vector<IntPair>& matches,const vector<Vector2d>& keys1,
  const vector<Vector2d>& keys2,Matrix3d *R,Vector3d *C);

// For every camera pair, estimates fundamental matrix 
// using PROSAC (that is why we need dists). Then, only matches
// which are inliers to the fundamental matrix are kept. (as well as
// corresponding dists.
YASFM_API void verifyMatchesGeometrically(const Options& opt, const ptr_vector<Camera>& cams,
  pair_umap<CameraPair> *pairs);

// Robust estimator, which finds such a fundamental matrix that
// pts2'*F*pts1 = 0 using 7 point algorithm as a minimal solver. 
// MIND THE ORDER of the input points.
// The algorithm samples matches with smaller dist more often than
// those with higher.
// Returns false if the estimated hypothesis was not supported by
// enough inliers.
YASFM_API bool estimateRelativePose7ptPROSAC(const OptionsRANSAC& opt,
  const vector<Vector2d>& keys1,const vector<Vector2d>& keys2,const CameraPair& pair,Matrix3d *F,
  vector<int> *inliers = nullptr);

// Robust estimator, which finds such a fundamental matrix that
// pts2'*F*pts1 = 0 using 7 point algorithm as a minimal solver. 
// MIND THE ORDER of the input points.
// Returns false if the estimated hypothesis was not supported by
// enough inliers.
YASFM_API bool estimateRelativePose7ptRANSAC(const OptionsRANSAC& opt,
  const vector<Vector2d>& keys1,const vector<Vector2d>& keys2,const vector<IntPair>& matches,
  Matrix3d *F,vector<int> *inliers = nullptr);

// Minimal solver, which finds such a fundamental matrix that
// pts2'*F*pts1 = 0 using 7 point algorithm. MIND THE ORDER of the input points.
// There are 1 to 3 solutions which are all possible and need to be 
// verified by computing support.
// Takes only the first 7 matches if the matches vector would contain more.
YASFM_API void estimateRelativePose7pt(const vector<Vector2d>& keys1, const vector<Vector2d>& keys2,
  const vector<IntPair>& matches,vector<Matrix3d> *Fs);

// Robust estimator, which finds such an essential matrix that
// (inv(K)*pts2)'*E*(inv(K2)*pts1) = 0 using 5 point algorithm. 
// MIND THE ORDER of the input points. 
// Returns false if the estimated hypothesis was not supported by
// enough inliers.
YASFM_API bool estimateRelativePose5ptRANSAC(const OptionsRANSAC& opt,
  const Camera& cam1,const Camera& cam2,const vector<IntPair>& matches,Matrix3d *E,
  vector<int> *inliers = nullptr);

// Minimal solver, which finds such an essential matrix that
// (inv(K)*pts2)'*E*(inv(K2)*pts1) = 0 using 5 point algorithm. 
// MIND THE ORDER of the input points.
// Takes only the first 5 matches if the matches vector would contain more.
YASFM_API void estimateRelativePose5pt(const vector<Vector3d>& pts1Norm,
  const vector<Vector3d>& pts2Norm,const vector<IntPair>& matches,
  vector<Matrix3d> *Es);

} // namespace yasfm

namespace
{

class Mediator7ptPROSAC : public MediatorPROSAC<Matrix3d>
{
public:
  // Mind the order of points. We estimate such F that pts2'*F*pts1 = 0.
  Mediator7ptPROSAC(const vector<Vector2d>& keys1,const vector<Vector2d>& keys2,
    const CameraPair& pair);
  virtual int numMatches() const;
  virtual int minMatches() const;
  virtual void computeTransformation(const vector<int>& idxs,vector<Matrix3d> *Fs) const;
  virtual double computeSquaredError(const Matrix3d& F,int matchIdx) const;
  virtual void refine(const vector<int>& inliers,Matrix3d *F) const;
  virtual bool isPermittedSelection(const vector<int>& idxs) const;
  virtual void computeFeaturesOrdering(vector<int>& order) const;

private:
  const int minMatches_;
  const vector<Vector2d>& keys1_;
  const vector<Vector2d>& keys2_;
  const CameraPair& pair_;
};

class Mediator7ptRANSAC : public MediatorRANSAC<Matrix3d>
{
public:
  // Mind the order of points. We estimate such F that pts2'*F*pts1 = 0.
  Mediator7ptRANSAC(const vector<Vector2d>& keys1,const vector<Vector2d>& keys2,
    const vector<IntPair>& matches);
  virtual int numMatches() const;
  virtual int minMatches() const;
  virtual void computeTransformation(const vector<int>& idxs,vector<Matrix3d> *Fs) const;
  virtual double computeSquaredError(const Matrix3d& F,int matchIdx) const;
  virtual void refine(const vector<int>& inliers,Matrix3d *F) const;
  virtual bool isPermittedSelection(const vector<int>& idxs) const;

private:
  const int minMatches_;
  const vector<Vector2d>& keys1_;
  const vector<Vector2d>& keys2_;
  const vector<IntPair>& matches_;
};

// Even though this mediator estimates E (essential matrix)
// it works with corresponding F fundamental matrices.
// That is due to computation of squared error.
// Therefore, you should transform the resulting F to E
// if that is what you want.
class Mediator5ptRANSAC : public MediatorRANSAC<Matrix3d>
{
public:
  // Mind the order of points. We estimate such E that (inv(K2)*pts2)'*E*(inv(K1)*pts1) = 0.
  Mediator5ptRANSAC(const Camera& cam1,const Camera& cam2,
    const vector<IntPair>& matches);
  virtual int numMatches() const;
  virtual int minMatches() const;
  virtual void computeTransformation(const vector<int>& idxs,vector<Matrix3d> *Fs) const;
  virtual double computeSquaredError(const Matrix3d& F,int matchIdx) const;
  virtual void refine(const vector<int>& inliers,Matrix3d *F) const;
  virtual bool isPermittedSelection(const vector<int>& idxs) const;

private:
  const int minMatches_;
  const Camera& cam1_;
  const Camera& cam2_;
  vector<Vector3d> pts1Norm_,pts2Norm_;
  const vector<IntPair>& matches_;
  Matrix3d invK1_,invK2_;
};

// First-order geometric error (Sampson distance) for
// pts2'*F*pts1 = 0. MIND THE ORDER of the input points.
double computeSampsonSquaredDistanceFundMat(const Vector2d& pt2,const Matrix3d& F,
  const Vector2d& pt1);

// Finds all DISTINCT real roots.
// Accepts coefficients ordered by descending powers, i.e.
// p(x) = 5x^3 - x has representation [5 0 -1 0]
void solveThirdOrderPoly(const Vector4d& coeffs, VectorXd *roots);
void solveSecondOrderPoly(const Vector3d& coeffs, VectorXd *roots);

} // namespace