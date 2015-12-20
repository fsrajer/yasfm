//----------------------------------------------------------------------------------------
/**
* \file       absolute_pose.h
* \brief      Absolute pose algorithms.
*
*  Absolute pose algorithms for estimating camera using camera to scene matches.
*
*/
//----------------------------------------------------------------------------------------

#pragma once

#include <vector>

#include "Eigen\Dense"

#include "defines.h"
#include "ransac.h"
#include "sfm_data.h"

using Eigen::Vector2d;
using Eigen::Vector3d;
using std::vector;
using namespace yasfm;

namespace yasfm
{

/// Find cameras with enough camera-to-scene matches.
/**
Find cam with maximum number of matches, say maxMatches.
Return all cameras having at least max(minMatchesThresh,(maxMatches*factor))

\param[in] minMatchesThresh Minimum matches threshold for a camera.
\param[in] factor See the function description.
\param[in] camToSceneMatches Camera-to-scene matches (a.k.a. keys-to-3Dpoints matches).
\param[out] wellMatchedCams Well matched cameras (see function description).
*/
YASFM_API void chooseWellMatchedCameras(int minMatchesThresh,double factor,
  const vector<vector<IntPair>>& camToSceneMatches,
  uset<int> *wellMatchedCams);

/// Compute average reprojection error over all the observations.
/**
\param[in] cams Cameras.
\param[in] points Points.
\return Average reprojection error.
*/
YASFM_API double computeAverageReprojectionError(const ptr_vector<Camera>& cams,
  const Points& points);

/// Find camera parameters by plugging 5,5pt solver into RANSAC.
/**
\param[in] opt RANSAC options.
\param[in] camToSceneMatches .first of the IntPair is key index and .second a point
index.
\param[in] points 3d points' coordinates.
\param[in,out] cam Camera to be estimated. Uses keys() as input and sets parameters
as output.
\param[out] inliers Inliers of the matches to the estimated parameters.
\return True if the camera was estimated succesfully.
*/
YASFM_API bool resectCamera5AndHalfPtRANSAC(const OptionsRANSAC& opt,
  const vector<IntPair>& camToSceneMatches,const vector<Vector3d>& points,
  Camera *cam,vector<int> *inliers = nullptr);

/// Find camera parameters by plugging 5,5pt solver into RANSAC.
/**
\param[in] opt RANSAC options.
\param[in] camToSceneMatches .first of the IntPair is key index and .second a point
index.
\param[in] keys Keys.
\param[in] points 3d points' coordinates.
\param[out] P The best found projection matrix.
\param[out] inliers Inliers of the matches to the estimated parameters.
\return Success. (False if the best hypothesis was not supported by enough inliers.)
*/
YASFM_API bool resectCamera5AndHalfPtRANSAC(const OptionsRANSAC& opt,
  const vector<IntPair>& camToSceneMatches,const vector<Vector2d>& keys,
  const vector<Vector3d>& points,
  Matrix34d *P,vector<int> *inliers = nullptr);

/// 5,5pt absolute pose minimal solver.
/**
6 points give 12 equations. P matrix has 11 degrees of freedom. This solver takes 
5 points for 10 equations and 6th point is used twice for the 11-th equation. 
This gives 2 solutions.

\param[in] keys Keys.
\param[in] points Points.
\param[in] camToSceneMatches .first of the IntPair is key index and .second a point
index.
\param[out] Ps Estimated projection matrices (there are 2 matrices).
*/
YASFM_API void resectCamera5AndHalfPt(const vector<Vector2d>& keys,
  const vector<Vector3d>& points,const vector<IntPair>& camToSceneMatches,
  vector<Matrix34d> *Ps);

/// Find camera parameters by plugging 6pt non-minimal solver into RANSAC.
/**
\param[in] opt RANSAC options.
\param[in] camToSceneMatches .first of the IntPair is key index and .second a point
index.
\param[in] points 3d points' coordinates.
\param[in,out] cam Camera to be estimated. Uses keys() as input and sets parameters
as output.
\param[out] inliers Inliers of the matches to the estimated parameters.
\return True if the camera was estimated succesfully.
*/
YASFM_API bool resectCamera6ptLSRANSAC(const OptionsRANSAC& opt,
  const vector<IntPair>& camToSceneMatches,const vector<Vector3d>& points,
  Camera *cam,vector<int> *inliers = nullptr);

/// Find camera parameters by plugging 6pt non-minimal solver into RANSAC.
/**
\param[in] opt RANSAC options.
\param[in] camToSceneMatches .first of the IntPair is key index and .second a point
index.
\param[in] keys Keys.
\param[in] points 3d points' coordinates.
\param[out] P The best found projection matrix.
\param[out] inliers Inliers of the matches to the estimated parameters.
\return Success. (False if the best hypothesis was not supported by enough inliers.)
*/
YASFM_API bool resectCamera6ptLSRANSAC(const OptionsRANSAC& opt,
  const vector<IntPair>& camToSceneMatches,const vector<Vector2d>& keys,
  const vector<Vector3d>& points,
  Matrix34d *P,vector<int> *inliers = nullptr);

/// Non-minimal solver for projection matrix.
/**
Assumes P(2,3) to be 1 and stacks two equations for every match onto each other.
Then uses SVD to find solution.

\param[in] keys Keys.
\param[in] points Points.
\param[in] camToSceneMatches .first of the IntPair is key index and .second a point
index.
\param[out] P Estimated projection matrix.
*/
YASFM_API void resectCameraLS(const vector<Vector2d>& keys,
  const vector<Vector3d>& points,const vector<IntPair>& camToSceneMatches,
  Matrix34d *P);

/// Implementation of common structures for resectioning mediators.
class MediatorResectioningRANSAC : public MediatorRANSAC<Matrix34d>
{
public:
  /// Constructor. 
  /** 
  \param[in] minMatches Minimum number of matches to estimate a transformation.
  \param[in] keys Keys.
  \param[in] points Points.
  \param[in] camToSceneMatches .first of the IntPair is key index and .second a point
  index.
  */
  MediatorResectioningRANSAC(int minMatches,const vector<Vector2d>& keys,
    const vector<Vector3d>& points,const vector<IntPair>& camToSceneMatches);

  /// \return Minimum number of matches to compute the transformation.
  virtual int minMatches() const;

  /// \return Total number of matches.
  virtual int numMatches() const;

  /// Compute squared error for one match.
  /**
  \param[in] P Projection matrix.
  \param[in] matchIdx Index of the match.
  \return Squared error.
  */
  virtual double computeSquaredError(const Matrix34d& P,int matchIdx) const;

  /// Refine the projection matrix using its inliers.
  /**
  \param[in] tolerance Tolerance for optimization termination (can be ignored if it
  is not neccessary).
  \param[in] inliers Inlier matches.
  \param[in,out] P The projection matrix to be refined on input and refined transformation
  on the output.
  */
  virtual void refine(double tolerance,const vector<int>& inliers,Matrix34d *P) const;

protected:
  const int minMatches_;           ///< Minimal number of matches.
  const vector<Vector2d>& keys_;   ///< Keys in the camera.
  const vector<Vector3d>& points_; ///< 3d points coordinates.
  const vector<IntPair>& camToSceneMatches_; ///< Keys to points matches.
};

/// Mediator for 5,5pt minimal solver.
class MediatorResectioning5AndHalfPtRANSAC : public MediatorResectioningRANSAC
{
public:
  /// Constructor. 
  /**
  \param[in] keys Keys.
  \param[in] points Points.
  \param[in] camToSceneMatches .first of the IntPair is key index and .second a point
  index.
  */
  MediatorResectioning5AndHalfPtRANSAC(const vector<Vector2d>& keys,
    const vector<Vector3d>& points,const vector<IntPair>& camToSceneMatches);

  /// Compute transformation from a minimal sample.
  /**
  \param[in] idxs Indices of matches from which to compute the transformation.
  \param[out] Ps Resulting projection matrices.
  */
  virtual void computeTransformation(const vector<int>& idxs,vector<Matrix34d> *Ps) const;
};

/// Mediator for 6pt non-minimal solver.
class MediatorResectioning6ptLSRANSAC : public MediatorResectioningRANSAC
{
public:
  /// Constructor. 
  /**
  \param[in] keys Keys.
  \param[in] points Points.
  \param[in] camToSceneMatches .first of the IntPair is key index and .second a point
  index.
  */
  MediatorResectioning6ptLSRANSAC(const vector<Vector2d>& keys,
    const vector<Vector3d>& points,const vector<IntPair>& camToSceneMatches);

  /// Compute transformation from a minimal sample.
  /**
  \param[in] idxs Indices of matches from which to compute the transformation.
  \param[out] Ps Resulting projection matrices.
  */
  virtual void computeTransformation(const vector<int>& idxs,vector<Matrix34d> *Ps) const;
};

} // namespace yasfm
