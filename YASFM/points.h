//----------------------------------------------------------------------------------------
/**
* \file       points.h
* \brief      Functions relevant to points and n-view matches.
*
*  Functions relevant to points and n-view matches
*
*/
//----------------------------------------------------------------------------------------

#pragma once

#include <vector>

#include "Eigen\Dense"

#include "defines.h"
#include "sfm_data.h"

using Eigen::Vector2d;
using Eigen::Vector3d;
using std::vector;
using namespace yasfm;

namespace yasfm
{

/// Find n-view matches from two-view matches.
/**
This tries to connect all the key matches into n-view matches. If a n-view match is
found to be inconsistent, it is discarded. A n-view match is inconsistent if it is
observed in any of the cameras more than once.

A word of warning if you have been matching features in both directions. 
E.g. from img0 to img1 and img1 to img0. Then you should have the matches consistent. 
Meaning that if feature0 in img0 matches to feature1 in img1, then no other feature 
from img1 should match to feature0.

\param[in] cams Cameras. Needed to know total number of keys in all cameras.
\param[in] pairs Two view matches.
\param[out] nViewMatches Found consistent n-view matches.
*/
YASFM_API void twoViewMatchesToNViewMatches(const ptr_vector<Camera>& cams,
  const pair_umap<CameraPair>& pairs,
  vector<NViewMatch> *nViewMatches);

/// Creates matches for one camera pair from n-view matches.
/**
\param[in] nViewMatches N-View matches.
\param[in] pair Camera pair indices.
\param[out] twoViewMatches Resulting two-view matches.
\param[out] nViewMatchesIdxs Corresponding indices of n-view matches
*/
YASFM_API void nViewMatchesToTwoViewMatches(const vector<NViewMatch>& nViewMatches,
  IntPair pair,vector<IntPair> *twoViewMatches,vector<int> *nViewMatchesIdxs);

/// Reconstruct points using two cameras.
/**
Trianguates all points given by two-view matches. A point is reconstructed only if 
it is triangulated in front of both cameras. Points colors are chosen from the first
key.

\param[in] camIdxs Camera pair indices.
\param[in] cam1 First camera.
\param[in] cam2 Second camera.
\param[in] nViewMatchIdxs Corresponding indices to points->matchesToReconstruct.
\params[out] points Points.
*/
YASFM_API void reconstructPoints(const IntPair& camsIdxs,const Camera& cam1,
  const Camera& cam2,const vector<int>& nViewMatchIdxs,Points *points);

/// Reconstruct points using any number of cameras.
/**
Trianguates all points given by n-view matches. A point is reconstructed only if
it is triangulated in front of all cameras. Points colors are chosen from one
observation.

\param[in] cams Cameras.
\param[in] matchesToReconstruct Points views.
\params[out] points Points.
*/
YASFM_API void reconstructPoints(const ptr_vector<Camera>& cams, 
  const vector<SplitNViewMatch>& matchesToReconstruct,Points *points);

/// Trianguates all points given by two-view matches.
/**
\param[in] P1 First projection matrix.
\param[in] P2 Second projection matrix.
\param[in] keys1 Keys of the first camera.
\param[in] keys2 Keys of the second camera.
\param[in] matches Matches.
\param[out] pts Points coordinates.
*/
YASFM_API void triangulate(const Matrix34d& P1,const Matrix34d& P2,
  const vector<Vector2d>& keys1,const vector<Vector2d>& keys2,
  const vector<IntPair>& matches,vector<Vector3d> *pts);

/// Trianguates one point (does numerical conditioning).
/**
\param[in] P1 First projection matrix.
\param[in] P2 Second projection matrix.
\param[in] key1 Key of the first camera.
\param[in] key2 Key of the second camera.
\param[out] pt Point coordinate.
*/
YASFM_API void triangulate(const Matrix34d& P1,const Matrix34d& P2,const Vector2d& key1,
  const Vector2d& key2,Vector3d *pt);

// Triangulate one point from n views using least squares.
/**
\param[in] Ps Projection matrices.
\param[in] PsToUse Which projection matrices to use for triangulation.
\param[in] keys Corresponding keys. The same size as PsToUse.
\param[out] pt Point.
*/
YASFM_API void triangulate(const vector<Matrix34d>& Ps,const vector<int>& PsToUse,
  const vector<Vector2d>& keys,Vector3d *pt);

/// Check if point is in front of a camera. 
/**
Does not assume any kind of normalization of P. Hence it has to compute 
determinant of M where P = [M|m]. Use isInFrontNormalizedP if you know that 
a > 0, where P = aK[R|t].

\param[in] P Projection matrix.
\param[in] point Point.
\return True if the point is in front of the camera.
*/
YASFM_API bool isInFront(const Matrix34d& P,const Vector3d& point);

/// Check if point is in front of a camera. 
/**
Assumes that a > 0, where P = aK[R|t].

\param[in] P Normalized projection matrix (see function description).
\param[in] point Point.
\return True if the point is in front of the camera.
*/
YASFM_API bool isInFrontNormalizedP(const Matrix34d& P,const Vector3d& point);

/// Extract candidate n-view matches for reconstruction.
/**
Extracts n-view matches that are seen by at least minObservingCams cameras and 
are well conditioned (have maximum angle of all pairs of rays at least rayAngleThresh). 

\param[in] minObservingCams Minimal number of reconstructed cameras that have to observe
a candidate point.
\param[in] rayAngleThresh Angle threshold in degrees.
\param[in] reconstructedCams Cameras that are reconstructed.
\param[in] cams All cameras.
\param[in,out] nViewMatches All n-view matches.
\param[out] extracted N-View matches that were removed from nViewMatches.
*/
YASFM_API void extractCandidateNewPoints(int minObservingCams,double rayAngleThresh,
  const uset<int>& reconstructedCams,const ptr_vector<Camera>& cams,
  vector<NViewMatch> *nViewMatches,vector<SplitNViewMatch> *extracted);

/// Find camera to scene (key to point) matches.
/**
\param[in] camsToIgnore Cameras to ignore.
\param[in] numCams Total number cameras.
\param[in] points Points.
\param[out] matches Cam to scene matches. matches[i] are matches for camera i.
*/
YASFM_API void findCamToSceneMatches(const uset<int>& camsToIgnore,int numCams,
  const Points& points,vector<vector<IntPair>> *matches);

/// Determine conditioning.
/**
A match is well conditioned if the maximum angle of all pairs of rays is at least 
rayAngleThresh. 
\param[in] rayAngleThresh Angle threshold in degrees.
\param[in] cams Cameras.
\param[in] pointViews N-View match (Point observations in images).
\return True if well conditioned.
*/
YASFM_API bool isWellConditioned(double rayAngleThresh,const ptr_vector<Camera>& cams,
  const NViewMatch& pointViews);

/// Compute angle between rays.
/**
Normalize keys using calibration, apply inverse rotation and compute angle between 
the resulting rays.

\param[in] cam1 First camera.
\param[in] key1Idx Index of the key in the first camera.
\param[in] cam2 Second camera.
\param[in] key2Idx Index of the key in the second camera.
\return Angle between rays in radians.
*/
YASFM_API double computeRayAngle(const Camera& cam1,int key1Idx,
  const Camera& cam2,int key2Idx);

/// Remove ill conditioned points.
/**
Remove points with the maximum angle of all pairs of rays smaller than rayAngleThresh. 
A ray is computed by subtracting a point and a camera center.

\param[in] rayAngleThresh Angle threshold in degrees.
\param[in] cams Cameras.
\param[in,out] pts Points.
*/
YASFM_API void removeIllConditionedPoints(double rayAngleThresh,
  const ptr_vector<Camera>& cams,Points *pts);

/// Remove points with high reprojection error.
/**
\param[in] avgReprojErrThresh Threshold for the average reprojection error.
\param[in] cams Cameras.
\param[in,out] pts Points.
*/
YASFM_API void removeHighReprojErrorPoints(double avgReprojErrThresh,
  const ptr_vector<Camera>& cams,Points *pts);

} // namespace yasfm

namespace
{

/** 
Searches for connected component which does not have more than one feature in 
any image. Updates visitedFeats. Pushes back the track into tracks if a track is 
succesfully found.
*/
void findNViewMatch(const vector<uset<int>>& matchedCams,
  const pair_umap<vector<int>>& matches,int startCamIdx,int startFeatIdx,
  vector<vector<bool>> *visitedFeats,vector<NViewMatch> *nViewMatches);

/**
matches here will represent mapping from feature1 to feature2. -1 means that 
a feature is not matched. The matches are meant to be symmetric to be easily 
searched through. matchedCams[i] is simply a set of cameras which are 
matched from/to i-th camera.
*/
void convertMatchesToLocalRepresentation(const ptr_vector<Camera>& cams,
  const pair_umap<CameraPair>& pairs,vector<uset<int>> *matchedCams,
  pair_umap<vector<int>> *matches);

} // namespace