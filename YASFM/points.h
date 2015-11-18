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

#include <list>
#include <string>
#include <unordered_map>
#include <unordered_set>
#include <vector>

#include "Eigen\Dense"

#include "defines.h"
#include "options.h"
#include "sfm_data.h"

using Eigen::Vector3d;
using Eigen::Vector4d;
using Eigen::Matrix3d;
using Eigen::Matrix4d;
using namespace yasfm;
using std::list;
using std::string;
using std::vector;
using std::unordered_map; 
using std::unordered_set;

namespace yasfm
{

// Chooses color of points based on features which are associated with 
// a given point, are in addedCams. Result has the same size as number 
// of points but masked ones will be omitted.
//YASFM_API void findPointColors(const vector<int>& points2tracks, const vector<bool>& pointsMask,
//  const vector<NViewMatch>& tracks,const ptr_vector<ICamera>& cams,
//  const unordered_set<int>& addedCams, vector<Vector3uc>& pointsColors);
//YASFM_API void findPointColors(const IDataset& dts, vector<Vector3uc>& pointsColors);

// A word of warning, if you have been matching features
// in both directions. E.g. from img0 to img1 and img1 to img0.
// Then you should have the matches consistent. Meaning that 
// if feature0 in img0 matches to feature1 in img1, then
// no other feature from img1 should match to feature0.
YASFM_API void twoViewMatchesToNViewMatches(const ptr_vector<Camera>& cams,
  const pair_umap<CameraPair>& pairs,
  vector<NViewMatch> *nViewMatches);

// Creates matches for one camera pair from n-view matches.
// nViewMatchesIdxs contains corresponding indices of n-view matches. 
YASFM_API void nViewMatchesToTwoViewMatches(const vector<NViewMatch>& nViewMatches,
  IntPair pair,vector<IntPair> *twoViewMatches,vector<int> *nViewMatchesIdxs);

// Trianguates all points given by two-view matches. Uses only projection matrices.
// nViewMatchIdxs are corresponding idxs to points->matchesToReconstruct.
// A point is reconstructed only if it is triangulated in front of both cameras.
YASFM_API void reconstructPoints(const IntPair& camsIdxs,const Camera& cam1,const Camera& cam2,
  const vector<int>& nViewMatchIdxs,Points *points);

// A point is reconstructed only if it is triangulated in front of all cameras.
YASFM_API void reconstructPoints(const ptr_vector<Camera>& cams, 
  const vector<SplitNViewMatch>& matchesToReconstruct,Points *points);

// Trianguates all points given by two-view matches. Uses only projection matrices.
YASFM_API void triangulate(const Matrix34d& P1,const Matrix34d& P2,const vector<Vector2d>& keys1,
  const vector<Vector2d>& keys2,const vector<IntPair>& matches,vector<Vector3d> *pts);

YASFM_API void triangulate(const Matrix34d& P1,const Matrix34d& P2,const Vector2d& key1,
  const Vector2d& key2,Vector3d *pt);

// Triangulate from n-views. Ps are any number of projection matrices.
// PsToUse and keys are the same size and correspond to how the point is
// seen by different cameras.
YASFM_API void triangulate(const vector<Matrix34d>& Ps,const vector<int>& PsToUse,
  const vector<Vector2d>& keys,Vector3d *pt);

// Check if point is in front of the camera. Does not assume any kind of
// normalization of P. Hence it has to compute determinant of M where P = [M|m].
// Use isInFrontNormalizedP if you know that a > 0, where P = aK[R|t].
YASFM_API bool isInFront(const Matrix34d& P,const Vector3d& point);
// Check if point is in front of the camera. Assumes that a > 0, where P = aK[R|t].
YASFM_API bool isInFrontNormalizedP(const Matrix34d& P,const Vector3d& point);

// Extracts n-view matches that are seen by at least minObservingCams cameras and
// are well conditioned (have maximum angle of all pairs of rays at least 
// rayAngleThresh). rayAngleThresh is in degrees.
YASFM_API void extractCandidateNewPoints(int minObservingCams,double rayAngleThresh,
  const uset<int>& reconstructedCams,const ptr_vector<Camera>& cams,
  vector<NViewMatch> *nViewMatches,vector<SplitNViewMatch> *extracted);

YASFM_API void findCamToSceneMatches(const uset<int>& camsToIgnore,int numCams,
  const Points& points,vector<vector<IntPair>> *pmatches);

// A match is well conditioned if the maximum angle of all pairs of rays is at 
// least rayAngleThresh. rayAngleThresh is in degrees.
YASFM_API bool isWellConditioned(double rayAngleThresh,const ptr_vector<Camera>& cams,
  const NViewMatch& pointViews);

// Normalize keys using calibration, apply inverse rotation and return
// angle between the resulting rays in radians.
YASFM_API double computeRayAngle(const Camera& cam1,int key1Idx,
  const Camera& cam2,int key2Idx);

// Remove points with the maximum angle of all pairs of rays smaller than
// rayAngleThresh. rayAngleThresh is in degrees. A ray is computed by subtracting
// a point and a camera center.
YASFM_API void removeIllConditionedPoints(double rayAngleThresh,
  const ptr_vector<Camera>& cams,Points *pts);

YASFM_API void removeHighReprojErrorPoints(double avgReprojErrThresh,const ptr_vector<Camera>& cams,
  Points *pts);

} // namespace yasfm

namespace
{

// Searches for connected component which does not have more
// than one feature in any image. Updates visitedFeats.
// Pushes back the track into tracks if a track is succesfully found.
void findNViewMatch(const vector<uset<int>>& matchedCams,
  const pair_umap<vector<int>>& matches,int startCamIdx,int startFeatIdx,
  vector<vector<bool>> *visitedFeats,vector<NViewMatch> *nViewMatches);

// matches here will represent mapping from feature1 to feature2.
// -1 means that a feature is not matched.
// The matches are meant to be symmetric to be easily searched through.
// matchedCams[i] is simply a set of cameras which are matched from/to i-th camera.
void convertMatchesToLocalRepresentation(const ptr_vector<Camera>& cams,
  const pair_umap<CameraPair>& pairs,vector<uset<int>> *matchedCams,
  pair_umap<vector<int>> *matches);

} // namespace