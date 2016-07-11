//----------------------------------------------------------------------------------------
/**
* \file       relative_pose.h
* \brief      Functions relevant to relative pose of cameras.
*
*  Functions relevant to relative pose of cameras.
*
*/
//----------------------------------------------------------------------------------------

#pragma once

#include <vector>
#include <memory>

#include "Eigen\Dense"

#include "defines.h"
#include "ransac.h"
#include "sfm_data.h"
#include "options_types.h"

using Eigen::ArrayXXd;
using Eigen::ArrayXXi;
using Eigen::Matrix3d;
using Eigen::Vector2d;
using Eigen::Vector3d;
using Eigen::Vector4d;
using Eigen::VectorXd;
using std::vector;
using std::make_unique;
using namespace yasfm;

////////////////////////////////////////////////////
///////////////   Declarations   ///////////////////
////////////////////////////////////////////////////

namespace yasfm
{

YASFM_API double computeHomologyDataFitScore(const Camera& cam1,
  const Camera& cam2,const vector<IntPair>& matches1,
  const vector<IntPair>& matches2,
  const OptionsRANSAC& ransacOpt,Matrix3d *H1,Matrix3d *H2);

YASFM_API double optimizeLine(const Matrix3d& H,const Vector3d& randVec,Vector3d *line);

/// Callback function for EG verification progress notifying
/**
\param[in] void pointer to the callee object.
\param[out] indices of the cameras.
\param[out] number of original matches.
\param[out] number of inliers after verification.
\param[out] progress indicator 0-1.

*/
typedef void(*GeomVerifyCallbackFunctionPtr)(void *, IntPair imgPair, int nMatches, int nInliers, double progress);

/// Callback function for Homography verification progress notifying
/**
\param[in] void pointer to the callee object
\param[out] progress indicator 0-1
*/
typedef void(*HomographyInliersCallbackFunctionPtr)(void *, double progress);

/// Convert F to a representation useful for non-linear optimization.
/**
See decompose function description.

\param[in] H Homography 3x3.
\param[in] v Vector 3.
\param[out] F Fundamental matrix.
*/
template<typename T>
void composeF(const T* const H,const T* const v,Matrix<T,3,3> *F);

/// Convert F from the representation for non-linear optimization.
/**
F = H*[v]x, where [v]x is a cross-product matrix and H is a homography.

\param[in] F Fundamental matrix.
\param[out] H Homography.
\param[out] v Vector.
*/
YASFM_API void decomposeF(const Matrix3d& F,Matrix3d *H,Vector3d *v);

/// Choose good camera pair for initialization (with the most matches).
/**
\param[in] minMatches Minimum matches needed for a camera pair to be enough.
\param[in] nCams Number of cameras.
\param[in] camsToIgnore Which cameras should not be considered.
\param[in] nViewMatches N-View matches.
\return Good initial camera pair. Returns (-1,-1) if there is no such pair.
*/
YASFM_API IntPair chooseInitialCameraPair(int minMatches,int nCams,
  const uset<int>& camsToIgnore,const vector<NViewMatch>& nViewMatches);

/// Choose good camera pair for initialization.
/**
Finds the camera pair with the most matches. First tries pairs where both cameras
are calibrated and if unsuccessful then other pairs are tried.

\param[in] minMatches Minimum matches needed for a camera pair to be enough.
\param[in] isCalibrated Which cameras are calibrated.
\param[in] camsToIgnore Which cameras should not be considered.
\param[in] nViewMatches N-View matches.
\return Good initial camera pair. Returns (-1,-1) if there is no such pair.
*/
YASFM_API IntPair chooseInitialCameraPair(int minMatches,
  const vector<bool>& isCalibrated,const uset<int>& camsToIgnore,
  const vector<NViewMatch>& nViewMatches);

/// Choose good camera pair for initialization.
/**
Finds a good initial camera pair. First tries pairs where both cameras
are calibrated and if unsuccessful then other pairs are tried. A pair is good 
if it has at least minScore and the most matches. If no pair has minScore then
a pair with maximum score is searched while having at least minMatches.

\param[in] minMatches Minimum matches needed for a camera pair to be enough.
\param[in] minScore Minimum score.
\param[in] isCalibrated Which cameras are calibrated.
\param[in] camsToIgnore Which cameras should not be considered.
\param[in] nViewMatches N-View matches.
\param[in] scores Camera pair scores. The bigger the better pair.
\return Good initial camera pair. Returns (-1,-1) if there is no such pair.
*/
YASFM_API IntPair chooseInitialCameraPair(int minMatches,double minScore,
  const vector<bool>& isCalibrated,const uset<int>& camsToIgnore,
  const vector<NViewMatch>& nViewMatches,const ArrayXXd& scores);

/// Choose good camera pair for initialization.
/**
Finds a good initial camera pair. First tries pairs where both cameras
are calibrated and if unsuccessful then other pairs are tried. A pair is good
if it has at least minScore and the most matches. If no pair has minScore then
a pair with maximum score is searched while having at least minMatches.

\param[in] minMatches Minimum matches needed for a camera pair to be enough.
\param[in] minScore Minimum score.
\param[in] isCalibrated Which cameras are calibrated.
\param[in] camsToIgnore Which cameras should not be considered.
\param[in] numMatches Numbers of matches for every camera pair.
\param[in] scores Camera pair scores. The bigger the better pair.
\return Good initial camera pair. Returns (-1,-1) if there is no such pair.
*/
YASFM_API IntPair chooseInitialCameraPair(int minMatches,double minScore,
  const vector<bool>& isCalibrated,const uset<int>& camsToIgnore,
  const ArrayXXi& numMatches,const ArrayXXd& scores);

/// Choose good camera pair for initialization.
/**
Finds a good initial camera pair. A pair is good if it has at least minScore 
and the most matches. If no pair has minScore then a pair with maximum score 
is searched while having at least minMatches.

\param[in] minMatches Minimum matches needed for a camera pair to be enough.
\param[in] minScore Minimum score.
\param[in] camsToUse Which cameras should be considered.
\param[in] numMatches Numbers of matches for every camera pair.
\param[in] scores Camera pair scores. The bigger the better pair.
\return Good initial camera pair. Returns (-1,-1) if there is no such pair.
*/
YASFM_API IntPair chooseInitialCameraPair(int minMatches,double minScore,
  const uset<int>& camsToUse,const ArrayXXi& numMatches,const ArrayXXd& scores);

/// Initialize reconstruction using uncalibrated camera pair.
/**
1) Initialize the initial pair cameras based on their relative position estimated 
   from common n-view matches using 7-pt algorithm. (Estimate F and decompose into
   Ps)
2) Triangulate the common n-view matches.
3) Remove high reprojection points.

\param[in] solverOpt Options for fundamental matrix estimation.
\param[in] pointsReprojErrorThresh Threshold for reprojection error of points.
\param[in] initPair Indices of the intial camera pair.
\param[in,out] data Data storage.
*/
YASFM_API void initReconstructionFromCamPair(const OptionsRANSAC& solverOpt,
  double pointsReprojErrorThresh,const IntPair& initPair,Dataset *data);

/// Initialize reconstruction using calibrated camera pair.
/**
1) Initialize the initial pair cameras based on their relative position estimated
   from common n-view matches using 5-pt algorithm. (Estimate E and decompose into
   Rs and ts)
2) Triangulate the common n-view matches.
3) Remove high reprojection points.

\param[in] solverOpt Options for essential matrix estimation.
\param[in] pointsReprojErrorThresh Threshold for reprojection error of points.
\param[in] initPair Indices of the intial camera pair.
\param[in,out] data Data storage.
*/
YASFM_API void initReconstructionFromCalibratedCamPair(const OptionsRANSAC& solverOpt,
  double pointsReprojErrorThresh,const IntPair& initPair,Dataset *data);

/// Decompose fundamental matrix into projection matrices.
/**
Initializes projection matrices of a pair of cameras from their fundamental matrix. 
The first camera P1 is assumed to be:
P1 = [1 0 0 0]
     [0 1 0 0]
     [0 0 1 0]
and P2 = [[e]_x*F | e] where e is the epipole (e*F = 0)

\param[in] F Fundamental matrix.
\param[out] P2 Projection matrix of the second camera. (Don't forget to set P1 as 
described.)
*/
YASFM_API void F2Ps(const Matrix3d& F,Matrix34d *P2);

/// Decompose fundamental matrix into essential matrix using known calibration matrix.
/**
Creates from equation E = K2^T * F * K1 and by forcing
both singular values of E to be equal (E(1,1) <- E(0,0)).
Note that this makes the original F inconsistent with E.

\param[in] F Fundamental matrix.
\param[in] K1 Calibration matrix of the first camera.
\param[in] K2 Calibration matrix of the second camera.
\param[out] E Essential matrix.
*/
YASFM_API void FK2E(const Matrix3d& F, const Matrix3d& K1, const Matrix3d& K2, 
  Matrix3d *E);

/// Decompose essential matrix into pose using known calibration and feature matches.
/**
Decomposes E. First camera is assumed to have R identity and C origin.
Second camera's R and C is returned.

\param[in] E Essential matrix.
\param[in] K1 Calibration matrix of the first camera.
\param[in] K2 Calibration matrix of the second camera.
\param[in] matches Matches of keys. .first is for the keys1 and .second for the keys2.
\param[in] keys1 Keys in the first camera.
\param[in] keys2 Keys in the second camera.
\param[out] R Rotation matrix of the second camera. (Don't forget to set the first 
camera.)
\param[out] C Camera center of the second camera. (Don't forget to set the first 
camera.)
*/
YASFM_API void E2RC(const Matrix3d& E,const Matrix3d& K1,const Matrix3d& K2,
  const vector<IntPair>& matches,const vector<Vector2d>& keys1,
  const vector<Vector2d>& keys2,Matrix3d *R,Vector3d *C);

/// Verify matches geometrically using epipolar geometry.
/**
For every camera pair, estimates fundamental matrix using PROSAC (that is why we
need dists). Then, only matches which are inliers to the fundamental matrix are
kept (as well as corresponding dists). The empty pairs get removed.

\param[in] solverOpt Options for estimating transformations.
\param[in] verbose Should this print status?
\param[in] useCalibratedEG Uses 5pt instead of 7pt. All cameras have to be calibrated.
\param[in] cams Cameras needed for the keys.
\param[in,out] pairs Camera pairs which will get verified.
\param[in] callback function for progress notification, called after each verified pair.
\param[in] pointer to an object whose callback function should be called.
*/
YASFM_API void verifyMatchesEpipolar(const OptionsRANSAC& solverOpt,
  bool verbose,bool useCalibratedEG,const ptr_vector<Camera>& cams,
  pair_umap<CameraPair> *pairs, 
  GeomVerifyCallbackFunctionPtr callbackFunction = NULL, void * callbackObjectPtr = NULL);

/// Sets verbosity to true and calls overloaded function.
/**
\param[in] solverOpt Options for estimating transformations.
\param[in] useCalibratedEG Uses 5pt instead of 7pt. All cameras have to be calibrated.
\param[in] cams Cameras needed for the keys.
\param[in,out] pairs Camera pairs which will get verified.
\param[in] callback function for progress notification, called after each verified pair.
\param[in] pointer to an object whose callback function should be called.
*/
YASFM_API void verifyMatchesEpipolar(const OptionsRANSAC& solverOpt,
  bool useCalibratedEG,const ptr_vector<Camera>& cams,
  pair_umap<CameraPair> *pairs, 
  GeomVerifyCallbackFunctionPtr callbackFunction = NULL, void * callbackObjectPtr = NULL);

/// Estimate fundamental matrix using PROSAC.
/**
Robust estimator, which finds such a fundamental matrix that 
pts2'*F*pts1 = 0 using 7 point algorithm as a minimal solver. 
MIND THE ORDER of the input points.

\param[in] opt Options.
\param[in] keys1 Keys in the first camera (see function description).
\param[in] keys2 Keys in the second camera (see function description).
\param[in] pair Camera pair with matches and distances.
\param[out] F Fundamental matrix.
\param[out] inliers Inliers to the best fundamental matrix.
\return False if the estimated hypothesis was not supported by enough inliers.
*/
YASFM_API bool estimateRelativePose7ptPROSAC(const OptionsRANSAC& opt,
  const vector<Vector2d>& keys1,const vector<Vector2d>& keys2,const CameraPair& pair,
  Matrix3d *F,vector<int> *inliers = nullptr);

/// Estimate fundamental matrix using RANSAC.
/**
Robust estimator, which finds such a fundamental matrix that
pts2'*F*pts1 = 0 using 7 point algorithm as a minimal solver.
MIND THE ORDER of the input points.

\param[in] opt Options.
\param[in] keys1 Keys in the first camera (see function description).
\param[in] keys2 Keys in the second camera (see function description).
\param[in] matches Keys matches.
\param[out] F Fundamental matrix.
\param[out] inliers Inliers to the best fundamental matrix.
\return False if the estimated hypothesis was not supported by enough inliers.
*/
YASFM_API bool estimateRelativePose7ptRANSAC(const OptionsRANSAC& opt,
  const vector<Vector2d>& keys1,const vector<Vector2d>& keys2,
  const vector<IntPair>& matches,Matrix3d *F,vector<int> *inliers = nullptr);

/// Estimate fundamental matrix (minimal solver).
/**
Minimal solver, which finds such a fundamental matrix that 
pts2'*F*pts1 = 0 using 7 point algorithm. 
MIND THE ORDER of the input points. 
There are 1 to 3 solutions which are all possible and need to be verified by 
computing support.

\param[in] keys1 Keys in the first camera (see function description).
\param[in] keys2 Keys in the second camera (see function description).
\param[in] matches Keys matches (of size 7).
\param[out] Fs Fundamental matrices.
*/
YASFM_API void estimateRelativePose7pt(const vector<Vector2d>& keys1, 
  const vector<Vector2d>& keys2,const vector<IntPair>& matches,vector<Matrix3d> *Fs);

/// Estimate essential matrix using RANSAC.
/**
Robust estimator, which finds such an essential matrix that 
(inv(K)*pts2)'*E*(inv(K2)*pts1) = 0 using 5 point algorithm.
MIND THE ORDER of the input points.

\param[in] opt Options.
\param[in] cam1 First camera with set keys and calibration.
\param[in] cam2 Second camera with set keys and calibration.
\param[in] matches Keys matches.
\param[out] E Essential matrix.
\param[out] inliers Inliers to the best matrix.
\return False if the estimated hypothesis was not supported by enough inliers.
*/
YASFM_API bool estimateRelativePose5ptRANSAC(const OptionsRANSAC& opt,
  const Camera& cam1,const Camera& cam2,const vector<IntPair>& matches,Matrix3d *E,
  vector<int> *inliers = nullptr);

/// Estimate essential matrix using PROSAC.
/**
Robust estimator, which finds such an essential matrix that
(inv(K1)*pts2)'*E*(inv(K2)*pts1) = 0 using 5 point algorithm.
MIND THE ORDER of the input points.

\param[in] opt Options.
\param[in] cam1 First camera with set keys and calibration.
\param[in] cam2 Second camera with set keys and calibration.
\param[in] matches Keys matches.
\param[out] E Essential matrix.
\param[out] inliers Inliers to the best matrix.
\return False if the estimated hypothesis was not supported by enough inliers.
*/
YASFM_API bool estimateRelativePose5ptPROSAC(const OptionsRANSAC& opt,
  const Camera& cam1,const Camera& cam2,const CameraPair& pair,Matrix3d *E,
  vector<int> *inliers = nullptr);

/// Estimate essential matrix (minimal solver).
/**
Minimal solver, which finds such an essential matrix that
(inv(K)*pts2)'*E*(inv(K2)*pts1) = 0 using 5 point algorithm.
MIND THE ORDER of the input points.

\param[in] pts1Norm Normalized points in the first camera (see function description).
\param[in] pts2Norm Normalized points in the second camera (see function description).
\param[in] matches Points matches (of size 5).
\param[out] Es Essential matrices.
*/
YASFM_API void estimateRelativePose5pt(const vector<Vector3d>& pts1Norm,
  const vector<Vector3d>& pts2Norm,const vector<IntPair>& matches,
  vector<Matrix3d> *Es);

/// Compute fundamental matrix from an all-inlier sample.
/**
Does least squares, closest rank 2 matrix and non-linear refinement.
Finds a solution for pts2'*F*pts1 = 0.
MIND THE ORDER of the input points.

\param[in] pts1 Points 1 (see function description).
\param[in] pts2 Points 2 (see function description).
\param[in] matches Points matches.
\param[in] matchesToUse Matches which should be used.
\param[in] tolerance Tolerance for refine optimization termination.
\param[out] F Fundamental matrix.
*/
YASFM_API void estimateFundamentalMatrix(const vector<Vector2d>& pts1,
  const vector<Vector2d>& pts2,const vector<IntPair>& matches,
  const vector<int>& matchesToUse,double tolerance,int maxOptIters,Matrix3d *F);

/// Refines fundamental matrix using levenberg-marquardt.
/**
Finds a solution for pts2'*F*pts1 = 0.
MIND THE ORDER of the input points.

\param[in] pts1 Points 1 (see function description).
\param[in] pts2 Points 2 (see function description).
\param[in] matches Points matches.
\param[in] matchesToUse Matches which should be used.
\param[in,out] F Fundamental matrix.
*/
YASFM_API void refineFundamentalMatrixNonLinear(const vector<Vector2d>& pts1,
  const vector<Vector2d>& pts2,const vector<IntPair>& matches,
  const vector<int>& matchesToUse,Matrix3d *F);

/// Refines fundamental matrix using levenberg-marquardt.
/**
Finds a solution for pts2'*F*pts1 = 0.
MIND THE ORDER of the input points.

\param[in] pts1 Points 1 (see function description).
\param[in] pts2 Points 2 (see function description).
\param[in] matches Points matches.
\param[in] matchesToUse Matches which should be used.
\param[in] tolerance Tolerance for optimization termination (try e.g. 1e-12).
\param[in] maxIters Maximum number of refine iterations
\param[in,out] F Fundamental matrix.
*/
YASFM_API void refineFundamentalMatrixNonLinear(const vector<Vector2d>& pts1,
  const vector<Vector2d>& pts2,const vector<IntPair>& matches,
  const vector<int>& matchesToUse,double tolerance,int maxIters,Matrix3d *F);

/// Find fundamental matrix inliers.
/**
For pts2'*F*pts1 = 0.
MIND THE ORDER of the input points.

\param[in] pts1 Points 1 (see function description).
\param[in] pts2 Points 2 (see function description).
\param[in] matches Points matches.
\param[in] F Fundamental matrix.
\param[out] inliers Inliers.
\return Number of inliers.
*/
YASFM_API int findFundamentalMatrixInliers(double thresh,const vector<Vector2d>& pts1,
  const vector<Vector2d>& pts2,const vector<IntPair>& matches,const Matrix3d& F,
  vector<int> *inliers);

/// Compute essential matrix from an all-inlier sample.
/**
Does least squares, closest essential matrix using svd (and non-linear refinement - not yet).
Finds a solution for (inv(K2)*pts2)'*E*(inv(K1)*pts1) = 0.
MIND THE ORDER of the input points.

\param[in] pts1 Points 1 (see function description).
\param[in] pts2 Points 2 (see function description).
\param[in] invK1 Inverse calibration matrix corresponding to pts1.
\param[in] invK2 Inverse calibration matrix corresponding to pts2.
\param[in] matches Points matches.
\param[in] matchesToUse Matches which should be used.
\param[in] tolerance Tolerance for refine optimization termination.
\param[out] E Essential matrix.
*/
YASFM_API void estimateEssentialMatrix(const vector<Vector2d>& pts1,
  const vector<Vector2d>& pts2,const Matrix3d& invK1,const Matrix3d& invK2,
  const vector<IntPair>& matches,const vector<int>& matchesToUse,
  double tolerance,Matrix3d *E);

/// Refines essential matrix using levenberg-marquardt.
/**
Finds a solution for (inv(K2)*pts2)'*E*(inv(K1)*pts1) = 0.
MIND THE ORDER of the input points.

\param[in] pts1 Points 1 (see function description).
\param[in] pts2 Points 2 (see function description).
\param[in] invK1 Inverse calibration matrix corresponding to pts1.
\param[in] invK2 Inverse calibration matrix corresponding to pts2.
\param[in] matches Points matches.
\param[in] matchesToUse Matches which should be used.
\param[in] tolerance Tolerance for optimization termination (try e.g. 1e-12).
\param[in,out] E Essential matrix.
*/
YASFM_API void refineEssentialMatrixNonLinear(const vector<Vector2d>& pts1,
  const vector<Vector2d>& pts2,const Matrix3d& invK1,const Matrix3d& invK2,
  const vector<IntPair>& matches,const vector<int>& matchesToUse,
  double tolerance,Matrix3d *E);

/// Compute proportion of the best homography inliers in the matches for all pairs.
/**
Estimate homography for every camera pair and save the proportion numInliers/numPoints 
into proportion. By convention unmatched camera pairs have the proportion 1.

\param[in] opt Options for estimating transformations.
\param[in] cams Cameras needed for the keys.
\param[in] pairs Camera pairs for which the homography should be computed.
\param[out] proportion Homography proportion. See function description.
*/
YASFM_API void computeHomographyInliersProportion(const OptionsRANSAC& opt,
  const ptr_vector<Camera>& cams,const pair_umap<CameraPair>& pairs,
  ArrayXXd *proportion, HomographyInliersCallbackFunctionPtr callbackFunction = NULL, void * callbackObjectPtr = NULL);

/// Estimate homography using PROSAC.
/**
Computes homography H from four matches using svd decomposition.
H*pt1 = lambda*pt2.

\param[in] opt Options.
\param[in] pts1 Points 1 (see function description).
\param[in] pts2 Points 2 (see function description).
\param[in] pair Camera pair.
\param[out] H Homography matrix.
\param[out] inliers Inliers to the best matrix.
\return False if the estimated hypothesis was not supported by enough inliers.
*/
YASFM_API bool estimateHomographyPROSAC(const OptionsRANSAC& opt,
  const vector<Vector2d>& pts1,const vector<Vector2d>& pts2,const CameraPair& pair,
  Matrix3d *H,vector<int> *inliers = nullptr);

/// Estimate homography using RANSAC.
/**
Computes homography H from four matches using svd decomposition.
H*pt1 = lambda*pt2.

\param[in] opt Options.
\param[in] pts1 Points 1 (see function description).
\param[in] pts2 Points 2 (see function description).
\param[in] matches Points matches.
\param[out] H Homography matrix.
\param[out] inliers Inliers to the best matrix.
\return False if the estimated hypothesis was not supported by enough inliers.
*/
YASFM_API bool estimateHomographyRANSAC(const OptionsRANSAC& opt,
  const vector<Vector2d>& pts1,const vector<Vector2d>& pts2,
  const vector<IntPair>& matches,Matrix3d *H,vector<int> *inliers = nullptr);

/// Options for geometric verificatio
/**
Fields:
/// Threshold defining inliers for estimated similarity.
double similarityThresh;

/// Threshold defining inliers for estimated affinity.
double affinityThresh;

/// Threshold defining inliers for estimated homography.
double homographyThresh;

/// Minimum number of inliers to one transformation.
int minInliersPerTransform;

/// Maximum number of transformations that should be estimated.
int maxTransforms;

/// Number of refine iterations that should be done to a transformation.
/// 1st iteration is estimated using key coordinates, scale and orientation.
/// 2-4th iterations are affinities
/// 5+th iterations are homographies
int nRefineIterations;

/// Minimum number of inliers to continue in further refining.
int minInliersToRefine;

/// Value in [0,1]. If there is this fraction of inliers found the
/// refine phase is terminated.
double stopInlierFraction;
**/
class OptionsGeometricVerification : public OptionsWrapper
{
public:
  YASFM_API OptionsGeometricVerification()
  {
    opt.emplace("similarityThresh",make_unique<OptTypeWithVal<double>>(20));
    opt.emplace("affinityThresh",make_unique<OptTypeWithVal<double>>(10));
    opt.emplace("homographyThresh",make_unique<OptTypeWithVal<double>>(5));
    opt.emplace("maxHs",make_unique<OptTypeWithVal<int>>(7));
    opt.emplace("minInliersPerH",make_unique<OptTypeWithVal<int>>(10));
    opt.emplace("nRefineIterations",make_unique<OptTypeWithVal<int>>(8));
    opt.emplace("minInliersToRefine",make_unique<OptTypeWithVal<int>>(4));

    opt.emplace("nOptIterations",make_unique<OptTypeWithVal<int>>(10));
    opt.emplace("refineTolerance",make_unique<OptTypeWithVal<double>>(1e-12));
    opt.emplace("fundMatThresh",make_unique<OptTypeWithVal<double>>(3.));

    opt.emplace("mergeThresh",make_unique<OptTypeWithVal<double>>(5.5));
  }
};

/// Verify matches geometrically.
/**
Verifies every camera pair. Firstly estimates a transformation and saves 
inliers, then estimates another transformation and so on until there is
enough matches. A transformation can be similarity, affinity or
homography.
Then, as output, only matches which are inliers are kept. Empty pairs get removed.

Inspired by:
https://github.com/vedaldi/practical-object-instance-recognition/blob/master/geometricVerification.m

\param[in] opt Options for estimating transformations.
\param[in] verbose Should this print status?
\param[in] cams Cameras.
\param[in,out] pairs Camera pairs which will get verified.
*/
YASFM_API void verifyMatchesGeometrically(const OptionsGeometricVerification& opt,
  bool verbose,const ptr_vector<Camera>& cams,pair_umap<CameraPair> *pairs);

/// Sets verbosity to true and calls overloaded function.
/**
\param[in] opt Options for estimating transformations.
\param[in] cams Cameras.
\param[in,out] pairs Camera pairs which will get verified.
*/
YASFM_API void verifyMatchesGeometrically(const OptionsGeometricVerification& opt,
  const ptr_vector<Camera>& cams,pair_umap<CameraPair> *pairs);

/// Verify matches geometrically.
/**
Detecting multiple motions

Homography estimation inspired by:
https://github.com/vedaldi/practical-object-instance-recognition/blob/master/geometricVerification.m

\param[in] opt Options for estimating transformations.
\param[in] cam1 First camera.
\param[in] cam2 Second camera.
\param[in] pair Used for matches and dists.
\param[out] inliers Indices of geometrically verified matches in the order of 
transformations from the most supported one.
\param[out] groups Detected match groups.
\return Number of inliers.
*/
YASFM_API int verifyMatchesGeometrically(const OptionsGeometricVerification& opt,
  const Camera& cam1,const Camera& cam2,const CameraPair& pair,
  vector<int> *inliers,vector<MatchGroup> *groups);

/// Greedy detection of multiple fundamental matrices with known homographies.
/**
\param[in] opt Options for estimating transformations.
\param[in] keys1 First camera.
\param[in] keys2 Second camera.
\param[in] pair Matches.
\param[in] groupsH Inliers to individual homographies.
\param[out] groupsF Inliers to individual Fs.
\param[out] Fs Fundamental matrices.
*/
YASFM_API void estimateFundamentalMatrices(const OptionsGeometricVerification& opt,
  const vector<Vector2d>& keys1,const vector<Vector2d>& keys2,const CameraPair& pair,
  const vector<vector<int>>& groupsH,vector<vector<int>> *groupsEG,vector<Matrix3d> *Fs);

YASFM_API double computePairwiseEigScore(const Matrix3d& H1,const Matrix3d& H2);

YASFM_API double computePairwiseEGScore(const OptionsGeometricVerification& opt,
  const vector<Vector2d>& pts1,const vector<Vector2d>& pts2,
  const vector<IntPair>& matches,const vector<int>& group1,
  const vector<int>& group2);

/// Estimate fundamental matrix using known homographies in the scene to avoid 
/// degeneracies.
YASFM_API bool estimateRelativePose7ptKnownHsLOPROSAC(const OptionsRANSAC& opt,
  const vector<Vector2d>& keys1,const vector<Vector2d>& keys2,
  const CameraPair& pair,int nGroups,const vector<int>& groupId,
  Matrix3d *F,vector<int> *inliers);

/// Refine fundamental matrix using known homographies.
YASFM_API void refineFKnownHs(const OptionsGeometricVerification& opt,
  const vector<Vector2d>& keys1,const vector<Vector2d>& keys2,const CameraPair& pair,
  vector<vector<int>> *groupsH,Matrix3d *F,vector<int> *inliersF);

/// Greedy detection of multiple homographies.
/**
\param[in] opt Options for estimating transformations.
\param[in] cam1 First camera.
\param[in] cam2 Second camera.
\param[in] matches Matches.
\param[out] groups Inliers to individual homographies.
\param[out] Hs Homographies.
*/
YASFM_API void growHomographies(const OptionsGeometricVerification& opt,
  const Camera& cam1,const Camera& cam2,const CameraPair& camPair,
  vector<vector<int>> *groups,vector<Matrix3d> *Hs);

/// Compute similarity transform given one feature match.
/**
Computes similarity S such that:
S*pt1 = lambda*pt2.

\param[in] coord1 First feature coordinates.
\param[in] scale1 First feature scale.
\param[in] orientation1 First feature orientation in radians.
\param[in] coord2 Second feature coordinates.
\param[in] scale2 Second feature scale.
\param[in] orientation2 Second feature orientation in radians.
\param[out] S Similarity transform.
*/
YASFM_API void computeSimilarityFromMatch(const Vector2d& coord1,double scale1,
  double orientation1,const Vector2d& coord2,double scale2,double orientation2,
  Matrix3d *S);

/// Compute affine transform given a set of matches.
/**
Computes affinity A using least squares such that:
A*pt1 = lambda*pt2.

\param[in] keys1 First keys.
\param[in] keys2 Second keys.
\param[in] matches Keys matches.
\param[in] matchesToUse Matches which should be used.
\param[out] A Affine transform.
*/
YASFM_API void estimateAffinity(const vector<Vector2d>& keys1,
  const vector<Vector2d>& keys2,const vector<IntPair>& matches,
  const vector<int>& matchesToUse,Matrix3d *A);

/// Compute homography.
/**
Computes homography H from matches using svd decomposition.
H*pt1 = lambda*pt2.

\param[in] pts1 Points 1 (see function description).
\param[in] pts2 Points 2 (see function description).
\param[in] matches Points matches.
\param[in] matchesToUse Matches which should be used.
\param[out] H Homography matrix.
*/
YASFM_API void estimateHomography(const vector<Vector2d>& pts1,
  const vector<Vector2d>& pts2,const vector<IntPair>& matches,
  const vector<int>& matchesToUse,Matrix3d *H);

/// Find homography inliers.
/**
H*pt1 = lambda*pt2.

\param[in] pts1 Points 1 (see function description).
\param[in] pts2 Points 2 (see function description).
\param[in] matches Points matches.
\param[in] H Homography matrix.
\param[out] inliers Inliers.
\return Number of inliers.
*/
YASFM_API int findHomographyInliers(double thresh,const vector<Vector2d>& pts1,
  const vector<Vector2d>& pts2,const vector<IntPair>& matches,const Matrix3d& H,
  vector<int> *inliers);

/// Implementation of mediator for 7pt solver.
class Mediator7ptRANSAC : public MediatorRANSAC<Matrix3d>
{
public:
  /// Constructor. Mind the order of points. We estimate such F that pts2'*F*pts1 = 0.
  /**
  \param[in] keys1 Keys in the first camera.
  \param[in] keys2 Keys in the second camera.
  \param[in] matches Keys matches.
  */
  Mediator7ptRANSAC(const vector<Vector2d>& keys1,const vector<Vector2d>& keys2,
    const vector<IntPair>& matches);

  /// \return Total number of matches.
  virtual int numMatches() const;

  /// \return Minimum number of matches to compute the transformation.
  virtual int minMatches() const;

  /// Compute transformation from a minimal sample.
  /**
  \param[in] idxs Indices of matches from which to compute the transformation.
  \param[out] Fs Resulting fundamental matrices.
  */
  virtual void computeTransformation(const vector<int>& idxs,vector<Matrix3d> *Fs) const;

  /// Compute symmetric squared distance.
  /**
  \param[in] F Fundamental matrix.
  \param[in] matchIdx Index of the match.
  \return Squared error.
  */
  virtual double computeSquaredError(const Matrix3d& F,int matchIdx) const;

  /// Refine a transformation using its inliers.
  /**
  \param[in] tolerance Tolerance for optimization termination.
  \param[in] inliers Inlier matches.
  \param[in,out] F The transformation to be refined on input and refined transformation
  on the output.
  */
  virtual void refine(double tolerance,const vector<int>& inliers,Matrix3d *F) const;

private:
  const int minMatches_;
  const vector<Vector2d>& keys1_;
  const vector<Vector2d>& keys2_;
  const vector<IntPair>& matches_;
};

/// Implementation of mediator for 5pt solver.
/**
Even though this mediator estimates E (essential matrix) it works with 
corresponding F fundamental matrices. That is due to simplification of call to 
computation of squared error. Therefore, you should transform the resulting F to E
after running RANSAC.
*/
class Mediator5ptRANSAC : public MediatorRANSAC<Matrix3d>
{
public:
  /// Constructor. Mind the order of points. Mind the order of points. 
  /// We estimate such E that (inv(K2)*pts2)'*E*(inv(K1)*pts1) = 0.
  /**
  \param[in] cam1 First camera (used for getting K and keys).
  \param[in] cam2 Second camera (used for getting K and keys).
  \param[in] matches Keys matches.
  */
  Mediator5ptRANSAC(const Camera& cam1,const Camera& cam2,
    const vector<IntPair>& matches);

  /// \return Total number of matches.
  virtual int numMatches() const;

  /// \return Minimum number of matches to compute the transformation.
  virtual int minMatches() const;

  /// Compute transformation from a minimal sample.
  /**
  \param[in] idxs Indices of matches from which to compute the transformation.
  \param[out] Fs Resulting fundamental matrices.
  */
  virtual void computeTransformation(const vector<int>& idxs,vector<Matrix3d> *Fs) const;

  /// Compute symmetric squared distance.
  /**
  \param[in] F Fundamental matrix.
  \param[in] matchIdx Index of the match.
  \return Squared error.
  */
  virtual double computeSquaredError(const Matrix3d& F,int matchIdx) const;

  /// Refine. Does nothing for now.
  virtual void refine(double tolerance,const vector<int>& inliers,Matrix3d *F) const;

private:
  const int minMatches_;
  const Camera& cam1_;
  const Camera& cam2_;
  vector<Vector3d> pts1Norm_; ///< Normalized keys of the first camera.
  vector<Vector3d> pts2Norm_; ///< Normalized keys of the second camera.
  const vector<IntPair>& matches_;
  Matrix3d invK1_; ///< Inverted K1.
  Matrix3d invK2_; ///< Inverted K2.
};

/// Implementation of mediator for homography minimal solver.
class MediatorHomographyRANSAC : public MediatorRANSAC<Matrix3d>
{
public:
  /// Constructor. We estimate: H*keys1 = lambda*keys2.
  /**
  \param[in] keys1 Keys in the first camera.
  \param[in] keys2 Keys in the second camera.
  \param[in] matches Keys matches.
  */
  MediatorHomographyRANSAC(const vector<Vector2d>& keys1,const vector<Vector2d>& keys2,
    const vector<IntPair>& matches);

  /// \return Total number of matches.
  virtual int numMatches() const;

  /// \return Minimum number of matches to compute the transformation.
  virtual int minMatches() const;

  /// Compute transformation from a minimal sample.
  /**
  \param[in] idxs Indices of matches from which to compute the transformation.
  \param[out] Hs Resulting transformations.
  */
  virtual void computeTransformation(const vector<int>& idxs,vector<Matrix3d> *Hs) const;

  /// Compute squared error for one match.
  /**
  \param[in] H A transformation.
  \param[in] matchIdx Index of the match.
  \return Squared error.
  */
  virtual double computeSquaredError(const Matrix3d& H,int matchIdx) const;

  /// Refine. Empty for now.
  virtual void refine(double tolerance,const vector<int>& inliers,Matrix3d *H) const;

private:
  const int minMatches_;
  const vector<Vector2d>& keys1_;
  const vector<Vector2d>& keys2_;
  const vector<IntPair>& matches_;
};

class Mediator7ptKnownHsRANSAC : public Mediator7ptRANSAC
{
public:
  Mediator7ptKnownHsRANSAC(const vector<Vector2d>& keys1,const vector<Vector2d>& keys2,
    const vector<IntPair>& matches,int nGroups,const vector<int>& groupId);

  /// Rejects configurations where 4 or more points belong to the same homography.
  virtual bool isPermittedSelection(const vector<int>& idxs) const;

private:
  int nGroups_;
  const vector<int>& groupId_;
};

} // namespace yasfm

namespace
{

/// Compute symmetric epipolar distance.
/**
See Hartley & Zisserman p. 278.
Compute for pts2'*F*pts1 = 0.
MIND THE ORDER of the input points.

\param[in] pt2 Point in the second camera.
\param[in] F Fundamental matrix.
\param[in] pt1 Point in the first camera.
\return Error (in pixels).
*/
template<typename T>
T computeSymmetricEpipolarDist(const Vector2d& pt2,
  const Matrix<T,3,3>& F,const Vector2d& pt1);

/// Compute squared symmetric epipolar distance.
/**
See Hartley & Zisserman p. 278.
Compute for pts2'*F*pts1 = 0.
MIND THE ORDER of the input points.

\param[in] pt2 Point in the second camera.
\param[in] F Fundamental matrix.
\param[in] pt1 Point in the first camera.
\return Squared error (in squared pixels).
*/
template<typename T>
T computeSymmetricEpipolarDistSquared(const Vector2d& pt2,
  const Matrix<T,3,3>& F,const Vector2d& pt1);

/// Function for cminpack call which computes symmetric epipolar distance.
/**
\param[in] data Pointer to EssentialMatrixRefineData.
\param[in] nPoints Number of matches/residuals.
\param[in] nParams Number of E parameters (9).
\param[in] params E parameters column wise.
\param[out] residuals Errors.
\param[in] iflag Is this call residual or Jacobian computation.
\return Flag. Negative value would terminate the optimization.
*/
int computeEssenMatSampsonDistCMINPACK(void *data,int nPoints,
  int nParams,const double* params,double* residuals,int iflag);

/// Structure for passing data into cminpack for refining.
struct EssentialMatrixRefineData
{
  const Matrix3d *invK1;
  const Matrix3d *invK2;
  const vector<Vector2d> *keys1;
  const vector<Vector2d> *keys2;
  const vector<IntPair> *matches; ///< Keys matches.
  const vector<int> *matchesToUse;     ///< Indices of inlier matches.
};

/// Compute squared first-order geometric error approximation (Sampson distance).
/**
Compute for pts2'*F*pts1 = 0. 
MIND THE ORDER of the input points.

\param[in] pt2 Point in the second camera.
\param[in] F Fundamental matrix.
\param[in] pt1 Point in the first camera.
\return Squared error (in squared pixels).
*/
template<typename T>
T computeFundMatSampsonDistSquared(const Vector2d& pt2,
  const Matrix<T,3,3>& F,const Vector2d& pt1);

/// Solve third order polynomial.
/**
Finds all DISTINCT real roots. Accepts coefficients ordered by descending powers, i.e.
p(x) = 5x^3 - x has representation [5 0 -1 0]

\param[in] coeffs Polynomial coefficients.
\param[out] roots Polynomial roots.
*/
void solveThirdOrderPoly(const Vector4d& coeffs, VectorXd *roots);

/// Solve second order polynomial.
/**
Accepts coefficients ordered by descending powers, i.e.
p(x) = 5x^3 - x has representation [5 0 -1 0]

\param[in] coeffs Polynomial coefficients.
\param[out] roots Polynomial roots.
*/
void solveSecondOrderPoly(const Vector3d& coeffs, VectorXd *roots);

/// Compute matrix transforming points to reasonably scaled data.
/**
Compute matrix moving mean to zero and standard deviation to 1.
If UseFirstMatch is true, then:
Use only points pts[matches[matchesToUse[i]].first].
else:
Use only points pts[matches[matchesToUse[i]].second].

\param[in] pts Points.
\param[in] matches Matches.
\param[in] matchesToUse Which matches to use.
\param[out] C Centering matrix.
*/
template<bool UseFirstMatch>
void matchedPointsCenteringMatrix(const vector<Vector2d>& pts,
  const vector<IntPair>& matches,const vector<int>& matchesToUse,Matrix3d *C);

} // namespace


////////////////////////////////////////////////////
///////////////   Definitions   ////////////////////
////////////////////////////////////////////////////

namespace yasfm
{

template<typename T>
void composeF(const T* const pH,const T* const v,Matrix<T,3,3> *F)
{
  Map<const Matrix<T,3,3>> H(pH);
  Matrix<T,3,3> vx;
  vx(0,0) = T(0); vx(0,1) = -v[2]; vx(0,2) = v[1];
  vx(1,0) = v[2]; vx(1,1) = T(0); vx(1,2) = -v[0];
  vx(2,0) = -v[1]; vx(2,1) = v[0]; vx(2,2) = T(0);
  F->noalias() = H * vx;
}

}

namespace
{

template<typename T>
void _commonComputeSymmetricEpipolarDist(const Vector2d& pt2,
  const Matrix<T,3,3>& F,const Vector2d& pt1,
  T *pt2Fpt1,T *sqNorm1,T *sqNorm2)
{
  Matrix<T,3,1> Fpt1 = F*pt1.homogeneous().cast<T>();
  Matrix<T,3,1> FTpt2 = F.transpose()*pt2.homogeneous().cast<T>();

  *pt2Fpt1 = pt2.homogeneous().cast<T>().dot(Fpt1);
  *sqNorm1 = Fpt1.topRows(2).squaredNorm();
  *sqNorm2 = FTpt2.topRows(2).squaredNorm();
}

template<typename T>
T computeSymmetricEpipolarDist(const Vector2d& pt2,
  const Matrix<T,3,3>& F,const Vector2d& pt1)
{
  T pt2Fpt1,sqNorm1,sqNorm2;
  _commonComputeSymmetricEpipolarDist(pt2,F,pt1,&pt2Fpt1,&sqNorm1,&sqNorm2);

  if(sqNorm1 == 0. || sqNorm2 == 0.)
  {
    return pt2Fpt1 * T(1e100);
  } else
  {
    return pt2Fpt1 *
      (T(1.) / sqrt(sqNorm1) + T(1.) / sqrt(sqNorm2));
  }
}

template<typename T>
T computeSymmetricEpipolarDistSquared(const Vector2d& pt2,
  const Matrix<T,3,3>& F,const Vector2d& pt1)
{
  T pt2Fpt1,sqNorm1,sqNorm2;
  _commonComputeSymmetricEpipolarDist(pt2,F,pt1,&pt2Fpt1,&sqNorm1,&sqNorm2);

  if(sqNorm1 == 0. || sqNorm2 == 0.)
  {
    return (pt2Fpt1*pt2Fpt1) * T(1e100);
  } else
  {
    return (pt2Fpt1*pt2Fpt1) *
      (T(1.) / sqNorm1 + T(1.) / sqNorm2);
  }
}

template<typename T>
T computeFundMatSampsonDistSquared(const Vector2d& pt2,
  const Matrix<T,3,3>& F,const Vector2d& pt1)
{
  Matrix<T,3,1> Fpt1 = F*pt1.homogeneous().cast<T>();
  Matrix<T,3,1> FTpt2 = F.transpose()*pt2.homogeneous().cast<T>();

  T pt2Fpt1 = pt2.homogeneous().cast<T>().dot(Fpt1);
  T sqNorm = (Fpt1.topRows(2).squaredNorm() + FTpt2.topRows(2).squaredNorm());
  if(sqNorm == 0.)
    return (pt2Fpt1*pt2Fpt1) * T(1e100);
  else
    return (pt2Fpt1*pt2Fpt1) * (T(1.) / sqNorm);
}

template<bool UseFirstMatch>
void matchedPointsCenteringMatrix(const vector<Vector2d>& pts,
  const vector<IntPair>& matches,const vector<int>& matchesToUse,Matrix3d *pC)
{
  auto& C = *pC;
  int n = static_cast<int>(matchesToUse.size());

  Vector2d mean(Vector2d::Zero());
  for(int i = 0; i < n; i++)
  {
    if(UseFirstMatch)
      mean += pts[matches[matchesToUse[i]].first];
    else
      mean += pts[matches[matchesToUse[i]].second];
  }
  mean /= n;

  Vector2d stdDev(Vector2d::Zero());
  for(int i = 0; i < n; i++)
  {
    const Vector2d *pt;
    if(UseFirstMatch)
      pt = &pts[matches[matchesToUse[i]].first];
    else
      pt = &pts[matches[matchesToUse[i]].second];
    stdDev += (*pt - mean).cwiseAbs2();
  }
  stdDev = (stdDev/n).cwiseSqrt();

  if(stdDev(0) < 0.1)
    stdDev(0) = 0.1;
  if(stdDev(1) < 0.1)
    stdDev(1) = 0.1;

  C << 1./stdDev(0),0.,-mean(0)/stdDev(0),
    0.,1./stdDev(1),-mean(1)/stdDev(1),
    0.,0.,1.;
}

} // namespace