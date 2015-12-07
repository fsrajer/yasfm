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

#include "Eigen\Dense"

#include "defines.h"
#include "options.h"
#include "ransac.h"
#include "sfm_data.h"

using Eigen::ArrayXXd;
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

/// Verify matches geometrically.
/**
For every camera pair, estimates fundamental matrix using PROSAC (that is why we
need dists). Then, only matches which are inliers to the fundamental matrix are
kept (as well as corresponding dists). Hence you may want to clear empty pairs after
this using removePoorlyMatchedPairs().

\param[in] solverOpt Options for estimating transformations.
\param[in] verbose Should this print status?
\param[in] cams Cameras needed for the keys.
\param[in,out] pairs Camera pairs which will get verified.
*/
YASFM_API void verifyMatchesGeometrically(const OptionsRANSAC& solverOpt, 
  bool verbose,const ptr_vector<Camera>& cams,pair_umap<CameraPair> *pairs);

/// Sets verbosity to true and calls overloaded function.
/**
\param[in] solverOpt Options for estimating transformations.
\param[in] cams Cameras needed for the keys.
\param[in,out] pairs Camera pairs which will get verified.
*/
YASFM_API void verifyMatchesGeometrically(const OptionsRANSAC& solverOpt,
  const ptr_vector<Camera>& cams,pair_umap<CameraPair> *pairs);

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
  ArrayXXd *proportion);

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

/// Compute homography (minimal solver).
/**
Computes homography H from four matches using svd decomposition.
H*pt1 = lambda*pt2.

\param[in] pts1 Points 1 (see function description).
\param[in] pts2 Points 2 (see function description).
\param[in] matches Points matches (of size 4).
\param[out] H Homography matrix.
*/
YASFM_API void estimateHomographyMinimal(const vector<Vector2d>& pts1,
  const vector<Vector2d>& pts2,const vector<IntPair>& matches,Matrix3d *H);

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

  /// Structure for passing data into cminpack for refining.
  struct RefineData
  {
    const vector<Vector2d> *keys1;
    const vector<Vector2d> *keys2;
    const vector<IntPair> *matches; ///< Keys matches.
    const vector<int> *inliers;     ///< Indices of inlier matches.
  };

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

} // namespace yasfm

namespace
{

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
double computeSymmetricEpipolarSquaredDistanceFundMat(const Vector2d& pt2,const Matrix3d& F,
  const Vector2d& pt1);

// Data are pointer to 
/// Function for cminpack call which computes symmetric epipolar distance.
/**
\param[in] data Pointer to Mediator7ptRANSAC::RefineData.
\param[in] nPoints Number of matches/residuals.
\param[in] nParams Number of F parameters (9).
\param[in] params F parameters column wise.
\param[out] residuals Errors.
\param[in] iflag Is this call residual or Jacobian computation.
\return Flag. Negative value would terminate the optimization.
*/
int computeSymmetricEpipolarDistanceFundMatCMINPACK(void *data,int nPoints,
  int nParams,const double* params,double* residuals,int iflag);

/// Compute squared first-order geometric error approximation (Sampson distance).
/**
Compute for pts2'*F*pts1 = 0. 
MIND THE ORDER of the input points.

\param[in] pt2 Point in the second camera.
\param[in] F Fundamental matrix.
\param[in] pt1 Point in the first camera.
\return Squared error (in squared pixels).
*/
double computeSampsonSquaredDistanceFundMat(const Vector2d& pt2,const Matrix3d& F,
  const Vector2d& pt1);

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

} // namespace