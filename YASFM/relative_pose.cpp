#include "relative_pose.h"

#include <ctime>
#include <iostream>

#include "5point/5point.h"
#include "ceres/ceres.h"

#include "points.h"

using Eigen::JacobiSVD;
using Eigen::MatrixXd;
using Eigen::Map;
using Eigen::Matrix;
using std::cerr;
using std::cout;

namespace yasfm
{

IntPair chooseInitialCameraPair(int minMatches,int nCams,
  const uset<int>& camsToIgnore,const vector<NViewMatch>& nViewMatches)
{
  vector<bool> isCalibrated(nCams,false);
  return chooseInitialCameraPair(minMatches,isCalibrated,camsToIgnore,nViewMatches);
}

IntPair chooseInitialCameraPair(int minMatches,
  const vector<bool>& isCalibrated,const uset<int>& camsToIgnore,
  const vector<NViewMatch>& nViewMatches)
{
  ArrayXXd scores(ArrayXXd::Zero(isCalibrated.size(),isCalibrated.size()));
  double minScore = -1.;
  return chooseInitialCameraPair(minMatches,minScore,isCalibrated,camsToIgnore,
    nViewMatches,scores);
}

IntPair chooseInitialCameraPair(int minMatches,double minScore,
  const vector<bool>& isCalibrated,const uset<int>& camsToIgnore,
  const vector<NViewMatch>& nViewMatches,const ArrayXXd& scores)
{
  size_t nCams = isCalibrated.size();
  ArrayXXi numMatches(ArrayXXi::Zero(nCams,nCams));

  for(const auto& nViewMatch : nViewMatches)
  {
    for(auto camKey1 = nViewMatch.begin(); camKey1 != nViewMatch.end(); ++camKey1)
    {
      int cam1 = camKey1->first;
      auto camKey2(camKey1);
      ++camKey2;
      for(; camKey2 != nViewMatch.end(); ++camKey2)
      {
        int cam2 = camKey2->first;
        numMatches(cam1,cam2)++;
        numMatches(cam2,cam1)++;
      }
    }
  }
  return chooseInitialCameraPair(minMatches,minScore,isCalibrated,camsToIgnore,
    numMatches,scores);
}

IntPair chooseInitialCameraPair(int minMatches,double minScore,
  const vector<bool>& isCalibrated,const uset<int>& camsToIgnore,
  const ArrayXXi& numMatches,const ArrayXXd& scores)
{
  int nCams = static_cast<int>(isCalibrated.size());
  uset<int> camsToUse;
  for(int i = 0; i < nCams; i++)
    if(isCalibrated[i] && camsToIgnore.count(i) == 0)
      camsToUse.insert(i);

  IntPair best = chooseInitialCameraPair(minMatches,minScore,camsToUse,
    numMatches,scores);

  if(best.first == -1 || best.second == -1)
  {
    for(int i = 0; i < nCams; i++)
      if(!isCalibrated[i]  && camsToIgnore.count(i) == 0)
        camsToUse.insert(i);

    best = chooseInitialCameraPair(minMatches,minScore,camsToUse,
      numMatches,scores);
  }
  return best;
}

IntPair chooseInitialCameraPair(int minMatches,double minScore,
  const uset<int>& camsToUse,const ArrayXXi& numMatches,const ArrayXXd& scores)
{
  int maxMatchesPrimary = 0;
  double maxScoreSecondary = DBL_MIN;
  IntPair bestPrimary,bestSecondary;
  for(auto it1 = camsToUse.begin(); it1 != camsToUse.end(); ++it1)
  {
    int i = *it1;
    auto it2 = it1;
    ++it2;
    for(; it2 != camsToUse.end(); ++it2)
    {
      int j = *it2;
      if(numMatches(i,j) > maxMatchesPrimary && scores(i,j) >= minScore)
      {
        bestPrimary = IntPair(i,j);
        maxMatchesPrimary = numMatches(i,j);
      }
      if(numMatches(i,j) >= minMatches && scores(i,j) > maxScoreSecondary)
      {
        bestSecondary = IntPair(i,j);
        maxScoreSecondary = scores(i,j);
      }
    }
  }
  if(maxMatchesPrimary >= minMatches)
    return bestPrimary;
  else if(maxScoreSecondary >= minScore)
    return bestSecondary;
  else
    return IntPair(-1,-1);
}

void initReconstructionFromCamPair(const OptionsRANSAC& solverOpt,
  double pointsReprojErrorThresh,const IntPair& initPair,Dataset *data)
{
  cout << "Estimating projection matrices of the initial pair ... ";
  auto& cam0 = data->cam(initPair.first);
  auto& cam1 = data->cam(initPair.second);

  data->markCamAsReconstructed(initPair.first);
  data->markCamAsReconstructed(initPair.second);

  vector<IntPair> initPairMatches;
  vector<int> nViewMatchesIdxs;
  nViewMatchesToTwoViewMatches(data->nViewMatches(),initPair,
    &initPairMatches,&nViewMatchesIdxs);

  Matrix3d F;
  bool success = estimateRelativePose7ptRANSAC(solverOpt,
    cam0.keys(),cam1.keys(),initPairMatches,&F);

  if(success)
  {
    cout << "successful\n";
    Matrix34d P1;
    F2Ps(F,&P1);
    cam0.setParams(Matrix34d::Identity());
    cam1.setParams(P1);

    int nReconstructed = reconstructPoints(data->nViewMatches(),nViewMatchesIdxs,
      initPair,&cam0,&cam1,&data->pts());
    filterOutOutliers(nViewMatchesIdxs,&data->nViewMatches());
    cout << "Reconstructing " << nReconstructed << " points\n";

    int nRemoved = removeHighReprojErrorPoints(
      pointsReprojErrorThresh,&data->cams(),&data->pts());
    cout << "Removing " << nRemoved << " points with high reprojection error\n";
  } else
  {
    cout << "unsuccessful\n";
  }
}

void initReconstructionFromCalibratedCamPair(const OptionsRANSAC& solverOpt,
  double pointsReprojErrorThresh,const IntPair& initPair,Dataset *data)
{
  cout << "Estimating camera poses of the initial pair ... ";
  auto& cam0 = data->cam(initPair.first);
  auto& cam1 = data->cam(initPair.second);

  data->markCamAsReconstructed(initPair.first);
  data->markCamAsReconstructed(initPair.second);

  vector<IntPair> initPairMatches;
  vector<int> nViewMatchesIdxs;
  nViewMatchesToTwoViewMatches(data->nViewMatches(),initPair,
    &initPairMatches,&nViewMatchesIdxs);

  Matrix3d E;
  bool success = estimateRelativePose5ptRANSAC(solverOpt,
    cam0,cam1,initPairMatches,&E);

  if(success)
  {
    cout << "successful\n";
    Matrix3d R;
    Vector3d C;
    E2RC(E,cam0.K(),cam1.K(),initPairMatches,cam0.keys(),cam1.keys(),&R,&C);
    cam0.setRotation(Matrix3d::Identity());
    cam0.setC(Vector3d::Zero());
    cam1.setRotation(R);
    cam1.setC(C);

    int nReconstructed = reconstructPoints(data->nViewMatches(),nViewMatchesIdxs,
      initPair,&cam0,&cam1,&data->pts());
    filterOutOutliers(nViewMatchesIdxs,&data->nViewMatches());
    cout << "Reconstructing " << nReconstructed << " points\n";

    int nRemoved = removeHighReprojErrorPoints(
      pointsReprojErrorThresh,&data->cams(),&data->pts());
    cout << "Removing " << nRemoved << " points with high reprojection error\n";
  } else
  {
    cout << "unsuccessful\n";
  }
}

void F2Ps(const Matrix3d& F,Matrix34d *P2)
{
  JacobiSVD<Matrix3d> svd(F,Eigen::ComputeFullU);
  Vector3d epipole2 = svd.matrixU().col(2);

  Matrix3d crossMat;
  crossProdMat(epipole2,&crossMat);
  P2->block(0,0,3,3).noalias() = crossMat*F;
  P2->col(3) = epipole2;
}

void FK2E(const Matrix3d& F,const Matrix3d& K1,const Matrix3d& K2,Matrix3d *pE)
{
  auto& E = *pE;
  E.noalias() = K2.transpose()*F*K1;

  JacobiSVD<Matrix3d> svd(E,Eigen::ComputeFullU | Eigen::ComputeFullV);
  const Vector3d& singularValues = svd.singularValues();

  Matrix3d S; 
  S.setZero();
  S(0,0) = singularValues(0);
  S(1,1) = singularValues(0);

  E.noalias() = svd.matrixU() * S * svd.matrixV().transpose();
}

void E2RC(const Matrix3d& E,const Matrix3d& K1,const Matrix3d& K2,
  const vector<IntPair>& matches,const vector<Vector2d>& keys1,
  const vector<Vector2d>& keys2,Matrix3d *R,Vector3d *C)
{
  JacobiSVD<Matrix3d> svd(E,Eigen::ComputeFullU | Eigen::ComputeFullV);
  Matrix3d W;
  W <<
    0,1,0,
    -1,0,0,
    0,0,1;

  Matrix3d U = svd.matrixU().determinant() * svd.matrixU();
  Matrix3d V = svd.matrixV().determinant() * svd.matrixV();

  Matrix3d Ra,Rb;
  Vector3d ta,tb; // not C
  Ra.noalias() = U*W*V.transpose();
  Rb.noalias() = U*W.transpose()*V.transpose();
  ta = U.col(2);
  tb = -ta;

  // We have four cases. We determine the correct solution
  // by triangulating points and checking that they are in
  // front of the both cameras. The four cases correspond
  // to the four combinations of "in front of the first camera"
  // and "in front of the second camera".
  Matrix34d P1(Matrix34d::Identity()),Pa,Pc;

  Pa.leftCols(3) = Ra;
  //Pb.leftCols(3) = Ra;
  Pc.leftCols(3) = Rb;
  //Pd.leftCols(3) = Rb;

  Pa.rightCols(1) = ta;
  //Pb.rightCols(1) = tb;
  Pc.rightCols(1) = ta;
  //Pd.rightCols(1) = tb;

  P1 = K1*P1;
  Pa = K2*Pa;
  //Pb = K2*Pb;
  Pc = K2*Pc;
  //Pd = K2*Pd;

  vector<Vector3d> points;

  triangulate(P1,Pa,keys1,keys2,matches,&points);
  int inFront1 = 0,inFront2 = 0;
  for(const auto& pt : points)
  {
    if(isInFrontNormalizedP(P1,pt))
      inFront1++;
    if(isInFrontNormalizedP(Pa,pt))
      inFront2++;
  }

  double halfPoints = 0.5*points.size();
  if(inFront1 >= halfPoints && inFront2 >= halfPoints)
  {
    *R = Ra;
    *C = -Ra.transpose()*ta;
  } else if(inFront1 <= halfPoints && inFront2 <= halfPoints)
  {
    *R = Ra;
    *C = -Ra.transpose()*tb;
  } else
  {
    points.clear();
    triangulate(P1,Pc,keys1,keys2,matches,&points);
    inFront1 = inFront2 = 0;
    for(const auto& pt : points)
    {
      if(isInFrontNormalizedP(P1,pt))
        inFront1++;
      if(isInFrontNormalizedP(Pc,pt))
        inFront2++;
    }
    if(inFront1 >= halfPoints && inFront2 >= halfPoints)
    {
      *R = Rb;
      *C = -Rb.transpose()*ta;
    } else if(inFront1 <= halfPoints && inFront2 <= halfPoints)
    {
      *R = Rb;
      *C = -Rb.transpose()*tb;
    } else
    {
      cerr << "ERROR: E2RC: None of the 4 decompositions is good\n";
      return;
    }
  }
}

void verifyMatchesEpipolar(const OptionsRANSAC& solverOpt,
	const ptr_vector<Camera>& cams, pair_umap<CameraPair> *pairs, 
	GeomVerifyCallbackFunctionPtr callbackFunction, void * callbackObjectPtr)
{
	verifyMatchesEpipolar(solverOpt, true, cams, pairs, callbackFunction, callbackObjectPtr);
}

void verifyMatchesEpipolar(const OptionsRANSAC& solverOpt,
  bool verbose,const ptr_vector<Camera>& cams,pair_umap<CameraPair> *pairs,
  GeomVerifyCallbackFunctionPtr callbackFunction, void * callbackObjectPtr)
{
  clock_t start,end;
  int pairsDone = 0;
  int nPrevMatches;
  for(auto it = pairs->begin(); it != pairs->end();)
  {
    IntPair camsIdx = it->first;
    auto &pair = it->second;

    if(verbose)
    {
      cout << "verifying: " << camsIdx.first << " -> " << camsIdx.second << "\t";
      cout << pair.matches.size();
      start = clock();
    }

    Matrix3d F;
    vector<int> inliers;
    bool success = estimateRelativePose7ptPROSAC(solverOpt,
      cams[camsIdx.first]->keys(),cams[camsIdx.second]->keys(),pair,&F,&inliers);
    if(success)
    {
      nPrevMatches = static_cast<int>(pair.matches.size());
      filterVector(inliers,&pair.matches);
      filterVector(inliers,&pair.dists);
      ++it;
	  pairsDone++;
    } else
    {
      it = pairs->erase(it);
    }

    int nInliers = static_cast<int>(inliers.size());
    if(callbackFunction != NULL&&callbackObjectPtr != NULL)
    {
      callbackFunction(callbackObjectPtr,camsIdx,nPrevMatches,nInliers,double(pairsDone) / pairs->size());
    }

    if(verbose)
    {
      end = clock();
      cout << "->" << nInliers << " matches" << "\t";
      cout << "took: " << (double)(end - start) / (double)CLOCKS_PER_SEC << "s\n";
    }
  }
}

bool estimateRelativePose7ptPROSAC(const OptionsRANSAC& opt,
  const vector<Vector2d>& keys1,const vector<Vector2d>& keys2,const CameraPair& pair,
  Matrix3d *F,vector<int> *inliers)
{
  Mediator7ptRANSAC m(keys1,keys2,pair.matches);
  vector<int> matchesOrder;
  yasfm::quicksort(pair.dists,&matchesOrder);
  int nInliers = estimateTransformPROSAC(m,opt,matchesOrder,F,inliers);
  return (nInliers > 0);
}

bool estimateRelativePose7ptRANSAC(const OptionsRANSAC& opt,
  const vector<Vector2d>& keys1,const vector<Vector2d>& keys2,const vector<IntPair>& matches,
  Matrix3d *F,vector<int> *inliers)
{
  Mediator7ptRANSAC m(keys1,keys2,matches);
  int nInliers = estimateTransformRANSAC(m,opt,F,inliers);
  return (nInliers > 0);
}

void estimateRelativePose7pt(const vector<Vector2d>& keys1,const vector<Vector2d>& keys2,
  const vector<IntPair>& matches,vector<Matrix3d> *pFs)
{
  const size_t minPts = 7;
  if(matches.size() < minPts)
  {
    cerr << "ERROR: estimateRelativePose7pt: "
      << "Cannot estimate transformation. " << matches.size()
      << " matches given but 7 needed.\n";
    pFs->clear();
    return;
  }

  // The equation pt2'*F*pt1 = 0 rewritten, i.e. one row
  // corresponds to this equation for one pair of points. 
  MatrixXd A;
  A.resize(7,9);
  for(size_t i = 0; i < minPts; i++)
  {
    const auto& pt1 = keys1[matches[i].first];
    const auto& pt2 = keys2[matches[i].second];
    A.row(i) <<
      pt1(0) * pt2(0),
      pt1(0) * pt2(1),
      pt1(0),
      pt1(1) * pt2(0),
      pt1(1) * pt2(1),
      pt1(1),
      pt2(0),
      pt2(1),
      1;
  }
  // A*f has 9 variables and 7 equations, therefore the
  // nullspace has dimensionality 2 (last 2 columns of V form SVD)
  JacobiSVD<MatrixXd> svd(A,Eigen::ComputeFullV);
  VectorXd f1 = svd.matrixV().col(7);
  VectorXd f2 = svd.matrixV().col(8);

  // The solutions space corresponds to lambda*f1 + mu*f2.
  // Since F is determined up to a scale, we can normalize these:
  // lambda + mu = 1. Hence lambda*f1 + (1-lambda)*f2 generates the 
  // solution space. We now enforce the rank 2 constraint by
  // det(lambda*F1 + (1-lambda)*F2) = 0 which is cubic equation.
  // (F1 and F2 correspond to f1 and f2 respectively)

  // Coefficients of the cubic polynomial ordered by descending powers (as in MATLAB).
  Vector4d coeffs;

  VectorXd f0 = f1 - f2;
  double term00 = f0(4)*f0(8) - f0(5)*f0(7);
  double term10 = f0(3)*f0(8) - f0(5)*f0(6);
  double term20 = f0(3)*f0(7) - f0(4)*f0(6);
  double term01 = f0(4)*f2(8) + f2(4)*f0(8) - f0(5)*f2(7) - f2(5)*f0(7);
  double term11 = f0(3)*f2(8) + f2(3)*f0(8) - f0(5)*f2(6) - f2(5)*f0(6);
  double term21 = f0(3)*f2(7) + f2(3)*f0(7) - f0(4)*f2(6) - f2(4)*f0(6);
  double term02 = f2(4)*f2(8) - f2(5)*f2(7);
  double term12 = f2(3)*f2(8) - f2(5)*f2(6);
  double term22 = f2(3)*f2(7) - f2(4)*f2(6);

  coeffs(0) = f0(0)*term00 - f0(1)*term10 + f0(2)*term20;
  coeffs(1) = f0(0)*term01 + f2(0)*term00
    - f0(1)*term11 - f2(1)*term10
    + f0(2)*term21 + f2(2)*term20;
  coeffs(2) = f0(0)*term02 + f2(0)*term01
    - f0(1)*term12 - f2(1)*term11
    + f0(2)*term22 + f2(2)*term21;
  coeffs(3) = f2(0)*term02 - f2(1)*term12 + f2(2)*term22;

  VectorXd roots;
  solveThirdOrderPoly(coeffs,&roots);

  auto& Fs = *pFs;
  Fs.resize(roots.rows());
  for(int i = 0; i < roots.rows(); i++)
  {
    double lambda = roots(i);
    f0 = lambda*f1 + (1 - lambda)*f2;
    Fs[i].col(0) = f0.middleRows(0,3);
    Fs[i].col(1) = f0.middleRows(3,3);
    Fs[i].col(2) = f0.middleRows(6,3);
  }
}

bool estimateRelativePose5ptRANSAC(const OptionsRANSAC& opt,
  const Camera& cam1,const Camera& cam2,const vector<IntPair>& matches,
  Matrix3d *E,vector<int> *inliers)
{
  Matrix3d F;
  Mediator5ptRANSAC m(cam1,cam2,matches);
  int nInliers = estimateTransformRANSAC(m,opt,&F,inliers);
  *E = cam2.K().transpose() * F * cam1.K();
  return (nInliers > 0);
}

void estimateRelativePose5pt(const vector<Vector3d>& pts1Norm,
  const vector<Vector3d>& pts2Norm,const vector<IntPair>& matches,
  vector<Matrix3d> *pEs)
{
  if(matches.size() < 5)
  {
    cerr << "ERROR: estimateRelativePose5pt: "
      << "Cannot estimate transformation. " << matches.size()
      << " matches given but 5 needed.\n";
    pEs->clear();
    return;
  }
  auto& Es = *pEs;
  double matchedPts1[5][3],matchedPts2[5][3];
  for(size_t i = 0; i < 5; i++)
  {
    memcpy(matchedPts1[i],pts1Norm[matches[i].first].data(),3 * sizeof(double));
    memcpy(matchedPts2[i],pts2Norm[matches[i].second].data(),3 * sizeof(double));
  }
  
  fivepoint::Ematrix EsRaw[fivepoint::Maxsolutions];
  int nRoots;
  bool optimized = true;
  fivepoint::compute_E_matrices(matchedPts1,matchedPts2,EsRaw,nRoots,optimized);
  
  Es.resize(nRoots);
  for(int i = 0; i < nRoots; i++)
  {
    // Es are column major. Hence we put rows in columns and then transpose.
    memcpy(Es[i].col(0).data(),&(EsRaw[i][0]),3 * sizeof(double));
    memcpy(Es[i].col(1).data(),&(EsRaw[i][1]),3 * sizeof(double));
    memcpy(Es[i].col(2).data(),&(EsRaw[i][2]),3 * sizeof(double));
    Es[i].transposeInPlace();
  }
}

void estimateFundamentalMatrix(const vector<Vector2d>& pts1,
  const vector<Vector2d>& pts2,const vector<IntPair>& matches,
  const vector<int>& matchesToUse,double tolerance,Matrix3d *pF)
{
  auto& F = *pF;
  int nUseful = static_cast<int>(matchesToUse.size());

  if(nUseful < 8)
  {
    cerr << "ERROR: estimateFundamentalMatrix: Too few matches handed."
      << " At least 8 needed and " << nUseful << " was given.\n";
    F.setZero();
    return;
  }

  Matrix3d C1,C2;
  matchedPointsCenteringMatrix<true>(pts1,matches,matchesToUse,&C1);
  matchedPointsCenteringMatrix<false>(pts2,matches,matchesToUse,&C2);

  MatrixXd A(nUseful,8);
  VectorXd b(nUseful);
  for(int i = 0; i < nUseful; i++)
  {
    Vector3d pt1 = C1 * pts1[matches[matchesToUse[i]].first].homogeneous();
    Vector3d pt2 = C2 * pts2[matches[matchesToUse[i]].second].homogeneous();

    A.row(i) <<
      pt1(0) * pt2(0),
      pt1(0) * pt2(1),
      pt1(0) * pt2(2),
      pt1(1) * pt2(0),
      pt1(1) * pt2(1),
      pt1(1) * pt2(2),
      pt1(2) * pt2(0),
      pt1(2) * pt2(1);

    b(i) = -(pt1(2)*pt2(2));
  }

  VectorXd f = A.jacobiSvd(Eigen::ComputeThinU | Eigen::ComputeThinV).solve(b);

  // Not rank 2 yet.
  Matrix3d F_;
  F_.col(0) = f.topRows(3);
  F_.col(1) = f.middleRows(3,3);
  F_(0,2) = f(6); F_(1,2) = f(7); F_(2,2) = 1.;

  closestRank2Matrix(F_,&F);

  // Un-normalize
  F = C2.transpose() * F * C1;

  refineFundamentalMatrixNonLinear(pts1,pts2,matches,matchesToUse,tolerance,&F);
}

void refineFundamentalMatrixNonLinear(const vector<Vector2d>& pts1,
  const vector<Vector2d>& pts2,const vector<IntPair>& matches,
  const vector<int>& matchesToUse,double tolerance,Matrix3d *F)
{
  int numPoints = static_cast<int>(matchesToUse.size());
  const int numParams = 9;

  FundamentalMatrixRefineData data;
  data.keys1 = &pts1;
  data.keys2 = &pts2;
  data.matches = &matches;
  data.matchesToUse = &matchesToUse;

  vector<double> residuals(numPoints);

  Matrix3d tmp = *F;
  nonLinearOptimLMCMINPACK(computeSymmetricEpipolarDistanceFundMatCMINPACK,
    &data,numPoints,numParams,tolerance,tmp.data(),&residuals[0]);

  closestRank2Matrix(tmp,F);
}

int findFundamentalMatrixInliers(double thresh,const vector<Vector2d>& pts1,
  const vector<Vector2d>& pts2,const vector<IntPair>& matches,const Matrix3d& F,
  vector<int> *pinliers)
{
  auto& inliers = *pinliers;
  int nMatches = static_cast<int>(matches.size());
  double sqThresh = thresh*thresh;
  inliers.clear();
  inliers.reserve(nMatches);
  for(int iMatch = 0; iMatch < nMatches; iMatch++)
  {
    double sqDist = computeSymmetricEpipolarSquaredDistanceFundMat(
      pts2[matches[iMatch].second],
      F,
      pts1[matches[iMatch].first]);
    if(sqDist < sqThresh)
    {
      inliers.push_back(iMatch);
    }
  }
  return static_cast<int>(inliers.size());
}

void estimateEssentialMatrix(const vector<Vector2d>& pts1,
  const vector<Vector2d>& pts2,const Matrix3d& invK1,const Matrix3d& invK2,
  const vector<IntPair>& matches,const vector<int>& matchesToUse,
  double tolerance,Matrix3d *pE)
{
  auto& E = *pE;
  int nUseful = static_cast<int>(matchesToUse.size());

  if(nUseful < 8)
  {
    cerr << "ERROR: estimateEssentialMatrix: Too few matches handed."
      << " At least 8 needed and " << nUseful << " was given.\n";
    E.setZero();
    return;
  }

  MatrixXd A(nUseful,8);
  VectorXd b(nUseful);
  for(int i = 0; i < nUseful; i++)
  {
    Vector3d pt1 = invK1 * pts1[matches[matchesToUse[i]].first].homogeneous();
    Vector3d pt2 = invK2 * pts2[matches[matchesToUse[i]].second].homogeneous();

    A.row(i) <<
      pt1(0) * pt2(0),
      pt1(0) * pt2(1),
      pt1(0) * pt2(2),
      pt1(1) * pt2(0),
      pt1(1) * pt2(1),
      pt1(1) * pt2(2),
      pt1(2) * pt2(0),
      pt1(2) * pt2(1);

    b(i) = -(pt1(2)*pt2(2));
  }

  VectorXd e = A.jacobiSvd(Eigen::ComputeThinU | Eigen::ComputeThinV).solve(b);

  // Not rank 2 yet. Nor has it the same 1st and 2nd singular values.
  Matrix3d E_;
  E_.col(0) = e.topRows(3);
  E_.col(1) = e.middleRows(3,3);
  E_(0,2) = e(6); E_(1,2) = e(7); E_(2,2) = 1.;

  closestEssentialMatrix(E_,&E);

  refineEssentialMatrixNonLinear(pts1,pts2,invK1,invK2,matches,matchesToUse,tolerance,&E);
}

void refineEssentialMatrixNonLinear(const vector<Vector2d>& pts1,
  const vector<Vector2d>& pts2,const Matrix3d& invK1,const Matrix3d& invK2,
  const vector<IntPair>& matches,const vector<int>& matchesToUse,
  double tolerance,Matrix3d *E)
{
  int numPoints = static_cast<int>(matchesToUse.size());
  const int numParams = 9;

  EssentialMatrixRefineData data;
  data.invK1 = &invK1;
  data.invK2 = &invK2;
  data.keys1 = &pts1;
  data.keys2 = &pts2;
  data.matches = &matches;
  data.matchesToUse = &matchesToUse;

  vector<double> residuals(numPoints);

  Matrix3d tmp = *E;
  nonLinearOptimLMCMINPACK(computeSymmetricEpipolarDistanceEssenMatCMINPACK,
    &data,numPoints,numParams,tolerance,tmp.data(),&residuals[0]);

  closestEssentialMatrix(tmp,E);
}

void computeHomographyInliersProportion(const OptionsRANSAC& opt,
  const ptr_vector<Camera>& cams,const pair_umap<CameraPair>& pairs,
  ArrayXXd *pproportion, HomographyInliersCallbackFunctionPtr callbackFunction, void * callbackObjectPtr)
{
  auto& proportion = *pproportion;
  proportion.resize(cams.size(),cams.size());
  proportion.fill(1.);
  int pairsDone = 0;
  for(const auto& entry : pairs)
  {
    int i = entry.first.first;
    int j = entry.first.second;
    const auto& pair = entry.second;

    Matrix3d H;
    vector<int> inliers;
    bool success = estimateHomographyPROSAC(opt,cams[i]->keys(),
      cams[j]->keys(),pair,&H,&inliers);
    if(success && pair.matches.size() > 0)
    {
      proportion(i,j) = static_cast<double>(inliers.size()) / pair.matches.size();
      proportion(j,i) = proportion(i,j);
    }
	pairsDone++;
	if (callbackFunction != NULL&&callbackObjectPtr != NULL){
		callbackFunction(callbackObjectPtr, pairsDone / double(pairs.size()));
	}
  }
}

bool estimateHomographyRANSAC(const OptionsRANSAC& opt,const vector<Vector2d>& pts1,
  const vector<Vector2d>& pts2,const vector<IntPair>& matches,Matrix3d *H,
  vector<int> *inliers)
{
  MediatorHomographyRANSAC m(pts1,pts2,matches);
  int nInliers = estimateTransformRANSAC(m,opt,H,inliers);
  return (nInliers > 0);
}

bool estimateHomographyPROSAC(const OptionsRANSAC& opt,const vector<Vector2d>& pts1,
  const vector<Vector2d>& pts2,const CameraPair& pair,Matrix3d *H,
  vector<int> *inliers)
{
  MediatorHomographyRANSAC m(pts1,pts2,pair.matches);
  vector<int> matchesOrder;
  yasfm::quicksort(pair.dists,&matchesOrder);
  int nInliers = estimateTransformPROSAC(m,opt,matchesOrder,H,inliers);
  return (nInliers > 0);
}

void verifyMatchesGeometrically(const OptionsGeometricVerification& opt,
  const ptr_vector<Camera>& cams,pair_umap<CameraPair> *pairs)
{
  verifyMatchesGeometrically(opt,true,cams,pairs);
}

void verifyMatchesGeometrically(const OptionsGeometricVerification& opt,
  bool verbose,const ptr_vector<Camera>& cams,pair_umap<CameraPair> *pairs)
{
  clock_t start,end;
  for(auto it = pairs->begin(); it != pairs->end();)
  {
    IntPair camsIdx = it->first;
    auto &pair = it->second;

    if(verbose)
    {
      cout << "verifying: " << camsIdx.first << " -> " << camsIdx.second << "\t";
      cout << pair.matches.size();
      start = clock();
    }

    vector<int> inliers;
    int nInliers = verifyMatchesGeometrically(opt,
      *cams[camsIdx.first],*cams[camsIdx.second],pair.matches,&inliers,
      &pair.supportSizes);
    if(nInliers >= opt.get<int>("minInliersPerTransform"))
    {
      // Filter and reorder
      vector<IntPair> newMatches(inliers.size());
      vector<double> newDists(inliers.size());
      for(size_t i = 0; i < inliers.size(); i++)
      {
        newMatches[i] = pair.matches[inliers[i]];
        newDists[i] = pair.dists[inliers[i]];
      }
      pair.matches = newMatches;
      pair.dists = newDists;
      ++it;
    } else
    {
      it = pairs->erase(it);
    }

    if(verbose)
    {
      end = clock();
      cout << "->" << inliers.size() << " matches" << "\t";
      cout << "took: " << (double)(end - start) / (double)CLOCKS_PER_SEC << "s\n";
    }
  }
}

struct GeomVerifCostFunctor
{
  GeomVerifCostFunctor(int nTrans,const vector<bool>& isGroupEG,
    const Matrix3d& invK1,const Matrix3d& invK2T,const Vector2d& x1,const Vector2d& x2)
    : nTrans(nTrans),isGroupEG(isGroupEG),invK1(invK1),invK2T(invK2T),x1(x1),x2(x2)
  {
  }

  template<typename T>
  bool operator()(T const* const* parameters,T* residuals) const
  {
    const T *alpha = parameters[0];

    residuals[0] = T(0.);
    T alphaSqSum = T(0.);
    for(int iT = 0; iT < nTrans; iT++)
    {
      T err;
      if(isGroupEG[iT])
      {
        const T *t = &parameters[iT+1][0];
        const T *rot = &parameters[iT+1][3];
        Matrix<T,3,3> A,R,F;

        A(0) = T(0.); A(3) = t[2];  A(6) = -t[1];
        A(1) = -t[2]; A(4) = T(0.); A(7) = t[0];
        A(2) = t[1];  A(5) = -t[0]; A(8) = T(0.);

        Map<const Matrix<T,3,1>> rotMap(rot);
        T angle = rotMap.norm();
        Eigen::AngleAxis<T> aa(angle,rotMap/angle);
        R = aa.toRotationMatrix();

        F = invK2T.cast<T>() * A * R * invK1.cast<T>();

        Matrix<T,3,1> Fx1 = F*x1.homogeneous().cast<T>();
        Matrix<T,3,1> FTx2 = F.transpose()*x2.homogeneous().cast<T>();

        T x2Fx1 = x2.homogeneous().cast<T>().dot(Fx1);

        err = sqrt( (x2Fx1*x2Fx1) *
          (T(1.) / Fx1.topRows(2).squaredNorm() + 
          T(1.) / FTx2.topRows(2).squaredNorm()) );
      } else
      {
        const T *H = parameters[iT+1];

        T pt[3];
        pt[0] = H[0] * T(x1(0)) + H[3] * T(x1(1)) + H[6];
        pt[1] = H[1] * T(x1(0)) + H[4] * T(x1(1)) + H[7];
        pt[2] = H[2] * T(x1(0)) + H[5] * T(x1(1)) + H[8];

        T diff[2];
        diff[0] = T(x2(0)) - (pt[0] / pt[2]);
        diff[1] = T(x2(1)) - (pt[1] / pt[2]);

        err = sqrt(diff[0]*diff[0] + diff[1]*diff[1]);
      }

      T aSq = alpha[iT]*alpha[iT];

      alphaSqSum += aSq;
      residuals[0] += aSq * err;
    }

    if(alphaSqSum != T(0.))
      residuals[0] /= alphaSqSum;

    return true;
  }

  int nTrans;
  const vector<bool> isGroupEG;
  const Matrix3d& invK1,invK2T;
  const Vector2d& x1,x2;
};

bool canBeMerged(const OptionsGeometricVerification& opt,
  const vector<int>& group1,const vector<int>& group2,
  const Camera& cam1,const Camera& cam2,const vector<IntPair>& matches)
{
  vector<int> bothGroups;
  bothGroups.reserve(group1.size() + group2.size());
  bothGroups.insert(bothGroups.begin(),group1.begin(),group1.end());
  bothGroups.insert(bothGroups.end(),group2.begin(),group2.end());

  Matrix3d invK1 = cam1.K().inverse();
  Matrix3d invK2 = cam2.K().inverse();
  Matrix3d E;
  estimateEssentialMatrix(cam1.keys(),cam2.keys(),invK1,invK2,matches,bothGroups,
    opt.get<double>("fundMatRefineTolerance"),&E);
  Matrix3d F = invK2.transpose() * E * invK1;

  vector<IntPair> relevantMatches(bothGroups.size());
  for(size_t i = 0; i < relevantMatches.size(); i++)
    relevantMatches[i] = matches[bothGroups[i]];

  vector<int> inliers;
  findFundamentalMatrixInliers(opt.get<double>("fundMatThresh"),
    cam1.keys(),cam2.keys(),relevantMatches,F,&inliers);

  return (inliers.size() >= (size_t)(0.95*relevantMatches.size()));
}

void enrichInliersWithEG(const OptionsGeometricVerification& opt,
  const Camera& cam1,const Camera& cam2,
  const vector<IntPair>& matches,vector<bool> *punassigned,
  vector<int> *pinliers,Matrix3d *pE)
{
  auto& unassigned = *punassigned;
  auto& inliers = *pinliers;
  auto& E = *pE;

  Matrix3d invK1 = cam1.K().inverse();
  Matrix3d invK2 = cam2.K().inverse();
  estimateEssentialMatrix(cam1.keys(),cam2.keys(),invK1,invK2,matches,inliers,
    opt.get<double>("fundMatRefineTolerance"),&E);
  Matrix3d F = invK2.transpose() * E * invK1;

  vector<int> newInliers;
  findFundamentalMatrixInliers(opt.get<double>("fundMatThresh"),
    cam1.keys(),cam2.keys(),matches,F,&newInliers);

  for(int idx : newInliers)
  {
    if(unassigned[idx])
    {
      unassigned[idx] = false;
      inliers.push_back(idx);
    }
  }
}

int verifyMatchesGeometrically(const OptionsGeometricVerification& opt,
  const Camera& cam1,const Camera& cam2,const vector<IntPair>& allMatches,
  vector<int> *poutInliers,vector<int> *pinlierSetSizes)
{
  auto& outInliers = *poutInliers;
  auto& inlierSetSizes = *pinlierSetSizes;
  vector<Matrix3d> Ts;

  int nAllMatches = static_cast<int>(allMatches.size());
  vector<IntPair> remainingMatches = allMatches;
  vector<int> remainingToAll(nAllMatches);
  for(int i = 0; i < nAllMatches; i++)
    remainingToAll[i] = i;

  Matrix3d bestH;
  vector<int> bestInliers,currInliers;
  bestInliers.reserve(nAllMatches);
  currInliers.reserve(nAllMatches);

  for(int iTransform = 0; iTransform < opt.get<int>("maxTransforms"); iTransform++)
  {
    bestInliers.clear();

    for(size_t iMatch = 0; iMatch < remainingMatches.size(); iMatch++)
    {
      int k1 = remainingMatches[iMatch].first;
      int k2 = remainingMatches[iMatch].second;
      currInliers.clear();
      Matrix3d currH;

      for(int iRefine = 0; iRefine < opt.get<int>("nRefineIterations"); iRefine++)
      {
        double thresh;
        if(iRefine == 0)
        {
          computeSimilarityFromMatch(cam1.key(k1),cam1.keysScales()[k1],
            cam1.keysOrientations()[k1],cam2.key(k2),cam2.keysScales()[k2],
            cam2.keysOrientations()[k2],&currH);
          thresh = opt.get<double>("similarityThresh");
        } else if(iRefine <= 4)
        {
          estimateAffinity(cam1.keys(),cam2.keys(),remainingMatches,
            currInliers,&currH);
          thresh = opt.get<double>("affinityThresh");
        } else
        {
          estimateHomography(cam1.keys(),cam2.keys(),remainingMatches,
            currInliers,&currH);
          thresh = opt.get<double>("homographyThresh");
        }

        currInliers.clear();
        findHomographyInliers(thresh,cam1.keys(),cam2.keys(),
          remainingMatches,currH,&currInliers);
        
        if(currInliers.size() < opt.get<int>("minInliersToRefine"))
          break;
        
        if(currInliers.size() > 
          opt.get<double>("stopInlierFraction")*remainingMatches.size())
          break;
      }

      if(currInliers.size() > bestInliers.size())
      {
        bestInliers = currInliers;
        bestH = currH;
      }
    }

    if(bestInliers.size() < opt.get<int>("minInliersPerTransform"))
      break;

    Ts.push_back(bestH);
    inlierSetSizes.push_back(static_cast<int>(bestInliers.size()));
    for(int remainingMatchesInlier : bestInliers)
      outInliers.push_back(remainingToAll[remainingMatchesInlier]);
    filterOutOutliers(bestInliers,&remainingMatches);
    filterOutOutliers(bestInliers,&remainingToAll);

    if(remainingMatches.size() < opt.get<int>("minInliersPerTransform"))
      break;
  }

  if(inlierSetSizes.size() > 1)
  {
    vector<vector<int>> groups(inlierSetSizes.size());
    vector<bool> isGroupEG(groups.size(),false);
    int idx = 0;
    for(size_t i = 0; i < inlierSetSizes.size(); i++)
    {
      groups[i].reserve(inlierSetSizes[i]);
      for(int j = 0; j < inlierSetSizes[i]; j++, idx++)
      {
        groups[i].push_back(outInliers[idx]);
      }
    }

    for(int i = 0; i < (int)groups.size(); i++)
    {
      for(int j = i+1; j < (int)groups.size(); j++)
      {
        if(canBeMerged(opt,groups[i],groups[j],cam1,cam2,allMatches))
        {
          groups[i].insert(groups[i].end(),groups[j].begin(),groups[j].end());
          groups.erase(groups.begin() + j);
          isGroupEG[i] = true;
          isGroupEG.erase(isGroupEG.begin() + j);
          Ts.erase(Ts.begin() + j);
          j--;
        }
      }
    }

    vector<bool> unassigned(allMatches.size(),true);
    for(int idx : outInliers)
      unassigned[idx] = false;

    for(size_t ig = 0; ig < groups.size(); ig++)
    {
      if(isGroupEG[ig])
        enrichInliersWithEG(opt,cam1,cam2,allMatches,&unassigned,&groups[ig],&Ts[ig]);
    }

    outInliers.clear();
    inlierSetSizes.clear();
    for(const auto& g : groups)
    {
      outInliers.insert(outInliers.end(),g.begin(),g.end());
      inlierSetSizes.push_back(static_cast<int>(g.size()));
    }

    int nTrans = (int)Ts.size();
    int nInliers = (int)outInliers.size();
    vector<double> alpha(nTrans*nInliers,0.);

    vector<VectorXd> EsParams(nTrans);
    for(int iT = 0; iT < nTrans; iT++)
    {
      if(isGroupEG[iT])
      {
        vector<IntPair> currMatches(groups[iT].size());
        for(size_t i = 0; i < currMatches.size(); i++)
          currMatches[i] = allMatches[groups[iT][i]];

        Matrix3d R;
        Vector3d t,C;
        E2RC(Ts[iT],cam1.K(),cam2.K(),currMatches,cam1.keys(),cam2.keys(),
          &R,&C);
        t = -R*C;

        Eigen::AngleAxisd aa;
        aa.fromRotationMatrix(R);

        EsParams[iT].resize(6);
        EsParams[iT].topRows(3) = t;
        EsParams[iT].bottomRows(3) = aa.angle() * aa.axis();
      }
    }

    // Initialize weights
    int ii = 0;
    vector<double *> alphas(nInliers);
    for(int iT = 0; iT < nTrans; iT++)
    {
      for(int i = 0; i < inlierSetSizes[iT]; i++,ii++)
      {
        alpha[ii*nTrans + iT] = 1.;
        alphas[ii] = &alpha[ii*nTrans];
      }
    }

    // Set-up the problem
    ceres::Problem problem;
    Matrix3d invK1 = cam1.K().inverse(),
      invK2T = cam2.K().inverse().transpose();
    for(int ii = 0; ii < nInliers; ii++)
    {
      IntPair match = allMatches[outInliers[ii]];
      const auto& pt1 = cam1.key(match.first);
      const auto& pt2 = cam2.key(match.second);

      // NULL specifies squared loss
      ceres::LossFunction *lossFun = NULL;

      auto costFun =
        new ceres::DynamicAutoDiffCostFunction<GeomVerifCostFunctor>(
        new GeomVerifCostFunctor(nTrans,isGroupEG,invK1,invK2T,pt1,pt2));

      costFun->SetNumResiduals(1);
      costFun->AddParameterBlock(nTrans);
      
      vector<double *> parameterBlocks(1+nTrans);
      parameterBlocks[0] = alphas[ii];
      for(int iT = 0; iT < nTrans; iT++)
      {
        if(isGroupEG[iT])
        {
          costFun->AddParameterBlock(6);
          parameterBlocks[1+iT] = &EsParams[iT](0);
        } else
        {
          costFun->AddParameterBlock(9);
          parameterBlocks[1+iT] = &Ts[iT](0,0);
        }
      }
      problem.AddResidualBlock(costFun,lossFun,parameterBlocks);
    }

    ceres::Solver::Summary summary;
    ceres::Solver::Options solverOpt;
    ceres::Solve(solverOpt,&problem,&summary);
    //std::cout << summary.FullReport() << "\n";

    // Compute weights in [0,1] from alpha
    vector<double> weights(nTrans*nInliers);
    for(int ii = 0; ii < nInliers; ii++)
    {
      double sum = 0.;
      for(size_t iH = 0; iH < nTrans; iH++)
        sum += alpha[ii*nTrans + iH] * alpha[ii*nTrans + iH];
      for(size_t iH = 0; iH < nTrans; iH++)
        weights[ii*nTrans + iH] = (alpha[ii*nTrans + iH] * alpha[ii*nTrans + iH]) / sum;
    }

    // Re-assign matches
    vector<vector<int>> finalInliers(Ts.size());
    for(int ii = 0; ii < nInliers; ii++)
    {
      int max = 0;
      for(int iT = 1; iT < nTrans; iT++)
      {
        if(weights[ii*nTrans + iT] > weights[ii*nTrans + max])
          max = iT;
      }
      finalInliers[max].push_back(outInliers[ii]);
    }
    outInliers.clear();
    for(int iT = 0; iT < nTrans; iT++)
    {
      inlierSetSizes[iT] = (int)finalInliers[iT].size();
      outInliers.insert(outInliers.end(),finalInliers[iT].begin(),
        finalInliers[iT].end());
    }
  }

  return static_cast<int>(outInliers.size());
}

void computeSimilarityFromMatch(const Vector2d& coord1,double scale1,
  double orientation1,const Vector2d& coord2,double scale2,double orientation2,
  Matrix3d *S)
{
  double c1 = cos(orientation1),
    s1 = sin(orientation1),
    c2 = cos(orientation2),
    s2 = sin(orientation2);

  Matrix3d A1,A2;
  A1 << scale1*c1,scale1*(-s1),coord1(0),
    scale1*s1,scale1*c1,coord1(1),
    0,0,1;
  A2 << scale2*c2,scale2*(-s2),coord2(0),
    scale2*s2,scale2*c2,coord2(1),
    0,0,1;

  *S = A2*A1.inverse();
}

void estimateAffinity(const vector<Vector2d>& keys1,
  const vector<Vector2d>& keys2,const vector<IntPair>& matches,
  const vector<int>& matchesToUse,Matrix3d *pA)
{
  auto& A = *pA;
  int nUseful = static_cast<int>(matchesToUse.size());
  MatrixXd M(MatrixXd::Zero(2*nUseful,6));
  VectorXd b(2*nUseful);
  for(int i = 0; i < nUseful; i++)
  {
    const auto& key1 = keys1[matches[matchesToUse[i]].first];
    const auto& key2 = keys2[matches[matchesToUse[i]].second];
    M.block(i,0,1,3) = key1.homogeneous().transpose();
    M.block(i+nUseful,3,1,3) = key1.homogeneous().transpose();
    b(i) = key2(0);
    b(i+nUseful) = key2(1);
  }
  VectorXd a = M.jacobiSvd(Eigen::ComputeThinU | Eigen::ComputeThinV).solve(b);
  A.row(0) = a.topRows(3).transpose();
  A.row(1) = a.bottomRows(3).transpose();
  A(2,0) = 0.;
  A(2,1) = 0.;
  A(2,2) = 1.;
}

void estimateHomography(const vector<Vector2d>& pts1,
  const vector<Vector2d>& pts2,const vector<IntPair>& matches,
  const vector<int>& matchesToUse,Matrix3d *pH)
{
  auto& H = *pH;
  int nUseful = static_cast<int>(matchesToUse.size());

  Matrix3d C1,C2;
  matchedPointsCenteringMatrix<true>(pts1,matches,matchesToUse,&C1);
  matchedPointsCenteringMatrix<false>(pts2,matches,matchesToUse,&C2);

  MatrixXd A(MatrixXd::Zero(2*nUseful,9));
  for(int i = 0; i < nUseful; i++)
  {
    Vector3d pt1 = C1 * pts1[matches[matchesToUse[i]].first].homogeneous();
    Vector3d pt2 = C2 * pts2[matches[matchesToUse[i]].second].homogeneous();

    A.block(i,0,1,3) = pt1.transpose();
    A.block(i,6,1,3) = -pt2(0) * pt1.transpose();
    A.block(i+nUseful,3,1,3) = pt1.transpose();
    A.block(i+nUseful,6,1,3) = -pt2(1) * pt1.transpose();
  }

  JacobiSVD<MatrixXd> svd(A,Eigen::ComputeThinU | Eigen::ComputeFullV);
  VectorXd h = svd.matrixV().rightCols(1);
  Matrix3d H0;
  H0.row(0) = h.topRows(3).transpose();
  H0.row(1) = h.middleRows(3,3).transpose();
  H0.row(2) = h.bottomRows(3).transpose();

  H = C2.inverse() * H0 * C1;
}

int findHomographyInliers(double thresh,const vector<Vector2d>& pts1,
  const vector<Vector2d>& pts2,const vector<IntPair>& matches,const Matrix3d& H,
  vector<int> *pinliers)
{
  auto& inliers = *pinliers;
  int nMatches = static_cast<int>(matches.size());
  double sqThresh = thresh*thresh;
  inliers.clear();
  inliers.reserve(nMatches);
  for(int iMatch = 0; iMatch < nMatches; iMatch++)
  {
    const auto& pt2 = pts2[matches[iMatch].second];
    Vector3d pt1t = H * pts1[matches[iMatch].first].homogeneous();
    double sqDist = (pt2 - pt1t.hnormalized()).squaredNorm();
    if(sqDist < sqThresh)
    {
      inliers.push_back(iMatch);
    }
  }
  return static_cast<int>(inliers.size());
}

Mediator7ptRANSAC::Mediator7ptRANSAC(const vector<Vector2d>& keys1,
  const vector<Vector2d>& keys2,const vector<IntPair>& matches)
  : minMatches_(7),keys1_(keys1),keys2_(keys2),matches_(matches)
{
}

int Mediator7ptRANSAC::numMatches() const
{ return static_cast<int>(matches_.size()); }

int Mediator7ptRANSAC::minMatches() const
{ return minMatches_; }

void Mediator7ptRANSAC::computeTransformation(const vector<int>& idxs,vector<Matrix3d> *Fs) const
{
  vector<IntPair> selectedMatches;
  selectedMatches.reserve(minMatches_);
  for(int idx : idxs)
    selectedMatches.push_back(matches_[idx]);
  estimateRelativePose7pt(keys1_,keys2_,selectedMatches,Fs);
}

double Mediator7ptRANSAC::computeSquaredError(const Matrix3d& F,int matchIdx) const
{
  IntPair match = matches_[matchIdx];
  return computeSymmetricEpipolarSquaredDistanceFundMat(
    keys2_[match.second],
    F,
    keys1_[match.first]);
}

void Mediator7ptRANSAC::refine(double tolerance,const vector<int>& inliers,
  Matrix3d *F) const
{
  refineFundamentalMatrixNonLinear(keys1_,keys2_,matches_,inliers,tolerance,F);
}

Mediator5ptRANSAC::Mediator5ptRANSAC(const Camera& cam1,const Camera& cam2,
  const vector<IntPair>& matches)
  : minMatches_(5),cam1_(cam1),cam2_(cam2),matches_(matches)
{
  invK1_ = cam1_.K().inverse();
  invK2_ = cam2_.K().inverse();

  pts1Norm_.resize(cam1_.keys().size());
  pts2Norm_.resize(cam2_.keys().size());
  for(IntPair match : matches_)
  {
    pts1Norm_[match.first] = invK1_ * cam1_.key(match.first).homogeneous();
    pts2Norm_[match.second] = invK2_ * cam2_.key(match.second).homogeneous();
  }
}

void Mediator5ptRANSAC::computeTransformation(const vector<int>& idxs,vector<Matrix3d> *pFs) const
{
  auto& Fs = *pFs;
  vector<IntPair> selectedMatches;
  selectedMatches.reserve(minMatches_);
  for(int idx : idxs)
    selectedMatches.push_back(matches_[idx]);
  // Compute Es.
  estimateRelativePose5pt(pts1Norm_,pts2Norm_,selectedMatches,&Fs);
  // Go from Es to Fs.
  for(size_t i = 0; i < Fs.size(); i++)
  {
    Fs[i] = invK2_.transpose() * Fs[i] * invK1_;
  }
}

double Mediator5ptRANSAC::computeSquaredError(const Matrix3d& F,int matchIdx) const
{
  return computeSymmetricEpipolarSquaredDistanceFundMat(
    cam2_.key(matches_[matchIdx].second),
    F,
    cam1_.key(matches_[matchIdx].first));
}

int Mediator5ptRANSAC::numMatches() const
{ return static_cast<int>(matches_.size()); }

int Mediator5ptRANSAC::minMatches() const
{ return minMatches_; }

void Mediator5ptRANSAC::refine(double tolerance,const vector<int>& inliers,
  Matrix3d *F) const
{
  // TODO: Find LM lib for non-linear minimization
  // change F to E before optimization.
}

MediatorHomographyRANSAC::MediatorHomographyRANSAC(const vector<Vector2d>& keys1,
  const vector<Vector2d>& keys2,const vector<IntPair>& matches)
  : minMatches_(4),keys1_(keys1),keys2_(keys2),matches_(matches)
{
}

int MediatorHomographyRANSAC::numMatches() const
{
  return static_cast<int>(matches_.size());
}

int MediatorHomographyRANSAC::minMatches() const
{
  return minMatches_;
}

void MediatorHomographyRANSAC::computeTransformation(const vector<int>& idxs,
  vector<Matrix3d> *Hs) const
{
  Hs->resize(1);
  estimateHomography(keys1_,keys2_,matches_,idxs,&(*Hs)[0]);
}

double MediatorHomographyRANSAC::computeSquaredError(const Matrix3d& H,int matchIdx) const
{
  IntPair match = matches_[matchIdx];
  Vector3d pt = H * keys1_[match.first].homogeneous();
  return (pt.hnormalized() - keys2_[match.second]).squaredNorm();
}

void MediatorHomographyRANSAC::refine(double tolerance,const vector<int>& inliers,
  Matrix3d *H) const
{
}

} // namespace yasfm

namespace
{

double computeSymmetricEpipolarSquaredDistanceFundMat(const Vector2d& pt2,
  const Matrix3d& F,const Vector2d& pt1)
{
  Vector3d Fpt1 = F*pt1.homogeneous();
  Vector3d FTpt2 = F.transpose()*pt2.homogeneous();

  double pt2Fpt1 = pt2.homogeneous().dot(Fpt1);

  return (pt2Fpt1*pt2Fpt1) *
    (1. / Fpt1.topRows(2).squaredNorm() + 1. / FTpt2.topRows(2).squaredNorm());
}

int computeSymmetricEpipolarDistanceFundMatCMINPACK(void *pdata,
  int nPoints,int nParams,const double* params,double* residuals,int iflag)
{
  Matrix3d F;
  closestRank2Matrix(params,F.data());

  const auto& data = *static_cast<FundamentalMatrixRefineData *>(pdata);
  for(int iInlier = 0; iInlier < nPoints; iInlier++)
  {
    IntPair match = (*data.matches)[(*data.matchesToUse)[iInlier]];
    residuals[iInlier] = sqrt(computeSymmetricEpipolarSquaredDistanceFundMat(
      (*data.keys2)[match.second],F,(*data.keys1)[match.first]));
  }
  return 0; // Negative value would terminate the optimization.
}

int computeSymmetricEpipolarDistanceEssenMatCMINPACK(void *pdata,
  int nPoints,int nParams,const double* params,double* residuals,int iflag)
{
  const auto& data = *static_cast<EssentialMatrixRefineData *>(pdata);

  Matrix3d E;
  closestEssentialMatrix(params,E.data());
  
  Matrix3d F = (*data.invK2).transpose() * E * (*data.invK1);

  for(int iInlier = 0; iInlier < nPoints; iInlier++)
  {
    IntPair match = (*data.matches)[(*data.matchesToUse)[iInlier]];
    residuals[iInlier] = sqrt(computeSymmetricEpipolarSquaredDistanceFundMat(
      (*data.keys2)[match.second],F,(*data.keys1)[match.first]));
  }
  return 0; // Negative value would terminate the optimization.
}

// dist = (pt2'*F*pt1)^2/((F*pt1)^2_1 + (F*pt1)^2_2 + (FT*pt2)^2_1 + (FT*pt2)^2_2)
// where FT is the transpose of F and _i is the i-th element of a vector
// Units are squared pixels.
double computeSampsonSquaredDistanceFundMat(const Vector2d& pt2,const Matrix3d& F,const Vector2d& pt1)
{
  Vector3d Fpt1 = F*pt1.homogeneous();
  Vector3d FTpt2 = F.transpose()*pt2.homogeneous();

  double pt2Fpt1 = pt2.homogeneous().dot(Fpt1);

  return (pt2Fpt1*pt2Fpt1) /
    (Fpt1.topRows(2).squaredNorm() + FTpt2.topRows(2).squaredNorm());
}

// Check: http://stackoverflow.com/questions/13328676/c-solving-cubic-equations
// and http://mathworld.wolfram.com/CubicFormula.html
void solveThirdOrderPoly(const Vector4d& coeffs,VectorXd *proots)
{
  auto& roots = *proots;
  // first verify the polynomial degree
  if(coeffs(0) == 0)
  {
    // second order / quadratic
    Vector3d coeffsReduced = coeffs.bottomRows(3);
    solveSecondOrderPoly(coeffsReduced,&roots);
  } else
  {
    double a1 = coeffs(1) / coeffs(0);
    double a2 = coeffs(2) / coeffs(0);
    double a3 = coeffs(3) / coeffs(0);

    double Q = (3 * a2 - a1*a1) * (1. / 9);
    double R = (9 * a1*a2 - 27 * a3 - 2 * a1*a1*a1) * (1. / 54);

    double D = Q*Q*Q + R*R;

    double term1 = a1 / 3;
    if(D == 0)
    {
      // All roots are real and two are equal. (Cardano's formula)
      roots.resize(2);
      double S = cbrt(-R);
      double T = S;
      roots(0) = S + T - term1;
      roots(1) = -S - term1;
    } else if(D < 0)
    {
      // All roots are real and unequal.
      roots.resize(3);
      double theta = acos(R / sqrt(-Q*Q*Q));
      double term2 = 2.0*sqrt(-Q);
      roots(0) = term2*cos(theta / 3.0) - term1;
      roots(1) = term2*cos((theta + 2.0*M_PI) / 3.0) - term1;
      roots(2) = term2*cos((theta + 4.0*M_PI) / 3.0) - term1;
    } else
    {
      // One root is real. (Cardano's formula)
      roots.resize(1);
      double S = cbrt(R + sqrt(D));
      double T = cbrt(R - sqrt(D));
      roots(0) = S + T - term1;
    }
  }
}

void solveSecondOrderPoly(const Vector3d& coeffs,VectorXd *proots)
{
  auto& roots = *proots;
  if(coeffs(0) == 0)
  {
    if(coeffs(1) == 0)
    {
      // zero order
      roots = VectorXd();
    } else
    {
      // first order / linear
      roots.resize(1);
      roots(0) = -coeffs(2) / coeffs(1);
    }
  } else
  {
    double discriminant = coeffs(1)*coeffs(1) - 4 * coeffs(0)*coeffs(2);
    if(discriminant == 0)
    {
      roots.resize(1);
      roots(0) = -coeffs(1) / (2 * coeffs(0));
    } else if(discriminant > 0)
    {
      roots.resize(2);
      roots(0) = (-coeffs(1) + sqrt(discriminant)) / (2 * coeffs(0));
      roots(0) = (-coeffs(1) - sqrt(discriminant)) / (2 * coeffs(0));
    } else
    {
      roots = VectorXd();
    }
  }
}

} // namespace