#include "relative_pose.h"

#include <ctime>
#include <iostream>
#include <list>

#include "5point/5point.h"
#include "ceres/ceres.h"

#include "points.h"

using Eigen::JacobiSVD;
using Eigen::MatrixXd;
using Eigen::Map;
using Eigen::Matrix;
using Eigen::Vector3cd;
using Eigen::AngleAxisd;
using Eigen::AngleAxis;
using Eigen::RowVectorXd;
using std::cerr;
using std::cout;
using std::list;

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
      pair.groups.resize(1);
      pair.groups[0].size = (int)pair.matches.size();
      pair.groups[0].type = 'F';
      pair.groups[0].T = F;
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
      *cams[camsIdx.first],*cams[camsIdx.second],pair,&inliers,
      &pair.groups);
    if(nInliers > 0)
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

void estimateFundamentalMatrixParallax(const vector<Vector2d>& keys1,
  const vector<Vector2d>& keys2,const Matrix3d& H,const vector<IntPair>& matches,
  const vector<int>& offPlaneMatches,Matrix3d *pF)
{
  auto& F = *pF;

  MatrixXd A(offPlaneMatches.size(),3);
  for(size_t iOffPlane = 0; iOffPlane < offPlaneMatches.size(); iOffPlane++)
  {
    IntPair match = matches[offPlaneMatches[iOffPlane]];
    const auto& pt1 = keys1[match.first];
    const auto& pt2 = keys2[match.second];

    Vector3d pt2Hom = pt2.homogeneous();
    // epipolar line
    A.row(iOffPlane) = pt2Hom.cross(H * pt1.homogeneous()).transpose();
  }

  JacobiSVD<MatrixXd> svd(A,Eigen::ComputeFullV);
  Vector3d epipole2 = svd.matrixV().rightCols(1);

  Matrix3d e2x;
  crossProdMat(epipole2,&e2x);
  F.noalias() = e2x * H;
}

void growHomographies(const OptionsGeometricVerification& opt,
  const Camera& cam1,const Camera& cam2,const vector<IntPair>& allMatches,
  vector<vector<int>> *pgroups,vector<Matrix3d> *pHs)
{
  auto& groups = *pgroups;
  auto& Hs = *pHs;

  int nAllMatches = static_cast<int>(allMatches.size());
  vector<IntPair> remainingMatches = allMatches;
  vector<int> remainingToAll(nAllMatches);
  for(int i = 0; i < nAllMatches; i++)
    remainingToAll[i] = i;

  Matrix3d bestH;
  vector<int> bestInliers,currInliers;
  bestInliers.reserve(nAllMatches);
  currInliers.reserve(nAllMatches);

  for(int iTransform = 0; iTransform < opt.get<int>("maxHs"); iTransform++)
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

    if(bestInliers.size() < opt.get<int>("minInliersPerH"))
      break;

    Hs.push_back(bestH);
    groups.emplace_back();
    for(int remainingMatchesInlier : bestInliers)
      groups.back().push_back(remainingToAll[remainingMatchesInlier]);

    filterOutOutliers(bestInliers,&remainingMatches);
    filterOutOutliers(bestInliers,&remainingToAll);

    if(remainingMatches.size() < opt.get<int>("minInliersPerH"))
      break;
  }
}

Mediator7ptKnownHsRANSAC::Mediator7ptKnownHsRANSAC(const vector<Vector2d>& keys1,
  const vector<Vector2d>& keys2,const vector<IntPair>& matches,int nGroups,
  const vector<int>& groupId)
  : Mediator7ptRANSAC(keys1,keys2,matches),nGroups_(nGroups),groupId_(groupId)
{
}

bool Mediator7ptKnownHsRANSAC::isPermittedSelection(const vector<int>& idxs) const
{
  // counts[0] corresponds to non-planar matches
  vector<int> counts(nGroups_+1,0);
  for(int idx : idxs)
    counts[groupId_[idx]+1]++;
  for(int ig = 0; ig < nGroups_; ig++)
  {
    // Degenerate selection?
    if(counts[ig+1] > 4)
      return false;
  }
  return true;
}

bool estimateRelativePose7ptKnownHsLOPROSAC(const OptionsRANSAC& opt,
  const vector<Vector2d>& keys1,const vector<Vector2d>& keys2,
  const CameraPair& pair,int nGroups,const vector<int>& groupId,
  Matrix3d *F,vector<int> *inliers)
{
  Mediator7ptKnownHsRANSAC m(keys1,keys2,pair.matches,nGroups,groupId);
  vector<int> matchesOrder;
  yasfm::quicksort(pair.dists,&matchesOrder);
  int nInliers = estimateTransformLOPROSAC(m,opt,matchesOrder,F,inliers);
  return (nInliers > 0);
}

template<typename T>
T robustifyRefineFKnownHsCostFunctor(double softThresh,T x)
{
  x /= T(softThresh);
  if(x < T(1))
    return T(-0.25)*(T(3.)*x*x*x - T(6.)*x*x);
  else if(x < T(2))
  {
    T tmp = T(2.) - x;
    return T(-0.25)*(tmp*tmp*tmp - T(4.));
  } else
    return T(1.);
}

struct RefineFKnownHsCostFunctor
{
  RefineFKnownHsCostFunctor(const vector<Vector2d>& keys1,const vector<Vector2d>& keys2,
    const CameraPair& pair,const vector<vector<int>>& groupsH,
    const vector<int>& others,double errThresh,double alpha)
    : keys1(keys1),keys2(keys2),pair(pair),groupsH(groupsH),others(others),
    errThresh(errThresh),alpha(alpha)
  {
  }

  template<typename T>
  bool operator()(T const* const* parameters,T* residuals) const
  {
    Matrix<T,3,3> F;
    composeFFromMinimalParams(parameters[0],&F);

    int iResidual = 0;
    for(size_t iH = 0; iH < groupsH.size(); iH++)
    {
      T avgErr = T(0.);
      for(int iMatch : groupsH[iH])
      {
        IntPair match = pair.matches[iMatch];
        const auto& x1 = keys1[match.first];
        const auto& x2 = keys2[match.second];

        T err = sqrt(computeSampsonSquaredDistanceFundMat(x2,F,x1));
        residuals[iResidual] =
          T(1. - alpha) * robustifyRefineFKnownHsCostFunctor(errThresh,err);
        avgErr += err;
        iResidual++;
      }
      avgErr /= T(double(groupsH[iH].size()));
      T robAvgErr = robustifyRefineFKnownHsCostFunctor(errThresh,avgErr);

      iResidual -= (int)groupsH[iH].size();
      for(int iMatch : groupsH[iH])
      {
        residuals[iResidual] += T(alpha) * robAvgErr;
        residuals[iResidual] *= T(1. - pair.dists[iMatch]);
        iResidual++;
      }
    }

    for(size_t io = 0; io < others.size(); io++)
    {
      IntPair match = pair.matches[others[io]];
      const auto& x1 = keys1[match.first];
      const auto& x2 = keys2[match.second];

      residuals[iResidual] = robustifyRefineFKnownHsCostFunctor(errThresh,
        sqrt(computeSampsonSquaredDistanceFundMat(x2,F,x1)));
      residuals[iResidual] *= T(1. - pair.dists[others[io]]);
      iResidual++;
    }

    return true;
  }

  const vector<Vector2d> &keys1,&keys2;
  const CameraPair& pair;
  const vector<vector<int>> groupsH;
  const vector<int>& others;
  /// This changes robustify function.
  double errThresh;
  /// Weights how do we compute error of a match which is modelled by homography.
  /// err = alpha * robustify(groupAverageError) + (1-alpha) * robustify(matchError)
  double alpha;
};

void refineFKnownHs(const OptionsGeometricVerification& opt,
  const vector<Vector2d>& keys1,const vector<Vector2d>& keys2,const CameraPair& pair,
  vector<vector<int>> *pgroupsH,Matrix3d *pF,vector<int> *pinliersF)
{
  auto& groupsH = *pgroupsH;
  auto& F = *pF;
  auto& inliersF = *pinliersF;
  const auto& matches = pair.matches;

  int nOptIters = opt.get<int>("nOptIterations");
  double errThresh = opt.get<double>("fundMatThresh");
  if(errThresh == 0.)
    errThresh = 1e-12; // We will divide by the errThresh

  vector<bool> isModelledByH(matches.size(),false);
  for(const auto& group : groupsH)
    for(int idx : group)
      isModelledByH[idx] = true;

  vector<int> others;
  for(int iMatch = 0; iMatch < (int)matches.size(); iMatch++)
    if(!isModelledByH[iMatch])
      others.push_back(iMatch);

  /// === Convert F into a minimal parameterization ===  
  VectorXd FParams(7);
  decomposeFToMinimalParams(F,&FParams);

  double alpha = 0.;
  for(int iIter = 0; iIter < nOptIters; iIter++)
  {
    /// === Set-up the problem ===
    ceres::Problem problem;
    ceres::Solver::Options solverOpt;
    ceres::LossFunction *lossFun = NULL;  // NULL specifies squared loss

    auto costFun =
      new ceres::DynamicAutoDiffCostFunction<RefineFKnownHsCostFunctor>(
      new RefineFKnownHsCostFunctor(keys1,keys2,pair,groupsH,others,errThresh*1.5,
      alpha));

    costFun->SetNumResiduals((int)matches.size());
    costFun->AddParameterBlock(7);
    problem.AddResidualBlock(costFun,lossFun,FParams.data());

    ceres::Solver::Summary summary;
    ceres::Solve(solverOpt,&problem,&summary);
#ifdef PRINT_STATUS
    std::cout << summary.FullReport() << "\n";
#endif
    alpha += 1./(nOptIters-1);
  }

  composeFFromMinimalParams(FParams.data(),&F);

  inliersF.clear();
  size_t largestHSz = 0;
  list<size_t> HsToRemove;
  for(size_t iH = 0; iH < groupsH.size(); iH++)
  {
    double avgErr = 0.;
    for(int iMatch : groupsH[iH])
    {
      IntPair match = matches[iMatch];
      const auto& x1 = keys1[match.first];
      const auto& x2 = keys2[match.second];

      avgErr += sqrt(computeSampsonSquaredDistanceFundMat(x2,F,x1));
    }
    avgErr /= double(groupsH[iH].size());
    if(avgErr < errThresh)
    {
      for(int iMatch : groupsH[iH])
        inliersF.push_back(iMatch);
      HsToRemove.push_front(iH);
      if(largestHSz < groupsH[iH].size())
        largestHSz = groupsH[iH].size();
    }
  }

  for(size_t io = 0; io < others.size(); io++)
  {
    IntPair match = matches[others[io]];
    const auto& x1 = keys1[match.first];
    const auto& x2 = keys2[match.second];

    double err = sqrt(computeSampsonSquaredDistanceFundMat(x2,F,x1));

    if(err < errThresh)
      inliersF.push_back(others[io]);
  }

  if(HsToRemove.size() < 2 &&
    (double(largestHSz)/inliersF.size() > opt.get<double>("maxHProportionInF") ||
    (inliersF.size()-largestHSz) < opt.get<int>("minOffHInliersInF")))
  {
    inliersF.clear();
  } else
  {
    for(size_t iH : HsToRemove)
      groupsH.erase(groupsH.begin() + iH);
  }
}

void estimateFundamentalMatrices(const OptionsGeometricVerification& opt,
  const vector<Vector2d>& keys1,const vector<Vector2d>& keys2,const CameraPair& allPair,
  const vector<vector<int>>& groupsH,vector<vector<int>> *pgroupsF,vector<Matrix3d> *pFs)
{
  auto& groupsF = *pgroupsF;
  auto& Fs = *pFs;
  int minInliersPerF = opt.get<int>("minInliersPerF");

  OptionsRANSAC ransacOpt(opt.get<int>("maxRansacRounds"),
    opt.get<double>("fundMatThresh"),minInliersPerF);

  int nAllMatches = static_cast<int>(allPair.matches.size());
  CameraPair remainingPair = allPair;
  vector<vector<int>> remainingGroupsH = groupsH;
  vector<int> remainingToAll(nAllMatches);
  for(int i = 0; i < nAllMatches; i++)
    remainingToAll[i] = i;

  vector<int> remainingGroupHId(nAllMatches,-1);
  for(int ig = 0; ig < (int)groupsH.size(); ig++)
    for(int idx : groupsH[ig])
      remainingGroupHId[idx] = ig;

  vector<int> inliers;
  inliers.reserve(nAllMatches);

  for(int iTransform = 0; iTransform < opt.get<int>("maxFs"); iTransform++)
  {
    inliers.clear();

    Matrix3d F;
    estimateRelativePose7ptKnownHsLOPROSAC(ransacOpt,keys1,keys2,
      remainingPair,(int)remainingGroupsH.size(),remainingGroupHId,&F,&inliers);

    if(inliers.size() < minInliersPerF)
      break;

    refineFKnownHs(opt,keys1,keys2,remainingPair,&remainingGroupsH,&F,&inliers);

    if(inliers.size() < minInliersPerF)
      break;

    // Save inliers as F group
    Fs.push_back(F);
    groupsF.emplace_back();
    for(int remainingMatchesInlier : inliers)
      groupsF.back().push_back(remainingToAll[remainingMatchesInlier]);

    // Update indices in H groups
    vector<bool> isInlier(remainingPair.matches.size(),false);
    for(int idx : inliers)
      isInlier[idx] = true;
    vector<int> remainingToNextRemaining(isInlier.size());
    int idxToNext = 0;
    for(size_t i = 0; i < isInlier.size(); i++)
    {
      remainingToNextRemaining[i] = idxToNext;
      idxToNext += (!isInlier[i]);
    }
    for(auto& group : remainingGroupsH)
      for(auto& idx : group)
        idx = remainingToNextRemaining[idx];

    filterOutOutliers(inliers,&remainingPair.matches);
    filterOutOutliers(inliers,&remainingPair.dists);
    filterOutOutliers(inliers,&remainingToAll);
    filterOutOutliers(inliers,&remainingGroupHId);

    // Update indices to H groups
    for(int ig = 0; ig < (int)remainingGroupsH.size(); ig++)
      for(int idx : remainingGroupsH[ig])
        remainingGroupHId[idx] = ig;

    if(remainingPair.matches.size() < minInliersPerF)
      break;
  }
}

int verifyMatchesGeometrically(const OptionsGeometricVerification& opt,
  const Camera& cam1,const Camera& cam2,const CameraPair& pair,
  vector<int> *poutInliers,vector<MatchGroup> *poutGroups)
{
  auto& outInliers = *poutInliers;
  auto& outGroups = *poutGroups;

  vector<vector<int>> groupsH;
  vector<Matrix3d> Hs;
  growHomographies(opt,cam1,cam2,pair.matches,&groupsH,&Hs);

  vector<vector<int>> groupsEG;
  vector<Matrix3d> Fs;
  estimateFundamentalMatrices(opt,cam1.keys(),cam2.keys(),pair,groupsH,&groupsEG,&Fs);

  // == Remove Hs modelled by EG ==
  vector<bool> isModelledByEG(pair.matches.size(),false);
  for(const auto& g : groupsEG)
    for(int idx : g)
      isModelledByEG[idx] = true;

  vector<size_t> nModelledByEG(Hs.size(),0);
  for(size_t iH = 0; iH < Hs.size(); iH++)
    for(int idx : groupsH[iH])
      nModelledByEG[iH] += isModelledByEG[idx];

  for(int iH = int(groupsH.size())-1; iH >= 0; iH--)
  {
    if(nModelledByEG[iH] == groupsH[iH].size())
    {
      groupsH.erase(groupsH.begin() + iH);
      Hs.erase(Hs.begin() + iH);
    }
  }

  outInliers.clear();
  outGroups.clear();
  for(size_t ig = 0; ig < groupsEG.size(); ig++)
  {
    outInliers.insert(outInliers.end(),groupsEG[ig].begin(),groupsEG[ig].end());
    outGroups.emplace_back();
    outGroups.back().size = (int)groupsEG[ig].size();
    outGroups.back().type = 'F';
    outGroups.back().T = Fs[ig];
  }
  for(size_t ig = 0; ig < groupsH.size(); ig++)
  {
    outInliers.insert(outInliers.end(),groupsH[ig].begin(),groupsH[ig].end());
    outGroups.emplace_back();
    outGroups.back().size = (int)groupsH[ig].size();
    outGroups.back().type = 'H';
    outGroups.back().T = Hs[ig];
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