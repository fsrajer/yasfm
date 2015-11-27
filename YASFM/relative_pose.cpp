/*
* Filip Srajer
* filip.srajer (at) fel.cvut.cz
* Center for Machine Perception
* Czech Technical University in Prague
*
* This software is under construction.
* 05/2015
*/

#include "relative_pose.h"

#include <ctime>
#include <iostream>

#include "5point/5point.h"

#include "points.h"

using Eigen::JacobiSVD;
using Eigen::MatrixXd;
using std::cerr;
using std::cout;

namespace yasfm
{

IntPair chooseInitialCameraPair(int minMatches,int nCams,
  const vector<NViewMatch>& nViewMatches)
{
  vector<bool> isCalibrated(nCams,false);
  return chooseInitialCameraPair(minMatches,isCalibrated,nViewMatches);
}

IntPair chooseInitialCameraPair(int minMatches,
  const vector<bool>& isCalibrated,const vector<NViewMatch>& nViewMatches)
{
  ArrayXXd scores(ArrayXXd::Zero(isCalibrated.size(),isCalibrated.size()));
  double minScore = -1.;
  return chooseInitialCameraPair(minMatches,minScore,isCalibrated,nViewMatches,
    scores);
}

IntPair chooseInitialCameraPair(int minMatches,double minScore,
  const vector<bool>& isCalibrated,const vector<NViewMatch>& nViewMatches,
  const ArrayXXd& scores)
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
  return chooseInitialCameraPair(minMatches,minScore,isCalibrated,numMatches,
    scores);
}

IntPair chooseInitialCameraPair(int minMatches,double minScore,
  const vector<bool>& isCalibrated,const ArrayXXi& numMatches,const ArrayXXd& scores)
{
  int nCams = static_cast<int>(isCalibrated.size());
  uset<int> camsToUse;
  for(int i = 0; i < nCams; i++)
    if(isCalibrated[i])
      camsToUse.insert(i);

  IntPair best = chooseInitialCameraPair(minMatches,minScore,camsToUse,
    numMatches,scores);

  if(best.first == -1 || best.second == -1)
  {
    for(int i = 0; i < nCams; i++)
      if(!isCalibrated[i])
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
  nViewMatchesToTwoViewMatches(data->points().matchesToReconstruct(),initPair,
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

    cout << "Reconstructing " << nViewMatchesIdxs.size() << " points\n";
    reconstructPoints(initPair,cam0,cam1,nViewMatchesIdxs,&data->points());

    removeHighReprojErrorPoints(pointsReprojErrorThresh,data->cams(),&data->points());
    cout << "Removing " << nViewMatchesIdxs.size()-data->points().numPts()
      << " points with high reprojection error\n";
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
  nViewMatchesToTwoViewMatches(data->points().matchesToReconstruct(),initPair,
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

    cout << "Reconstructing " << nViewMatchesIdxs.size() << " points\n";
    reconstructPoints(initPair,cam0,cam1,nViewMatchesIdxs,&data->points());

    removeHighReprojErrorPoints(pointsReprojErrorThresh,data->cams(),&data->points());
    cout << "Removing " << nViewMatchesIdxs.size()-data->points().numPts()
      << " points with high reprojection error\n";
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

void verifyMatchesGeometrically(const OptionsRANSAC& solverOpt,
  const ptr_vector<Camera>& cams,pair_umap<CameraPair> *pairs)
{
  verifyMatchesGeometrically(solverOpt,true,cams,pairs);
}

void verifyMatchesGeometrically(const OptionsRANSAC& solverOpt,
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

    Matrix3d F;
    vector<int> inliers;
    bool success = estimateRelativePose7ptPROSAC(solverOpt,
      cams[camsIdx.first]->keys(),cams[camsIdx.second]->keys(),pair,&F,&inliers);
    if(success)
    {
      filterVector(inliers,&pair.matches);
      filterVector(inliers,&pair.dists);
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

  // The equation pt1'*F*pt2 = 0 rewritten, i.e. one row
  // corresponds to this equation for one pair of points. 
  MatrixXd A;
  A.resize(7,9);
  for(size_t i = 0; i < minPts; i++)
  {
    const auto& pt1 = keys1[matches[i].first];
    const auto& pt2 = keys2[matches[i].second];
    A.row(i) <<
      pt1(0) * pt2(0),
      pt1(1) * pt2(0),
      pt2(0),
      pt1(0) * pt2(1),
      pt1(1) * pt2(1),
      pt2(1),
      pt1(0),
      pt1(1),
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
    Fs[i].row(0) = f0.middleRows(0,3).transpose();
    Fs[i].row(1) = f0.middleRows(3,3).transpose();
    Fs[i].row(2) = f0.middleRows(6,3).transpose();
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

void computeHomographyInliersProportion(const OptionsRANSAC& opt,
  const ptr_vector<Camera>& cams,const pair_umap<CameraPair>& pairs,
  ArrayXXd *pproportion)
{
  auto& proportion = *pproportion;
  proportion.resize(cams.size(),cams.size());
  proportion.fill(1.);
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

void estimateHomographyMinimal(const vector<Vector2d>& pts1,const vector<Vector2d>& pts2,
  const vector<IntPair>& matches,Matrix3d *H)
{
  if(matches.size() < 4)
  {
    cerr << "ERROR: estimateHomographyMinimal: "
      << "Cannot estimate homography. " << matches.size() 
      << " matches given but 4 needed.\n";
    H->setZero();
    return;
  }

  MatrixXd A(MatrixXd::Zero(8,9));
  for(int i = 0; i < 4; i++)
  {
    const auto& pt1 = pts1[matches[i].first];
    const auto& pt2 = pts2[matches[i].second];
    
    A.block(i,0,1,3) = pt1.homogeneous().transpose();
    A.block(i+4,3,1,3) = pt1.homogeneous().transpose();
    A.block(i,6,1,3) = -pt2(0) * pt1.homogeneous().transpose();
    A.block(i+4,6,1,3) = -pt2(1) * pt1.homogeneous().transpose();
  }

  JacobiSVD<MatrixXd> svd(A,Eigen::ComputeThinU | Eigen::ComputeFullV);
  VectorXd h = svd.matrixV().rightCols(1);
  H->row(0) = h.topRows(3).transpose();
  H->row(1) = h.middleRows(3,3).transpose();
  H->row(2) = h.bottomRows(3).transpose();
}

} // namespace yasfm


namespace
{

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

void Mediator7ptRANSAC::refine(const vector<int>& inliers,Matrix3d *F) const
{
  // TODO: Find LM lib for non-linear minimization (cost will be sampson distance)
  // according to H&Z book
}

bool Mediator7ptRANSAC::isPermittedSelection(const vector<int>& idxs) const
{
  // TODO: Should we exclude some cases?
  return true;
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

void Mediator5ptRANSAC::refine(const vector<int>& inliers,Matrix3d *F) const
{
  // TODO: Find LM lib for non-linear minimization
  // change F to E before optimization.
}

bool Mediator5ptRANSAC::isPermittedSelection(const vector<int>& idxs) const
{
  // TODO: Should we exclude some cases?
  return true;
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
  vector<IntPair> selectedMatches;
  selectedMatches.reserve(minMatches_);
  for(int idx : idxs)
    selectedMatches.push_back(matches_[idx]);
  Hs->resize(1);
  estimateHomographyMinimal(keys1_,keys2_,selectedMatches,&(*Hs)[0]);
}

double MediatorHomographyRANSAC::computeSquaredError(const Matrix3d& H,int matchIdx) const
{
  IntPair match = matches_[matchIdx];
  Vector3d pt = H * keys1_[match.first].homogeneous();
  return (pt.hnormalized() - keys2_[match.second]).squaredNorm();
}

void MediatorHomographyRANSAC::refine(const vector<int>& inliers,Matrix3d *H) const
{
}

bool MediatorHomographyRANSAC::isPermittedSelection(const vector<int>& idxs) const
{
  // TODO: Should we exclude some cases?
  return true;
}

double computeSymmetricEpipolarSquaredDistanceFundMat(const Vector2d& pt2,const Matrix3d& F,
  const Vector2d& pt1)
{
  Vector3d Fpt1 = F*pt1.homogeneous();
  Vector3d FTpt2 = F.transpose()*pt2.homogeneous();

  double pt2Fpt1 = pt2.homogeneous().dot(Fpt1);

  return (pt2Fpt1*pt2Fpt1) *
    (1. / Fpt1.topRows(2).squaredNorm() + 1. / FTpt2.topRows(2).squaredNorm());
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