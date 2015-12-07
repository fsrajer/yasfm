/*
* Filip Srajer
* filip.srajer (at) fel.cvut.cz
* Center for Machine Perception
* Czech Technical University in Prague
*
* This software is under construction.
* 02/2015
*/

#include "utils.h"

#include <iostream>

using Eigen::JacobiSVD;
using Eigen::Map;
using Eigen::MatrixXd;
using Eigen::Vector2d;
using Eigen::VectorXd;
using std::cerr;
using std::cout;

namespace yasfm
{

string joinPaths(const string& p1,const string& p2)
{
  string out(p1);
  if(!p1.empty() && !p2.empty() &&
    p1[p1.length() - 1] != '\\' && p1[p1.length() - 1] != '/')
  {
    out += '/';
  }
  out += p2;
  return out;
}

string extractPath(const string& filepath)
{
  size_t idx = filepath.find_last_of("/\\");
  if(idx == string::npos)
    return "";
  else
    return filepath.substr(0,idx);
}

string extractFilename(const string& filepath)
{
  size_t idx = filepath.find_last_of("/\\");
  if(idx == string::npos)
    return filepath;
  else
    return filepath.substr(idx+1);
}

void crossProdMat(const Vector3d& vec,Matrix3d *mat)
{
  *mat <<  0,-vec(2),vec(1),
    vec(2),0,-vec(0),
    -vec(1),vec(0),0;
}

void closestRank2Matrix(const double* const pA,double* pB)
{
  Map<const Matrix3d> A(pA);
  Map<Matrix3d> B(pB);

  JacobiSVD<Matrix3d> svd(A,Eigen::ComputeFullU | Eigen::ComputeFullV);

  Matrix3d S(Matrix3d::Zero());
  S(0,0) = svd.singularValues()(0);
  S(1,1) = svd.singularValues()(1);

  B.noalias() = svd.matrixU() * S * svd.matrixV().transpose();
}

void closestRank2Matrix(const Matrix3d& A,Matrix3d *B)
{
  closestRank2Matrix(A.data(),B->data());
}

void RQDecomposition(const Matrix3d& A,Matrix3d *pR,Matrix3d *pQ)
{
  auto& R = *pR;
  auto& Q = *pQ;

  Matrix3d Qx,Qy,Qz,T;

  double term = sqrt(A(2,2)*A(2,2) + A(2,1)*A(2,1));
  double s = A(2,1) / term;
  double c = -A(2,2) / term;
  Qx << 1,0,0,
    0,c,-s,
    0,s,c;
  T.noalias() = A*Qx;

  term = sqrt(T(2,2)*T(2,2) + T(2,0)*T(2,0));
  s = T(2,0) / term;
  c = T(2,2) / term;
  Qy << c,0,s,
    0,1,0,
    -s,0,c;
  T = T*Qy;

  term = sqrt(T(1,0)*T(1,0) + T(1,1)*T(1,1));
  s = T(1,0) / term;
  c = -T(1,1) / term;
  Qz << c,-s,0,
    s,c,0,
    0,0,1;
  R = T*Qz;

  Q.noalias() = Qz.transpose()*Qy.transpose()*Qx.transpose();
  R(1,0) = R(2,0) = R(2,1) = 0; // replace potential imprecisions
}

void P2KRC(const Matrix34d& P,Matrix3d *pK,Matrix3d *pR,Vector3d *pC)
{
  auto& K = *pK;
  auto& R = *pR;
  RQDecomposition(P.leftCols(3),&K,&R);

  Matrix3d T = Matrix3d::Identity();
  if(K(0,0) < 0)
    T(0,0) = -1;
  if(K(1,1) < 0)
    T(1,1) = -1;
  if(K(2,2) < 0)
    T(2,2) = -1;

  K = K*T;
  R = T.inverse()*R;
  pC->noalias() = -R.transpose()*K.inverse()*P.rightCols(1);

  K /= K(2,2);
  R *= R.determinant();
}

void unzipPairsVectorFirst(const vector<IntPair>& pairs,vector<int> *firsts)
{
  firsts->reserve(pairs.size());
  for(const auto& p : pairs)
  {
    firsts->push_back(p.first);
  }
}

void unzipPairsVectorSecond(const vector<IntPair>& pairs,vector<int> *seconds)
{
  seconds->reserve(pairs.size());
  for(const auto& p : pairs)
  {
    seconds->push_back(p.second);
  }
}

void approximateInverseRadialDistortion(int nForwardParams,int nInverseParams,
  double maxRadius,const double* const radParams,double* invRadParams)
{
  const int cNumSamples = 20;

  MatrixXd A(cNumSamples,nInverseParams);
  VectorXd b(cNumSamples);

  double radiusIncrement = maxRadius/(cNumSamples-1);
  for(int iSample = 0; iSample < cNumSamples; iSample++)
  {
    double radius = iSample*radiusIncrement;

    double radiusPower = radius;
    double distortion = 1.;
    for(int i = 0; i < nForwardParams; i++)
    {
      distortion += radiusPower*radParams[i];
      radiusPower *= radius;
    }

    double radiusDistorted = radius * distortion;
    double coeff = radiusDistorted * distortion;
    for(int i = 0; i < nInverseParams; i++)
    {
      A(iSample,i) = coeff;
      coeff *= radiusDistorted;
    }
    b(iSample) = 1 - distortion;
  }
  Map<VectorXd> invRadParamsMap(invRadParams,nInverseParams);
  invRadParamsMap = A.jacobiSvd(Eigen::ComputeThinU | Eigen::ComputeThinV).solve(b);
}

void nonLinearOptimLMCMINPACK(cminpack_func_mn residualFuncHandle,
  void *data,int m,int n,double tolerance,double *params,double *residuals)
{
  int doubleWorkArraySize = m*n+5*n+m;
  vector<double> doubleWorkArray(doubleWorkArraySize);
  vector<int> integerWorkArray(n);

  int info = lmdif1(residualFuncHandle,data,m,n,params,residuals,tolerance,
    &integerWorkArray[0],&doubleWorkArray[0],doubleWorkArraySize);
}

} //namespace yasfm