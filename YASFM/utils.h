/*
* Filip Srajer
* filip.srajer (at) fel.cvut.cz
* Center for Machine Perception
* Czech Technical University in Prague
*
* This software is under construction.
* 02/2015
*/

#pragma once

#include <string>
#include <utility>
#include <vector>

#include "ceres/ceres.h"
#include "Eigen/Dense"
#include "cminpack/cminpack.h"

#include "defines.h"
#include "sfm_data.h"

using Eigen::Matrix3d;
using Eigen::Vector3d;
using std::string;
using std::vector;

////////////////////////////////////////////////////
///////////////   Declarations   ///////////////////
////////////////////////////////////////////////////

namespace yasfm
{

// Forward declarations
class Camera;
class Points;
class Dataset;

YASFM_API string joinPaths(const string& p1, const string& p2);
YASFM_API string extractPath(const string& filepath);
YASFM_API string extractFilename(const string& filepath);

inline double deg2Rad(double d);
inline double rad2Deg(double r);

// Sorts in ascending order. order is the output. It is the mapping 
// from the new ordered indices to the original ones. 
template<typename T>
void quicksort(const vector<T>& arr,vector<int> *order);

template<typename T>
void filterVector(const vector<bool>& keep,vector<T> *arr);
template<typename T>
void filterVector(const vector<int>& toKeep,vector<T> *arr);
template<typename T>
void filterOutOutliers(const vector<int>& outliers,vector<T> *arr);

// Create matrix simulating cross product. Often symbolized as [vec]_x.
YASFM_API void crossProdMat(const Vector3d& vec,Matrix3d *mat);

// Find a rank 2 matrix B closest to A using svd.
YASFM_API void closestRank2Matrix(const double* const A,double* B);
// Find a rank 2 matrix B closest to A using svd.
YASFM_API void closestRank2Matrix(const Matrix3d& A,Matrix3d *B);

// Decomposes A so that A = R*Q, where R is upper triangular
// and Q is orthogonal. This decomposition is based on Givens
// rotations and is non-unique. The elements on the 
// diagonal of R are determined up to sign. You can 
// transform the results as R*T*T^-1*Q, where T=diag(-1,-1,1) e.g.
YASFM_API void RQDecomposition(const Matrix3d& A,Matrix3d *R,Matrix3d *Q);

// Convert projection matrix into upper triangular calibration matrix,
// orthogonal rotation matrix with determinant +1 and camera center. 
// Works only for finite cameras.
// P = K*R*[I -C], where I is 3x3 identity matrix.
YASFM_API void P2KRC(const Matrix34d& P,Matrix3d *K,Matrix3d *R,Vector3d *C);

// Extract .first from all pairs.
YASFM_API void unzipPairsVectorFirst(const vector<IntPair>& pairs,vector<int> *firsts);
// Extract .second from all pairs.
YASFM_API void unzipPairsVectorSecond(const vector<IntPair>& pairs,vector<int> *seconds);

// Find cam with maximum number of matches (maxMatches)
// Return all cameras having at least max(minMatchesThresh,(maxMatches*factor))
YASFM_API void chooseWellMatchedCameras(int minMatchesThresh,double factor,
  const vector<vector<IntPair>>& camToSceneMatches,
  uset<int> *wellMatchedCams);

YASFM_API double computeAverageReprojectionError(const ptr_vector<Camera>& cams,
  const Points& points);

template<unsigned int N>
ceres::CostFunction* generateConstraintsCostFunction(const double* const constraints,
  const double* const weights);

// Compute distortion parameters for inverse distortion. Having a point [x y] and
// a distortion radParams. We compute r = sqrt(x*x + y*y) and d = 1 + r*radParams[0] +
// r*r*radParams[1] + ... Then distorted point would be d*[x y].
// The resulting inverse distortion invRadParams is used similarly. Having distorted
// point [x' v'], we compute radius r' = sqrt(x'*x' + y'*y') and d' = 1 +
// r'*invRadParams[0] + r'*r'*invRadParams[1] + ... And the final undistorted point is
// approximately d'*[x' y'].
YASFM_API void approximateInverseRadialDistortion(int nForwardParams,int nInverseParams,
  double maxRadius,const double* const radParams,double* invRadParams);

// Driver for the cminpack function lmdif1.
// Brief description of lmdif1:
// the purpose of lmdif1 is to minimize the sum of the squares of 
// m nonlinear functions in n variables by a modification of the 
// levenberg-marquardt algorithm. this is done by using the more 
// general least-squares solver lmdif. the user must provide a 
// subroutine which calculates the functions. the jacobian is 
// then calculated by a forward-difference approximation. 
// m is the number of residuals
// n is the number of parameters
YASFM_API void nonLinearOptimLMCMINPACK(cminpack_func_mn residualFuncHandle,
  void *data,int m,int n,double tolerance,double *params,double *residuals);

} // namespace yasfm

namespace
{

// Main function for the header in the yasfm namespace.  Sorts in ascending order.
template<typename T>
void quicksort(int left,int right,const vector<T>& arr,vector<int> *order);

template<unsigned int N>
class CameraConstraintsFunctor
{
public:
  CameraConstraintsFunctor(const double* const constraints,const double* const weights);

  template<typename T>
  bool operator()(const T* const params,T* error)const;

private:
  const double* const constraints_;
  const double* const weights_;
};

} // namespace

////////////////////////////////////////////////////
///////////////   Definitions   ////////////////////
////////////////////////////////////////////////////

namespace yasfm
{

inline double deg2Rad(double d)
{
  return ((d)* (M_PI / 180.));
}

inline double rad2Deg(double r)
{
  return ((r)* (180. / M_PI));
}

// order is the output. It is the mapping from the new ordered indices
// to the original ones.
template<typename T>
void quicksort(const vector<T>& arr,vector<int> *order)
{
  int nElem = static_cast<int>(arr.size());
  order->resize(arr.size());
  for(int i = 0; i < nElem; i++)
  {
    (*order)[i] = i;
  }
  ::quicksort(0,nElem - 1,arr,order);
}

template<typename T>
void filterVector(const vector<bool>& keep,vector<T> *parr)
{
  auto& arr = *parr;
  size_t i = 0;
  while(i < keep.size() && keep[i])
    i++;

  size_t tail = i;
  for(; i < keep.size(); i++)
  {
    if(keep[i])
    {
      std::swap(arr[tail],arr[i]);
      tail++;
    }
  }
  arr.erase(arr.begin() + tail,arr.end());
}

template<typename T>
void filterVector(const vector<int>& toKeep,vector<T> *parr)
{
  vector<bool> keep(parr->size(),false);
  for(int i : toKeep)
    keep[i] = true;

  filterVector(keep,parr);
}
template<typename T>
void filterOutOutliers(const vector<int>& outliers,vector<T> *parr)
{
  vector<bool> keep(parr->size(),true);
  for(int i : outliers)
    keep[i] = false;

  filterVector(keep,parr);
}

template<unsigned int N>
ceres::CostFunction* generateConstraintsCostFunction(const double* const constraints,
  const double* const weights)
{
  return new ceres::AutoDiffCostFunction<CameraConstraintsFunctor<N>,N,N>(
    new CameraConstraintsFunctor<N>(constraints,weights));
}

} //namespace yasfm

namespace
{

template<typename T>
void quicksort(int left,int right,const vector<T>& arr,vector<int> *porder)
{
  auto& order = *porder;
  if(left >= right)
    return;
  int mid = left;
  for(int i = left + 1; i <= right; i++)
  {
    if(arr[order[i]] < arr[order[left]])
      std::swap(order[++mid],order[i]);
  }
  std::swap(order[mid],order[left]);
  quicksort(left,mid - 1,arr,&order);
  quicksort(mid + 1,right,arr,&order);
}

template<unsigned int N>
CameraConstraintsFunctor<N>::CameraConstraintsFunctor(const double* const constraints,
  const double* const weights)
  : constraints_(constraints),weights_(weights)
{
}

template<unsigned int N>
template<typename T>
bool CameraConstraintsFunctor<N>::operator()(const T* const params,T* error)const
{
  for(unsigned int i = 0; i < N; i++)
    error[i] = weights_[i] * (constraints_[i] - params[i]);
  return true;
}

} // namespace