//----------------------------------------------------------------------------------------
/**
* \file       utils.h
* \brief      Various utility functions.
*
*  Various utility functions.
*
*/
//----------------------------------------------------------------------------------------

#pragma once

#include <string>
#include <utility>
#include <vector>

#include "ceres/ceres.h"
#include "Eigen/Dense"
#include "cminpack/cminpack.h"

#include "defines.h"
#include "camera.h"

using Eigen::Matrix3d;
using Eigen::Matrix;
using Eigen::Vector3d;
using Eigen::VectorXd;
using std::string;
using std::vector;

////////////////////////////////////////////////////
///////////////   Declarations   ///////////////////
////////////////////////////////////////////////////

namespace yasfm
{

/// Correctly join to paths.
/**
Handles checking for empty paths and checking of slashes and backslashes.

\param[in] p1 First path.
\param[in] p2 Second path.
\return p1/p2 or p1p2. Depends which is suitable.
*/
YASFM_API string joinPaths(const string& p1, const string& p2);

/// Extract path to dir from path to a file.
/**
Finds last slash or backslash and returns path before it.

\param[in] filepath Path to a file.
\return Path to directory.
*/
YASFM_API string extractPath(const string& filepath);

/// Extract filename from path to file.
/**
Finds last slash or backslash and returns filename after it.

\param[in] filepath Path to a file.
\return Filename.
*/
YASFM_API string extractFilename(const string& filepath);

/// Convert degrees to radians.
/// \param[in] d Angle in degrees.
/// \return Angle in radians.
inline double deg2Rad(double d);

/// Convert radians to degrees.
/// \param[in] r Angle in radians.
/// \return Angle in degrees. 
inline double rad2Deg(double r);

/// Finds ascending ordering.
/**
\param[in] arr Array of elements.
\param[out] order Mapping from the new ordered indices to the original ones. 
*/
template<typename T>
void quicksort(int n,const T* const arr,int *order);

/// Finds ascending ordering.
/**
\param[in] arr Array of elements.
\param[out] order Mapping from the new ordered indices to the original ones.
*/
template<typename T>
void quicksort(const vector<T>& arr,vector<int> *order);

/// Effectively remove multiple elements from a vector.
/**
\param[in] keep Which elements to keep.
\param[in,out] arr Array of elements.
*/
template<typename T>
void filterVector(const vector<bool>& keep,vector<T> *arr);

/// Effectively remove multiple elements from a vector.
/**
\param[in] toKeep Which elements to keep.
\param[in,out] arr Array of elements.
*/
template<typename T>
void filterVector(const vector<int>& toKeep,vector<T> *arr);

/// Effectively remove multiple elements from a vector.
/**
\param[in] outliers Which elements to remove.
\param[in,out] arr Array of elements.
*/
template<typename T>
void filterOutOutliers(const vector<int>& outliers,vector<T> *arr);

/// Create matrix simulating cross product. Often symbolized as [vec]_x.
/**
\param[in] vec Vector on the left side of the cross product.
\param[out] mat Matrix simulating cross product.
*/
YASFM_API void crossProdMat(const Vector3d& vec,Matrix3d *mat);

/// Find a rank 2 matrix B closest to A using svd.
/**
\param[in] A 3x3 matrix.
\param[out] B 3x3 rank 2 matrix.
*/
YASFM_API void closestRank2Matrix(const double* const A,double* B);

/// Find a rank 2 matrix B closest to A using svd.
/**
\param[in] A 3x3 matrix.
\param[out] B 3x3 rank 2 matrix.
*/
YASFM_API void closestRank2Matrix(const Matrix3d& A,Matrix3d *B);

/// Find closest essential matrix using svd decomposition.
/**
\param[in] A 3x3 matrix.
\param[out] E Essential matrix (rank 2 + 1st and 2nd singular values the same).
*/
YASFM_API void closestEssentialMatrix(const double* const A,double* E);

/// Find closest essential matrix using svd decomposition.
/**
\param[in] A 3x3 matrix.
\param[out] E Essential matrix (rank 2 + 1st and 2nd singular values the same).
*/
YASFM_API void closestEssentialMatrix(const Matrix3d& A,Matrix3d *E);

/// RQ decomposition
/**
Decomposes A so that A = R*Q, where R is upper triangular and Q is orthogonal. 
This decomposition is based on Givens rotations and is non-unique. The elements on the 
diagonal of R are determined up to sign. You can transform the results 
as R*T*T^-1*Q, where T=diag(-1,-1,1) e.g.

\param[in] A Matrix to be decomposed.
\param[out] R Upper triangular matrix.
\param[out] Q Orthogonal matrix.
*/
YASFM_API void RQDecomposition(const Matrix3d& A,Matrix3d *R,Matrix3d *Q);

/// Decompose projection matrix on calibration and pose.
/**
Convert projection matrix into upper triangular calibration matrix, orthogonal 
rotation matrix with determinant +1 and camera center. Works only for finite cameras.
P = K*R*[I -C], where I is 3x3 identity matrix.

\param[in] P Projection matrix.
\param[out] K Calibration matrix.
\param[out] R Rotation matrix.
\param[out] C Camera center.
*/
YASFM_API void P2KRC(const Matrix34d& P,Matrix3d *K,Matrix3d *R,Vector3d *C);

/// Extract .first from all pairs.
/**
\param[in] pairs Pairs to be decomposed.
\param[out] firsts All .first from pairs.
*/
YASFM_API void unzipPairsVectorFirst(const vector<IntPair>& pairs,vector<int> *firsts);

/// Extract .second from all pairs.
/**
\param[in] pairs Pairs to be decomposed.
\param[out] seconds All .second from pairs.
*/
YASFM_API void unzipPairsVectorSecond(const vector<IntPair>& pairs,vector<int> *seconds);

/// Generate ceres cost function for constraining parameters.
/**
residual[i] = weights[i]*(params[i]-constraints[i])

\param[in] constraints Constraints.
\param[in] weights Constraints weights.
\return Ceres cost function. Deletion is assumed to be done by ceres.
*/
template<unsigned int N>
ceres::CostFunction* generateConstraintsCostFunction(const double* const constraints,
  const double* const weights);

/// Compute distortion parameters for inverse distortion. 
/**
Having a point [x y] and a distortion radParams. We compute r = sqrt(x*x + y*y) 
and d = 1 + r*radParams[0] + r*r*radParams[1] + ... Then distorted point would be 
d*[x y]. The resulting inverse distortion invRadParams is used similarly. Having a 
distorted point [x' v'], we compute radius r' = sqrt(x'*x' + y'*y') and d' = 1 +
r'*invRadParams[0] + r'*r'*invRadParams[1] + ... And the final undistorted point is
approximately d'*[x' y'].

\param[in] nForwardParams Number of parameters of the radial distortion in forward 
direction.
\param[in] nInverseParams Number of parameters of the radial distortion in inverse 
direction.
\param[in] maxRadius Maximum used radius (0.5*[imgWidth imgHeight]).
\param[in] radParams Parameters of the radial distortion in forward direction.
\param[out] invRadParams Parameters of the radial distortion in inverse direction.
*/
YASFM_API void approximateInverseRadialDistortion(int nForwardParams,int nInverseParams,
  double maxRadius,const double* const radParams,double* invRadParams);

/// Driver for the cminpack function lmdif1.
/**
Brief description of lmdif1:
the purpose of lmdif1 is to minimize the sum of the squares of m nonlinear functions 
in n variables by a modification of the levenberg-marquardt algorithm. this is done 
by using the more general least-squares solver lmdif. the user must provide a subroutine 
which calculates the functions. the jacobian is then calculated by a forward-difference 
approximation. 

\param[in] residualFuncHandle Function handle for computing residuals.
\param[in] data Pointer to data that will be passed to residualFuncHandle.
\param[in] m The number of residuals.
\param[in] n The number of parameters.
\param[in] tolerance Termination tolerance.
\param[in,out] params Parameters.
\param[out] residuals Residuals.
*/
YASFM_API void nonLinearOptimLMCMINPACK(cminpack_func_mn residualFuncHandle,
  void *data,int m,int n,double tolerance,double *params,double *residuals);

} // namespace yasfm

namespace
{

/// Main function for the header in the yasfm namespace. Sorts in ascending order.
template<typename T>
void quicksort(int left,int right,const T* const arr,int* order);

/// Functor for computing error using ceres cost function.
template<unsigned int N>
class CameraConstraintsFunctor
{
public:
  /// Constructor.
  /// \param[in] constraints Constraints.
  /// \param[in] weights Constraints weights.
  CameraConstraintsFunctor(const double* const constraints,const double* const weights);

  /// Compute residuals.
  /// \param[in] params Parameters.
  /// \param[out] error Residuals.
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
void quicksort(int nElem,const T* const arr,int *order)
{
  for(int i = 0; i < nElem; i++)
  {
    order[i] = i;
  }
  ::quicksort(0,nElem - 1,arr,order);
}

// order is the output. It is the mapping from the new ordered indices
// to the original ones.
template<typename T>
void quicksort(const vector<T>& arr,vector<int> *order)
{
  int n = static_cast<int>(arr.size());
  order->resize(n);
  ::quicksort(n,arr.data(),order->data());
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
void quicksort(int left,int right,const T* const arr,int* order)
{
  if(left >= right)
    return;
  int mid = left;
  for(int i = left + 1; i <= right; i++)
  {
    if(arr[order[i]] < arr[order[left]])
      std::swap(order[++mid],order[i]);
  }
  std::swap(order[mid],order[left]);
  quicksort(left,mid - 1,arr,order);
  quicksort(mid + 1,right,arr,order);
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