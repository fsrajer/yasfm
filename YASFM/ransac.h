//----------------------------------------------------------------------------------------
/**
* \file       ransac.h
* \brief      General framework for RANSAC and RANSAC like algorithms.
*
*  General framework for RANSAC and RANSAC like algorithms. The file also gives
*  MediatorRANSAC which is used as an interface to the data.
*
*/
//----------------------------------------------------------------------------------------

#pragma once

#include <iostream>
#include <ostream>
#include <vector>
#include <memory>

#include "defines.h"
#include "options_types.h"

using std::vector;
using std::make_unique;
using std::ostream;
using std::cerr;
using std::cout;

////////////////////////////////////////////////////
///////////////   Declarations   ///////////////////
////////////////////////////////////////////////////

namespace yasfm
{

/// Options for running RANSAC like algorithms.
/**
Fields:
/// Maximum number of iterations.
int maxRounds;

/// Error threshold.
double errorThresh;

/// Minimum number of inliers.
int minInliers;

/// Find a good hypothesis with this confidence. The values are from range [0,1].
double confidence;

/// The tolerance for refining the result on inliers. Used for example to terminate
// optimization of non-linear function using LM method. 
// (Not all ransac mediators implement refine phase.)
double refineTolerance;
*/
class OptionsRANSAC : public OptionsWrapper
{
public:
  /// Constructor.
  YASFM_API OptionsRANSAC(int maxRounds,double errorThresh,int minInliers)
  {
    opt.emplace("maxRounds",make_unique<OptTypeWithVal<int>>(maxRounds));
    opt.emplace("errorThresh",make_unique<OptTypeWithVal<double>>(errorThresh));
    opt.emplace("minInliers",make_unique<OptTypeWithVal<int>>(minInliers));
    opt.emplace("confidence",make_unique<OptTypeWithVal<double>>(0.95));
    opt.emplace("refineTolerance",make_unique<OptTypeWithVal<double>>(1e-12));
    opt.emplace("maxSampleSelectionSkips",make_unique<OptTypeWithVal<int>>(100000));
  }

  /// Constructor.
  YASFM_API OptionsRANSAC(int maxRounds,double errorThresh,int minInliers,double confidence)
  {
    opt.emplace("maxRounds",make_unique<OptTypeWithVal<int>>(maxRounds));
    opt.emplace("errorThresh",make_unique<OptTypeWithVal<double>>(errorThresh));
    opt.emplace("minInliers",make_unique<OptTypeWithVal<int>>(minInliers));
    opt.emplace("confidence",make_unique<OptTypeWithVal<double>>(confidence));
    opt.emplace("refineTolerance",make_unique<OptTypeWithVal<double>>(1e-12));
    opt.emplace("maxSampleSelectionSkips",make_unique<OptTypeWithVal<int>>(100000));
  }

  // Shortcuts
  YASFM_API int maxRounds() const { return get<int>("maxRounds"); }
  YASFM_API double errorThresh() const { return get<double>("errorThresh"); }
  YASFM_API int minInliers() const { return get<int>("minInliers"); }
  YASFM_API double confidence() const { return get<double>("confidence"); }
  YASFM_API double refineTolerance() const { return get<double>("refineTolerance"); }
  YASFM_API int maxSampleSelectionSkips() const
  { return get<int>("maxSampleSelectionSkips"); }
};

/// Base, interface like, class for access to data used in RANSAC like frameworks.
template<typename MatType>
class MediatorRANSAC
{
public:

  /// \return Total number of matches.
  virtual int numMatches() const = 0;

  /// \return Minimum number of matches to compute the transformation.
  virtual int minMatches() const = 0;

  /// Compute transformation from a minimal sample.
  /**
  \param[in] idxs Indices of matches from which to compute the transformation.
  \param[out] Ms Resulting transformations.
  */
  virtual void computeTransformation(const vector<int>& idxs,vector<MatType> *Ms) const = 0;
  
  /// Compute squared error for one match.
  /**
  \param[in] M A transformation.
  \param[in] matchIdx Index of the match.
  \return Squared error.
  */
  virtual double computeSquaredError(const MatType& M,int matchIdx) const = 0;
  
  /// Refine a transformation using its inliers.
  /**
  \param[in] tolerance Tolerance for optimization termination (can be ignored if it
  is not neccessary).
  \param[in] inliers Inlier matches.
  \param[in,out] M The transformation to be refined on input and refined transformation
  on the output.
  */
  virtual void refine(double tolerance,const vector<int>& inliers,MatType *M) const = 0;

  /// Are these indices permitted to be used for computating the transformation?
  /**
  \param[in] idxs Indices of matches.
  \return True if the matches selection is permitted.
  */
  virtual bool isPermittedSelection(const vector<int>& idxs) const 
  { return true; }
};

/// Compute a transformation robustly using RANSAC.
/**
Run simple RANSAC, i.e.:
for numRounds:
  generate random minimal sample
  compute the transformation
  for every found transformation
    count support
    if better than the best so far
      remember
      update numRounds
refine using inliers

\param[in] m Interface to the data.
\param[in] opt Options.
\param[out] M Resulting best transformation.
\param[out] inliers Inliers to the best found transformation. If set to nullptr, the
parameter is ignored
\return Number of inliers. This is 0 if less than opt.minInliers was found.
*/
template<typename MatType>
int estimateTransformRANSAC(const MediatorRANSAC<MatType>& m,const OptionsRANSAC& opt,
  MatType *M,vector<int> *inliers = nullptr);

/// Compute a transformation robustly using PROSAC.
/**
Run PROSAC (termination criterion is slighly different that in the original paper), 
i.e.:
for numRounds:
  generate random minimal sample (prefer first the matches earlier in the ordering)
  compute the transformation
  for every found transformation
    count support
    if better than the best so far
      remember
      update numRounds
refine using inliers

\param[in] m Interface to the data.
\param[in] opt Options.
\param[in] matchesOrder Ordering of matches based on their score (the better 
scoring ones should be in the beginning).
\param[out] M Resulting best transformation.
\param[out] inliers Inliers to the best found transformation. If set to nullptr, the
parameter is ignored
\return Number of inliers. This is 0 if less than opt.minInliers was found.
*/
template<typename MatType>
int estimateTransformPROSAC(const MediatorRANSAC<MatType>& m,const OptionsRANSAC& opt,
  const vector<int>& matchesOrder,MatType *M,vector<int> *inliers = nullptr);

/// Find or just count inliers to a transformation.
/**
Find or count inliers by computing squared error.

\param[in] m Interface to data.
\param[in] M Transformation.
\param[in] errSqThresh Squared threshold on the error.
\param[out] inliers Inliers. If set to nullptr, then the inliers are only summed.
\return Number of inliers.
*/
template<typename MatType>
int findInliers(const MediatorRANSAC<MatType>& m,const MatType& M,
  double errSqThresh,vector<int> *inliers = nullptr);

/// Generates random indices. 
/**
Generate non-repeating numToGenerate integers in the interval [0,numOverall-1].

\param[in] numToGenerate Number of indices to generate.
\param[in] numOverall Total number of data.
\param[out] idxs Generated indices.
*/
void generateRandomIndices(int numToGenerate,int numOverall,vector<int> *idxs);

/// Determine sufficient number of rounds that must happen.
/**
The equation determines how many rounds should happen to get at least one 
all inliers sample with given confidence.

\param[in] nInliers Number of inliers.
\param[in] nPoints Total number of data.
\param[in] sampleSize Number of data in a minimal sample.
\param[in] confidence Confidence. (Try values like 0.95.)
\return Sufficient number of rounds.
*/
int sufficientNumberOfRounds(int nInliers,int nPoints,int sampleSize,
  double confidence);

/// PROSAC auxiliary to find out from which sample set to generate random indices.
double computeInitAvgSamplesDrawnPROSAC(int ransacRounds,int nMatches,int minMatches);

/// PROSAC auxiliary to find out from which sample set to generate random indices.
double computeNextAvgSamplesDrawnPROSAC(double avgSamplesDrawn,int nUsedMatches,
  int minMatches);

} // namespace yasfm

////////////////////////////////////////////////////
///////////////   Definitions    ///////////////////
////////////////////////////////////////////////////

namespace yasfm
{

template<typename MatType>
int estimateTransformRANSAC(const MediatorRANSAC<MatType>& m,const OptionsRANSAC& opt,
  MatType *pM,vector<int> *inliers)
{
  int minMatches = m.minMatches();
  int nMatches = m.numMatches();

  auto& M = *pM;
  if(nMatches < minMatches)
  {
    YASFM_PRINT_ERROR("Cannot estimate transformation (too few points). "
      << nMatches << " given but " << minMatches << " needed.");
    M.setZero();
    if(inliers)
      inliers->clear();
    return -1;
  }

  int ransacRounds = opt.maxRounds();
  double sqThresh = opt.errorThresh()*opt.errorThresh();
  double confidence = opt.confidence();
  int maxInliers = -1;
  vector<int> idxs;
  idxs.resize(minMatches);
  int nSampleSelectionSkips = 0;
  for(int round = 0; round < (ransacRounds+nSampleSelectionSkips); round++)
  {
    generateRandomIndices(minMatches,nMatches,&idxs);

    if(!m.isPermittedSelection(idxs))
    {
      nSampleSelectionSkips++;
      if(nSampleSelectionSkips < opt.maxSampleSelectionSkips())
        continue;
      else
        break;
    }

    vector<MatType> hypotheses;
    m.computeTransformation(idxs,&hypotheses);

    for(const auto& Mcurr : hypotheses)
    {
      int nInliers = findInliers(m,Mcurr,sqThresh);
      if(maxInliers < nInliers)
      {
        maxInliers = nInliers;
        M = Mcurr;

        ransacRounds = std::min(ransacRounds,
          sufficientNumberOfRounds(maxInliers,nMatches,minMatches,confidence));
      }
    }
  }

  if(maxInliers >= opt.minInliers())
  {
    vector<int> tentativeInliers;
    findInliers(m,M,sqThresh,&tentativeInliers);
    m.refine(opt.refineTolerance(),tentativeInliers,&M);

    if(inliers)
      maxInliers = findInliers(m,M,sqThresh,inliers);
    else
      maxInliers = findInliers(m,M,sqThresh);

    return maxInliers;
  } else
  {
    M.setZero();
    if(inliers)
      inliers->clear();
    return 0;
  }
}

template<typename MatType,bool DoLocalOpt>
int _commonEstimateTransformLOPROSAC(const MediatorRANSAC<MatType>& m,const OptionsRANSAC& opt,
  const vector<int>& matchesOrder,MatType *pM,vector<int> *inliers)
{
  int minMatches = m.minMatches();
  int nMatches = m.numMatches();

  auto& M = *pM;
  if(nMatches < minMatches)
  {
    YASFM_PRINT_ERROR("Cannot estimate transformation (too few points). "
      << nMatches << " given but " << minMatches << " needed.");
    M.setZero();
    if(inliers)
      inliers->clear();
    return -1;
  }

  int ransacRounds = opt.maxRounds();
  double sqThresh = opt.errorThresh() * opt.errorThresh();
  double confidence = opt.confidence();
  int maxInliers = -1;
  vector<int> idxs;
  idxs.resize(minMatches);
  vector<int> tentativeInliers;
  int nUsedMatches = minMatches;
  // number of drawn samples containing only data points from [1,nUsedPts]
  int nSamplesDrawn = 1;
  int nSampleSelectionSkips = 0;
  double avgSamplesDrawn = computeInitAvgSamplesDrawnPROSAC(ransacRounds,nMatches,minMatches);

  for(int round = 0; round < (ransacRounds+nSampleSelectionSkips); round++)
  {
    // === choose a subset from which we will take random points ===
    if(round == nSamplesDrawn && nUsedMatches < nMatches)
    {
      nUsedMatches++;
      double nextAvgSamplesDrawn =
        computeNextAvgSamplesDrawnPROSAC(avgSamplesDrawn,nUsedMatches,minMatches);
      nSamplesDrawn += static_cast<int>(ceil(nextAvgSamplesDrawn - avgSamplesDrawn));
      avgSamplesDrawn = nextAvgSamplesDrawn;
    }

    // === generate random indices ===
    if(round > nSamplesDrawn)
    {
      generateRandomIndices(minMatches,nMatches,&idxs);
    } else
    {
      generateRandomIndices(minMatches - 1,nUsedMatches - 1,&idxs);
      idxs[minMatches - 1] = nUsedMatches - 1;
      for(size_t i = 0; i < idxs.size(); i++)
      {
        // map the pseudo-indices to the real indices
        idxs[i] = matchesOrder[idxs[i]];
      }
    }

    if(!m.isPermittedSelection(idxs))
    {
      nSampleSelectionSkips++;
      if(nSampleSelectionSkips < opt.maxSampleSelectionSkips())
        continue;
      else
        break;
    }

    vector<MatType> hypotheses;
    m.computeTransformation(idxs,&hypotheses);

    for(auto& Mcurr : hypotheses)
    {
      tentativeInliers.clear();
      int nInliers = findInliers(m,Mcurr,sqThresh,&tentativeInliers);
      if(maxInliers < nInliers)
      {
        if(DoLocalOpt)
        {
          m.refine(opt.refineTolerance(),tentativeInliers,&Mcurr);
          nInliers = findInliers(m,Mcurr,sqThresh);
        }

        maxInliers = nInliers;
        M = Mcurr;

        ransacRounds = std::min(ransacRounds,
          sufficientNumberOfRounds(maxInliers,nMatches,minMatches,confidence));
      }
    }
  }

  if(maxInliers >= opt.minInliers())
  {
    tentativeInliers.clear();
    findInliers(m,M,sqThresh,&tentativeInliers);
    m.refine(opt.refineTolerance(),tentativeInliers,&M);

    if(inliers)
      maxInliers = findInliers(m,M,sqThresh,inliers);
    else
      maxInliers = findInliers(m,M,sqThresh);

    return maxInliers;
  } else
  {
    M.setZero();
    if(inliers)
      inliers->clear();
    return 0;
  }
}

template<typename MatType>
int estimateTransformPROSAC(const MediatorRANSAC<MatType>& m,const OptionsRANSAC& opt,
  const vector<int>& matchesOrder,MatType *pM,vector<int> *inliers)
{
  return _commonEstimateTransformLOPROSAC<MatType,false>(m,opt,matchesOrder,pM,inliers);
}

template<typename MatType>
int estimateTransformLOPROSAC(const MediatorRANSAC<MatType>& m,const OptionsRANSAC& opt,
  const vector<int>& matchesOrder,MatType *pM,vector<int> *inliers)
{
  return _commonEstimateTransformLOPROSAC<MatType,true>(m,opt,matchesOrder,pM,inliers);
}

template<typename MatType>
int findInliers(const MediatorRANSAC<MatType>& m,const MatType& M,
  double errSqThresh,vector<int> *inliers)
{
  int nInliers = 0;
  if(inliers)
  {
    inliers->clear();
    inliers->reserve(m.numMatches());
  }
  for(int i = 0; i < m.numMatches(); i++)
  {
    double sqErr = m.computeSquaredError(M,i);

    if(sqErr < errSqThresh)
    {
      if(inliers)
      {
        inliers->push_back(i);
      }
      nInliers++;
    }
  }
  return nInliers;
}

} // namespace yasfm