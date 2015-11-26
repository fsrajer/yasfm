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

#include <iostream>
#include <vector>

#include "options.h"

using std::vector;

using std::cerr;
using std::cout;

////////////////////////////////////////////////////
///////////////   Declarations   ///////////////////
////////////////////////////////////////////////////

namespace yasfm
{

template<typename MatType>
class MediatorRANSAC
{
public:
  virtual int numMatches() const = 0;
  // Minimum number of matching points to compute the transformation.
  virtual int minMatches() const = 0;
  virtual void computeTransformation(const vector<int>& idxs,vector<MatType> *Ms) const = 0;
  virtual double computeSquaredError(const MatType& M,int matchIdx) const = 0;
  virtual void refine(const vector<int>& inliers,MatType *M) const = 0;

  // Are these indices permitted to be used for the computation?
  virtual bool isPermittedSelection(const vector<int>& idxs) const 
  { return true; }
};

// Returns number of inliers.
template<typename MatType>
int estimateTransformRANSAC(const MediatorRANSAC<MatType>& m,const OptionsRANSAC& opt,
  MatType *M,vector<int> *inliers = nullptr);
// Returns number of inliers.
template<typename MatType>
int estimateTransformPROSAC(const MediatorRANSAC<MatType>& m,const OptionsRANSAC& opt,
  const vector<int>& matchesOrder,MatType *M,vector<int> *inliers = nullptr);

// Returns number of inliers and also returns 
// all inliers if inliers vector is not null
template<typename MatType>
int findInliers(const MediatorRANSAC<MatType>& m,const MatType& M,
  double errSqThresh,vector<int> *inliers = nullptr);

// Generates non-repeating numToGenerate integers in 
// the interval [0,numOverall-1].
void generateRandomIndices(int numToGenerate,int numOverall,vector<int>& idxs);

// Determine sufficient number of rounds that must happen.
int sufficientNumberOfRounds(int nInliers,int nPoints,int sampleSize,
  double confidence);

// PROSAC auxiliary functions to find out from which sample set
// to generate random indices.
double computeInitAvgSamplesDrawnPROSAC(int ransacRounds,int nMatches,int minMatches);
double computeNextAvgSamplesDrawnPROSAC(double avgSamplesDrawn,int nUsedMatches,int minMatches);

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
    cerr << "estimateTransformRANSAC: cannot estimate transformation (too few points)\n";
    M.setZero();
    return -1;
  }

  int ransacRounds = opt.maxRounds;
  double sqThresh = opt.errorThresh * opt.errorThresh;
  int maxInliers = -1;
  vector<int> idxs;
  idxs.resize(minMatches);
  for(int round = 0; round < ransacRounds; round++)
  {
    generateRandomIndices(minMatches,nMatches,idxs);

    if(!m.isPermittedSelection(idxs))
      continue;

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
          sufficientNumberOfRounds(maxInliers,nMatches,minMatches,opt.confidence));
      }
    }
  }

  if(maxInliers >= opt.minInliers)
  {
    vector<int> tentativeInliers;
    findInliers(m,M,sqThresh,&tentativeInliers);
    m.refine(tentativeInliers,&M);

    if(inliers)
      findInliers(m,M,sqThresh,inliers);

    return maxInliers;
  } else
  {
    return 0;
  }
}

template<typename MatType>
int estimateTransformPROSAC(const MediatorRANSAC<MatType>& m,const OptionsRANSAC& opt,
  const vector<int>& matchesOrder,MatType *pM,vector<int> *inliers)
{
  int minMatches = m.minMatches();
  int nMatches = m.numMatches();

  auto& M = *pM;
  if(nMatches < minMatches)
  {
    cerr << "estimateTransformPROSAC: cannot estimate transformation (too few points)\n";
    M.setZero();
    return -1;
  }

  int ransacRounds = opt.maxRounds;
  double sqThresh = opt.errorThresh * opt.errorThresh;
  int maxInliers = -1;
  vector<int> idxs;
  idxs.resize(minMatches);
  int nUsedMatches = minMatches;
  // number of drawn samples containing only data points from [1,nUsedPts]
  int nSamplesDrawn = 1;
  double avgSamplesDrawn = computeInitAvgSamplesDrawnPROSAC(ransacRounds,nMatches,minMatches);

  for(int round = 0; round < ransacRounds; round++)
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
      generateRandomIndices(minMatches,nMatches,idxs);
    } else
    {
      generateRandomIndices(minMatches - 1,nUsedMatches - 1,idxs);
      idxs[minMatches - 1] = nUsedMatches - 1;
      for(size_t i = 0; i < idxs.size(); i++)
      {
        // map the pseudo-indices to the real indices
        idxs[i] = matchesOrder[idxs[i]];
      }
    }

    if(!m.isPermittedSelection(idxs))
      continue;

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
          sufficientNumberOfRounds(maxInliers,nMatches,minMatches,opt.confidence));
      }
    }
  }

  if(maxInliers >= opt.minInliers)
  {
    vector<int> tentativeInliers;
    findInliers(m,M,sqThresh,&tentativeInliers);
    m.refine(tentativeInliers,&M);

    if(inliers)
      findInliers(m,M,sqThresh,inliers);

    return maxInliers;
  } else
  {
    return 0;
  }
}

template<typename MatType>
int findInliers(const MediatorRANSAC<MatType>& m,const MatType& M,
  double errSqThresh,vector<int> *inliers)
{
  int nInliers = 0;
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