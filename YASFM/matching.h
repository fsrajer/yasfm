/*
* Filip Srajer
* filip.srajer (at) fel.cvut.cz
* Center for Machine Perception
* Czech Technical University in Prague
*
* This software is under construction.
* 10/2015
*/

// NOTE on FLANN: There have been 3 problems.
// 1) One warning was theated as error. Therefore, project using YASFM
// must set project properties->Configuration Properties->C/C++->General->SDL checks->No
// See this link:
// http://stackoverflow.com/questions/20448102/why-does-visual-studio-2013-error-on-c4996.
// 2) One harmless warning might show up. If you want just add its id, which is 4996
// to project properties->Configuration Properties->C/C++->Advanced->Disable Specific Warnings
// 3) The following lines had to be added to serialization.h on lines 92-94
// #ifdef _MSC_VER
//   BASIC_TYPE_SERIALIZER(unsigned __int64);
// #endif
// See this link:
// https://github.com/chambbj/osgeo-superbuild/issues/3

#pragma once

#include <set>
#include <vector>

// VS2013 compiler gave many warnings about:
// 1) conversion size_t <-> int
// 2) no appropriate delete operator to overloaded new
// These are in FLANN by design, therefore, we ignore the warnings.
#pragma warning(push) 
#pragma warning(disable : 4267 4291)
#include "FLANN\flann.hpp"
#pragma warning(pop)

#include "defines.h"
#include "options.h"
#include "sfm_data.h"

using std::set;
using std::vector;
using namespace yasfm;

namespace yasfm
{

// Removes pairs which have low number of matches.
YASFM_API void removePoorlyMatchedPairs(int minNumMatches,pair_umap<CameraPair> *pairs);

// Finds matching features. Finds for matches from img i to img j, where
// i is from the set queries[j]. Based on Options, different algorithms 
// can be used as FLANN allows that. Matches can be directly filtered
// using ratio of distances of the first over the second nearest neighbor.
// Also, matches can be filtered as to contain only unique ones. See Options.
YASFM_API void matchFeatFLANN(const OptionsFLANN& opt,const ptr_vector<Camera>& cams,
  const vector<set<int>>& queries,pair_umap<CameraPair> *pairs);

// Calls overloaded function. Sets matching of every 
// pair i,j such that i<j.
YASFM_API void matchFeatFLANN(const OptionsFLANN& opt,const ptr_vector<Camera>& cams,
  pair_umap<CameraPair> *pairs);

void matchFeatFLANN(const OptionsFLANN& opt,const vector<flann::Matrix<float>>& descr,
  const vector<set<int>>& queries,pair_umap<CameraPair> *pairs);

// Finds matches based on already built and ready flann::Index.
// Optionally filters using ratio and/or uniqueness.
void matchFeatFLANN(const OptionsFLANN& opt,const flann::Index<flann::L2<float>>& index,
  const flann::Matrix<float>& queryDescr,CameraPair *pair);

// Unique matches are those a feature from the second image is not matched by 
// more than one feature from the first image.
void findUniqueMatches(const vector<IntPair>& matches,size_t numFeats2,
  vector<bool> *unique);

} // namespace yasfm

namespace
{

// Takes care of memory management of flann::Matrix<T>.
template<typename T>
class AutoMemReleaseFlannMatrix
{
public:
  AutoMemReleaseFlannMatrix(int nRows,int nCols)
    : data(new T[nRows*nCols],nRows,nCols)
  {
  }
  ~AutoMemReleaseFlannMatrix() { delete[] data.ptr(); }

  flann::Matrix<T> data;
private:
  AutoMemReleaseFlannMatrix(const AutoMemReleaseFlannMatrix& o) {}
  AutoMemReleaseFlannMatrix& operator=(const AutoMemReleaseFlannMatrix& o) {}
};

} // namespace