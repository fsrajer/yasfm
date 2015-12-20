//----------------------------------------------------------------------------------------
/**
* \file       matching.h
* \brief      Functions for matching features.
*
*  Functions for matching features.
*
*/
//----------------------------------------------------------------------------------------

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
#include "sfm_data.h"

using std::set;
using std::vector;
using namespace yasfm;

namespace yasfm
{

/// Removes pairs which have low number of matches.
/**
\param[in] minNumMatches Minimal number of matches.
\param[in,out] pairs Camera pairs.
*/
YASFM_API void removePoorlyMatchedPairs(int minNumMatches,pair_umap<CameraPair> *pairs);

/// Options for matching features using FLANN.
struct OptionsFLANN
{
  /// Constructor setting defaults.
  YASFM_API OptionsFLANN()
  {
    indexParams = flann::KDTreeIndexParams();
    searchParams = flann::SearchParams();
    ratioThresh = 0.6f;
    onlyUniques = true;
    verbose = true;
  }

  /// \return True if the matches should be filtered by ratio of distances of 
  /// the first over the second nearest neighbors.
  bool filterByRatio() const;

  /// Write to a file to record which parameters were used.
  /// \param[in,out] file Opened output file.
  YASFM_API void write(ostream& file) const;

  /// What method to use for matching. See FLANN for more details.
  flann::IndexParams indexParams;

  /// Nearest neighbor search parameters, eg. num of checks. See FLANN for more details.
  flann::SearchParams searchParams;

  /// Threshold for the ratio d1/d2, where di is distance to the i-th nearest neighbor
  /// Negative values disable this filter and only the the nearest is searched.
  float ratioThresh;

  /// Discards non-unique matches, i.e., those for which two or more 
  /// different features in feats1 matched to the same feature in feats2
  bool onlyUniques;

  /// Verbosity.
  bool verbose;
};

/// Match features.
/**
Finds for all queries matches from img i to img j, where i is from the set queries[j]. 
Based on Options, different algorithms can be used as FLANN allows that. 
See options.

\param[in] opt Options.
\param[in] cams Cameras. Have to have descriptors.
\param[in] queries See function description.
\param[out] pairs Resulting matched camera pairs.
*/
YASFM_API void matchFeatFLANN(const OptionsFLANN& opt,const ptr_vector<Camera>& cams,
  const vector<set<int>>& queries,pair_umap<CameraPair> *pairs);

/// Match features.
/** 
Calls overloaded function. Sets queries so that every pair i,j such that i<j 
gets matched.

\param[in] opt Options.
\param[in] cams Cameras. Have to have descriptors.
\param[out] pairs Resulting matched camera pairs.
*/
YASFM_API void matchFeatFLANN(const OptionsFLANN& opt,const ptr_vector<Camera>& cams,
  pair_umap<CameraPair> *pairs);

/// Match features.
/**
Finds for all queries matches from img i to img j, where i is from the set queries[j].
Based on Options, different algorithms can be used as FLANN allows that.
See options.

\param[in] opt Options.
\param[in] descr Descriptors for all cameras.
\param[in] queries See function description.
\param[out] pairs Resulting matched camera pairs.
*/
void matchFeatFLANN(const OptionsFLANN& opt,const vector<flann::Matrix<float>>& descr,
  const vector<set<int>>& queries,pair_umap<CameraPair> *pairs);

/// Match features.
/**
Finds matches based on already built and ready flann::Index. Optionally filters 
using ratio and/or uniqueness.

\param[in] opt Options.
\param[in] index Built FLANN index, e.g., a kd-tree.
\param[in] queryDescr Query descriptors.
\param[out] pair Resulting matched camera pair.
*/
void matchFeatFLANN(const OptionsFLANN& opt,const flann::Index<flann::L2<float>>& index,
  const flann::Matrix<float>& queryDescr,CameraPair *pair);

/// Find unique matches.
/**
Unique matches are those a feature from the second image is not matched by more than 
one feature from the first image.

\param[in] matches Matches.
\param[in] numFeats2 Total number of features in the second camera.
\param[out] unique Which matches are unique.
*/
void findUniqueMatches(const vector<IntPair>& matches,size_t numFeats2,
  vector<bool> *unique);

} // namespace yasfm

namespace
{

/// Takes care of memory management of flann::Matrix<T>.
template<typename T>
class AutoMemReleaseFlannMatrix
{
public:

  /// Constructor. Allocates data.
  AutoMemReleaseFlannMatrix(int nRows,int nCols)
    : data(new T[nRows*nCols],nRows,nCols)
  {
  }

  /// Destructor. Deallocates data.
  ~AutoMemReleaseFlannMatrix() { delete[] data.ptr(); }

  flann::Matrix<T> data;

private:
  /// Forbidden.
  AutoMemReleaseFlannMatrix(const AutoMemReleaseFlannMatrix& o) {}

  /// Forbidden.
  AutoMemReleaseFlannMatrix& operator=(const AutoMemReleaseFlannMatrix& o) {}
};

} // namespace