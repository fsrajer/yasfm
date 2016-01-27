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
#include "options_types.h"

using std::set;
using std::vector;
using namespace yasfm;

namespace yasfm
{

/// Callback function for progress notifying
/**
\param[in,out] object void pointer to the callee object
\param[in] camIdxs indices of the cameras
\param[in] nMatches Number of found matches for the pair.
\param[in] progress indicator 0-1
*/
	typedef void(*MatchingCallbackFunctionPtr)(void *object, IntPair camIdxs, int nMatches,
  double progress);

/// Removes pairs which have low number of matches.
/**
\param[in] minNumMatches Minimal number of matches.
\param[in,out] pairs Camera pairs.
*/
YASFM_API void removePoorlyMatchedPairs(int minNumMatches,pair_umap<CameraPair> *pairs);

/// Options for matching features using FLANN.
/**
Fields:
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
*/
class OptionsFLANN : public OptionsWrapper
{
public:
  /// Constructor setting defaults.
  YASFM_API OptionsFLANN()
  {
    opt.emplace("indexParams",
      make_unique<OptTypeWithVal<flann::IndexParams>>(flann::KDTreeIndexParams()));
    opt.emplace("searchParams",
      make_unique<OptTypeWithVal<flann::SearchParams>>(flann::SearchParams()));
    opt.emplace("ratioThresh",make_unique<OptTypeWithVal<float>>(0.6f));
    opt.emplace("onlyUniques",make_unique<OptTypeWithVal<bool>>(true));
    opt.emplace("verbose",make_unique<OptTypeWithVal<bool>>(true));
  }

  /// \return True if the matches should be filtered by ratio of distances of 
  /// the first over the second nearest neighbors.
  bool filterByRatio() const
  {
    return (get<float>("ratioThresh") >= 0.f);
  }
};

/// Match features.
/** 
Calls overloaded function. Sets queries so that every pair i,j such that i<j 
gets matched.

\param[in] opt Options.
\param[in] cams Cameras. Have to have descriptors.
\param[out] pairs Resulting matched camera pairs.
\param[out] callbackFunction Optional. Function to be called after finishing 
matching of one pair.
\param[out] callbackObjectPtr Optional. Object to be passed to callbackFunction.
*/
YASFM_API void matchFeatFLANN(const OptionsFLANN& opt,const ptr_vector<Camera>& cams,
	pair_umap<CameraPair> *pairs, MatchingCallbackFunctionPtr callbackFunction = NULL, void * callbackObjectPtr = NULL);

/// Match features.
/**
Finds for all queries matches from img i to img j, where i is from the set queries[j].
Based on Options, different algorithms can be used as FLANN allows that.
See options.

\param[in] opt Options.
\param[in] cams Cameras. Have to have descriptors.
\param[in] queries See function description.
\param[out] pairs Resulting matched camera pairs.
\param[out] callbackFunction Optional. Function to be called after finishing 
matching of one pair.
\param[out] callbackObjectPtr Optional. Object to be passed to callbackFunction.
*/
YASFM_API void matchFeatFLANN(const OptionsFLANN& opt,const ptr_vector<Camera>& cams,
	const vector<set<int>>& queries, pair_umap<CameraPair> *pairs, 
	MatchingCallbackFunctionPtr callbackFunction = NULL, void * callbackObjectPtr = NULL);

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