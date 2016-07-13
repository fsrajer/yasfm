#include "matching.h"

#include "Eigen/Dense"

#include <ctime>
#include <iostream>

using std::cerr;
using std::cout;
using Eigen::MatrixXf;

namespace yasfm
{

void removePoorlyMatchedPairs(int minNumMatches,pair_umap<CameraPair> *pairs)
{
  for(auto it = pairs->begin(); it != pairs->end();)
  {
    auto &pair = it->second;
    if(pair.matches.size() < minNumMatches)
    {
      it = pairs->erase(it);
    } else
    {
      ++it;
    }
  }
}

void matchFeatFLANN(const OptionsFLANN& opt,const ptr_vector<Camera>& cams,
	pair_umap<CameraPair> *pairs, MatchingCallbackFunctionPtr callbackFunction, void * callbackObjectPtr)
{
  vector<set<int>> queries;
  int nImages = static_cast<int>(cams.size());
  queries.resize(nImages);
  for(int i = 0; i < nImages; i++)
  {
    for(int j = i + 1; j < nImages; j++)
    {
      queries[j].insert(i);
    }
  }
  matchFeatFLANN(opt, cams, queries, pairs, callbackFunction,callbackObjectPtr);
}

void matchFeatFLANN(const OptionsFLANN& opt,const ptr_vector<Camera>& cams,
	const vector<set<int>>& queries, pair_umap<CameraPair> *ppairs, 
  MatchingCallbackFunctionPtr callbackFunction, void * callbackObjectPtr)
{
  int pairsDone = 0;
  auto& pairs = *ppairs;
  bool verbose = opt.get<bool>("verbose");
  size_t sz = 0;
  for(const auto& entry : queries)
  {
    sz += entry.size();
  }
  pairs.reserve(sz);

  clock_t start,end;
  int numQueries = static_cast<int>(queries.size());
  for(int j = 0; j < numQueries; j++)
  {
    if(queries[j].empty() || cams[j]->keys().empty())
      continue; // no reason to build the trees

    // Make sure that we own these descriptors and they do not get deleted.
    MatrixXf targetDescr = cams[j]->descr();
    // Switch to row major and transpose (=exchange nrows and ncols)
    flann::Matrix<float> targetDescrFlann(
      const_cast<float*>(targetDescr.data()),targetDescr.cols(),targetDescr.rows());

    flann::Index<flann::L2<float>> index(targetDescrFlann,
      opt.get<flann::IndexParams>("indexParams"));
    index.buildIndex();

    for(int i : queries[j])
    {
      const auto& queryDescr = cams[i]->descr();
      flann::Matrix<float> queryDescrFlann(
        const_cast<float*>(queryDescr.data()),queryDescr.cols(),queryDescr.rows());
      if(verbose)
      {
        cout << "matching: " << i << " -> " << j << "\t";
        start = clock();
      }
      IntPair pairIdx(i,j);
      matchFeatFLANN(opt,index,queryDescrFlann,&pairs[pairIdx]);
      pairsDone++;
      int nMatches = static_cast<int>(pairs[pairIdx].matches.size());
      if(callbackFunction != NULL&&callbackObjectPtr != NULL)
      {
        double progress = static_cast<double>(pairsDone) / sz;
        callbackFunction(callbackObjectPtr,pairIdx,nMatches,progress);
      }
      if(verbose)
      {
        end = clock();
        cout << "found " << nMatches << " matches" << "\t";
        cout << "took: " << (double)(end - start) / (double)CLOCKS_PER_SEC << "s\n";
      }
    }
  }
}

void matchFeatFLANN(const OptionsFLANN& opt,const flann::Index<flann::L2<float>>& index,
  const flann::Matrix<float>& queryDescr,CameraPair *pair)
{
  int numQueries = static_cast<int>(queryDescr.rows);
  AutoMemReleaseFlannMatrix<size_t> nearestNeighbors(numQueries,2);
  AutoMemReleaseFlannMatrix<float> dists(numQueries,2);
  auto& outMatches = pair->matches;
  auto& outDists = pair->dists;
  const auto& searchParams = opt.get<flann::SearchParams>("searchParams");
  outMatches.clear();
  outDists.clear();
  outMatches.reserve(numQueries);
  outDists.reserve(numQueries);

  if(opt.filterByRatio())
  {
    float thresh = opt.get<float>("ratioThresh");
    float sqThresh = thresh*thresh; // flann returns squared distances
    index.knnSearch(queryDescr,nearestNeighbors.data,dists.data,2,searchParams);
    for(int i = 0; i < numQueries; i++)
    {
      // Ratio of the distance to the nearest neighbor over the distance 
      // to the second nearest neighbor
      float ratio = (dists.data[i][0] / dists.data[i][1]);
      if(ratio < sqThresh)
      {
        outMatches.emplace_back(i,static_cast<int>(nearestNeighbors.data[i][0]));
        outDists.push_back(sqrt(ratio)); // Do not use actual distance of descriptors
      }
    }
  } else
  {
    index.knnSearch(queryDescr,nearestNeighbors.data,dists.data,1,searchParams);
    for(int i = 0; i < numQueries; i++)
    {
      outMatches.emplace_back(i,static_cast<int>(nearestNeighbors.data[i][0]));
      outDists.push_back(dists.data[i][0]);
    }
  }

  vector<bool> unique; // empty means that the unique option is turned off
  if(opt.get<bool>("onlyUniques"))
  {
    findUniqueMatches(outMatches,index.size(),&unique);
  }

  if(!unique.empty())
  {
    filterVector(unique,&outMatches);
    filterVector(unique,&outDists);
  }
}

void findUniqueMatches(const vector<IntPair>& matches,size_t numFeats2,
  vector<bool> *puniqueMatches)
{
  auto& uniqueMatches = *puniqueMatches;
  // -1 means that a feature from feats2 was not matched to any from feats1
  vector<int> target2match(numFeats2,-1);

  int numMatches = static_cast<int>(matches.size());
  uniqueMatches.resize(numMatches,true);
  for(int matchIdx = 0; matchIdx < numMatches; matchIdx++)
  {
    int prevMatch = target2match[matches[matchIdx].second];
    if(prevMatch == -1)
    {
      target2match[matches[matchIdx].second] = matchIdx;
    } else
    {
      uniqueMatches[matchIdx] = false;
      uniqueMatches[prevMatch] = false;
    }
  }
}

} // namespace yasfm