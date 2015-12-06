//----------------------------------------------------------------------------------------
/**
* \file       defines.h
* \brief      Useful defines and typedefs.
*
*  Useful defines and typedefs.
*
*/
//----------------------------------------------------------------------------------------

#pragma once

#ifdef YASFM_STATIC
  #define YASFM_API
#elif defined YASFM_EXPORT
	#define YASFM_API __declspec(dllexport)
#else
	#define YASFM_API __declspec(dllimport)
#endif

#include <memory>
#include <unordered_map>
#include <unordered_set>
#include <utility>
#include <vector>
#include "Eigen\Dense"

////////////////////////////////////////////////////
///////////////   Declarations   ///////////////////
////////////////////////////////////////////////////

namespace yasfm
{

typedef Eigen::Matrix<unsigned char,3,1> Vector3uc;
typedef Eigen::Matrix<double,3,4> Matrix34d;
typedef std::pair<int,int> IntPair;

template<typename T>
using ptr_vector = std::vector < std::unique_ptr< T > > ;

/// N-View match are multiple pairs of a camera and key indices.
/**
An n-view match is a map with keys as camera indices and values respective 
feature indices, i.e., the entries correspond to how is a point seen in 
different images.
*/
typedef std::unordered_map<int,int> NViewMatch;

/// N-View match split into part which is seen by cameras that were reconstructed
/// and those that weren't.
typedef struct SplitNViewMatch
{
  NViewMatch observedPart;
  NViewMatch unobservedPart;
} SplitNViewMatch;

template<typename T>
using uset = std::unordered_set< T >;

template<typename K,typename T>
using umap = std::unordered_map < K,T >;

/// Hashing functor for std::pair.
template <class A,class B>
class PairHash
{
public:
  std::size_t operator()(const std::pair<A,B>& v) const;
};

/// Used for hashing lists of more standard types.
template <class T>
void hash_combine(std::size_t & seed,const T & v);

typedef PairHash<int,int> IntPairHash;

template<typename T>
using pair_umap = std::unordered_map < IntPair,T,IntPairHash >;

} // namespace yasfm

////////////////////////////////////////////////////
///////////////   Definitions   ////////////////////
////////////////////////////////////////////////////

namespace yasfm
{

template <class A,class B>
std::size_t PairHash<A,B>::operator()(const std::pair<A,B>& v) const
{
  std::size_t seed = 0;
  hash_combine(seed,v.first);
  hash_combine(seed,v.second);
  return seed;
}

template <class T>
void hash_combine(std::size_t & seed,const T & v)
{
  std::hash<T> hasher;
  seed ^= hasher(v) + 0x9e3779b9 + (seed << 6) + (seed >> 2);
}

} // namespace yasfm