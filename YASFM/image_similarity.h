//----------------------------------------------------------------------------------------
/**
* \file       image_similarity.h
* \brief      Functions relevant to computing image similarity.
*
*  Functions relevant to computing image similarity, visual vocabulary, etc.
*
*/
//----------------------------------------------------------------------------------------

#pragma once

#include <vector>
#include <set>

#include "Eigen/Dense"

#include "defines.h"
#include "camera.h"

using std::vector;
using std::set;
using Eigen::MatrixXf;
using Eigen::VectorXf;

namespace yasfm
{

/// Visual vocabulary.
struct VisualVocabulary
{
  MatrixXf words;  ///< Individual words in columns.
  VectorXf idf;    ///< Inverse document frequency.
};

/// Find similar cameras for every camera.
/**
\param[in] cams Cameras with descriptors.
\param[in] maxVocabularySize Maximum size of vocabulary.
\param[in] nSimilar Number of similar cameras for every camera.
\param[in] verbose Print status?
\param[out] queries For direct pluggin to matching functions. queries[i] are all similar 
to i-th camera and are all smaller than i (their index is smaller).
*/
YASFM_API void findSimilarCameraPairs(const ptr_vector<Camera>& cams,
  int maxVocabularySize,int nSimilar,bool verbose,
  vector<set<int>> *queries);

/// Compute image level similarity (assumes normalized features).
/**
Randomly sample descriptors to create vocabulary. Compute tf-idf for
every image and return cosine similarity between them (i.e. normalize and
do scalar product).

\param[in] cams Cameras with normalized features.
\param[in] maxVocabularySize Maximum size of vocabulary.
\param[in] verbose Print status?
\param[out] similarity Symmetric matrix of image level similarity.
\param[out] voc Used vocabulary.
*/
YASFM_API void computeImagesSimilarity(const ptr_vector<Camera>& cams,
  int maxVocabularySize,bool verbose,MatrixXf *similarity,
  VisualVocabulary *voc);

/// Samples visual words.
/**
Randomly samples descriptors from every camera to create visual vocabulary.

\param[in] cams Cameras with features with normalized descriptors.
\param[in] maxVocabularySize Maximum size of vocabulary.
\param[out] visualWords Visual words in columns.
*/
YASFM_API void randomlySampleVisualWords(const ptr_vector<Camera>& cams,
  int maxVocabularySize,MatrixXf *visualWords);

/// Determine the closest visual words to every descriptor in every camera.
/**
\param[in] cams Cameras with features.
\param[in] visualWords Vocabulary.
\param[in] verbose Print status?
\param[out] closestVisualWord Closest visual word for every camera and every
feature in that camera.
*/
YASFM_API void findClosestVisualWords(const ptr_vector<Camera>& cams,
  const MatrixXf& visualWords,bool verbose,vector<vector<int>> *closestVisualWord);

/// Compute tf-idf (term frequency - inverse document frequency) vectors for every camera.
/**
\param[in] nVisualWords Number of visual words.
\param[in] closestVisualWord Closest visual word for every camera and every
feature in that camera.
\param[out] idf Inverse document frequency.
\param[out] tfidf tf-idf for every camera.
*/
YASFM_API void computeTFIDF(size_t nVisualWords,const vector<vector<int>>& closestVisualWord,
  VectorXf *idf,MatrixXf *tfidf);

} // namespace yasfm

namespace
{

/// Compute dot product using SIMD instructions.
/**
WARNING: dim has to be divisible by 4 and x and y have to be 16-byte aligned
which is the default for Eigen vectors and matrices.
\param[in] dim Dimension.
\param[in] x A vector.
\param[in] y A vector.
\return Dot product.
*/
float computeDotSIMD(size_t dim,const float* const x,
  const float* const y);

} // namespace