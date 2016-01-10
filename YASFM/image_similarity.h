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

#include "Eigen/Dense"

#include "defines.h"
#include "camera.h"

using std::vector;
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

/// Compute image level similarity (assumes normalized features).
/**
Randomly sample descriptors to create vocabulary. Compute tf-idf for
every image and return cosine similarity between them (i.e. normalize and
do scalar product).

\param[in] cams Cameras with normalized features.
\param[in] vocabularySampleSizeFraction In interval [0,1]. Defines the 
sample size per image. The given sample of features is used to create the vocabulary.
\param[out] similarity Symmetric matrix of image level similarity.
\param[out] voc Used vocabulary.
*/
YASFM_API void computeImagesSimilarity(const ptr_vector<Camera>& cams,
  double vocabularySampleSizeFraction,MatrixXf *similarity,VisualVocabulary *voc);

/// Samples visual words.
/**
Randomly samples descriptors from every camera to create visual vocabulary.

\param[in] cams Cameras with features with normalized descriptors.
\param[in] sampleSizeFraction In interval [0,1]. Defines the sample size.
\param[out] visualWords Visual words in columns.
*/
YASFM_API void randomlySampleVisualWords(const ptr_vector<Camera>& cams,
  double sampleSizeFraction,MatrixXf *visualWords);

/// Determine the closest visual words to every descriptor in every camera.
/**
\param[in] cams Cameras with features.
\param[in] visualWords Vocabulary.
\param[out] closestVisualWord Closest visual word for every camera and every
feature in that camera.
*/
YASFM_API void findClosestVisualWords(const ptr_vector<Camera>& cams,
  const MatrixXf& visualWords,vector<vector<int>> *closestVisualWord);

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