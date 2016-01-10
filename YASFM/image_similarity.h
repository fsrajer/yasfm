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

#include "Eigen/Dense"

#include "defines.h"
#include "camera.h"

using Eigen::ArrayXXf;

namespace yasfm
{

/// Compute visual vocabulary (assumes normalized features).
/**
Randomly samples descriptors from every camera to create visual vocabulary.

\param[in] cams Cameras with features with normalized descriptors.
\param[in] sampleSizeFraction In interval [0,1]. Defines the sample size.
\param[out] vocabulary Visual words in columns (words are normalized).
*/
YASFM_API void createVisualVocabulary(const ptr_vector<Camera>& cams,
  double sampleSizeFraction,ArrayXXf *vocabulary);

} // namespace yasfm