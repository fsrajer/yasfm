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

/// Samples visual words.
/**
Randomly samples descriptors from every camera to create visual vocabulary.

\param[in] cams Cameras with features with normalized descriptors.
\param[in] sampleSizeFraction In interval [0,1]. Defines the sample size.
\param[out] visualWords Visual words in columns.
*/
YASFM_API void randomlySampleVisualWords(const ptr_vector<Camera>& cams,
  double sampleSizeFraction,MatrixXf *visualWords);

} // namespace yasfm