#include "image_similarity.h"

#include <random>

using Eigen::ArrayXi;
using std::uniform_int_distribution;

namespace yasfm
{

void randomlySampleVisualWords(const ptr_vector<Camera>& cams,
  double sampleSizeFraction,MatrixXf *pvisualWords)
{
  auto& visualWords = *pvisualWords;
  int nCams = static_cast<int>(cams.size());
  if(nCams == 0)
    return;

  size_t dim = cams[0]->descr().rows();
  ArrayXi sampleSizes(cams.size());
  for(int i = 0; i < nCams; i++)
    sampleSizes(i) = static_cast<int>(cams[i]->keys().size() * sampleSizeFraction);
  int vocSize = sampleSizes.sum();

  std::default_random_engine generator;

  visualWords.resize(dim,vocSize);
  int idx = 0;
  for(int iCam = 0; iCam < nCams; iCam++)
  {
    std::uniform_int_distribution<size_t> distribution(0,cams[iCam]->keys().size()-1);
    uset<size_t> indices;
    while(indices.size() < sampleSizes(iCam))
      indices.insert(distribution(generator));
    for(size_t iKey : indices)
    {
      visualWords.col(idx) = cams[iCam]->descr().col(iKey);
      idx++;
    }
  }
}

} // namespace yasfm