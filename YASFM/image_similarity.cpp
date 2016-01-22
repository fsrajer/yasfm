#include "image_similarity.h"

#include <random>
#include <iostream>

#include "utils.h"

using Eigen::ArrayXi;
using std::uniform_int_distribution;
using std::cout;
using std::cerr;

namespace yasfm
{

void findSimilarCameraPairs(const ptr_vector<Camera>& cams,
  double vocabularySampleSizeFraction,int nSimilar,bool verbose,
  vector<set<int>> *pqueries)
{
  auto& queries = *pqueries;
  MatrixXf similarity;
  VisualVocabulary voc;
  computeImagesSimilarity(cams,vocabularySampleSizeFraction,verbose,
    &similarity,&voc);

  int nCams = static_cast<int>(cams.size());
  queries.resize(nCams);
  for(int iCurr = 0; iCurr < nCams; iCurr++)
  {
    vector<int> idxs(nCams);
    quicksort(nCams,similarity.col(iCurr).data(),&idxs[0]);

    for(int i = nCams-1; i >= std::max(0,nCams-nSimilar); i--)
    {
      if(iCurr < idxs[i])
      {
        queries[idxs[i]].insert(iCurr);
      } else if(iCurr > idxs[i])
      {
        queries[iCurr].insert(idxs[i]);
      }
    }
  }
}

void computeImagesSimilarity(const ptr_vector<Camera>& cams,
  double vocabularySampleSizeFraction,bool verbose,MatrixXf *psimilarity,
  VisualVocabulary *voc)
{
  auto& similarity = *psimilarity;
  auto& visualWords = voc->words;
  auto& idf = voc->idf;

  if(verbose)
    cout << "Sampling words to create vocabulary ... ";
  randomlySampleVisualWords(cams,vocabularySampleSizeFraction,&visualWords);
  if(verbose)
    cout << visualWords.cols() << " words used.\n";

  if(verbose)
    cout << "Looking for closest visual words for image:\n";
  vector<vector<int>> closestVisualWord;
  findClosestVisualWords(cams,visualWords,verbose,&closestVisualWord);

  MatrixXf tfidf;
  computeTFIDF(visualWords.cols(),closestVisualWord,&idf,&tfidf);

  similarity.noalias() = tfidf.transpose() * tfidf;
  similarity.diagonal().setZero();
}

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

void findClosestVisualWords(const ptr_vector<Camera>& cams,const MatrixXf& visualWords,
  bool verbose,vector<vector<int>> *pclosestVisualWord)
{
  auto& closestVisualWord = *pclosestVisualWord;
  closestVisualWord.resize(cams.size());
  VectorXf cosineSimilarity(visualWords.cols());
  for(size_t iCam = 0; iCam < cams.size(); iCam++)
  {
    if(verbose)
      cout << "  " << iCam << "/" << cams.size() << "\n";
    size_t nKeys = cams[iCam]->keys().size();
    closestVisualWord[iCam].resize(nKeys);
    for(size_t iKey = 0; iKey < nKeys; iKey++)
    {
      cosineSimilarity.noalias() = visualWords.transpose() * cams[iCam]->descr().col(iKey);
      cosineSimilarity.maxCoeff(&closestVisualWord[iCam][iKey]);
    }
  }
}

void computeTFIDF(size_t nVisualWords,const vector<vector<int>>& closestVisualWord,
  VectorXf *pidf,MatrixXf *ptfidf)
{
  auto& idf = *pidf;
  auto& tfidf = *ptfidf;
  int nCams = static_cast<int>(closestVisualWord.size());

  idf.resize(nVisualWords);
  idf.setZero();
  // We will need sparse matrix for bigger vocabularies.
  MatrixXf tf(MatrixXf::Zero(nVisualWords,nCams));
  for(int iCam = 0; iCam < nCams; iCam++)
  {
    for(int iWord : closestVisualWord[iCam])
    {
      if(tf(iWord,iCam) == 0.f)
        idf(iWord) += 1.f;
      tf(iWord,iCam) += 1.f;
    }
  }
  for(int i = 0; i < idf.rows(); i++)
  {
    if(idf(i) != 0.f)
      idf(i) = log(nCams / idf(i));
  }

  tfidf.noalias() = (tf.array().colwise() * idf.array()).matrix();
  tfidf.colwise().normalize();
}

} // namespace yasfm