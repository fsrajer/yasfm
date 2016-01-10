#include "stdafx.h"
#include "CppUnitTest.h"

#include "image_similarity.h"
#include "standard_camera.h"

using namespace Microsoft::VisualStudio::CppUnitTestFramework;

using namespace yasfm;

namespace yasfm_tests
{
	TEST_CLASS(image_similarity_tests)
	{
	public:

    TEST_METHOD(computeImagesSimilarityTest)
    {
      ptr_vector<Camera> cams;
      double vocabularySampleSizeFraction = 0.5;
      int nCams = 3;
      int nFeatures = 50;
      int dim = 50;
      double count = 0.f;
      for(int i = 0; i < nCams; i++)
      {
        cams.emplace_back(new StandardCamera);
        cams[i]->resizeFeatures(nFeatures,dim);
        for(int iFeat = 0; iFeat < nFeatures; iFeat++)
        {
          VectorXf descr = VectorXf::Random(dim);
          cams[i]->setFeature(iFeat,0,0,0,0,descr.data());
          count += i;
        }
      }
      for(int i = 0; i < nFeatures; i++)
      {
        cams[0]->setFeature(i,0,0,0,0,cams[1]->descr().col(i).data());
      }

      MatrixXf similarity;
      VisualVocabulary voc;
      computeImagesSimilarity(cams,vocabularySampleSizeFraction,&similarity,&voc);
      Assert::IsTrue(similarity(0,1) > 0.99f);
      Assert::IsTrue(similarity(0,2) < 0.99f);
    }
		
    TEST_METHOD(randomlySampleVisualWordsTest)
		{
      ptr_vector<Camera> cams;
      double sampleSizeFraction = 0.;
      MatrixXf vw;
      randomlySampleVisualWords(cams,sampleSizeFraction,&vw);
      Assert::IsTrue(vw.cols() == 0);

      int nCams = 2;
      int nFeatures = 10;
      int dim = 10;
      double count = 0.f;
      for(int i = 0; i < nCams; i++)
      {
        cams.emplace_back(new StandardCamera);
        cams[i]->resizeFeatures(nFeatures,dim);
        VectorXf descr(dim);
        descr.fill(float(i));
        for(int iFeat = 0; iFeat < nFeatures; iFeat++)
        {
          cams[i]->setFeature(iFeat,0,0,0,0,descr.data());
          count += i;
        }
      }

      randomlySampleVisualWords(cams,sampleSizeFraction,&vw);
      Assert::IsTrue(vw.cols() == 0);

      sampleSizeFraction = 0.5;
      randomlySampleVisualWords(cams,sampleSizeFraction,&vw);
      Assert::IsTrue(vw.cols() == nFeatures*sampleSizeFraction*nCams);
      Assert::IsTrue(vw.rows() == dim);
      count *= sampleSizeFraction;
      for(int i = 0; i < vw.cols(); i++)
      {
        count -= vw(0,i);
      }
      Assert::IsTrue(count == 0.);
		}

    TEST_METHOD(findClosestVisualWordsTest)
    {
      ptr_vector<Camera> cams;
      MatrixXf vw;
      vector<vector<int>> closestVisualWord;
      int nFeatures = 10;
      int dim = 4;
      cams.emplace_back(new StandardCamera);
      cams[0]->resizeFeatures(nFeatures,dim);
      VectorXf descr(dim);
      for(int iFeat = 0; iFeat < nFeatures; iFeat++)
      {
        descr = VectorXf::Random(dim);
        cams[0]->setFeature(iFeat,0,0,0,0,descr.data());
      }

      vw = MatrixXf::Random(dim,200);
      vw.col(0) << 10,0,0,0;
      cams[0]->setFeature(0,0,0,0,0,vw.data());

      findClosestVisualWords(cams,vw,&closestVisualWord);
      Assert::IsTrue(closestVisualWord.size() == 1);
      Assert::IsTrue(closestVisualWord[0].size() == nFeatures);
      Assert::IsTrue(closestVisualWord[0][0] == 0);
    }

    TEST_METHOD(computeTFIDFTest)
    {
      size_t nVisualWords = 100;
      vector<vector<int>> closestVisualWord(3);
      closestVisualWord[0].push_back(0);
      closestVisualWord[0].push_back(1);
      closestVisualWord[0].push_back(2);
      closestVisualWord[0].push_back(3);
      closestVisualWord[1].push_back(0);
      closestVisualWord[1].push_back(1);
      closestVisualWord[1].push_back(2);
      closestVisualWord[1].push_back(3);
      closestVisualWord[1].push_back(4);
      closestVisualWord[2].push_back(0);
      closestVisualWord[2].push_back(10);
      closestVisualWord[2].push_back(11);
      closestVisualWord[2].push_back(12);
      VectorXf idf;
      MatrixXf tfidf;
      computeTFIDF(nVisualWords,closestVisualWord,&idf,&tfidf);
      Assert::IsTrue(idf(0) == float(log(1.f)));
      Assert::IsTrue(idf(89) == 0.f);
      Assert::IsTrue(tfidf(0,0) == 1.f * idf(0));
    }

	};
}