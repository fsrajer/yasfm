#include "stdafx.h"
#include "CppUnitTest.h"
#include "bundle_adjust.h"
#include "utils_tests.h"
#include "standard_camera.h"
#include "absolute_pose.h"

using namespace Microsoft::VisualStudio::CppUnitTestFramework;
using namespace yasfm;

namespace yasfm_tests
{
	TEST_CLASS(bundle_adjust_tests)
	{
	public:
		
		TEST_METHOD(bundleAdjustTest)
		{
      OptionsBundleAdjustment opt;
      ptr_vector<Camera> cams;
      vector<Point> pts;
      bundleAdjust(opt,&cams,&pts);

      int nCams = 4;
      int nPts = 10;
      for(int camIdx = 0; camIdx < nCams; camIdx++)
      {
        cams.push_back(make_unique<StandardCamera>("../UnitTests/test_dataset/test0.JPG",""));
        cams.back()->setParams(generateRandomProjection());
        cams.back()->resizeFeatures(nPts,0);
      }

      pts.resize(nPts);
      for(int iPt = 0; iPt < nPts; iPt++)
      {
        pts[iPt].coord = Vector3d::Random();
        for(int camIdx = 0; camIdx < nCams; camIdx++)
        {
          pts[iPt].views.emplace(camIdx,iPt);
          Vector2d p = cams[camIdx]->project(pts[iPt].coord);
          p += 0.1*Vector2d::Random(); // add noise
          float dummy;
          cams[camIdx]->setFeature(iPt,p(0),p(1),0,0,&dummy);
        }
      }

      double origErr = computeAverageReprojectionError(cams,pts);

      ptr_vector<Camera> cams1;
      for(int i = 0; i < nCams; i++)
        cams1.push_back(cams[i]->clone());
      vector<Point> pts1 = pts;
      bundleAdjustPoints(opt,&cams1,&pts1);
      double err1 = computeAverageReprojectionError(cams1,pts1);
      for(int i = 0; i < nCams; i++)
      {
        Matrix34d P = cams[i]->P(),P1 = cams1[i]->P();
        Assert::IsTrue(P.isApprox(P1));
      }
      Assert::IsTrue(origErr >= err1);

      cams1.clear();
      for(int i = 0; i < nCams; i++)
        cams1.push_back(cams[i]->clone());
      pts1 = pts;
      bundleAdjustCams(opt,&cams1,&pts1);
      err1 = computeAverageReprojectionError(cams1,pts1);
      for(int i = 0; i < nPts; i++)
      {
        Assert::IsTrue(pts[i].coord == pts1[i].coord);
      }
      Assert::IsTrue(origErr >= err1);

      cams1.clear();
      for(int i = 0; i < nCams; i++)
        cams1.push_back(cams[i]->clone());
      pts1 = pts;
      bundleAdjust(opt,&cams1,&pts1);
      err1 = computeAverageReprojectionError(cams1,pts1);
      Assert::IsTrue(origErr >= err1);
		}

    TEST_METHOD(bundleAdjustOneCamTest)
    {
      OptionsBundleAdjustment opt;
      ptr_vector<Camera> cams;
      vector<Point> pts;
      bundleAdjust(opt,&cams,&pts);

      int nCams = 1;
      int nPts = 10;
      for(int camIdx = 0; camIdx < nCams; camIdx++)
      {
        cams.push_back(make_unique<StandardCamera>("../UnitTests/test_dataset/test0.JPG",""));
        cams.back()->setParams(generateRandomProjection());
        cams.back()->resizeFeatures(nPts,0);
      }

      pts.resize(nPts);
      for(int iPt = 0; iPt < nPts; iPt++)
      {
        pts[iPt].coord = Vector3d::Random();
        for(int camIdx = 0; camIdx < nCams; camIdx++)
        {
          pts[iPt].views.emplace(camIdx,iPt);
          Vector2d p = cams[camIdx]->project(pts[iPt].coord);
          p += 0.1*Vector2d::Random(); // add noise
          float dummy;
          cams[camIdx]->setFeature(iPt,p(0),p(1),0,0,&dummy);
        }
      }

      double origErr = computeAverageReprojectionError(cams,pts);

      ptr_vector<Camera> cams1;
      for(int i = 0; i < nCams; i++)
        cams1.push_back(cams[i]->clone());
      vector<Point> pts1 = pts;
      bundleAdjustOneCam(opt,0,&(*cams1[0]),&pts1);
      double err1 = computeAverageReprojectionError(cams1,pts1);
      Assert::IsTrue(origErr >= err1);
    }

	};
}