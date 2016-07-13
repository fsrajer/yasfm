#include "stdafx.h"
#include "CppUnitTest.h"
#include "absolute_pose.h"
#include "utils_tests.h"

using namespace Microsoft::VisualStudio::CppUnitTestFramework;
using namespace yasfm;

namespace yasfm_tests
{
	TEST_CLASS(absolute_pose_tests)
	{
	public:
		
    TEST_METHOD(resectCameraLSTest)
		{
      int nPts = 15;
      vector<Vector2d> keys;
      vector<Point> pts(nPts);
      vector<IntPair> matches;
      vector<Matrix34d> _Ps(1);

      Matrix34d P(generateRandomProjection());
      for(int i = 0; i < nPts; i++)
      {
        pts[i].coord = Vector3d::Random();
        Vector3d proj = P * pts[i].coord.homogeneous();
        keys.push_back(proj.hnormalized());
        matches.emplace_back(i,i);
      }

      resectCameraLS(keys,pts,matches,&_Ps[0]);
      Assert::IsFalse(_Ps.empty());
      P /= P(2,3);
      bool goodP = false;
      for(auto& _P : _Ps)
      {
        _P /= _P(2,3);
        goodP |= (P - _P).norm() < 1e-10;
      }
      Assert::IsTrue(goodP);
		}

    TEST_METHOD(resectCamera5AndHalfPtTest)
    {
      int nPts = 15;
      vector<Vector2d> keys;
      vector<Point> pts;
      vector<IntPair> matches;
      vector<Matrix34d> _Ps;
      resectCamera5AndHalfPt(keys,pts,matches,&_Ps);
      Assert::IsTrue(_Ps.empty());

      pts.resize(nPts);
      Matrix34d P(generateRandomProjection());
      for(int i = 0; i < nPts; i++)
      {
        pts[i].coord = Vector3d::Random();
        Vector3d proj = P * pts[i].coord.homogeneous();
        keys.push_back(proj.hnormalized());
        matches.emplace_back(i,i);
      }

      resectCamera5AndHalfPt(keys,pts,matches,&_Ps);
      Assert::IsFalse(_Ps.empty());
      P /= P(2,3);
      bool goodP = false;
      for(auto& _P : _Ps)
      {
        _P /= _P(2,3);
        goodP |= (P - _P).norm() < 1e-10;
      }
      Assert::IsTrue(goodP);

      OptionsRANSAC opt(4096,1.,8);
      Matrix34d _P;
      vector<int> inliers;
      resectCamera5AndHalfPtRANSAC(opt,matches,keys,pts,&_P,&inliers);
      Assert::IsTrue(inliers.size() > opt.minInliers() || inliers.empty());

      for(int i : inliers)
      {
        Vector3d proj = _P * pts[i].coord.homogeneous();
        Assert::IsTrue(
          (proj.hnormalized() - keys[i]).norm() < opt.errorThresh());
      }
    }

	TEST_METHOD(resectCamera3ptTest)
  {
    int nPts = 15;
		vector<Vector2d> keys;
		vector<Point> pts;
		vector<IntPair> matches;
		vector<Matrix34d> _Ps;
		resectCamera3pt(keys, pts, matches, &_Ps);
		Assert::IsTrue(_Ps.empty());

		// 3d points must lie in front of the camera
		Matrix34d P;
		Vector3d shift;
		shift << 0, 0, 3;
		P << generateRandomRotation(), Vector3d().Random()+shift;

    pts.resize(nPts);
		for (int i = 0; i < nPts; i++)
		{
			pts[i].coord = Vector3d::Random();
			Vector3d proj = P * pts[i].coord.homogeneous();
			keys.push_back(proj.hnormalized());
			matches.emplace_back(i, i);
		}

		resectCamera3pt(keys, pts, matches, &_Ps);
		Assert::IsFalse(_Ps.empty());
		P.normalize();
		bool goodP = false;
		for (auto& _P : _Ps)
		{
			_P.normalize();
			goodP |= (P - _P).norm() < 1e-5;
		}
		Assert::IsTrue(goodP);

		OptionsRANSAC opt(4096, 1., 8);
		Matrix34d _P;
		vector<int> inliers;
		resectCamera3ptRANSAC(opt, matches, keys, pts, &_P, &inliers);
		Assert::IsTrue(inliers.size() > opt.minInliers() || inliers.empty());

		for (int i : inliers)
		{
			Vector3d proj = _P * pts[i].coord.homogeneous();
			Assert::IsTrue((proj.hnormalized() - keys[i]).norm() < opt.errorThresh());
		}
	}

};
}