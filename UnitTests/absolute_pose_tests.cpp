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
		
    TEST_METHOD(resectCamera6ptTest)
		{
      vector<Vector2d> keys;
      vector<Vector3d> pts;
      vector<IntPair> matches;
      vector<Matrix34d> _Ps;
      resectCamera6ptMinimal(keys,pts,matches,&_Ps);
      Assert::IsTrue(_Ps.empty());

      Matrix34d P(generateRandomProjection());
      for(int i = 0; i < 15; i++)
      {
        pts.push_back(Vector3d::Random());
        Vector3d proj = P * pts.back().homogeneous();
        keys.push_back(proj.hnormalized());
        matches.emplace_back(i,i);
      }

      resectCamera6ptMinimal(keys,pts,matches,&_Ps);
      Assert::IsFalse(_Ps.empty());
      P.normalize();
      bool goodP = false;
      for(auto& _P : _Ps)
      {
        _P.normalize();
        goodP |= (P - _P).norm() < 1e-10;
      }
      Assert::IsTrue(goodP);

      OptionsRANSAC opt(4096,1.,8);
      Matrix34d _P;
      vector<int> inliers;
      resectCamera6ptRANSAC(opt,matches,keys,pts,&_P,&inliers);
      Assert::IsTrue(inliers.size() > opt.minInliers_ || inliers.empty());

      for(int i : inliers)
      {
        Vector3d proj = _P * pts[i].homogeneous();
        Assert::IsTrue((proj.hnormalized() - keys[i]).norm() < opt.errorThresh_);
      }
		}

	};
}