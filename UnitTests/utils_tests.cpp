#include "stdafx.h"
#include "CppUnitTest.h"

#include "utils_tests.h"
#include "utils.h"

using namespace Microsoft::VisualStudio::CppUnitTestFramework;
using Eigen::Vector4d;
using namespace yasfm;

namespace yasfm_tests
{

void assertVectorEquality(const vector<int>& v1,const vector<int>& v2)
{
  Assert::AreEqual(v1.size(),v2.size());
  for(size_t i = 0; i < v1.size(); i++)
  {
    Assert::AreEqual(v1[i],v2[i]);
  }
}

void assertStringEquality(const string& s1,const string& s2)
{
  Assert::IsTrue(s1.compare(s2) == 0);
}

Matrix3d generateRandomRotation()
{
  return Eigen::AngleAxisd(Vector4d::Random()).toRotationMatrix();
}

Matrix3d generateRandomCalibration()
{
  Matrix3d K = Matrix3d::Random().triangularView<Eigen::Upper>();
  K.diagonal() = K.diagonal().cwiseAbs();
  if(K(2,2) == 0.)
    K(2,2) = 1.;
  else
    K /= K(2,2);
  return K;
}

Matrix34d generateRandomProjection()
{
  Matrix3d K = generateRandomCalibration();
  Matrix3d R = generateRandomRotation();
  Vector3d C = Vector3d::Random();
  Matrix34d P;
  P.setIdentity();
  P.rightCols(1) = -C;
  P = K*R*P * 47.; // 47 is there to test scaling
  return P;
}

	TEST_CLASS(utils_tests)
	{
  public:

    TEST_METHOD(joinPathsTest)
    {
      assertStringEquality("",joinPaths("",""));

      assertStringEquality("foo",joinPaths("foo",""));
      assertStringEquality("foo/",joinPaths("foo/",""));
      assertStringEquality("foo\\",joinPaths("foo\\",""));

      assertStringEquality("foo",joinPaths("","foo"));
      assertStringEquality("foo/",joinPaths("","foo/"));
      assertStringEquality("foo\\",joinPaths("","foo\\"));

      assertStringEquality("foo/bar",joinPaths("foo","bar"));
      assertStringEquality("foo/bar",joinPaths("foo/","bar"));
      assertStringEquality("foo\\bar",joinPaths("foo\\","bar"));
    }

    TEST_METHOD(extractPathTest)
    {
      string filepath("a.txt");
      Assert::IsTrue(extractPath(filepath) == "");

      filepath = "xx/a.txt";
      Assert::IsTrue(extractPath(filepath) == "xx");

      filepath = "xx\\a.txt";
      Assert::IsTrue(extractPath(filepath) == "xx");
    }

    TEST_METHOD(extractFilenameTest)
    {
      string filepath("a.txt");
      Assert::IsTrue(extractFilename(filepath) == "a.txt");

      filepath = "xx/a.txt";
      Assert::IsTrue(extractFilename(filepath) == "a.txt");

      filepath = "x/xx\\a.txt";
      Assert::IsTrue(extractFilename(filepath) == "a.txt");
    }

    TEST_METHOD(quicksortTest)
    {
      vector<int> arr,order;
      quicksort(arr,&order);
      Assert::IsTrue(order.empty());

      vector<int> arr2 = {1,3,2,5,4,0},
        orderTrue = {5,0,2,1,4,3};
      quicksort(arr2,&order);

      assertVectorEquality(orderTrue,order);
    }

    TEST_METHOD(filterVectorTest)
    {
      vector<bool> keep(5,true);
      vector<int> a(5),b;
      for(int i = 0; i < 5; i++)
        a[i] = i;
      b = a;

      filterVector(keep,&a);
      keep[0] = false;
      filterVector(keep,&b);

      Assert::IsTrue(a.size() == 5);
      Assert::IsTrue(b.size() == 4);
      for(int i = 0; i < 4; i++)
      {
        Assert::AreEqual(a[i],i);
        Assert::AreEqual(b[i],i+1);
      }
      Assert::AreEqual(a[4],4);
    }

    TEST_METHOD(filterVectorTest2)
    {
      vector<int> toKeep(5);
      vector<int> a(5),b;
      for(int i = 0; i < 5; i++)
        a[i] = i;
      b = a;
      toKeep = a;

      filterVector(toKeep,&a);
      toKeep.erase(toKeep.begin());
      filterVector(toKeep,&b);

      Assert::IsTrue(a.size() == 5);
      Assert::IsTrue(b.size() == 4);
      for(int i = 0; i < 4; i++)
      {
        Assert::AreEqual(a[i],i);
        Assert::AreEqual(b[i],i+1);
      }
      Assert::AreEqual(a[4],4);
    }

    TEST_METHOD(filterOutOutliersTest)
    {
      vector<int> toRemove;
      vector<int> a(5),b;
      for(int i = 0; i < 5; i++)
        a[i] = i;
      b = a;

      filterOutOutliers(toRemove,&a);
      toRemove.push_back(0);
      filterOutOutliers(toRemove,&b);

      Assert::IsTrue(a.size() == 5);
      Assert::IsTrue(b.size() == 4);
      for(int i = 0; i < 4; i++)
      {
        Assert::AreEqual(a[i],i);
        Assert::AreEqual(b[i],i+1);
      }
      Assert::AreEqual(a[4],4);
    }

    TEST_METHOD(crossProdMatTest)
    {
      Vector3d x = Vector3d::Random();
      Matrix3d X;
      crossProdMat(x,&X);
      Assert::IsTrue(X.diagonal().isZero() &&
        X(0,1) == -x(2) && X(0,2) == x(1) &&
        X(1,0) == x(2) && X(1,2) == -x(0) &&
        X(2,0) == -x(1) && X(2,1) == x(0));
      Vector3d a = Vector3d::Random();
      Vector3d res1 = x.cross(a);
      Vector3d res2 = X*a;
      Assert::IsTrue(res1 == res2);
    }

    TEST_METHOD(RQDecompositionTest)
    {
      const double cDesiredPrecision = 1e-14;
      Matrix3d R = generateRandomCalibration();
      Matrix3d Q = generateRandomRotation();
      Matrix3d A = R * Q;
      Matrix3d R_,Q_;
      RQDecomposition(A,&R_,&Q_);
      Matrix3d T = Matrix3d::Identity();
      if(R_(0,0) < 0)
        T(0,0) = -1;
      if(R_(1,1) < 0)
        T(1,1) = -1;
      if(R_(2,2) < 0)
        T(2,2) = -1;
      R_ = R_*T;
      Q = T.inverse()*Q;
      Assert::IsTrue(R.isApprox(R_,cDesiredPrecision));
      Assert::IsTrue(Q.isApprox(Q_,cDesiredPrecision));
    }

    TEST_METHOD(P2KRCTest)
    {
      const double desiredPrecision = 1e-13;
      Matrix3d K = generateRandomCalibration();
      Matrix3d R = generateRandomRotation();
      Vector3d C = Vector3d::Random();
      Matrix34d P;
      P.setIdentity();
      P.rightCols(1) = -C;
      P = K*R*P * 47.; // 47 is there to test scaling
      Matrix3d _K,_R;
      Vector3d _C;
      P2KRC(P,&_K,&_R,&_C);
      Assert::IsTrue(K.isApprox(_K,desiredPrecision));
      Assert::IsTrue(R.isApprox(_R,desiredPrecision));
      Assert::IsTrue(C.isApprox(_C,desiredPrecision));
    }

    TEST_METHOD(unzipPairsVectorTest)
    {
      vector<IntPair> pairs;
      vector<int> firsts,seconds;

      unzipPairsVectorFirst(pairs,&firsts);
      unzipPairsVectorSecond(pairs,&seconds);
      Assert::IsTrue(firsts.empty());
      Assert::IsTrue(seconds.empty());

      pairs.emplace_back(0,5);
      pairs.emplace_back(1,6);
      unzipPairsVectorFirst(pairs,&firsts);
      unzipPairsVectorSecond(pairs,&seconds);
      Assert::IsTrue(firsts.size() == 2);
      Assert::IsTrue(seconds.size() == 2);
      Assert::AreEqual(firsts[0],0);
      Assert::AreEqual(firsts[1],1);
      Assert::AreEqual(seconds[0],5);
      Assert::AreEqual(seconds[1],6);
    }

    TEST_METHOD(chooseWellMatchedCamerasTest)
    {
      int minMatchesThresh = 10;
      double factor = 0.8;

      vector<vector<IntPair>> cam2SceneMatches;
      uset<int> wellMatchedCams;
      chooseWellMatchedCameras(minMatchesThresh,factor,cam2SceneMatches,&wellMatchedCams);
      Assert::IsTrue(wellMatchedCams.empty());

      cam2SceneMatches.resize(3);
      chooseWellMatchedCameras(minMatchesThresh,factor,cam2SceneMatches,&wellMatchedCams);
      Assert::IsTrue(wellMatchedCams.empty());

      cam2SceneMatches[0].resize(10);
      cam2SceneMatches[1].resize(8);
      cam2SceneMatches[2].resize(7);
      chooseWellMatchedCameras(minMatchesThresh,factor,cam2SceneMatches,&wellMatchedCams);
      Assert::IsTrue(wellMatchedCams.size() == 1);
      Assert::IsTrue(wellMatchedCams.count(0) >= 0);

      wellMatchedCams.clear();
      minMatchesThresh = 1;
      chooseWellMatchedCameras(minMatchesThresh,factor,cam2SceneMatches,&wellMatchedCams);
      Assert::IsTrue(wellMatchedCams.size() == 2);
      Assert::IsTrue(wellMatchedCams.count(0) >= 0);
      Assert::IsTrue(wellMatchedCams.count(1) >= 0);
    }

    TEST_METHOD(computeAverageReprojectionErrorTest)
    {
      ptr_vector<Camera> cams;
      Points points;
      double err = computeAverageReprojectionError(cams,points);
      Assert::AreEqual(err,0.);

      Matrix3d K = generateRandomCalibration();
      Matrix3d R = generateRandomRotation();
      Vector3d C = Vector3d::Random();
      Matrix34d P;
      P.setIdentity();
      P.rightCols(1) = -C;

      vector<Vector3d> ptCoord(1);
      vector<Vector3uc> colors(1);
      ptCoord[0] = Vector3d::Random();
      vector<SplitNViewMatch> ptViews(1);
      ptViews[0].observedPart.emplace(0,0);
      ptViews[0].observedPart.emplace(1,0);
      points.addPoints(ptCoord,colors,ptViews);

      cams.push_back(make_unique<StandardCamera>("../UnitTests/test_dataset/test0.JPG"));
      cams.push_back(make_unique<StandardCamera>("../UnitTests/test_dataset/test1.JPG"));
      cams[0]->setParams(P);
      cams[1]->setParams(P);
      cams[0]->resizeFeatures(1,1);
      cams[1]->resizeFeatures(1,1);

      Vector2d proj = cams[0]->project(ptCoord[0]);
      float descr = 0;

      cams[0]->setFeature(0,proj(0),proj(1),&descr);
      cams[1]->setFeature(0,proj(0),proj(1),&descr);

      err = computeAverageReprojectionError(cams,points);

      Assert::AreEqual(err,0.);
    }

    TEST_METHOD(approximateInverseRadialDistortionTest)
    {
      // example data
      Vector4d fwd;
      fwd << 0.,0.00322511,0.,9.24195e-5;
      array<double,4> inv;
      double maxRadius = 0.29225;

      approximateInverseRadialDistortion(4,4,maxRadius,&fwd(0),&inv[0]);
      
      // distort
      Vector2d pt(Vector2d::Random());
      double r = pt.norm();
      double rPower = r;
      double d = 1.;
      for(int i = 0; i < 4; i++)
      {
        d += rPower*fwd(i);
        rPower *= r;
      }
      Vector2d ptD = d*pt;

      // undistort 
      r = ptD.norm();
      rPower = r;
      d = 1.;
      for(int i = 0; i < 4; i++)
      {
        d += rPower*inv[i];
        rPower *= r;
      }
      Vector2d ptApprox = d*ptD;

      Assert::IsTrue(pt.isApprox(ptApprox,1e-5));
    }

  private:

    /*
    TEST_METHOD(findPointColorsTest)
    {
      vector<int> points2tracks;
      vector<bool> pointsMask;
      vector<Track> tracks;
      ptr_vector<ICamera> cams;
      unordered_set<int> addedCams;
      vector<Vector3uc> pointsColors;

      //findPointColors(points2tracks, pointsMask, tracks, cams, addedCams, pointsColors);
      //Assert::IsTrue(pointsColors.empty());

      points2tracks = { 0, 1 };
      YASFM_TESTS_EXPECT_EXCEPTION(findPointColors(points2tracks, pointsMask,
        tracks, cams, addedCams, pointsColors));
      
      //tracks[0] = Track();
      //tracks[1] = Track();
    }
    */

	};
}