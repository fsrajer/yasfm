#include "stdafx.h"
#include "CppUnitTest.h"

#include "relative_pose.h"
#include "utils_tests.h"
#include "standard_camera.h"

using namespace Microsoft::VisualStudio::CppUnitTestFramework;

namespace yasfm_tests
{
	TEST_CLASS(relative_pose_tests)
	{
	public:

    TEST_METHOD(chooseInitialCameraPairTest)
    {
      int minMatches = 1;
      double minScore = 1.;
      int numCams = 4;
      ArrayXXi numMatches(ArrayXXi::Zero(numCams,numCams));
      ArrayXXd scores(ArrayXXd::Zero(numCams,numCams));
      uset<int> camsToUse;

      IntPair initPair = chooseInitialCameraPair(minMatches,minScore,camsToUse,numMatches,
        scores);
      Assert::IsTrue(initPair.first == -1 && initPair.second == -1);

      camsToUse.insert(0);
      camsToUse.insert(1);
      initPair = chooseInitialCameraPair(minMatches,minScore,camsToUse,numMatches,
        scores);
      Assert::IsTrue(initPair.first == -1 && initPair.second == -1);

      numMatches(1,0) = numMatches(0,1) = 1;
      numMatches(2,1) = numMatches(1,2) = 2;
      scores(1,0) = scores(0,1) = 1.;
      scores(2,1) = scores(1,2) = 2.;

      initPair = chooseInitialCameraPair(minMatches,minScore,camsToUse,numMatches,
        scores);
      Assert::IsTrue(initPair.first == 0 && initPair.second == 1);

      camsToUse.insert(2);
      initPair = chooseInitialCameraPair(minMatches,minScore,camsToUse,numMatches,
        scores);
      Assert::IsTrue(initPair.first == 1 && initPair.second == 2);

      scores(2,1) = scores(1,2) = .5;
      initPair = chooseInitialCameraPair(minMatches,minScore,camsToUse,numMatches,
        scores);
      Assert::IsTrue(initPair.first == 0 && initPair.second == 1);
    }
    
    TEST_METHOD(F2PsTest)
    {
      Matrix34d P = generateRandomProjection();
      Matrix3d ex;
      crossProdMat(P.col(3),&ex);
      Matrix3d F = ex*P.leftCols(3);

      Matrix34d _P;
      F2Ps(F,&_P);

      crossProdMat(_P.col(3),&ex);
      Matrix3d _F = ex*_P.leftCols(3);

      F.normalize();
      _F.normalize();
      F *= sgn(F(0,0));
      _F *= sgn(_F(0,0));

      Assert::IsTrue(_F.isApprox(F));
    }

    TEST_METHOD(FK2ETest)
    {
      Matrix3d K1,K2,R;
      K1 = generateRandomCalibration();
      K2 = generateRandomCalibration();
      R = generateRandomRotation();
      Vector3d t(Vector3d::Random());
      Matrix3d tx;
      crossProdMat(t,&tx);
      
      Matrix3d E = tx*R;
      Matrix3d F = K2.inverse().transpose() * E * K1.inverse();

      Matrix3d _E;
      FK2E(F,K1,K2,&_E);

      E.normalize();
      _E.normalize();
      E *= sgn(E(0,0));
      _E *= sgn(_E(0,0));

      Assert::IsTrue(_E.isApprox(E));
    }

    TEST_METHOD(E2RCTest)
    {
      Vector3d axis(Vector3d::Random());
      axis.normalize();
      AngleAxisd aa(0.1,axis);
      Matrix3d R = aa.toRotationMatrix();
      Vector3d C(Vector3d::Random());

      Matrix3d K1 = generateRandomCalibration();
      Matrix3d K2 = generateRandomCalibration();

      Matrix34d P1(Matrix34d::Identity()),P2(Matrix34d::Identity());
      P1 = K1 * P1;
      P2.col(3) = -C;
      P2 = K2 * R * P2;
      
      Matrix3d E,tx;
      Vector3d t = R * (-C);
      crossProdMat(t,&tx);
      E = tx * R;

      Vector3d pt;
      pt << 0,0,100;
      vector<IntPair> matches(1);
      matches[0] = IntPair(0,0);
      vector<Vector2d> keys1(1),keys2(1);
      Vector3d tmp = P1 * pt.homogeneous();
      keys1[0] = tmp.hnormalized();
      tmp = P2 * pt.homogeneous();
      keys2[0] = tmp.hnormalized();

      Matrix3d _R;
      Vector3d _C;
      E2RC(E,K1,K2,matches,keys1,keys2,&_R,&_C);

      C.normalize();
      _C.normalize();
      Assert::IsTrue(R.isApprox(_R));
      Assert::IsTrue(C.isApprox(_C));
    }

    TEST_METHOD(estimateRelativePose7ptTest)
    {
      Matrix34d P1(Matrix34d::Identity()),P2(generateRandomProjection());
      Matrix3d ex;
      crossProdMat(P2.col(3),&ex);
      Matrix3d F = ex*P2.leftCols(3);

      vector<Vector2d> keys1(15),keys2(15);
      vector<IntPair> matches(15);
      for(int i = 0; i < 15; i++)
      {
        Vector3d pt(Vector3d::Random());
        Vector3d tmp = P1 * pt.homogeneous();
        keys1[i] = tmp.hnormalized();
        tmp = P2 * pt.homogeneous();
        keys2[i] = tmp.hnormalized();
        matches[i] = IntPair(i,i);
      }

      vector<Matrix3d> _Fs;
      estimateRelativePose7pt(keys1,keys2,matches,&_Fs);

      bool goodF = false;
      F.normalize();
      F *= sgn(F(0,0));
      for(auto& _F : _Fs)
      {
        _F.normalize();
        _F *= sgn(_F(0,0));
        goodF |= (_F - F).norm() < 1e-10;
          
        Assert::IsTrue(abs(_F.determinant()) < 1e-10);
      }
      Assert::IsTrue(goodF);

      Matrix3d _F;
      OptionsRANSAC opt(512,1.,10);
      vector<int> inliers;
      estimateRelativePose7ptRANSAC(opt,keys1,keys2,matches,&_F,&inliers);
      Assert::IsTrue(inliers.size() >= opt.minInliers() || inliers.empty());
      for(size_t i = 0; i < inliers.size(); i++)
      {
        auto& pt1 = keys1[inliers[i]];
        auto& pt2 = keys2[inliers[i]];
        Vector3d Fpt1 = F*pt1.homogeneous();
        Vector3d FTpt2 = F.transpose()*pt2.homogeneous();
        double pt2Fpt1 = pt2.homogeneous().dot(Fpt1);
        double e = (pt2Fpt1*pt2Fpt1) /
          (Fpt1.topRows(2).squaredNorm() + FTpt2.topRows(2).squaredNorm());
        Assert::IsTrue(e <= opt.errorThresh());
      }

      CameraPair pair;
      pair.matches = matches;
      pair.dists.resize(matches.size(),0);
      inliers.clear();
      estimateRelativePose7ptPROSAC(opt,keys1,keys2,pair,&_F,&inliers);
      Assert::IsTrue(inliers.size() >= opt.minInliers() || inliers.empty());
      for(size_t i = 0; i < inliers.size(); i++)
      {
        auto& pt1 = keys1[inliers[i]];
        auto& pt2 = keys2[inliers[i]];
        Vector3d Fpt1 = F*pt1.homogeneous();
        Vector3d FTpt2 = F.transpose()*pt2.homogeneous();
        double pt2Fpt1 = pt2.homogeneous().dot(Fpt1);
        double e = (pt2Fpt1*pt2Fpt1) /
          (Fpt1.topRows(2).squaredNorm() + FTpt2.topRows(2).squaredNorm());
        Assert::IsTrue(e <= opt.errorThresh());
      }

      keys1.clear();
      keys2.clear();
      matches.clear();
      pair.matches.clear();
      pair.dists.clear();
      inliers.clear();
      estimateRelativePose7ptRANSAC(opt,keys1,keys2,matches,&_F,&inliers);
      Assert::IsTrue(inliers.size() == 0);
      estimateRelativePose7ptPROSAC(opt,keys1,keys2,pair,&_F,&inliers);
      Assert::IsTrue(inliers.size() == 0);
    }

    TEST_METHOD(Mediator7ptRANSACRefineTest)
    {
      const int nMatches = 1000;
      Matrix34d P1(Matrix34d::Identity()),P2(generateRandomProjection());
      Matrix3d ex;
      crossProdMat(P2.col(3),&ex);
      Matrix3d F = ex*P2.leftCols(3);

      vector<Vector2d> keys1(nMatches),keys2(nMatches);
      vector<IntPair> matches(nMatches);
      vector<int> inliers(nMatches);
      for(int i = 0; i < nMatches; i++)
      {
        Vector3d pt(Vector3d::Random());
        Vector3d tmp = P1 * pt.homogeneous();
        keys1[i] = tmp.hnormalized();
        tmp = P2 * pt.homogeneous();
        keys2[i] = tmp.hnormalized();
        matches[i] = IntPair(i,i);
        inliers[i] = i;
      }

      double mag = F.norm();
      F += 0.001*mag*Matrix3d::Random();
      
      Mediator7ptRANSAC m(keys1,keys2,matches);
      m.refine(1e-12,inliers,&F);

      double sqthresh = 0.001;
      Assert::IsTrue(findInliers(m,F,sqthresh) == nMatches);
    }

    TEST_METHOD(estimateRelativePose5ptTest)
    {
      Matrix34d P1(Matrix34d::Identity()),P2(Matrix34d::Identity());
      Matrix3d K1(generateRandomCalibration()),K2(generateRandomCalibration()),
        R(generateRandomRotation());
      Vector3d C(Vector3d::Random());
      P1 = K1*P1;
      P2.col(3) = -C;
      P2 = R * P2;
      Matrix3d ex;
      crossProdMat(P2.col(3),&ex);
      Matrix3d E = ex*R;
      P2 = K2 * P2;

      vector<Vector2d> keys1(15),keys2(15);
      vector<Vector3d> keys1Norm(15),keys2Norm(15);
      vector<IntPair> matches(15);
      for(int i = 0; i < 15; i++)
      {
        Vector3d pt(Vector3d::Random());
        Vector3d tmp = P1 * pt.homogeneous();
        keys1[i] = tmp.hnormalized();
        tmp = P2 * pt.homogeneous();
        keys2[i] = tmp.hnormalized();
        matches[i] = IntPair(i,i);
        keys1Norm[i] = K1.inverse() * keys1[i].homogeneous();
        keys2Norm[i] = K2.inverse() * keys2[i].homogeneous();
      }

      vector<Matrix3d> _Es;
      estimateRelativePose5pt(keys1Norm,keys2Norm,matches,&_Es);

      bool goodE = false;
      for(auto& _E : _Es)
      {
        size_t numGood = 0;
        for(size_t i = 0; i < keys1Norm.size(); i++)
        {
          auto& pt1 = keys1Norm[i];
          auto& pt2 = keys2Norm[i];
          Vector3d Ept1 = _E*pt1;
          Vector3d ETpt2 = _E.transpose()*pt2;
          double pt2Ept1 = pt2.dot(Ept1);
          double e = (pt2Ept1*pt2Ept1) /
            (Ept1.topRows(2).squaredNorm() + ETpt2.topRows(2).squaredNorm());
          
          numGood += (e <= 1e10); // TODO: edit
        }
        if(numGood == keys1Norm.size() && abs(_E.determinant()) < 1e10)// TODO: edit
        {
          goodE = true;
          break;
        }
      }
      Assert::IsTrue(goodE);

      StandardCamera cam1,cam2;
      OptionsRANSAC opt(512,1.,10);
      vector<IntPair> matches_empty;
      Matrix3d _E;
      vector<int> inliers;
      estimateRelativePose5ptRANSAC(opt,cam1,cam2,matches_empty,&_E,&inliers);
      Assert::IsTrue(inliers.empty());

      cam1.setParams(P1);
      cam2.setParams(P2);
      cam1.resizeFeatures((int)keys1.size(),0);
      cam2.resizeFeatures((int)keys2.size(),0);
      for(size_t i = 0; i < keys1.size(); i++)
      {
        float dummy;
        cam1.setFeature((int)i,keys1[i](0),keys1[i](1),0,0,&dummy);
        cam2.setFeature((int)i,keys2[i](0),keys2[i](1),0,0,&dummy);
      }

      inliers.clear();
      estimateRelativePose5ptRANSAC(opt,cam1,cam2,matches,&_E,&inliers);
      Assert::IsTrue(inliers.size() >= opt.minInliers() || inliers.empty());
      for(size_t i = 0; i < inliers.size(); i++)
      {
        auto& pt1 = keys1Norm[inliers[i]];
        auto& pt2 = keys2Norm[inliers[i]];
        Vector3d Ept1 = E*pt1;
        Vector3d ETpt2 = E.transpose()*pt2;
        double pt2Ept1 = pt2.dot(Ept1);
        double e = (pt2Ept1*pt2Ept1) /
          (Ept1.topRows(2).squaredNorm() + ETpt2.topRows(2).squaredNorm());
        Assert::IsTrue(e <= opt.errorThresh());
      }
    }

    TEST_METHOD(estimateFundamentalMatrixTest)
    {
      double tolerance = 1e-12;
      Matrix34d P1(Matrix34d::Identity()),P2(generateRandomProjection());
      Matrix3d ex;
      crossProdMat(P2.col(3),&ex);
      Matrix3d F = ex*P2.leftCols(3);
      F /= F(2,2);

      int n = 15;
      vector<Vector2d> keys1(n),keys2(n);
      vector<IntPair> matches;
      vector<int> matchesToUse;
      for(int i = 0; i < n; i++)
      {
        Vector3d pt(Vector3d::Random());
        Vector3d tmp = P1 * pt.homogeneous();
        keys1[i] = tmp.hnormalized();
        keys1[i] += 1e-8*Vector2d::Random();
        tmp = P2 * pt.homogeneous();
        keys2[i] = tmp.hnormalized();
        keys2[i] += 1e-8*Vector2d::Random();

        matches.emplace_back(i,i);
        matchesToUse.push_back(i);
      }
      Matrix3d _F;
      estimateFundamentalMatrix(keys1,keys2,matches,matchesToUse,tolerance,&_F);
      _F /= _F(2,2);
      Assert::IsTrue(F.isApprox(_F,1e-5));
    }

    TEST_METHOD(estimateEssentialMatrixTest)
    {
      double tolerance = 1e-12;
      Matrix3d E;
      closestEssentialMatrix(Matrix3d::Random(),&E);
      Matrix3d K1 = generateRandomCalibration(),
        K2 = generateRandomCalibration();

      int n = 15;
      vector<Vector2d> keys1(n),keys2(n);
      vector<IntPair> matches;
      vector<int> matchesToUse;
      for(int i = 0; i < n; i++)
      {
        Vector3d pt2 = Vector3d::Random();
        keys2[i] = (K2 * pt2).hnormalized();

        Vector3d pt1 = Vector3d::Random();
        Vector3d tmp = pt2.transpose() * E;
        pt1(0) = -(tmp(1)*pt1(1) + tmp(2)*pt1(2))/tmp(0);

        keys1[i] = (K1 * pt1).hnormalized();

        matches.emplace_back(i,i);
        matchesToUse.push_back(i);

        // add noise
        keys1[i] += 1e-9 * Vector2d::Random();
        keys2[i] += 1e-9 * Vector2d::Random();
      }

      Matrix3d _E;
      estimateEssentialMatrix(keys1,keys2,K1.inverse(),K2.inverse(),
        matches,matchesToUse,tolerance,&_E);
      _E /= _E(2,2);
      E /= E(2,2);

      Assert::IsTrue(E.isApprox(_E,1e-8));

      _E += 1e-4 * Matrix3d::Random();
      double cumErr = 0;
      for(int i = 0; i < n; i++)
        cumErr += abs((K2.inverse()*keys2[i].homogeneous()).transpose() * _E *
        (K1.inverse() * keys1[i].homogeneous()));
      
      refineEssentialMatrixNonLinear(keys1,keys2,K1.inverse(),K2.inverse(),
        matches,matchesToUse,tolerance,&_E);
      double _cumErr = 0;
      for(int i = 0; i < n; i++)
        _cumErr += abs((K2.inverse()*keys2[i].homogeneous()).transpose() * _E *
        (K1.inverse() * keys1[i].homogeneous()));
      
      Assert::IsTrue(cumErr > _cumErr);
    }

    TEST_METHOD(computeSimilarityFromMatchTest)
    {
      const double cPrecision = 1e-8;
      Vector2d k1(Vector2d::Zero()),k2(Vector2d::Zero());
      double s1 = 1.,s2 = 1.,o1 = 0.,o2 = 0.;

      // none
      Matrix3d S;
      computeSimilarityFromMatch(k1,s1,o1,k2,s2,o2,&S);
      Assert::IsTrue(S.isIdentity());

      // scale
      s1 = 2.;
      s2 = 4.;
      computeSimilarityFromMatch(k1,s1,o1,k2,s2,o2,&S);
      Assert::IsTrue(S(0,0) == 2.);
      Assert::IsTrue(S(1,1) == 2.);
      S(0,0) = S(1,1) = 1.;
      Assert::IsTrue(S.isIdentity());
      s1 = 1.;
      s2 = 1.;

      // translation
      k1 << 4,5;
      k2 << 1,1;
      computeSimilarityFromMatch(k1,s1,o1,k2,s2,o2,&S);
      Assert::IsTrue(abs(S(0,2) + 3.) < cPrecision);
      Assert::IsTrue(abs(S(1,2) + 4.) < cPrecision);
      S(0,2) = S(1,2) = 0.;
      Assert::IsTrue(S.isIdentity());
      k1 << 0,0;
      k2 << 0,0;

      // rotation
      o1 = M_PI/4;
      o2 = 3*M_PI/4;
      computeSimilarityFromMatch(k1,s1,o1,k2,s2,o2,&S);
      Assert::IsTrue(abs(S(0,0)) < cPrecision);
      Assert::IsTrue(abs(S(1,1)) < cPrecision);
      Assert::IsTrue(abs(S(0,1) + 1.) < cPrecision);
      Assert::IsTrue(abs(S(1,0) - 1.) < cPrecision);
      S(0,0) = S(1,1) = 1.;
      S(0,1) = S(1,0) = 0.;
      Assert::IsTrue(S.isIdentity());
    }

    TEST_METHOD(estimateAffinityTest)
    {
      int n = 10;
      vector<Vector2d> keys1(n),keys2(n);
      vector<IntPair> matches;
      vector<int> matchesToUse;
      Matrix3d A(Matrix3d::Random());
      A(2,0) = 0.;
      A(2,1) = 0.;
      A(2,2) = 1.;
      for(int i = 0; i < n; i++)
      {
        keys1[i] = Vector2d::Random();
        keys2[i] = A.topRows(2) * keys1[i].homogeneous();
        keys2[i] += 1e-8*Vector2d::Random();
        matches.emplace_back(i,i);
        matchesToUse.push_back(i);
      }
      Matrix3d _A;
      estimateAffinity(keys1,keys2,matches,matchesToUse,&_A);
      Assert::IsTrue(A.isApprox(_A,1e-5));
    }

    TEST_METHOD(estimateHomographyTest)
    {
      int n = 10;
      vector<Vector2d> keys1(n),keys2(n);
      vector<IntPair> matches;
      vector<int> matchesToUse;
      Matrix3d H(Matrix3d::Random());
      H(2,2) = 1.;
      for(int i = 0; i < n; i++)
      {
        keys1[i] = Vector2d::Random();
        Vector3d tmp = H * keys1[i].homogeneous();
        keys2[i] = tmp.hnormalized();
        keys2[i] += 1e-8*Vector2d::Random();
        matches.emplace_back(i,i);
        matchesToUse.push_back(i);
      }
      Matrix3d _H;
      estimateHomography(keys1,keys2,matches,matchesToUse,&_H);
      _H /= _H(2,2);
      Assert::IsTrue(H.isApprox(_H,1e-5));
    }

    TEST_METHOD(findHomographyInliersTest)
    {
      int n = 10;
      vector<Vector2d> keys1(n),keys2(n);
      vector<IntPair> matches;
      vector<int> matchesToUse;
      Matrix3d H(Matrix3d::Random());
      for(int i = 0; i < n; i++)
      {
        keys1[i] = Vector2d::Random();
        Vector3d tmp = H * keys1[i].homogeneous();
        keys2[i] = tmp.hnormalized();
        matches.emplace_back(i,i);
        matchesToUse.push_back(i);
      }
      keys2[0](0) += 2.;
      double thresh = 1.9;
      vector<int> inliers;
      findHomographyInliers(thresh,keys1,keys2,matches,H,&inliers);

      Assert::IsTrue(inliers.size() == n-1);
    }
	};
}