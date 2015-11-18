#include "stdafx.h"
#include "CppUnitTest.h"

#include "points.h"
#include "utils_tests.h"

using namespace Microsoft::VisualStudio::CppUnitTestFramework;

namespace yasfm_tests
{
	TEST_CLASS(points_tests)
	{
	public:

    TEST_METHOD(twoViewMatchesToNViewMatchesTest)
    {
      ptr_vector<Camera> cams;
      pair_umap<CameraPair> pairs;
      vector<NViewMatch> matches;
      twoViewMatchesToNViewMatches(cams,pairs,&matches);
      Assert::IsTrue(matches.empty());

      cams.push_back(make_unique<Camera>(""));
      cams.push_back(make_unique<Camera>(""));
      cams.push_back(make_unique<Camera>(""));
      cams.push_back(make_unique<Camera>(""));
      for(size_t camIdx = 0; camIdx < cams.size(); camIdx++)
      {
        cams[camIdx]->reserveFeatures(5,0);
        for(int i = 0; i < 5; i++)
        {
          float dummy;
          cams[camIdx]->addFeature(0.,0.,&dummy);
        }
      }

      pairs[IntPair(0,1)].matches.emplace_back(0,0);
      pairs[IntPair(1,2)].matches.emplace_back(0,0);
      pairs[IntPair(2,3)].matches.emplace_back(0,0);
      twoViewMatchesToNViewMatches(cams,pairs,&matches);
      Assert::IsTrue(matches.size() == 1);
      Assert::IsTrue(matches[0].size() == 4);
      Assert::IsTrue(matches[0].count(0) == 1);
      Assert::IsTrue(matches[0].count(1) == 1);
      Assert::IsTrue(matches[0].count(2) == 1);
      Assert::IsTrue(matches[0].count(3) == 1);

      pairs.clear();
      matches.clear();
      pairs[IntPair(0,1)].matches.emplace_back(0,0);
      pairs[IntPair(2,3)].matches.emplace_back(0,0);
      twoViewMatchesToNViewMatches(cams,pairs,&matches);
      Assert::IsTrue(matches.size() == 2);
      Assert::IsTrue(matches[0].size() == 2);
      Assert::IsTrue(matches[1].size() == 2);

      pairs.clear();
      matches.clear();
      pairs[IntPair(0,2)].matches.emplace_back(0,0);
      pairs[IntPair(1,2)].matches.emplace_back(0,0);
      twoViewMatchesToNViewMatches(cams,pairs,&matches);
      Assert::IsTrue(matches.size() == 1);
      Assert::IsTrue(matches[0].size() == 3);
      Assert::IsTrue(matches[0].count(0) == 1);
      Assert::IsTrue(matches[0].count(1) == 1);
      Assert::IsTrue(matches[0].count(2) == 1);

      pairs.clear();
      matches.clear();
      pairs[IntPair(0,1)].matches.emplace_back(0,0);
      pairs[IntPair(1,2)].matches.emplace_back(0,0);
      pairs[IntPair(0,2)].matches.emplace_back(1,0);
      twoViewMatchesToNViewMatches(cams,pairs,&matches);
      Assert::IsTrue(matches.size() == 0);

      pairs.clear();
      matches.clear();
      pairs[IntPair(0,1)].matches.emplace_back(0,0);
      pairs[IntPair(1,2)].matches.emplace_back(0,0);
      pairs[IntPair(0,2)].matches.emplace_back(0,0);
      twoViewMatchesToNViewMatches(cams,pairs,&matches);
      Assert::IsTrue(matches.size() == 1);
      Assert::IsTrue(matches[0].size() == 3);
      Assert::IsTrue(matches[0].count(0) == 1);
      Assert::IsTrue(matches[0].count(1) == 1);
      Assert::IsTrue(matches[0].count(2) == 1);
    }

    TEST_METHOD(nViewMatchesToTwoViewMatchesTest)
    {
      vector<NViewMatch> nViewMatches;
      IntPair cams(0,1);
      vector<IntPair> matches;
      vector<int> idxs;
      nViewMatchesToTwoViewMatches(nViewMatches,cams,&matches,&idxs);
      Assert::IsTrue(matches.empty());
      Assert::IsTrue(idxs.empty());

      nViewMatches.resize(4);
      nViewMatches[0].emplace(1,1);
      nViewMatches[0].emplace(2,2);
      nViewMatches[0].emplace(3,3);
      nViewMatches[1].emplace(0,4);
      nViewMatches[1].emplace(1,5);
      nViewMatches[1].emplace(2,6);
      nViewMatches[1].emplace(3,7);
      nViewMatches[2].emplace(0,8);
      nViewMatches[2].emplace(1,9);
      nViewMatchesToTwoViewMatches(nViewMatches,cams,&matches,&idxs);
      Assert::IsTrue(matches.size() == 2);
      Assert::IsTrue(idxs.size() == 2);
      Assert::IsTrue(matches[0].first == 4);
      Assert::IsTrue(matches[0].second == 5);
      Assert::IsTrue(matches[1].first == 8);
      Assert::IsTrue(matches[1].second == 9);
      Assert::IsTrue(idxs[0] == 1);
      Assert::IsTrue(idxs[1] == 2);
    }

    TEST_METHOD(reconstructPointsTest)
    {
      IntPair camsIdxs(0,1);
      StandardCamera cam0(""),cam1("");
      vector<int> nViewMatchIdxs;
      Points pts;
      reconstructPoints(camsIdxs,cam0,cam1,nViewMatchIdxs,&pts);
      Assert::IsTrue(pts.matchesToReconstruct().empty());
      Assert::IsTrue(pts.ptCoord().empty());
      Assert::IsTrue(pts.ptData().empty());

      Vector3d axis(Vector3d::Random());
      axis.normalize();
      AngleAxisd aa(5,axis);
      Matrix3d R = aa.toRotationMatrix();
      Vector3d C(Vector3d::Random());
      Matrix34d P0(Matrix34d::Identity()),P1(Matrix34d::Identity());
      P0 = generateRandomCalibration() * P0;
      P1.col(3) = -C;
      P1 = generateRandomCalibration() * R * P1;
      cam0.setParams(P0);
      cam1.setParams(P1);
      P0 = cam0.P();
      P1 = cam1.P();
      int n = 2;
      vector<Vector3d> coord(n);
      cam0.reserveFeatures(n,0);
      cam1.reserveFeatures(n,0);
      coord[0] << 0,0,100;
      coord[1] << 0,0,-100;
      for(int i = 0; i < n; i++)
      {
        float dummy;
        Vector3d proj = P0 * coord[i].homogeneous();
        Vector2d p = proj.hnormalized();
        cam0.addFeature(p(0),p(1),&dummy);
        proj = P1 * coord[i].homogeneous();
        p = proj.hnormalized();
        cam1.addFeature(p(0),p(1),&dummy);
        pts.matchesToReconstruct().emplace_back();
        pts.matchesToReconstruct().back().emplace(0,i);
        pts.matchesToReconstruct().back().emplace(1,i);
        nViewMatchIdxs.push_back(i);
      }
      pts.matchesToReconstruct()[0].emplace(2,0);
      reconstructPoints(camsIdxs,cam0,cam1,nViewMatchIdxs,&pts);
      Assert::IsTrue(pts.matchesToReconstruct().size() == 1);
      Assert::IsTrue(pts.ptCoord().size() == n-1);
      Assert::IsTrue(pts.ptData().size() == n-1);
      const double precision = 1e-10;
      Assert::IsTrue(pts.ptCoord()[0].isApprox(coord[0]));
      auto& rec = pts.ptData().back().reconstructed;
      Assert::IsTrue(rec.size() == 2);
      Assert::IsTrue(rec.count(0) == 1);
      Assert::IsTrue(rec.count(1) == 1);
      auto& toRec = pts.ptData().back().toReconstruct;
      Assert::IsTrue(toRec.size() == 1);
      Assert::IsTrue(toRec.count(2) == 1);

      ptr_vector<Camera> cams;
      vector<SplitNViewMatch> matches(1);
      for(int i = 0; i < 4; i++)
      {
        cams.push_back(make_unique<StandardCamera>(""));
        cams[i]->setParams(generateRandomProjection());
        Vector3d axis(Vector3d::Random());
        axis.normalize();
        AngleAxisd aa(5,axis);
        cams[i]->setRotation(aa.toRotationMatrix());
        cams[i]->reserveFeatures(n,0);
        for(int iPt = 0; iPt < n; iPt++)
        {
          float dummy;
          Vector3d proj = cams[i]->P() * coord[iPt].homogeneous();
          Vector2d p = proj.hnormalized();
          cams[i]->addFeature(p(0),p(1),&dummy);
          matches[0].observedPart.emplace(i,0);
        }
      }
      matches[0].unobservedPart.emplace(45,0);
      reconstructPoints(cams,matches,&pts);
      Assert::IsTrue(pts.ptCoord().back().isApprox(coord[0]));
      Assert::IsTrue(pts.ptData().back().reconstructed.size() == cams.size());
      Assert::IsTrue(pts.ptData().back().toReconstruct.size() == 1);
    }

    TEST_METHOD(triangulateTest)
    {
      Matrix34d P0(generateRandomProjection()),P1(generateRandomProjection());
      Vector3d pt(Vector3d::Random());
      Vector3d proj = P0 * pt.homogeneous();
      Vector2d key0 = proj.hnormalized();
      proj = P1 * pt.homogeneous();
      Vector2d key1 = proj.hnormalized();

      Vector3d _pt;
      triangulate(P0,P1,key0,key1,&_pt);

      Assert::IsTrue(pt.isApprox(_pt));

      int n = 5;
      vector<Matrix34d> Ps(n);
      vector<int> PsToUse(n);
      vector<Vector2d> keys(n);
      for(int i = 0; i < n; i++)
      {
        Ps[i] = generateRandomProjection();
        proj = Ps[i] * pt.homogeneous();
        keys[i] = proj.hnormalized();
        PsToUse[i] = i;
      }
      triangulate(Ps,PsToUse,keys,&_pt);

      Assert::IsTrue(pt.isApprox(_pt));
    }

    TEST_METHOD(isInFrontTest)
    {
      Matrix3d K(generateRandomCalibration()),R(Matrix3d::Identity());
      Vector3d C(Vector3d::Random()),pt;
      pt = C;
      pt(2) += 100;
      Matrix34d P(Matrix34d::Identity());
      P.col(3) = -C;
      P = K*R*P;
      Assert::IsTrue(isInFront(P,pt));
      P *= 4;
      Assert::IsTrue(isInFront(P,pt));
      P *= -1;
      Assert::IsTrue(isInFront(P,pt));

      AngleAxisd aa(DEG2RAD(180),Vector3d::UnitX());
      R = aa.toRotationMatrix();
      P.col(3) = -C;
      P = K*R*P;
      Assert::IsFalse(isInFront(P,pt));
      P *= 4;
      Assert::IsFalse(isInFront(P,pt));
      P *= -1;
      Assert::IsFalse(isInFront(P,pt));
    }

    TEST_METHOD(extractCandidateNewPointsTest)
    {
      int minObservingCams = 2;
      // Do not test angles, that is done in a different test
      double angleThresh = 0.;
      uset<int> reconstructedCams;
      ptr_vector<Camera> cams;
      for(int i = 0; i < 5; i++)
      {
        cams.push_back(make_unique<StandardCamera>(""));
        cams[i]->setParams(generateRandomProjection()); 
        cams[i]->reserveFeatures(2,0);
        for(int j = 0; j < 2; j++)
        {
          Vector2d p = Vector2d::Random();
          float dummy;
          cams[i]->addFeature(p(0),p(1),&dummy);
        }
      }
      vector<NViewMatch> matches;
      vector<SplitNViewMatch> extracted;
      extractCandidateNewPoints(minObservingCams,
        angleThresh,reconstructedCams,cams,&matches,&extracted);
      Assert::IsTrue(extracted.empty());

      matches.resize(2);
      matches[0].emplace(0,0);
      matches[0].emplace(1,0);
      matches[0].emplace(2,0);
      matches[1].emplace(1,0);
      matches[1].emplace(2,0);
      matches[1].emplace(3,0);
      extractCandidateNewPoints(minObservingCams,angleThresh,
        reconstructedCams,cams,&matches,&extracted);
      Assert::IsTrue(matches.size() == 2);
      Assert::IsTrue(extracted.empty());

      reconstructedCams.insert(0);
      reconstructedCams.insert(1);
      minObservingCams = 3;
      extractCandidateNewPoints(minObservingCams,angleThresh,
        reconstructedCams,cams,&matches,&extracted);
      Assert::IsTrue(matches.size() == 2);
      Assert::IsTrue(extracted.empty());

      minObservingCams = 2;
      extractCandidateNewPoints(minObservingCams,angleThresh,
        reconstructedCams,cams,&matches,&extracted);
      Assert::IsTrue(matches.size() == 1);
      Assert::IsTrue(extracted.size() == 1);
      Assert::IsTrue(extracted[0].observedPart.size() == 2);
      Assert::IsTrue(extracted[0].observedPart.count(0) == 1);
      Assert::IsTrue(extracted[0].observedPart.count(1) == 1);
      Assert::IsTrue(extracted[0].unobservedPart.size() == 1);
      Assert::IsTrue(extracted[0].unobservedPart.count(2) == 1);
    }

    TEST_METHOD(findCamToSceneMatchesTest)
    {
      uset<int> ignore;
      int n = 0;
      Points pts;
      vector<vector<IntPair>> matches;
      findCamToSceneMatches(ignore,n,pts,&matches);
      Assert::IsTrue(matches.empty());

      n = 1;
      findCamToSceneMatches(ignore,n,pts,&matches);
      Assert::IsTrue(matches.size() == 1);
      Assert::IsTrue(matches[0].empty());

      matches.clear();
      vector<Vector3d> coord(5);
      vector<SplitNViewMatch> views(5);
      pts.addPoints(coord,views);
      findCamToSceneMatches(ignore,n,pts,&matches);
      Assert::IsTrue(matches.size() == 1);
      Assert::IsTrue(matches[0].empty());

      matches.clear();
      n = 3;
      views[0].observedPart.emplace(0,0);
      views[0].unobservedPart.emplace(1,0);
      views[0].unobservedPart.emplace(2,0);
      pts.addPoints(coord,views);
      findCamToSceneMatches(ignore,n,pts,&matches);
      Assert::IsTrue(matches.size() == 3);
      Assert::IsTrue(matches[0].empty());
      Assert::IsTrue(matches[1].size() == 1);
      Assert::IsTrue(matches[2].size() == 1);

      matches.clear();
      ignore.insert(1);
      findCamToSceneMatches(ignore,n,pts,&matches);
      Assert::IsTrue(matches.size() == 3);
      Assert::IsTrue(matches[0].empty());
      Assert::IsTrue(matches[1].empty());
      Assert::IsTrue(matches[2].size() == 1);
    }

    TEST_METHOD(removeHighReprojErrorPointsTest)
    {
      double avgReprojErrThresh = 1;
      ptr_vector<Camera> cams;
      Points pts;

      for(int i = 0; i < 2; i++)
      {
        cams.push_back(make_unique<StandardCamera>(""));
        cams[i]->setParams(generateRandomProjection());
        cams[i]->reserveFeatures(2,0);
      }

      vector<Vector3d> coord(2);
      vector<SplitNViewMatch> views(2);
      for(int i = 0; i < (int)coord.size(); i++)
      {
        coord[i] = Vector3d::Random();
        views[i].observedPart.emplace(0,i);
        views[i].observedPart.emplace(1,i);
      }
      float dummy;
      Vector2d p;
      cams[0]->project(coord[0],&p);
      cams[0]->addFeature(p(0),p(1),&dummy);
      cams[1]->project(coord[0],&p);
      cams[1]->addFeature(p(0),p(1),&dummy);
      cams[0]->project(coord[1],&p);
      cams[0]->addFeature(p(0),p(1),&dummy);
      cams[1]->project(coord[1],&p);
      p(0) += 100;
      cams[1]->addFeature(p(0),p(1),&dummy);
      views[1].unobservedPart.emplace(11,1);
      pts.addPoints(coord,views);

      removeHighReprojErrorPoints(avgReprojErrThresh,cams,&pts);
      Assert::IsTrue(pts.numPts() == 1);
      Assert::IsTrue(pts.ptData()[0].toReconstruct.empty());
    }

    TEST_METHOD(isWellConditionedTest)
    {
      double rayAngleThresh = 2; 
      ptr_vector<Camera> cams;
      NViewMatch views;
      Assert::IsFalse(isWellConditioned(rayAngleThresh,cams,views));

      Vector3d pt(Vector3d::Random());
      for(int i = 0; i < 2; i++)
      {
        cams.push_back(make_unique<StandardCamera>(""));
        cams[i]->setParams(generateRandomProjection());
        cams[i]->reserveFeatures(1,0);
        Vector2d p;
        cams[i]->project(pt,&p);
        float dummy;
        cams[i]->addFeature(p(0),p(1),&dummy);
        views.emplace(i,0);
      }

      Vector3d ray1 = pt - cams[0]->C();
      Vector3d ray2 = pt - cams[1]->C();
      double angle = RAD2DEG(acos(ray1.dot(ray2) / (ray1.norm() * ray2.norm())));

      rayAngleThresh = angle-1;
      Assert::IsTrue(isWellConditioned(rayAngleThresh,cams,views));

      rayAngleThresh = angle+1;
      Assert::IsFalse(isWellConditioned(rayAngleThresh,cams,views));

      double angle2 = RAD2DEG(computeRayAngle(*cams[0],0,*cams[1],0));
      Assert::IsTrue(abs(angle-angle2) < 1e-10);
    }

  private:

	};
}