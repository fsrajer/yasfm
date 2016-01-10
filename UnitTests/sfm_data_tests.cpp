#include "stdafx.h"
#include "CppUnitTest.h"

#include "sfm_data.h"
#include "utils_tests.h"
#include "standard_camera.h"
#include "standard_camera_radial.h"

using namespace Microsoft::VisualStudio::CppUnitTestFramework;
using namespace yasfm;
using Eigen::VectorXf;

namespace yasfm_tests
{
TEST_CLASS(sfm_data_tests)
{
public:

  TEST_METHOD(addCameraTest)
  {
    Dataset data("../UnitTests/test_dataset");
    data.addCamera<StandardCamera>("test0.JPG");
    data.addCamera<StandardCamera>("test1.JPG");

    string fn0 = joinPaths(data.dir(),"test0.JPG");
    string fn1 = joinPaths(data.dir(),"test1.JPG");

    Assert::AreEqual(data.numCams(),2);
    Assert::IsTrue(data.cam(0).imgFilename().compare(fn0) == 0);
    Assert::IsTrue(data.cam(1).imgFilename().compare(fn1) == 0);
  }

  TEST_METHOD(addCamerasTest)
  {
    Dataset data("../UnitTests/test_dataset");
    data.addCameras<StandardCamera>("");

    Assert::AreEqual(data.numCams(),3);

    string fn0 = joinPaths(data.dir(),"test0.JPG");
    string fn1 = joinPaths(data.dir(),"test1.JPG");
    string fn2 = joinPaths(data.dir(),"test2.JPG");
    Assert::IsTrue(data.cam(0).imgFilename().compare(fn0) == 0);
    Assert::IsTrue(data.cam(1).imgFilename().compare(fn1) == 0);
    Assert::IsTrue(data.cam(2).imgFilename().compare(fn2) == 0);
  }

  TEST_METHOD(markCamAsReconstructedTest)
  {
    Dataset data("../UnitTests/test_dataset");
    data.addCameras<StandardCamera>("");

    data.markCamAsReconstructed(1);

    Assert::IsTrue(data.reconstructedCams().size() == 1);
    Assert::IsTrue(data.reconstructedCams().count(1) == 1);
  }

  TEST_METHOD(PointsTest)
  {
    // addPoints - overload1
    Points pts;
    auto& matches = pts.matchesToReconstruct();
    matches.resize(2);
    matches[0].emplace(0,1);
    matches[0].emplace(1,1);
    matches[0].emplace(2,1);
    matches[1].emplace(1,1);
    matches[1].emplace(2,1);
    vector<Vector3d> coord(1);
    vector<Vector3uc> colors(1);
    coord[0] = Vector3d::Random();
    IntPair camsIdxs(0,1);
    vector<int> matchesIdxs(1);
    matchesIdxs[0] = 0;

    pts.addPoints(camsIdxs,matchesIdxs,coord,colors);

    Assert::IsTrue(pts.matchesToReconstruct().size() == 1);
    Assert::IsTrue(pts.numPtsAll() == 1);
    Assert::IsTrue(pts.ptCoord()[0] == coord[0]);
    auto& rec = pts.ptData()[0].reconstructed;
    auto& toRec = pts.ptData()[0].toReconstruct;
    Assert::IsTrue(rec.size() == 2);
    Assert::IsTrue(rec.count(camsIdxs.first) == 1);
    Assert::IsTrue(rec.count(camsIdxs.second) == 1);
    Assert::IsTrue(toRec.size() == 1);
    Assert::IsTrue(toRec.count(2) == 1);

    // addPoints - overload2
    coord[0] = Vector3d::Random();
    vector<SplitNViewMatch> views(1);
    views[0].observedPart.emplace(4,4);
    views[0].unobservedPart.emplace(5,5);

    pts.addPoints(coord,colors,views);

    Assert::IsTrue(pts.numPtsAll() == 2);
    Assert::IsTrue(pts.ptCoord()[1] == coord[0]);
    auto& rec2 = pts.ptData()[1].reconstructed;
    auto& toRec2 = pts.ptData()[1].toReconstruct;
    Assert::IsTrue(rec2.size() == 1);
    Assert::IsTrue(rec2.count(4) == 1);
    Assert::IsTrue(toRec2.size() == 1);
    Assert::IsTrue(toRec2.count(5) == 1);

    // removePoints
    vector<bool> keep(2,true);
    keep[0] = false;

    pts.removePointsViews(keep);

    Assert::IsTrue(pts.numPtsAlive() == 1);
    auto& rec3 = pts.ptData()[0].reconstructed;
    auto& toRec3 = pts.ptData()[0].toReconstruct;
    Assert::IsTrue(rec3.size() == 0);
    Assert::IsTrue(toRec3.size() == 0);

    pts.removePointsViews(keep);
    Assert::IsTrue(pts.numPtsAlive() == 1);
    Assert::IsTrue(rec3.size() == 0);
    Assert::IsTrue(toRec3.size() == 0);

    // markCamAsReconstructed
    pts.markCamAsReconstructed(5);
    auto& rec4 = pts.ptData()[1].reconstructed;
    auto& toRec4 = pts.ptData()[1].toReconstruct;
    Assert::IsTrue(rec4.size() == 2);
    Assert::IsTrue(rec4.count(4) == 1);
    Assert::IsTrue(rec4.count(5) == 1);
    Assert::IsTrue(toRec4.size() == 0);
  }

  TEST_METHOD(StandardCameraTest)
  {
    testCameraFuctionality<StandardCamera>();
    testCameraRotationSetterAndGetter<StandardCamera>();
    testCameraCSetterAndGetter<StandardCamera>();
    testCameraPoseGetter<StandardCamera>();
  }

  TEST_METHOD(StandardCameraRadialTest)
  {
    testCameraFuctionality<StandardCameraRadial>();
    testCameraRotationSetterAndGetter<StandardCameraRadial>();
    testCameraCSetterAndGetter<StandardCameraRadial>();
    testCameraPoseGetter<StandardCameraRadial>();
  }

  template<class T>
  void testCameraFuctionality()
  {
    // Constructor
    string dir("../UnitTests/test_dataset");
    string fn = joinPaths(dir,"test0.JPG");

    T cam(fn);
    Assert::IsTrue(cam.imgFilename().compare(fn) == 0);
    Assert::IsTrue(cam.imgHeight() == 850);
    Assert::IsTrue(cam.imgWidth() == 1100);

    // reserveFeatures
    int nFeats = 4;
    int descrDim = 5;
    cam.resizeFeatures(nFeats,descrDim);

    double x = 3,y = 5;
    VectorXf descr(VectorXf::Random(descrDim));
    for(int i = 0; i < nFeats; i++)
      cam.setFeature(i,x,y,0,0,&descr(0));

    Assert::IsTrue(cam.keys().size() == nFeats);
    Assert::IsTrue(cam.descr().cols() == nFeats);
    Assert::IsTrue(cam.descr().rows() == descrDim);
    Assert::AreEqual(x,cam.key(1)(0));
    Assert::AreEqual(y,cam.key(1)(1));
    Assert::IsTrue(descr.isApprox(cam.descr().col(1)));

    // copy
    unique_ptr<Camera> pcam2(cam.clone());
    Assert::IsTrue(instanceof<T,Camera>(&(*pcam2)));
  }

  template<class T>
  void testCameraPSetterAndGetter()
  {
    Matrix34d P = generateRandomProjection();
    cam.setParams(P);
    Matrix34d _P = cam.P();
    P /= P(2,3);
    _P /= _P(2,3);
    Assert::IsTrue(P.isApprox(_P,1e-8));
  }

  template<class T>
  void testCameraRotationSetterAndGetter()
  {
    T cam;
    Matrix3d R = generateRandomRotation();
    cam.setRotation(R);
    Matrix3d _R = cam.R();
    Assert::IsTrue(R.isApprox(_R,1e-8));
  }

  template<class T>
  void testCameraCSetterAndGetter()
  {
    T cam;
    Vector3d C = Vector3d::Random();
    cam.setC(C);
    Assert::IsTrue(C.isApprox(cam.C(),1e-8));
  }

  template<class T>
  void testCameraPoseGetter()
  {
    const double cPrecision = 1e-8;
    T cam;
    Matrix3d R = generateRandomRotation();
    Vector3d C = Vector3d::Random();
    cam.setRotation(R);
    cam.setC(C);
    Matrix34d Rt = cam.pose();
    Matrix3d _R = Rt.leftCols(3);
    Assert::IsTrue(R.isApprox(_R,cPrecision));
    Vector3d _C = -R.transpose()*Rt.rightCols(1);
    Assert::IsTrue(C.isApprox(_C,cPrecision));
  }

  template<typename B,typename T>
  inline bool instanceof(const T *ptr) {
    return dynamic_cast<const B*>(ptr) != nullptr;
  }

private:

};
}