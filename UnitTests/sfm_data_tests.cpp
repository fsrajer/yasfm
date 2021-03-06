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
    string keysDir = joinPaths(dir,"keys");

    T cam(fn,keysDir);
    Assert::IsTrue(cam.imgFilename().compare(fn) == 0);
    Assert::IsTrue(cam.imgHeight() == 850);
    Assert::IsTrue(cam.imgWidth() == 1100);

    // reserveFeatures
    int nFeats = 4;
    int descrDim = 128;
    cam.resizeFeatures(nFeats,descrDim);

    double x = 3,y = 5;
    // A sift descriptor - important to be normalizable so that no element
    // is higher than 0.5
    VectorXf descr;
    descr.resize(descrDim);
    descr << 4,5,16,4,11,15,12,31,24,15,11,0,0,0,4,57,14,0,0,7
      ,9,1,1,24,0,0,0,2,1,0,0,0,8,34,149,112,10,2,0,2
      ,149,144,72,3,3,0,0,10,45,7,0,51,58,1,0,6,0,0,0,9
      ,5,0,0,0,50,24,53,92,17,0,5,127,149,39,8,2,6,13,23,149
      ,37,2,0,28,79,103,14,33,0,0,0,3,5,6,0,0,34,1,1,5
      ,14,9,42,149,20,0,0,0,0,16,40,149,1,0,0,0,4,105,34,17
      ,0,0,0,0,0,4,1,0;
    descr.normalize();
    for(int i = 0; i < nFeats; i++)
      cam.setFeature(i,x,y,0,0,&descr(0));

    Assert::IsTrue(cam.keys().size() == nFeats);
    Assert::IsTrue(cam.descr().cols() == nFeats);
    Assert::IsTrue(cam.descr().rows() == descrDim);
    Assert::AreEqual(x,cam.key(1)(0));
    Assert::AreEqual(y,cam.key(1)(1));
    Assert::IsTrue(descr.isApprox(cam.descr().col(1)));

    // descriptor writing and loading
    cam.writeFeatures();
    cam.clearDescriptors();
    cam.readFeatures(Camera::ReadDescriptors);
    Assert::IsTrue(cam.descr().col(1).isApprox(descr,1e-2f));

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