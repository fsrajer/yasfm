#include "stdafx.h"
#include "CppUnitTest.h"
#include "utils_io.h"

using namespace Microsoft::VisualStudio::CppUnitTestFramework;

#define YASFM_UNIT_TESTS_DIR "../UnitTests/test_dataset"

namespace yasfm_tests
{
	TEST_CLASS(utils_io_tests)
	{
	public:
		
    TEST_METHOD(listImgFilenamesTest)
		{
      vector<string> fns;
      listImgFilenames(YASFM_UNIT_TESTS_DIR,&fns);
      Assert::IsTrue(fns.size() == 3);
      Assert::IsTrue(fns[0].compare("test0.JPG") == 0);
      Assert::IsTrue(fns[1].compare("test1.JPG") == 0);
      Assert::IsTrue(fns[2].compare("test2.JPG") == 0);
    }

    TEST_METHOD(listFilenamesTest)
    {
      vector<string> fns;
      listFilenames(YASFM_UNIT_TESTS_DIR,&fns);
      Assert::IsTrue(fns.size() == 4);
      Assert::IsTrue(fns[0].compare("sample_rubbish.txt") == 0);
      Assert::IsTrue(fns[1].compare("test0.JPG") == 0);
      Assert::IsTrue(fns[2].compare("test1.JPG") == 0);
      Assert::IsTrue(fns[3].compare("test2.JPG") == 0);

      fns.clear();
      vector<string> ext(1);
      ext[0] = "txt";
      listFilenames(YASFM_UNIT_TESTS_DIR,ext,&fns);
      Assert::IsTrue(fns.size() == 1);
      Assert::IsTrue(fns[0].compare("sample_rubbish.txt") == 0);
    }

    TEST_METHOD(getImgDimsTest)
    {
      string fn = joinPaths(YASFM_UNIT_TESTS_DIR,"test0.JPG");
      int w,h;
      getImgDims(fn,&w,&h);
      Assert::IsTrue(w == 1100);
      Assert::IsTrue(h == 850);
    }

    TEST_METHOD(findFocalLengthInEXIFTest)
    {
      string fn = joinPaths(YASFM_UNIT_TESTS_DIR,"test0.JPG");
      string db = "../resources/camera_ccd_widths.txt";
      Camera cam(fn);
      double f = findFocalLengthInEXIF(db,cam,false);
      Assert::IsTrue(f == 2441.0003935338095);
    }

  private:

	};
}