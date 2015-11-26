#include "stdafx.h"
#include "CppUnitTest.h"

#include "ransac.h"
#include "utils_tests.h"

using namespace Microsoft::VisualStudio::CppUnitTestFramework;
using namespace yasfm;

namespace UnitTests
{
	TEST_CLASS(ransac_tests)
	{
	public:
		
    TEST_METHOD(sufficientNumberOfRoundsTest)
		{
      Assert::AreEqual(104,sufficientNumberOfRounds(50,100,5,.95));
      Assert::AreEqual(INT_MAX,sufficientNumberOfRounds(50,100,5,1.));
      Assert::AreEqual(INT_MAX,sufficientNumberOfRounds(1,100000,5,.95));
		}

	};
}