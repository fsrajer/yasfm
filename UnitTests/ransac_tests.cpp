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
      Assert::IsTrue(sufficientNumberOfRounds(50,100,5,.95) == 104);
		}

	};
}