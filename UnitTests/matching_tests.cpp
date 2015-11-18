#include "stdafx.h"
#include "CppUnitTest.h"

#include "matching.h"

using namespace Microsoft::VisualStudio::CppUnitTestFramework;

namespace yasfm_tests
{
	TEST_CLASS(matching_tests)
	{
	public:

    TEST_METHOD(removePoorlyMatchedPairsTest)
    {
      int minMatches = 1;
      pair_umap<CameraPair> pairs;
      removePoorlyMatchedPairs(minMatches,&pairs); // empty set test
      Assert::IsTrue(pairs.empty());

      IntPair pair0(0,0),pair1(0,1),pair2(0,2);
      CameraPair p1,p2,p3;
      p1.matches.resize(minMatches - 1);
      p1.dists.resize(minMatches - 1);
      p2.matches.resize(minMatches);
      p2.dists.resize(minMatches);
      p3.matches.resize(minMatches + 1);
      p3.dists.resize(minMatches + 1);

      pairs.emplace(pair0,p1);
      pairs.emplace(pair1,p2);
      pairs.emplace(pair2,p3);

      removePoorlyMatchedPairs(minMatches,&pairs);
      Assert::IsTrue(pairs.size() == 2);
      Assert::IsTrue(pairs.count(pair1) >= 0);
      Assert::IsTrue(pairs.count(pair2) >= 0);
    }

    TEST_METHOD(findUniqueMatchesTest)
    {
      vector<IntPair> matches;
      size_t numFeats2 = 0;
      vector<bool> unique;
      findUniqueMatches(matches,numFeats2,&unique);
      Assert::IsTrue(unique.empty());

      numFeats2 = 10;
      matches.resize(3);
      matches[0].first = 0;
      matches[1].first = 1;
      matches[2].first = 2;
      matches[0].second = 0;
      matches[1].second = 1;
      matches[2].second = 0;
      findUniqueMatches(matches,numFeats2,&unique);
      Assert::IsTrue(unique.size() == 3);
      Assert::IsFalse(unique[0]);
      Assert::IsTrue(unique[1]);
      Assert::IsFalse(unique[2]);
    }

	};
}