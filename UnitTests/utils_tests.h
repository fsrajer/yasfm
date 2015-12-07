#include <vector>
#include <string>

#include "Eigen/Dense"
#include "utils.h"

using namespace yasfm;
using std::vector;
using std::string;
using Eigen::Matrix3d;

namespace yasfm_tests
{

void assertVectorEquality(const vector<int>& v1,const vector<int>& v2);
void assertStringEquality(const string& s1,const string& s2);
Matrix3d generateRandomRotation();
Matrix3d generateRandomCalibration();
Matrix34d generateRandomProjection();

template <typename T> int sgn(T val) {
  return (T(0) < val) - (val < T(0));
}

}