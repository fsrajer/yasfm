/*
* Filip Srajer
* filip.srajer (at) fel.cvut.cz
* Center for Machine Perception
* Czech Technical University in Prague
*
* This software is under construction.
* 02/2015
*/

#include "options.h"

namespace yasfm
{

OptionsSIFT::OptionsSIFT() 
  : maxWorkingDimension_(-1),firstOctave_(-1), maxOctaves_(-1),
  dogLevelsInAnOctave_(-1),dogThresh_(-1), edgeThresh_(-1), detectUprightSIFT_(false)
{
}

bool OptionsSIFT::isSetMaxWorkingDimension() const { return maxWorkingDimension_ >= 0; }
bool OptionsSIFT::isSetMaxOctaves() const { return maxOctaves_ >= 0; }
bool OptionsSIFT::isSetDogLevelsInAnOctave() const { return dogLevelsInAnOctave_ >= 0; }
bool OptionsSIFT::isSetDogThresh() const { return dogThresh_ >= 0; }
bool OptionsSIFT::isSetEdgeThresh() const { return edgeThresh_ >= 0; }

OptionsFLANN::OptionsFLANN() 
  : indexParams_(flann::KDTreeIndexParams()), searchParams_(),
  ratioThresh_(0.8f), onlyUniques_(true)
{
}

bool OptionsFLANN::filterByRatio() const { return ratioThresh_ >= 0; }

OptionsRANSAC::OptionsRANSAC(int ransacRounds,double errorThresh,
  int minInliers)
  : ransacRounds_(ransacRounds),errorThresh_(errorThresh),
  minInliers_(minInliers),inliersEnough_(1.)
{
}

OptionsRANSAC::OptionsRANSAC(int ransacRounds,double errorThresh,
  int minInliers,double inliersEnough)
  : ransacRounds_(ransacRounds),errorThresh_(errorThresh),
  minInliers_(minInliers),inliersEnough_(inliersEnough)
{
}

Options::Options() 
  : ccdDBFilename_("../resources/camera_ccd_widths.txt"),sift_(),
  matchingFLANN_(),geometricVerification_(2048,4.,16),absolutePose_(4096,4.,16),
  relativePose_(512,1.25,10),verbosityLevel_(1),minNumMatches_(16),
  wellMatchedCamsFactor_(0.75),minNumCam2SceneMatches_(16),
  pointsReprojErrorThresh_(16),rayAngleThresh_(2.)
{
}

}