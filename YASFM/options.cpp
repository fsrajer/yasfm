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

bool OptionsSIFTGPU::isSetMaxWorkingDimension() const { return maxWorkingDimension >= 0; }
bool OptionsSIFTGPU::isSetMaxOctaves() const { return maxOctaves >= 0; }
bool OptionsSIFTGPU::isSetDogLevelsInAnOctave() const { return dogLevelsInAnOctave >= 0; }
bool OptionsSIFTGPU::isSetDogThresh() const { return dogThresh >= 0; }
bool OptionsSIFTGPU::isSetEdgeThresh() const { return edgeThresh >= 0; }

bool OptionsFLANN::filterByRatio() const { return ratioThresh >= 0.f; }

}