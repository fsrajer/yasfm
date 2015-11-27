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

void OptionsSIFTGPU::write(ostream& file) const
{
  file << " maxWorkingDimension: " << maxWorkingDimension << "\n";
  file << " firstOctave: " << firstOctave << "\n";
  file << " maxOctaves: " << maxOctaves << "\n";
  file << " dogLevelsInAnOctave: " << dogLevelsInAnOctave << "\n";
  file << " dogThresh: " << dogThresh << "\n";
  file << " edgeThresh: " << edgeThresh << "\n";
  file << " detectUprightSIFT: " << detectUprightSIFT << "\n";
  file << " verbosityLevel: " << verbosityLevel << "\n";
}

bool OptionsFLANN::filterByRatio() const { return ratioThresh >= 0.f; }

void OptionsFLANN::write(ostream& file) const
{
  file << " indexParams:\n";
  for(const auto& entry : indexParams)
  {
    file << "  " << entry.first << ": " << entry.second << "\n";
  }
  file << " searchParams:\n";
  file << "  checks: " << searchParams.checks << "\n";
  file << "  eps: " << searchParams.eps << "\n";
  file << "  sorted: " << searchParams.sorted << "\n";
  file << "  max_neighbors: " << searchParams.max_neighbors << "\n";
  file << "  cores: " << searchParams.cores << "\n";
  file << " ratioThresh: " << ratioThresh << "\n";
  file << " onlyUniques: " << onlyUniques << "\n";
  file << " verbose: " << verbose << "\n";
}

void OptionsRANSAC::write(ostream& file) const
{
  file << " maxRounds: " << maxRounds << "\n";
  file << " errorThresh: " << errorThresh << "\n";
  file << " minInliers: " << minInliers << "\n";
  file << " confidence: " << confidence << "\n";
}

void OptionsBundleAdjustment::write(ostream& file) const
{
  file << " solverOptions (only some of them):\n";
  file << "  max_num_iterations: " << solverOptions.max_num_iterations << "\n";
  file << "  num_threads: " << solverOptions.num_threads << "\n";
  file << "  function_tolerance: " << solverOptions.function_tolerance << "\n";
  file << "  parameter_tolerance: " << solverOptions.parameter_tolerance << "\n";
  file << "  gradient_tolerance: " << solverOptions.gradient_tolerance << "\n";
  file << "  minimizer_type: " << solverOptions.minimizer_type << "\n";
  file << "  linear_solver_type: " << solverOptions.linear_solver_type << "\n";
  file << " robustify: " << robustify << "\n";
}

}