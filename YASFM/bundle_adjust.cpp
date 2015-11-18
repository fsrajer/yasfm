/*
* Filip Srajer
* filip.srajer (at) fel.cvut.cz
* Center for Machine Perception
* Czech Technical University in Prague
*
* This software is under construction.
* 05/2015
*/

#include "bundle_adjust.h"

#include <iostream>

#include "ceres/ceres.h"

using std::cerr;
using std::cout;

namespace yasfm
{

void bundleAdjust(const OptionsBundleAdjustment& opt,ptr_vector<Camera> *pcams,Points *ppts)
{
  vector<bool> constantCams(pcams->size(),false),constantPts(ppts->numPts(),false);
  bundleAdjust(opt,constantCams,constantPts,pcams,ppts);
}

void bundleAdjustCams(const OptionsBundleAdjustment& opt,ptr_vector<Camera> *pcams,Points *ppts)
{
  vector<bool> constantCams(pcams->size(),false),constantPts(ppts->numPts(),true);
  bundleAdjust(opt,constantCams,constantPts,pcams,ppts);
}

void bundleAdjustPoints(const OptionsBundleAdjustment& opt,ptr_vector<Camera> *pcams,Points *ppts)
{
  vector<bool> constantCams(pcams->size(),true),constantPts(ppts->numPts(),false);
  bundleAdjust(opt,constantCams,constantPts,pcams,ppts);
}

void bundleAdjust(const OptionsBundleAdjustment& opt,const vector<bool>& constantCams,
  const vector<bool>& constantPoints,ptr_vector<Camera> *pcams,Points *ppts)
{
  cout << "Preparing for bundle adjustment\n";
  auto& cams = *pcams;
  auto& pts = *ppts;

  vector<vector<double>> camParams(cams.size());
  vector<bool> camParamsUsed(cams.size(),false);

  ceres::Problem problem;
  for(int ptIdx = 0; ptIdx < pts.numPts(); ptIdx++)
  {
    auto& projections = pts.ptData()[ptIdx].reconstructed;
    for(const auto& camKey : projections)
    {
      int camIdx = camKey.first;
      auto& cam = *cams[camIdx];
      if(!camParamsUsed[camIdx])
      {
        cam.params(&camParams[camIdx]);
        camParamsUsed[camIdx] = true;
      }

      int keyIdx = camKey.second;
      ceres::CostFunction *costFunction = cam.costFunction(keyIdx);

      // NULL specifies squared loss
      ceres::LossFunction *lossFunction = opt.robustify ? new ceres::HuberLoss(1.0) : NULL;
      
      problem.AddResidualBlock(costFunction,
        lossFunction,
        &camParams[camIdx][0],
        pts.ptCoord(ptIdx));
    }
    if(constantPoints[ptIdx])
      problem.SetParameterBlockConstant(pts.ptCoord(ptIdx));
  }

  for(size_t camIdx = 0; camIdx < cams.size(); camIdx++)
  {
    if(camParamsUsed[camIdx])
    {
      ceres::CostFunction *costFunction = cams[camIdx]->constraintsCostFunction();

      problem.AddResidualBlock(costFunction,
        NULL,
        &camParams[camIdx][0]);

      if(constantCams[camIdx])
        problem.SetParameterBlockConstant(&camParams[camIdx][0]);
    }
  }

  cout << "Running bundle adjustment\n";
  ceres::Solver::Summary summary;
  ceres::Solve(opt.solverOptions,&problem,&summary);
  //std::cout << summary.FullReport() << "\n";

  for(size_t camIdx = 0; camIdx < cams.size(); camIdx++)
  {
    if(camParamsUsed[camIdx])
    {
      cams[camIdx]->setParams(camParams[camIdx]);
    }
  }
}

void bundleAdjustOneCam(const OptionsBundleAdjustment& opt,int camIdx,Camera *pcam,
  Points *ppts)
{
  vector<bool> constantPts(ppts->numPts(),false);
  bundleAdjustOneCam(opt,camIdx,constantPts,pcam,ppts);
}

void bundleAdjustOneCam(const OptionsBundleAdjustment& opt,
  int camIdx,const vector<bool>& constantPoints,Camera *pcam,Points *ppts)
{
  cout << "Preparing for bundle adjustment\n";
  auto& cam = *pcam;
  auto& pts = *ppts;

  vector<double> camParams;
  cam.params(&camParams);

  ceres::Problem problem;
  for(int ptIdx = 0; ptIdx < pts.numPts(); ptIdx++)
  {
    auto& projections = pts.ptData()[ptIdx].reconstructed;
    try
    {
      int keyIdx = projections.at(camIdx);
      ceres::CostFunction *costFunction = cam.costFunction(keyIdx);

      // NULL specifies squared loss
      ceres::LossFunction *lossFunction = opt.robustify ? new ceres::HuberLoss(1.0) : NULL;

      problem.AddResidualBlock(costFunction,
        lossFunction,
        &camParams[0],
        pts.ptCoord(ptIdx));

    } catch(const std::out_of_range&)
    {
      // This point is not seen by our camera.
      continue;
    }

    if(constantPoints[ptIdx])
      problem.SetParameterBlockConstant(pts.ptCoord(ptIdx));
  }

  {
    ceres::CostFunction *costFunction = cam.constraintsCostFunction();
    
    problem.AddResidualBlock(costFunction,
      NULL,
      &camParams[0]);
  }

  cout << "Running bundle adjustment\n";
  ceres::Solver::Summary summary;
  ceres::Solve(opt.solverOptions,&problem,&summary);
  //std::cout << summary.FullReport() << "\n";

  cam.setParams(camParams);
}

} // namespace yasfm

namespace
{

} // namespace