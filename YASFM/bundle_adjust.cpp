#include "bundle_adjust.h"

#include <iostream>

using std::cerr;
using std::cout;

namespace yasfm
{

void bundleAdjust(const OptionsBundleAdjustment& opt,ptr_vector<Camera> *pcams,
  vector<Point> *ppts)
{
  vector<bool> constantCams(pcams->size(),false),constantPts(ppts->size(),false);
  bundleAdjust(opt,constantCams,constantPts,pcams,ppts);
}

void bundleAdjustCams(const OptionsBundleAdjustment& opt,ptr_vector<Camera> *pcams,
  vector<Point> *ppts)
{
  vector<bool> constantCams(pcams->size(),false),constantPts(ppts->size(),true);
  bundleAdjust(opt,constantCams,constantPts,pcams,ppts);
}

void bundleAdjustPoints(const OptionsBundleAdjustment& opt,ptr_vector<Camera> *pcams,
  vector<Point> *ppts)
{
  vector<bool> constantCams(pcams->size(),true),constantPts(ppts->size(),false);
  bundleAdjust(opt,constantCams,constantPts,pcams,ppts);
}

void bundleAdjust(const OptionsBundleAdjustment& opt,const vector<bool>& constantCams,
  const vector<bool>& constantPoints,ptr_vector<Camera> *pcams,vector<Point> *ppts)
{
  auto& cams = *pcams;
  auto& pts = *ppts;
  bool robustify = opt.get<bool>("robustify");

  vector<vector<double>> camParams(cams.size());
  vector<bool> camParamsUsed(cams.size(),false);

  ceres::Problem problem;
  for(int ptIdx = 0; ptIdx < pts.size(); ptIdx++)
  {
    auto& pt = pts[ptIdx];
    for(const auto& camKey : pt.views)
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
      ceres::LossFunction *lossFunction = robustify ? new ceres::HuberLoss(1.0) : NULL;
      
      problem.AddResidualBlock(costFunction,
        lossFunction,
        &camParams[camIdx][0],
        &pt.coord(0));
    }
    if(constantPoints[ptIdx] && !pt.views.empty())
      problem.SetParameterBlockConstant(&pt.coord(0));
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

  ceres::Solver::Summary summary;
  ceres::Solve(opt.get<ceres::Solver::Options>("solverOptions"),&problem,&summary);
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
  vector<Point> *ppts)
{
  vector<bool> constantPts(ppts->size(),true);
  bundleAdjustOneCam(opt,camIdx,constantPts,pcam,ppts);
}

void bundleAdjustOneCam(const OptionsBundleAdjustment& opt,
  int camIdx,const vector<bool>& constantPoints,Camera *pcam,vector<Point> *ppts)
{
  auto& cam = *pcam;
  auto& pts = *ppts;
  bool robustify = opt.get<bool>("robustify");

  vector<double> camParams;
  cam.params(&camParams);

  ceres::Problem problem;
  for(int ptIdx = 0; ptIdx < pts.size(); ptIdx++)
  {
    auto& pt = pts[ptIdx];
    try
    {
      int keyIdx = pt.views.at(camIdx);
      ceres::CostFunction *costFunction = cam.costFunction(keyIdx);

      // NULL specifies squared loss
      ceres::LossFunction *lossFunction = robustify ? new ceres::HuberLoss(1.0) : NULL;

      problem.AddResidualBlock(costFunction,
        lossFunction,
        &camParams[0],
        &pt.coord(0));

    } catch(const std::out_of_range&)
    {
      // This point is not seen by our camera.
      continue;
    }

    if(constantPoints[ptIdx] && !pt.views.empty())
      problem.SetParameterBlockConstant(&pt.coord(0));
  }

  {
    ceres::CostFunction *costFunction = cam.constraintsCostFunction();
    
    problem.AddResidualBlock(costFunction,
      NULL,
      &camParams[0]);
  }

  ceres::Solver::Summary summary;
  ceres::Solve(opt.get<ceres::Solver::Options>("solverOptions"),&problem,&summary);
  //std::cout << summary.FullReport() << "\n";

  cam.setParams(camParams);
}

} // namespace yasfm

namespace
{

} // namespace