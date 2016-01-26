#include "absolute_pose.h"

#include <algorithm>
#include <iostream>

using Eigen::JacobiSVD;
using Eigen::MatrixXd;
using Eigen::VectorXd;

using std::cerr;
using std::cout;

namespace yasfm
{
void chooseWellMatchedCameras(int minMatchesThresh,double factor,
  const vector<vector<IntPair>>& camToSceneMatches,
  uset<int> *wellMatchedCams)
{
  int maxMatches = 0;
  for(const auto& camMatches : camToSceneMatches)
  {
    if(camMatches.size() > maxMatches)
    {
      maxMatches = static_cast<int>(camMatches.size());
    }
  }
  int minMatches = static_cast<int>(floor(maxMatches*factor));
  int thresh = std::max<int>(minMatchesThresh,minMatches);
  for(int i = 0; i < static_cast<int>(camToSceneMatches.size()); i++)
  {
    if(camToSceneMatches[i].size() >= thresh)
    {
      wellMatchedCams->insert(i);
    }
  }
}

double computeAverageReprojectionError(const ptr_vector<Camera>& cams,
  const Points& points)
{
  double cumulativeError = 0.;
  int observationsCount = 0;
  for(int iPt = 0; iPt < points.numPtsAll(); iPt++)
  {
    for(const auto& camKey : points.ptData()[iPt].reconstructed)
    {
      const auto& cam = *cams[camKey.first];
      const auto& key = cam.key(camKey.second);
      Vector2d proj = cam.project(points.ptCoord()[iPt]);
      cumulativeError += (proj - key).norm();
      observationsCount++;
    }
  }

  if(observationsCount > 0)
    return cumulativeError / observationsCount;
  else
    return 0.;
}

bool resectCamera5AndHalfPtRANSAC(const OptionsRANSAC& opt,
  const vector<IntPair>& camToSceneMatches,const vector<Vector3d>& points,
  Camera *cam,vector<int> *inliers)
{
  Matrix34d P;
  bool success = resectCamera5AndHalfPtRANSAC(opt,camToSceneMatches,cam->keys(),
    points,&P,inliers);
  if(success)
    cam->setParams(P);
  return success;
}

bool resectCamera5AndHalfPtRANSAC(const OptionsRANSAC& opt,
  const vector<IntPair>& camToSceneMatches,const vector<Vector2d>& keys,
  const vector<Vector3d>& points,
  Matrix34d *P,vector<int> *inliers)
{
  MediatorResectioning5AndHalfPtRANSAC m(keys,points,camToSceneMatches);
  int nInliers = estimateTransformRANSAC(m,opt,P,inliers);
  return (nInliers > 0);
}

void resectCamera5AndHalfPt(const vector<Vector2d>& keys,const vector<Vector3d>& points,
  const vector<IntPair>& camToSceneMatches,vector<Matrix34d> *Ps)
{
  const size_t minPts = 6;
  if(camToSceneMatches.size() < minPts)
  {
    cerr << "ERROR: resectCamera5AndHalfPt: "
      << "Cannot estimate transform. " << camToSceneMatches.size()
      << " points given, but 6 needed.\n";
    return;
  }

  MatrixXd Ms[2];
  Ms[0].resize(11,12);
  Ms[0].setZero();
  for(size_t i = 0; i < 6; i++)
  {
    const auto& key = keys[camToSceneMatches[i].first];
    int ptIdx = camToSceneMatches[i].second;
    const auto& pt = points[ptIdx].homogeneous().transpose();

    Ms[0].block(2 * i,0,1,4) = pt;
    Ms[0].block(2 * i,8,1,4) = -key.x() * pt;

    if(i != 5)
    {
      Ms[0].block(2 * i + 1,4,1,4) = pt;
      Ms[0].block(2 * i + 1,8,1,4) = -key.y() * pt;
    } else
    {
      Ms[1] = Ms[0];
      Ms[1].block(2 * i,4,1,4) = pt;
      Ms[1].block(2 * i,8,1,4) = -key.y() * pt;
    }
  }

  Ps->resize(2);
  for(size_t i = 0; i < 2; i++)
  {
    JacobiSVD<MatrixXd> svd(Ms[i],Eigen::ComputeFullV);

    const auto& p = svd.matrixV().rightCols(1);
    for(size_t j = 0; j < 3; j++)
    {
      (*Ps)[i].row(j) = p.block(j * 4,0,4,1).transpose();
    }
  }
}

bool resectCamera6ptLSRANSAC(const OptionsRANSAC& opt,
  const vector<IntPair>& camToSceneMatches,const vector<Vector3d>& points,
  Camera *cam,vector<int> *inliers)
{
  Matrix34d P;
  bool success = resectCamera6ptLSRANSAC(opt,camToSceneMatches,cam->keys(),points,&P,inliers);
  if(success)
    cam->setParams(P);
  return success;
}

bool resectCamera6ptLSRANSAC(const OptionsRANSAC& opt,
  const vector<IntPair>& camToSceneMatches,const vector<Vector2d>& keys,
  const vector<Vector3d>& points,
  Matrix34d *P,vector<int> *inliers)
{
  MediatorResectioning6ptLSRANSAC m(keys,points,camToSceneMatches);
  int nInliers = estimateTransformRANSAC(m,opt,P,inliers);
  return (nInliers > 0);
}

void resectCameraLS(const vector<Vector2d>& keys,const vector<Vector3d>& points,
  const vector<IntPair>& camToSceneMatches,Matrix34d *pP)
{
  MatrixXd A(MatrixXd::Zero(2 * camToSceneMatches.size(),11));
  VectorXd b(VectorXd::Zero(2 * camToSceneMatches.size()));

  for(size_t i = 0; i < camToSceneMatches.size(); i++)
  {
    int keyIdx = camToSceneMatches[i].first;
    const auto& key = keys[keyIdx];
    int ptIdx = camToSceneMatches[i].second;
    const auto& pt = points[ptIdx];

    A.block(2*i,0,1,4) = pt.homogeneous().transpose();
    A.block(2*i,8,1,3) = -key.x() * pt.transpose();
    A.block(2*i+1,4,1,4) = pt.homogeneous().transpose();
    A.block(2*i+1,8,1,3) = -key.y() * pt.transpose();

    b(2*i) = key.x();
    b(2*i+1) = key.y();
  }

  MatrixXd X = A.jacobiSvd(Eigen::ComputeThinU | Eigen::ComputeThinV).solve(b);

  auto& P = (*pP);
  P.row(0) = X.topRows(4).transpose();
  P.row(1) = X.middleRows(4,4).transpose();
  P.row(2).leftCols(3) = X.bottomRows(3).transpose();
  P(2,3) = 1.;
}

bool resectCamera3ptRANSAC(const OptionsRANSAC& opt,
	const vector<IntPair>& camToSceneMatches, const vector<Vector3d>& points,
	Camera *cam, vector<int> *inliers)
{
	Matrix34d P;
	Matrix3d Kinv = cam->K().inverse();
	vector<Vector2d> calibratedKeys;
	for (int i = 0; i < cam->keys().size(); i++)
	{
		calibratedKeys.push_back((Kinv*cam->keys()[i].homogeneous()).head(2));
	}
	bool success = resectCamera3ptRANSAC(opt, camToSceneMatches, calibratedKeys,
		points,Kinv, &P, inliers);
	if (success)
		cam->setParams(cam->K()*P);
	return success;
}

bool resectCamera3ptRANSAC(const OptionsRANSAC& opt,
	const vector<IntPair>& camToSceneMatches, const vector<Vector2d>& keys,
	const vector<Vector3d>& points, const Matrix3d & Kinv,
	Matrix34d *P, vector<int> *inliers)
{
	
	MediatorResectioning3ptRANSAC m(keys, points, camToSceneMatches);
	int nInliers = estimateTransformRANSAC(m, opt, P, inliers);
	return (nInliers > 0);
}

void resectCamera3pt(const vector<Vector2d>& keys,
	const vector<Vector3d>& points, const vector<IntPair>& camToSceneMatches, 
	vector<Matrix34d> *Ps){

	const size_t minPts = 3;
	if (camToSceneMatches.size() < minPts)
	{
		cerr << "ERROR: resectCamera3Pt: "
			<< "Cannot estimate transform. " << camToSceneMatches.size()
			<< " points given, but 3 needed.\n";
		return;
	}

	double a, b, c;

	a = (points[camToSceneMatches[1].second] - points[camToSceneMatches[2].second]).norm();
	b = (points[camToSceneMatches[0].second] - points[camToSceneMatches[2].second]).norm();
	c = (points[camToSceneMatches[0].second] - points[camToSceneMatches[1].second]).norm();

	if (a==0||b==0||c==0)
	{
		return;
	}

	Eigen::Matrix3d u;

	for (size_t i = 0; i < 3; i++)
	{
		int keyIdx = camToSceneMatches[i].first;
		const auto& key = keys[keyIdx];
		// normalized unit rays
		double norm = sqrt(1+ key.x()*key.x() + key.y()*key.y());
		u(0, i) = key.x()/norm;
		u(1, i) = key.y()/norm;
		u(2, i) = 1/norm;
	}
	

	double ca, cb, cg;
	ca = u.col(1).transpose()*u.col(2);
	cb = u.col(0).transpose()*u.col(2);
	cg = u.col(0).transpose()*u.col(1);

	double aa, bb, cc;
	aa = a*a; bb = b*b; cc = c*c;

	double q1, q2, q3, q4;
	q1 = (aa - cc) / bb;
	q2 = (aa + cc) / bb;
	q3 = (bb - cc) / bb;
	q4 = (bb - aa) / bb;

	double caca, cbcb, cgcg;
	caca = ca*ca;
	cbcb = cb*cb;
	cgcg = cg*cg;

	Eigen::VectorXd A(5);

	A(4) = (q1 - 1) * (q1 - 1) - 4 * cc*caca / bb;
	A(3) = 4 * (q1*(1 - q1)*cb - (1 - q2)*ca*cg + 2 * cc*caca*cb / bb);
	A(2) = 2 * (q1*q1 - 1 + 2 * q1*q1* cbcb + 2 * q3*caca - 4 * q2*ca*cb*cg + 2 * q4*cgcg);
	A(1) = 4 * (-q1*(1 + q1)*cb + 2 * aa*cgcg*cb / bb - (1 - q2)*ca*cg);
	A(0) = (1 + q1)*(1 + q1) - 4 * aa*cgcg / bb;


	Eigen::MatrixXd CM(4, 4);
	CM.setZero();
	CM.bottomLeftCorner(3, 3).diagonal() << 1, 1, 1;
	CM.col(3) << -A(0) / A(4), -A(1) / A(4), -A(2) / A(4), -A(3) / A(4);
	Eigen::EigenSolver<Eigen::MatrixXd> es;
	es.compute(CM);

	Eigen::VectorXcd v = es.eigenvalues();
	std::vector<double> U, V;
	int nreal = 0;
	for (int i = 0; i < 4; i++)
	{
		if (v(i).imag() == 0){
			nreal++;
			V.push_back(v(i).real());
			U.push_back(((q1 - 1)*V[nreal - 1] * V[nreal - 1] - 2 * q1*cb*V[nreal - 1] + 1 + q1) / (2 * (cg - V[nreal - 1] * ca)));
		}
	}

	Eigen::MatrixXd s;
	s.resize(nreal, 3);
	for (int i = 0; i < nreal; i++)
	{
		s.row(i) << sqrt(aa / (U[i] * U[i] + V[i] * V[i] - 2 * U[i] * V[i] * ca)), sqrt(bb / (1 + V[i] * V[i] - 2 * V[i] * cb)), sqrt(cc / (1 + U[i] * U[i] - 2 * U[i] * cg));
	}

	Eigen::VectorXd s1, s2, s3;
	s1.resize(nreal);
	s2.resize(nreal);
	s3.resize(nreal);

	std::vector<Eigen::Matrix3d> R;
	std::vector<Eigen::Vector3d> t;

	for (int i = 0; i < nreal; i++)
	{
		s1(i) = s(i, 1);
		s2(i) = U[i] * s1(i);
		s3(i) = V[i] * s1(i);
	}
	std::vector<Eigen::Matrix3d> XX;
	for (int i = 0; i < nreal; i++)
	{
		XX.push_back(Eigen::Matrix3d());
		for (int j = 0; j < 3; j++)
		{
			XX[i].row(j) << u(j, 0)*s1(i), u(j, 1)*s2(i), u(j, 2)*s3(i);
		}

	}
	//points relative to centroids
	Eigen::Vector3d mean1, mean2;
	Eigen::Matrix3d X;
	for (int j = 0; j < 3; j++)
	{
		mean1(j) = (points[camToSceneMatches[0].second](j) + points[camToSceneMatches[1].second](j) + points[camToSceneMatches[2].second](j)) / 3;
	}

	for (int j = 0; j < 3; j++)
	{
		X.col(j) = points[camToSceneMatches[j].second] - mean1;
	}
	//find rigid transform between X and XX for all solutions
	for (int i = 0; i < nreal; i++)
	{
		for (int j = 0; j < 3; j++)
		{
			mean2(j) = XX[i].row(j).mean();
		}

		for (int j = 0; j < 3; j++)
		{
			XX[i].col(j) = XX[i].col(j) - mean2;
		}

		//ROTATION
		Eigen::Matrix3d cross;
		crossProdMat(X.col(0), &cross);
		Eigen::Vector3d n1 = cross*X.col(2);
		crossProdMat(XX[i].col(0), &cross);
		Eigen::Vector3d n2 = cross*XX[i].col(2);
		n1.normalize();
		n2.normalize();

		//first rotation
		Eigen::Vector3d a;
		crossProdMat(n1, &cross);
		a = cross*n2;
		a.normalize();

		double c_alpha, s_alpha;
		c_alpha = n1.transpose() *n2;
		s_alpha = sqrt(1 - c_alpha*c_alpha);
		Eigen::Matrix3d A;
		crossProdMat(a, &A);
		Eigen::Matrix3d R1 = Eigen::Matrix3d::Identity() + A*s_alpha + A*A*(1 - c_alpha);
		Eigen::Matrix3d X1a = R1*X;

		//second rotation
		c_alpha = XX[i].col(0).normalized().transpose()*X1a.col(0).normalized();
		s_alpha = sqrt(1 - c_alpha*c_alpha);
		crossProdMat(n2, &A);
		Eigen::Matrix3d R2 = Eigen::Matrix3d::Identity() + A*s_alpha + A*A*(1 - c_alpha);

		//which direction
		double e_plus = (XX[i] - R2*X1a).norm();
		double e_minus = (XX[i] - R2.transpose()*X1a).norm();
		Eigen::Matrix3d R;
		if (e_plus < e_minus){
			R = R2*R1;
		}
		else{
			R = R2.transpose()*R1;
		}
		Eigen::Vector3d t = mean2 - R*mean1;
		Matrix34d Pi(3, 4);
		Pi << R, t;
		Ps->push_back(Pi);

	}

}

MediatorResectioningRANSAC::MediatorResectioningRANSAC(int minMatches,
  const vector<Vector2d>& keys,const vector<Vector3d>& points,
  const vector<IntPair>& camToSceneMatches)
  : minMatches_(minMatches),keys_(keys),points_(points),
  camToSceneMatches_(camToSceneMatches)
{
}

int MediatorResectioningRANSAC::numMatches() const
{
  return static_cast<int>(camToSceneMatches_.size());
}

int MediatorResectioningRANSAC::minMatches() const
{ 
  return minMatches_; 
}

double MediatorResectioningRANSAC::computeSquaredError(const Matrix34d& P,
  int matchIdx) const
{
  const auto& pt = points_[camToSceneMatches_[matchIdx].second];
  const auto& key = keys_[camToSceneMatches_[matchIdx].first];

  Vector3d proj = (P*pt.homogeneous());
  return (key - proj.hnormalized()).squaredNorm();
}

void MediatorResectioningRANSAC::refine(double tolerance,const vector<int>& inliers,
  Matrix34d *P) const
{
}

MediatorResectioning5AndHalfPtRANSAC::MediatorResectioning5AndHalfPtRANSAC(
  const vector<Vector2d>& keys,const vector<Vector3d>& points,
  const vector<IntPair>& camToSceneMatches)
  : MediatorResectioningRANSAC(6,keys,points,camToSceneMatches)
{
}

void MediatorResectioning5AndHalfPtRANSAC::computeTransformation(const vector<int>& idxs,
  vector<Matrix34d> *Ps) const
{
  vector<IntPair> selectedMatches;
  selectedMatches.reserve(minMatches_);
  for(int idx : idxs)
    selectedMatches.push_back(camToSceneMatches_[idx]);
  resectCamera5AndHalfPt(keys_,points_,selectedMatches,Ps);
}

MediatorResectioning6ptLSRANSAC::MediatorResectioning6ptLSRANSAC(
  const vector<Vector2d>& keys,const vector<Vector3d>& points,
  const vector<IntPair>& camToSceneMatches)
  : MediatorResectioningRANSAC(6,keys,points,camToSceneMatches)
{
}

void MediatorResectioning6ptLSRANSAC::computeTransformation(const vector<int>& idxs,
  vector<Matrix34d> *Ps) const
{
  vector<IntPair> selectedMatches;
  selectedMatches.reserve(minMatches_);
  for(int idx : idxs)
    selectedMatches.push_back(camToSceneMatches_[idx]);
  Ps->resize(1);
  resectCameraLS(keys_,points_,selectedMatches,&(*Ps)[0]);
}

MediatorResectioning3ptRANSAC::MediatorResectioning3ptRANSAC(
	const vector<Vector2d>& keys, const vector<Vector3d>& points,
	const vector<IntPair>& camToSceneMatches)
	: MediatorResectioningRANSAC(3, keys, points, camToSceneMatches)
{
}

void MediatorResectioning3ptRANSAC::computeTransformation(const vector<int>& idxs,
	vector<Matrix34d> *Ps) const
{
	vector<IntPair> selectedMatches;
	selectedMatches.reserve(minMatches_);
	for (int idx : idxs)
		selectedMatches.push_back(camToSceneMatches_[idx]);
	resectCamera3pt(keys_, points_, selectedMatches, Ps);
}

} // namespace yasfm