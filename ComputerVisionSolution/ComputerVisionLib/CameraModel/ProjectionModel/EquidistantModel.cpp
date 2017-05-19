#include "EquidistantModel.h"

Cvl::EquidistantModel::EquidistantModel(double focalLengthX, double focalLengthY, double principalPointX, double principalPointY) :
ProjectionModel(focalLengthX, focalLengthY, principalPointX, principalPointY)
{
}

Cvl::ProjectionModel::Uptr Cvl::EquidistantModel::clone() const
{
	return ProjectionModel::Uptr(new EquidistantModel(mFocalLengthX, mFocalLengthY, mPrincipalPointX, mPrincipalPointY));
}

Eigen::Array2Xd Cvl::EquidistantModel::project(Eigen::Array2Xd const& distortedPoints) const
{
	Eigen::Array<double, 1, Eigen::Dynamic> norms = distortedPoints.matrix().colwise().norm();
	Eigen::Array<double, 1, Eigen::Dynamic> angles = norms.atan();
	return (((distortedPoints.colwise() * Eigen::Array2d(mFocalLengthX, mFocalLengthY)).rowwise() * angles).rowwise() / norms).colwise() + Eigen::Array2d(mPrincipalPointX, mPrincipalPointY);
}

//Eigen::Array2Xd Cvl::EquidistantModel::unproject(Eigen::Array2Xd const& imagePoints) const
//{
//	//imagePoints.colwise() -= Eigen::Array2d(mPrincipalPointX, mPrincipalPointY);
//	//Eigen::ArrayXd norms = imagePoints.colwise().norm();
//	return Eigen::Array2Xd();
//}
