#include "EquisolidModel.h"

Cvl::EquisolidModel::EquisolidModel(double focalLengthX, double focalLengthY, double principalPointX, double principalPointY) :
	ProjectionModel(focalLengthX, focalLengthY, principalPointX, principalPointY)
{
}

Cvl::ProjectionModel::Uptr Cvl::EquisolidModel::clone() const
{
	return ProjectionModel::Uptr(new EquisolidModel(mFocalLengthX, mFocalLengthY, mPrincipalPointX, mPrincipalPointY));
}

Eigen::Array2Xd Cvl::EquisolidModel::project(Eigen::Array2Xd const& distortedPoints) const
{
	Eigen::Array<double, 1, Eigen::Dynamic> norms = distortedPoints.matrix().colwise().norm();
	Eigen::Array<double, 1, Eigen::Dynamic> angles = (norms.atan()*0.5).sin();
	return (((distortedPoints.colwise() * Eigen::Array2d(2.0*mFocalLengthX, 2.0*mFocalLengthY)).rowwise() * angles).rowwise() / norms).colwise() + Eigen::Array2d(mPrincipalPointX, mPrincipalPointY);
}

