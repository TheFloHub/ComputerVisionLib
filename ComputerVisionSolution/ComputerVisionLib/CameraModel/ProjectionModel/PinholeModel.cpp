#include "PinholeModel.h"

Cvl::PinholeModel::PinholeModel(double focalLengthX, double focalLengthY, double principalPointX, double principalPointY) :
ProjectionModel(focalLengthX, focalLengthY, principalPointX, principalPointY)
{
}

Cvl::ProjectionModel::Uptr Cvl::PinholeModel::clone() const
{
	return ProjectionModel::Uptr(new PinholeModel(mFocalLengthX, mFocalLengthY, mPrincipalPointX, mPrincipalPointY));
}

Eigen::Array2Xd Cvl::PinholeModel::project(Eigen::Array2Xd const& distortedPoints) const
{
	return (distortedPoints.colwise() * Eigen::Array2d(mFocalLengthX, mFocalLengthY)).colwise() + Eigen::Array2d(mPrincipalPointX, mPrincipalPointY); // TODO: negate focal length?
}

Eigen::Array2Xd Cvl::PinholeModel::unproject(Eigen::Array2Xd const& imagePoints) const
{
	return (imagePoints.colwise() - Eigen::Array2d(mPrincipalPointX, mPrincipalPointY)).colwise() / Eigen::Array2d(mFocalLengthX, mFocalLengthY); // TODO: negate focal length?
}
