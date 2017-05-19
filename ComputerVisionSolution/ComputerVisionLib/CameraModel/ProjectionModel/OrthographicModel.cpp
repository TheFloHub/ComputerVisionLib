#include "OrthographicModel.h"

Cvl::OrthographicModel::OrthographicModel(double focalLengthX, double focalLengthY, double principalPointX, double principalPointY) :
	ProjectionModel(focalLengthX, focalLengthY, principalPointX, principalPointY)
{
}

Cvl::ProjectionModel::Uptr Cvl::OrthographicModel::clone() const
{
	return ProjectionModel::Uptr(new OrthographicModel(mFocalLengthX, mFocalLengthY, mPrincipalPointX, mPrincipalPointY));
}

Eigen::Array2Xd Cvl::OrthographicModel::project(Eigen::Array2Xd const& distortedPoints) const
{
	// r_s = f * sin(phi) = f * r_c / sqrt(1 + r_c^2)
	// x_s = x_c *s_x * f * (r_c / sqrt(1 + r_c^2))/r_c = x_c * s_x * f / sqrt(1 + r_c^2)
	Eigen::Array<double, 1, Eigen::Dynamic> norms = distortedPoints.matrix().colwise().norm();
	return ((distortedPoints.colwise() * Eigen::Array2d(mFocalLengthX, mFocalLengthY)).rowwise() / (norms.square() + 1.0).sqrt()).colwise() + Eigen::Array2d(mPrincipalPointX, mPrincipalPointY);
}

