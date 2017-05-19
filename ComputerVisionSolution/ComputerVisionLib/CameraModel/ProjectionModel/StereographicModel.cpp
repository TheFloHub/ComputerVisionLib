#include "StereographicModel.h"

Cvl::StereographicModel::StereographicModel(double focalLengthX, double focalLengthY, double principalPointX, double principalPointY) :
	ProjectionModel(focalLengthX, focalLengthY, principalPointX, principalPointY)
{
}

Cvl::ProjectionModel::Uptr Cvl::StereographicModel::clone() const
{
	return ProjectionModel::Uptr(new StereographicModel(mFocalLengthX, mFocalLengthY, mPrincipalPointX, mPrincipalPointY));
}

Eigen::Array2Xd Cvl::StereographicModel::project(Eigen::Array2Xd const& distortedPoints) const
{
	// r_s = 2*f * tan(phi/2) = 2*f * tan(atan(r_c)/2) = 2*f*r_c / (sqrt(r_c^2 + 1) + 1)
	// x_s = x_c *s_x * 2*f * r_c / (sqrt(r_c^2 + 1) + 1)/r_c = x_c *s_x * 2*f / (sqrt(r_c^2 + 1) + 1))
	Eigen::Array<double, 1, Eigen::Dynamic> norms = distortedPoints.matrix().colwise().norm();
	return ((distortedPoints.colwise() * Eigen::Array2d(2.0*mFocalLengthX, 2.0*mFocalLengthY)).rowwise() / ((norms.square() + 1.0).sqrt()+1.0)).colwise() + Eigen::Array2d(mPrincipalPointX, mPrincipalPointY);
}

