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

Eigen::Array2Xd Cvl::StereographicModel::unproject(Eigen::Array2Xd const& imagePoints) const
{
	// this is the same solution as below to test eigen broadcasting and reduction features
	//Eigen::Array2Xd result(2, imagePoints.cols());
	//double pixelLengthRatio = mFocalLengthX / mFocalLengthY;
	//double pixelLengthRatioSquared = pixelLengthRatio*pixelLengthRatio;
	//Eigen::Array2d temp1;
	//double radiusX = 0.0;
	//double fxs = mFocalLengthX*mFocalLengthX;
	//double radiusCamera = 0.0;
	//for (int i = 0; i<imagePoints.cols(); ++i)
	//{
	//	temp1 = imagePoints.col(i) - Eigen::Array2d(mPrincipalPointX, mPrincipalPointY);
	//	radiusX = std::sqrt(temp1.x()*temp1.x() + temp1.y()*temp1.y()*pixelLengthRatioSquared);
	//	radiusCamera = (radiusX * mFocalLengthX) / (fxs - 0.25 * radiusX * radiusX);
	//	result(0, i) = temp1.x() * radiusCamera / radiusX;
	//	result(1, i) = temp1.y() * radiusCamera * pixelLengthRatio / radiusX;
	//}
	//return result;

	Eigen::Array2Xd centeredPoints = imagePoints.colwise() - Eigen::Array2d(mPrincipalPointX, mPrincipalPointY);
	centeredPoints.row(1) *= mFocalLengthX / mFocalLengthY;
	Eigen::Array<double, 1, Eigen::Dynamic> radiX = centeredPoints.matrix().colwise().norm().array();
	return (centeredPoints.rowwise() * ( (radiX*mFocalLengthX) / (mFocalLengthX*mFocalLengthX - 0.25 * radiX.square()) ) ).rowwise() / radiX;
}