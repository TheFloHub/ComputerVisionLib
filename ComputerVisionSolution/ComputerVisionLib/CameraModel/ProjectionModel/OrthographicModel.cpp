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

Eigen::Array2Xd Cvl::OrthographicModel::unproject(Eigen::Array2Xd const& imagePoints) const
{
	// this is the same solution as below to test eigen broadcasting and reduction features
	//Eigen::Array2Xd result(2, imagePoints.cols());
	//double pixelLengthRatio = mFocalLengthX / mFocalLengthY;
	//double pixelLengthRatioSquared = pixelLengthRatio*pixelLengthRatio;
	//Eigen::Array2d temp1;
	//double radiusX = 0.0;
	//double radiusCamera = 0.0;
	//double temp = 0.0;
	//for (int i=0; i<imagePoints.cols(); ++i)
	//{
	//	temp1 = imagePoints.col(i) - Eigen::Array2d(mPrincipalPointX, mPrincipalPointY);
	//	radiusX = std::sqrt(temp1.x()*temp1.x() + temp1.y()*temp1.y()*pixelLengthRatioSquared);
	//	temp = radiusX / mFocalLengthX;
	//	radiusCamera = temp / std::sqrt(1.0 - temp*temp);
	//	result(0, i) = temp1.x() * radiusCamera / radiusX;
	//	result(1, i) = temp1.y() * radiusCamera * pixelLengthRatio / radiusX;
	//}
	//return result;

	Eigen::Array2Xd centeredPoints = imagePoints.colwise() - Eigen::Array2d(mPrincipalPointX, mPrincipalPointY);
	centeredPoints.row(1) *= mFocalLengthX / mFocalLengthY;
	Eigen::Array<double, 1, Eigen::Dynamic> radiX = centeredPoints.matrix().colwise().norm().array();
	return (centeredPoints.rowwise() * ((radiX/mFocalLengthX) / (1.0 - (radiX/mFocalLengthX).square()).sqrt())).rowwise() / radiX;
}
