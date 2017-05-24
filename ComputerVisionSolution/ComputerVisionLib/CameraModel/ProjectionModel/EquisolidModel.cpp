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

Eigen::Array2Xd Cvl::EquisolidModel::unproject(Eigen::Array2Xd const& imagePoints) const
{
	// this is the same solution as below to test eigen broadcasting and reduction features
	//Eigen::Array2Xd result(2, imagePoints.cols());
	//double pixelLengthRatio = mFocalLengthX / mFocalLengthY;
	//double pixelLengthRatioSquared = pixelLengthRatio*pixelLengthRatio;
	//Eigen::Array2d temp1;
	//double radiusX = 0.0;
	//double radiusTemp1 = 0.0;
	//double radiusTemp2 = 0.0;
	//double radiusCamera = 0.0;
	//for (int i=0; i<imagePoints.cols(); ++i)
	//{
	//	temp1 = imagePoints.col(i) - Eigen::Array2d(mPrincipalPointX, mPrincipalPointY);
	//	radiusX = std::sqrt(temp1.x()*temp1.x() + temp1.y()*temp1.y()*pixelLengthRatioSquared);
	//	radiusTemp1 = 0.5*radiusX / mFocalLengthX;
	//	radiusTemp2 = radiusTemp1 * radiusTemp1;
	//	radiusCamera = (2.0 * radiusTemp1 * sqrt(1.0 - radiusTemp2)) / (1.0 - 2.0 * radiusTemp2);
	//	result(0, i) = temp1.x() * radiusCamera / radiusX;
	//	result(1, i) = temp1.y() * radiusCamera * pixelLengthRatio / radiusX;
	//}
	//return result;

	Eigen::Array2Xd centeredPoints = imagePoints.colwise() - Eigen::Array2d(mPrincipalPointX, mPrincipalPointY);
	centeredPoints.row(1) *= mFocalLengthX / mFocalLengthY;
	Eigen::Array<double, 1, Eigen::Dynamic> radiX = centeredPoints.matrix().colwise().norm().array();
	Eigen::Array<double, 1, Eigen::Dynamic> radiX2f = radiX/(2.0*mFocalLengthX);
	Eigen::Array<double, 1, Eigen::Dynamic> radiX2fSquared = radiX2f.square();
	return (centeredPoints.rowwise() * ( (2.0 * radiX2f * (1.0 - radiX2fSquared).sqrt()) / (1.0 - 2.0 * radiX2fSquared) ) ).rowwise() / radiX;
}