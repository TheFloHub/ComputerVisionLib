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

Eigen::Array2Xd Cvl::EquidistantModel::unproject(Eigen::Array2Xd const& imagePoints) const
{
	// this is the same solution as below to test eigen broadcasting and reduction features
	//Eigen::Array2Xd result(2, imagePoints.cols());
	//double pixelLengthRatio = mFocalLengthX / mFocalLengthY;
	//double pixelLengthRatioSquared = pixelLengthRatio*pixelLengthRatio;
	//Eigen::Array2d temp1;
	//double radiusX = 0.0;
	//double radiusCamera = 0.0;
	//for (int i=0; i<imagePoints.cols(); ++i)
	//{
	//	temp1 = imagePoints.col(i) - Eigen::Array2d(mPrincipalPointX, mPrincipalPointY);
	//	radiusX = std::sqrt(temp1.x()*temp1.x() + temp1.y()*temp1.y()*pixelLengthRatioSquared);
	//	radiusCamera = std::tan(radiusX/ mFocalLengthX);
	//	result(0, i) = temp1.x() * radiusCamera / radiusX;
	//	result(1, i) = temp1.y() * radiusCamera * pixelLengthRatio / radiusX;
	//}

	Eigen::Array2Xd centeredPoints = imagePoints.colwise() - Eigen::Array2d(mPrincipalPointX, mPrincipalPointY);
	centeredPoints.row(1) *= mFocalLengthX / mFocalLengthY;
	Eigen::Array<double, 1, Eigen::Dynamic> radiX = centeredPoints.matrix().colwise().norm().array();
	return ((centeredPoints.rowwise() * (radiX / mFocalLengthX).tan()).rowwise() / radiX);

}
