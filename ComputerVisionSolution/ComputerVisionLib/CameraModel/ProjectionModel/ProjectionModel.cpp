#include "ProjectionModel.h"

Eigen::Matrix3d Cvl::ProjectionModel::getPinholeCameraMatrix() const
{
	return (Eigen::Matrix3d(3, 3) <<
		mFocalLengthX, 0.0, mPrincipalPointX,
		0.0, mFocalLengthY, mPrincipalPointY,
		0.0, 0.0, 1.0).finished();
}

Eigen::Matrix3d Cvl::ProjectionModel::getInversePinholeCameraMatrix() const
{
	// the inverse of the intrinsic camera matrix
	// fx	s	px
	// 0	fy	py
	// 0	0	1
	//
	// is
	//
	// 1/fx		-s/(fx*fy)		s*py/(fx*fy) - px/fx
	// 0		1/fy			-py/fy
	// 0		0				1
	return (Eigen::Matrix3d(3, 3) <<
		1.0/mFocalLengthX, 0.0, -mPrincipalPointX/mFocalLengthX,
		0.0, 1.0/mFocalLengthY, -mPrincipalPointY/mFocalLengthY,
		0.0, 0.0, 1.0).finished();
}

void Cvl::ProjectionModel::setParameters(double focalLengthX, double focalLengthY, double principalPointX, double principalPointY)
{
	mFocalLengthX = focalLengthX;
	mFocalLengthY = focalLengthY;
	mPrincipalPointX = principalPointX;
	mPrincipalPointY = principalPointY;
}

void Cvl::ProjectionModel::setParameters(Eigen::Vector4d const & parameters)
{
	mFocalLengthX = parameters(0);
	mFocalLengthY = parameters(1);
	mPrincipalPointX = parameters(2);
	mPrincipalPointY = parameters(3);
}

Eigen::Vector4d Cvl::ProjectionModel::getParameters() const
{
	return Eigen::Vector4d(mFocalLengthX, mFocalLengthY, mPrincipalPointX, mPrincipalPointY);
}
