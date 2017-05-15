#include "DistortionModel.h"

Cvl::DistortionModel::DistortionModel(double tangentialParameter1, double tangentialParameter2) :
mTangentialParameter1(tangentialParameter1),
mTangentialParameter2(tangentialParameter2)
{
}

Cvl::DistortionModel::~DistortionModel()
{
}

Eigen::Array2Xd Cvl::DistortionModel::distort(Eigen::Array2Xd const& normalizedCameraPoints) const
{
	Eigen::Array<double, 1, Eigen::Dynamic> squaredNorms = normalizedCameraPoints.matrix().colwise().squaredNorm();
	// (xd, yd)^T = (xu, yu)^T * radialFactor + tangentialOffsets
	return (normalizedCameraPoints.rowwise() * radialDistortionFactors(squaredNorms)) + tangentialDistortionOffsets(normalizedCameraPoints, squaredNorms);
}

Eigen::VectorXd Cvl::DistortionModel::getParameters() const
{
	Eigen::VectorXd radialParameters = getRadialDistortionParameters();
	return (Eigen::VectorXd(2+radialParameters.rows()) << mTangentialParameter1, mTangentialParameter2, radialParameters).finished();
}

void Cvl::DistortionModel::setParameters(Eigen::VectorXd const & parameters)
{
	mTangentialParameter1 = parameters(0);
	mTangentialParameter2 = parameters(1);
	setRadialDistortionParameters(parameters.tail(parameters.rows()-2));
}

void Cvl::DistortionModel::resetParameters()
{
	mTangentialParameter1 = 0.0;
	mTangentialParameter2 = 0.0;
	resetRadialDistortionParameters();
}

Eigen::Array2Xd Cvl::DistortionModel::tangentialDistortionOffsets(
	Eigen::Array2Xd const & undistortedPoints,
	Eigen::Array<double, 1, Eigen::Dynamic> const& squaredNorms) const
{
	// x' = p1 * 2 * x * y  + p2 * (r^2 + 2 * x^2)
	// y' = p2 * 2 * x * y  + p1 * (r^2 + 2 * y^2)
	return ((2.0 * Eigen::Vector2d(mTangentialParameter1, mTangentialParameter2)) * (undistortedPoints.row(0) * undistortedPoints.row(1)).matrix()).array()
		+ ((2.0 * undistortedPoints.square()).rowwise() + squaredNorms).colwise() * Eigen::Array2d(mTangentialParameter2, mTangentialParameter1);
}

