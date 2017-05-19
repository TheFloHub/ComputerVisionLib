#include "IntrinsicFishEyeCalibration.h"

Cvl::IntrinsicFishEyeCalibration::IntrinsicFishEyeCalibration()
{
}

Cvl::IntrinsicFishEyeCalibration::~IntrinsicFishEyeCalibration()
{
}

std::tuple<bool, double> Cvl::IntrinsicFishEyeCalibration::calibrate(
	Eigen::Array2Xd const & templatePoints, 
	std::vector<Eigen::Array2Xd> const & imagePointsPerFrame, 
	std::vector<std::vector<Match>> const & matchesPerFrame, 
	CameraModel & cameraModel)
{
	return std::tuple<bool, double>();
}
