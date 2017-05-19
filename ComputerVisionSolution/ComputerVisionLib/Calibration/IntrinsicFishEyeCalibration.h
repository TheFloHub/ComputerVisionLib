/// 
/// @author	Florian Feuerstein
/// 
/// @date   02/2017
/// 
/// @class  IntrinsicFishEyeCalibration
/// 
///	@brief  
///

#pragma once

#include <ComputerVisionLib/CameraModel/CameraModel.h>
#include <ComputerVisionLib/Common/Match.h>
#include <Eigen/Core>
#include <vector>
#include <tuple>

namespace Cvl
{

	class IntrinsicFishEyeCalibration
	{

	public:

		IntrinsicFishEyeCalibration();

		~IntrinsicFishEyeCalibration();

		static std::tuple<bool, double> calibrate(
			Eigen::Array2Xd const& templatePoints,
			std::vector<Eigen::Array2Xd> const & imagePointsPerFrame,
			std::vector<std::vector<Match>> const & matchesPerFrame,
			CameraModel & cameraModel);

	private:



	};
}





