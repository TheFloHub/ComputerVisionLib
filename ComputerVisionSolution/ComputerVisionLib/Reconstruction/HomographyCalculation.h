/// 
/// @author	Florian Feuerstein
/// 
/// @date   02/2017
/// 
/// @class  HomographyCalculation
/// 
///	@brief  
///

#pragma once

#include <ComputerVisionLib/Common/Match.h>
#include <Eigen/Geometry>
#include <vector>
#include <tuple>

namespace Cvl
{

	class HomographyCalculation
	{

	public:

		HomographyCalculation();

		~HomographyCalculation();

		static std::tuple<bool, Eigen::Matrix3d> calculate(
			Eigen::Array2Xd const& srcPoints,
			Eigen::Array2Xd const& dstPoints);

		static double rmse(
			Eigen::Matrix3d const & homography,
			Eigen::Array2Xd const& srcPoints,
			Eigen::Array2Xd const& dstPoints);

	private:

	};
}





