/// 
/// @author	Florian Feuerstein
/// 
/// @date   02/2017
/// 
/// @class  ModelViewFromHomography
/// 
///	@brief  
///

#pragma once

#include <ComputerVisionLib/CameraModel/CameraModel.h>
#include <Eigen/Geometry>
#include <tuple>

namespace Cvl
{

	class ModelViewFromHomography
	{

	public:

		ModelViewFromHomography();

		~ModelViewFromHomography();

		static std::tuple<bool, double, Eigen::Affine3d> calculate(
			CameraModel const & cameraModel,
			Eigen::Matrix3d const & homography,
			Eigen::Array2Xd const & srcPoints,
			Eigen::Array2Xd const & dstPoints);

	private:

		static Eigen::Affine3d algebraic(
			CameraModel const & cameraModel,
			Eigen::Matrix3d const & homography);
	};
}





