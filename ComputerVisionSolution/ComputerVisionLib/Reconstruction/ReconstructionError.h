/// 
/// @author	Florian Feuerstein
/// 
/// @date   02/2017
/// 
/// @class  ReconstructionError
/// 
///	@brief  
///

#pragma once

#include <ComputerVisionLib/CameraModel/CameraModel.h>
#include <Eigen/Core>
#include <Eigen/Geometry>

namespace Cvl
{

	class ReconstructionError
	{

	public:

		ReconstructionError();

		~ReconstructionError();

		static Eigen::VectorXd calculateDiff(
			Eigen::Affine3d const& modelView,
			CameraModel const& cameraModel,
			Eigen::Array2Xd const& srcPoints,
			Eigen::Array2Xd const& dstPoints);

		static Eigen::VectorXd calculateDiff(
			Eigen::Matrix<double, 6, 1> const& modelViewParameter,
			CameraModel const& cameraModel,
			Eigen::Array2Xd const& srcPoints,
			Eigen::Array2Xd const& dstPoints);

		static double calculateRMSE(
			Eigen::Affine3d const& modelView,
			CameraModel const& cameraModel,
			Eigen::Array2Xd const& srcPoints,
			Eigen::Array2Xd const& dstPoints);

	private:

	};
}





