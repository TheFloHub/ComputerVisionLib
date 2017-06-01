/// 
/// @author	Florian Feuerstein
/// 
/// @date   02/2017
/// 
/// @class  IntrinsicCameraCalibration
/// 
///	@brief  
///

#pragma once

#include <ComputerVisionLib/CameraModel/CameraModel.h>
#include <ComputerVisionLib/Common/Match.h>
#include <ComputerVisionLib/Common/EigenFunctorBase.h>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <vector>
#include <tuple>

namespace Cvl
{

	class IntrinsicCameraCalibration
	{

	public:

		static std::tuple<bool, double> calibrate(
			Eigen::Array2Xd const& templatePoints,
			std::vector<Eigen::Array2Xd> const & imagePointsPerFrame,
			std::vector<std::vector<Match>> const & matchesPerFrame,
			CameraModel & cameraModel);

	private:

		IntrinsicCameraCalibration() = delete;

		~IntrinsicCameraCalibration() = delete;

		static std::vector<std::tuple<Eigen::Array2Xd, Eigen::Array2Xd>> alignPoints(
			Eigen::Array2Xd const& templatePoints,
			std::vector<Eigen::Array2Xd> const & imagePointsPerFrame,
			std::vector<std::vector<Match>> const & matchesPerFrame);

		static std::vector<Eigen::Matrix3d> calculateHomographies(
			std::vector<std::tuple<Eigen::Array2Xd, Eigen::Array2Xd>> & alignedPointsPerFrame);

		static std::vector<Eigen::Matrix3d> calculateHomographies2(
			std::vector<std::tuple<Eigen::Array2Xd, Eigen::Array2Xd>> & alignedPinholePointsPerFrame,
			std::vector<std::tuple<Eigen::Array2Xd, Eigen::Array2Xd>> & alignedPointsPerFrame);

		static bool initializeWithHomographies(
			std::vector<Eigen::Matrix3d> const& homographies,
			CameraModel & cameraModel);

		static std::tuple<bool, double> optimize(
			std::vector<std::tuple<Eigen::Array2Xd, Eigen::Array2Xd>> const & alignedPointsPerFrame,
			std::vector<Eigen::Affine3d> & modelViews,
			CameraModel & cameraModel);


		class Functor : public FunctorBase<double>
		{
		public:
			Functor(
				size_t numberOfIntrinsicParameters, 
				std::vector<std::tuple<Eigen::Array2Xd, Eigen::Array2Xd>> const & alignedPointsPerFrame,
				CameraModel & cameraModel);
			int operator() (Eigen::VectorXd const & x, Eigen::VectorXd & fvec) const;
		private:
			size_t mNumberOfIntrinsicParameters;
			std::vector<std::tuple<Eigen::Array2Xd, Eigen::Array2Xd>> const & mAlignedPointsPerFrame;
			CameraModel & mCameraModel;
		};

	};
}





