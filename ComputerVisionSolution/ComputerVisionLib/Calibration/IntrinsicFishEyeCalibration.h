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
#include <ComputerVisionLib/Calibration/Circle.h>
#include <ComputerVisionLib/Common/EigenFunctorBase.h>
#include <Eigen/Core>
#include <vector>
#include <tuple>

namespace Cvl
{

	class IntrinsicFishEyeCalibration
	{

	public:

		static bool calibrate(
			Eigen::Array2Xd const& templatePoints,
			std::vector<Eigen::Array2Xd> const & imagePointsPerFrame,
			std::vector<std::vector<Match>> const & matchesPerFrame,
			CameraModel & cameraModel);

	private:

		IntrinsicFishEyeCalibration() = delete;

		~IntrinsicFishEyeCalibration() = delete;

		/// @brief calculates an initial estimation of the focal length and
		/// principal point from the collected data of one frame.
		static std::tuple<bool, Eigen::Array2Xd> calculateIntrinsicParametersOfFrame(
			std::vector<Eigen::VectorXd> & matches,
			CameraModel & cameraModel);

		/// @brief Intersects all circles and calculates the two vanishing points.
		static bool getVanishingPoints(std::vector<Circle> const & circles, Eigen::Vector2d & vanishingPoint1, Eigen::Vector2d & vanishingPoint2);

		/// @brief compares two matches column major.
		static bool compareColumnMajor(Eigen::VectorXd const & match1, Eigen::VectorXd const & match2);

		/// @brief compares two matches row major.
		static bool compareRowMajor(Eigen::VectorXd const & match1,	Eigen::VectorXd const & match2);

		static Eigen::Array2Xd vectorToEigen(std::vector<Eigen::VectorXd> const & vector);

		class Functor : public FunctorBase<double>
		{
		public:
			Functor(CameraModel & cameraModel, Eigen::Array2Xd const & points);
			int operator() (Eigen::VectorXd const & x, Eigen::VectorXd & fvec) const;
		private:
			CameraModel & mCameraModel;
			Eigen::Array2Xd const & mPoints;
		};

	};
}





