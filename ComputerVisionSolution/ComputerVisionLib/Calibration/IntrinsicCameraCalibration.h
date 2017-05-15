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
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <vector>
#include <tuple>

namespace Cvl
{

	class IntrinsicCameraCalibration
	{

	public:

		IntrinsicCameraCalibration();

		~IntrinsicCameraCalibration();

		static std::tuple<bool, double> calibrate(
			Eigen::Array2Xd const& templatePoints,
			std::vector<Eigen::Array2Xd> const & imagePointsPerFrame,
			std::vector<std::vector<Match>> const & matchesPerFrame,
			CameraModel & cameraModel);

	private:

		static std::vector<std::tuple<Eigen::Array2Xd, Eigen::Array2Xd>> alignPoints(
			Eigen::Array2Xd const& templatePoints,
			std::vector<Eigen::Array2Xd> const & imagePointsPerFrame,
			std::vector<std::vector<Match>> const & matchesPerFrame);

		static std::vector<Eigen::Matrix3d> calculateHomographies(
			std::vector<std::tuple<Eigen::Array2Xd, Eigen::Array2Xd>> & alignedPointsPerFrame);

		static bool initializeWithHomographies(
			std::vector<Eigen::Matrix3d> const& homographies,
			CameraModel & cameraModel);

		static std::tuple<bool, double> optimize(
			std::vector<std::tuple<Eigen::Array2Xd, Eigen::Array2Xd>> const & alignedPointsPerFrame,
			std::vector<Eigen::Affine3d> & modelViews,
			CameraModel & cameraModel);


		class Functor
		{
		public:

			typedef double Scalar;
			enum {
				InputsAtCompileTime = Eigen::Dynamic,
				ValuesAtCompileTime = Eigen::Dynamic
			};
			typedef Eigen::Matrix<Scalar, InputsAtCompileTime, 1> InputType;
			typedef Eigen::Matrix<Scalar, ValuesAtCompileTime, 1> ValueType;
			typedef Eigen::Matrix<Scalar, ValuesAtCompileTime, InputsAtCompileTime> JacobianType;
			typedef Eigen::ColPivHouseholderQR<JacobianType> QRSolver;

			Functor(
				size_t numberOfIntrinsicParameters, 
				std::vector<std::tuple<Eigen::Array2Xd, Eigen::Array2Xd>> const & alignedPointsPerFrame,
				CameraModel & cameraModel);
			int inputs() const { return mNumberOfInputs; }
			int values() const { return mNumberOfValues; }
			int operator() (Eigen::VectorXd const & x, Eigen::VectorXd & fvec) const;
		private:
			int mNumberOfInputs;
			int mNumberOfValues;
			size_t mNumberOfIntrinsicParameters;
			std::vector<std::tuple<Eigen::Array2Xd, Eigen::Array2Xd>> const & mAlignedPointsPerFrame;
			CameraModel & mCameraModel;
		};

	};
}





