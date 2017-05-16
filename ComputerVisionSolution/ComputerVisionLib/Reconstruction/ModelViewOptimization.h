/// 
/// @author	Florian Feuerstein
/// 
/// @date   02/2017
/// 
/// @class  ModelViewOptimization
/// 
///	@brief  
///

#pragma once

#include <ComputerVisionLib/CameraModel/CameraModel.h>
#include <Eigen/Geometry>
#include <tuple>

namespace Cvl
{

	class ModelViewOptimization
	{

	public:

		ModelViewOptimization();

		~ModelViewOptimization();

		static std::tuple<bool, double, Eigen::Affine3d> optimize(
			CameraModel const & cameraModel,
			Eigen::Affine3d const & modelView,
			Eigen::Array2Xd const & srcPoints,
			Eigen::Array2Xd const & dstPoints);

	private:

		class Functor2d
		{
		public:

			typedef double Scalar;
			enum {
				InputsAtCompileTime = Eigen::Dynamic, // 6???
				ValuesAtCompileTime = Eigen::Dynamic
			};

			typedef Eigen::Matrix<Scalar, InputsAtCompileTime, 1> InputType;
			typedef Eigen::Matrix<Scalar, ValuesAtCompileTime, 1> ValueType;
			typedef Eigen::Matrix<Scalar, ValuesAtCompileTime, InputsAtCompileTime> JacobianType;
			typedef Eigen::ColPivHouseholderQR<JacobianType> QRSolver;

			Functor2d(
				Eigen::Array2Xd const & srcPoints,
				Eigen::Array2Xd const & dstPoints,
				CameraModel const & cameraModel);

			int inputs() const { return mNumberOfInputs; }
			int values() const { return mNumberOfValues; }
			int operator() (Eigen::VectorXd const & x, Eigen::VectorXd & fvec) const;
		private:
			int mNumberOfInputs;
			int mNumberOfValues;
			Eigen::Array2Xd const & mSrcPoints;
			Eigen::Array2Xd const & mDstPoints;
			CameraModel const & mCameraModel;
		};

	};
}





