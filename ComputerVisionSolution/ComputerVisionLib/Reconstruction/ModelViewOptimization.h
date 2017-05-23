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
#include <ComputerVisionLib/Common/EigenFunctorBase.h>
#include <Eigen/Geometry>
#include <tuple>

namespace Cvl
{

	class ModelViewOptimization
	{

	public:

		static std::tuple<bool, double, Eigen::Affine3d> optimize(
			CameraModel const & cameraModel,
			Eigen::Affine3d const & modelView,
			Eigen::Array2Xd const & srcPoints,
			Eigen::Array2Xd const & dstPoints);

	private:

		ModelViewOptimization() = delete;

		~ModelViewOptimization() = delete;

		class Functor2d : public FunctorBase<double>
		{
		public:
			Functor2d(
				Eigen::Array2Xd const & srcPoints,
				Eigen::Array2Xd const & dstPoints,
				CameraModel const & cameraModel);
			int operator() (Eigen::VectorXd const & x, Eigen::VectorXd & fvec) const;
		private:
			Eigen::Array2Xd const & mSrcPoints;
			Eigen::Array2Xd const & mDstPoints;
			CameraModel const & mCameraModel;
		};

	};
}





