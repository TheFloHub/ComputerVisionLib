/// 
/// @author	Florian Feuerstein
/// 
/// @date   02/2017
/// 
/// @class  DistortionModel
/// 
///	@brief  
///

#pragma once

#include <Eigen/Core>
#include <memory>

namespace Cvl
{
	class DistortionModel
	{

	public:

		using Uptr = std::unique_ptr<DistortionModel>;

		DistortionModel(double tangentialParameter1, double tangentialParameter2);

		virtual ~DistortionModel();

		virtual DistortionModel::Uptr clone() const = 0;

		Eigen::Array2Xd distort(Eigen::Array2Xd const & normalizedCameraPoints) const;

		Eigen::Array2Xd undistort(Eigen::Array2Xd const & distortedPoints) const;

		Eigen::VectorXd getParameters() const;

		void setParameters(Eigen::VectorXd const & parameters);

		void resetParameters();

	protected:

		Eigen::Matrix2d derivative(Eigen::Array2d const & normalizedCameraPoint, double radialFactor) const;

		virtual Eigen::Array<double, 1, Eigen::Dynamic> radialDistortionFactors(Eigen::Array<double, 1, Eigen::Dynamic> const & squaredNorms) const = 0;

		virtual double radialDistortionDerivative(double r) const = 0;

		Eigen::Array2Xd tangentialDistortionOffsets(Eigen::Array2Xd const& undistortedPoints, Eigen::Array<double, 1, Eigen::Dynamic> const & squaredNorms) const;

		virtual Eigen::VectorXd getRadialDistortionParameters() const = 0;

		virtual void setRadialDistortionParameters(Eigen::VectorXd const & parameters) = 0;

		virtual void resetRadialDistortionParameters() = 0;

		double mTangentialParameter1;

		double mTangentialParameter2;

	};
}





