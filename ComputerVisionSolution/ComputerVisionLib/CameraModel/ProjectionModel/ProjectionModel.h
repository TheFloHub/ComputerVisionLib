/// 
/// @author	Florian Feuerstein
/// 
/// @date   02/2017
/// 
/// @class  ProjectionModel
/// 
///	@brief  
///

#pragma once

#include <Eigen/Core>
#include <memory>

namespace Cvl
{
	class ProjectionModel
	{

	public:

		using Uptr = std::unique_ptr<ProjectionModel>;

		ProjectionModel(
			double focalLengthX,
			double focalLengthY,
			double principalPointX,
			double principalPointY) :
			mFocalLengthX(focalLengthX),
			mFocalLengthY(focalLengthY),
			mPrincipalPointX(principalPointX),
			mPrincipalPointY(principalPointY)
		{}

		virtual ~ProjectionModel() {}

		virtual ProjectionModel::Uptr clone() const = 0;

		virtual Eigen::Array2Xd project(Eigen::Array2Xd const& distortedPoints) const = 0;

		virtual Eigen::Array2Xd unproject(Eigen::Array2Xd const& imagePoints) const = 0;

		Eigen::Matrix3d getPinholeCameraMatrix() const;

		Eigen::Matrix3d getInversePinholeCameraMatrix() const;

		void setParameters(double focalLengthX, double focalLengthY, double principalPointX, double principalPointY);

		void setParameters(Eigen::Vector4d const & parameters);

		Eigen::Vector4d getParameters() const;

	protected:

		double mFocalLengthX;

		double mFocalLengthY;

		double mPrincipalPointX;

		double mPrincipalPointY;

	};
}




