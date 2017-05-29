/// 
/// @author	Florian Feuerstein
/// 
/// @date   02/2017
/// 
/// @class  CameraModel
/// 
///	@brief  
///

#pragma once

#include "ProjectionModel\ProjectionModel.h"
#include "DistortionModel\DistortionModel.h"
#include <Eigen/Core>
#include <memory>
#include <type_traits>

namespace Cvl
{

	class CameraModel
	{

	public:

		CameraModel(DistortionModel::Uptr pDistortionModel, ProjectionModel::Uptr pProjectionModel);

		explicit CameraModel(ProjectionModel::Uptr pProjectionModel);
		
		CameraModel(CameraModel const& other);

		~CameraModel();

		CameraModel& operator=(CameraModel const& other);

		Eigen::Array2Xd distortAndProject(Eigen::Array2Xd const& normalizedCameraPoints) const;

		Eigen::Array2Xd unprojectAndUndistort(Eigen::Array2Xd const& imagePoints) const;
		
		Eigen::Array2Xd transformTo(CameraModel const& other, Eigen::Array2Xd const& imagePoints) const;

		Eigen::Array2Xd transformToPinhole(Eigen::Array2Xd const& imagePoints) const;

		bool isPinholeModel() const;

		Eigen::Matrix3d getPinholeCameraMatrix() const;

		Eigen::Matrix3d getInversePinholeCameraMatrix() const;

		Eigen::Vector4d getProjectionParameters() const;

		Eigen::VectorXd getDistortionParameters() const;

		Eigen::VectorXd getAllParameters() const;

		void setProjectionParameters(double focalLengthX, double focalLengthY, double principalPointX, double principalPointY);

		void setProjectionParameters(Eigen::Vector4d const & parameters);

		void setDistortionParameters(Eigen::VectorXd const & parameters);

		void resetDistortionParameters();

		void setAllParameters(Eigen::VectorXd const & parameters);

		template <class P>
		static CameraModel create(double focalLengthX, double focalLengthY, double principalPointX, double principalPointY);

		template <class P>
		static CameraModel create();

		template <class D, class P>
		static CameraModel create(double focalLengthX, double focalLengthY, double principalPointX, double principalPointY);

		template <class D, class P>
		static CameraModel create();

	private:

		DistortionModel::Uptr mpDistortionModel;

		ProjectionModel::Uptr mpProjectionModel;

	};

	template<class P>
	inline CameraModel CameraModel::create(double focalLengthX, double focalLengthY, double principalPointX, double principalPointY)
	{
		static_assert(std::is_base_of<ProjectionModel, P>::value, "P must be a descendant of ProjectionModel");
		return CameraModel(std::unique_ptr<P>(new P(focalLengthX, focalLengthY, principalPointX, principalPointY)));
	}

	template<class P>
	inline CameraModel CameraModel::create()
	{
		return CameraModel::create<P>(0.0, 0.0, 0.0, 0.0);
	}

	template <class D, class P>
	inline CameraModel CameraModel::create(double focalLengthX, double focalLengthY, double principalPointX, double principalPointY)
	{
		static_assert(std::is_base_of<DistortionModel, D>::value, "D must be a descendant of DistortionModel");
		static_assert(std::is_base_of<ProjectionModel, P>::value, "P must be a descendant of ProjectionModel");
		return CameraModel(
			std::unique_ptr<D>(new D),
			std::unique_ptr<P>(new P(focalLengthX, focalLengthY, principalPointX, principalPointY)));
	}

	template <class D, class P>
	inline CameraModel CameraModel::create()
	{
		return CameraModel::create<D, P>(0.0, 0.0, 0.0, 0.0);
	}
}





