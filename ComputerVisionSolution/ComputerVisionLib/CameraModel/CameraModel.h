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

namespace Cvl
{

	class CameraModel
	{

	public:

		CameraModel(DistortionModel::Uptr pDistortionModel, ProjectionModel::Uptr pProjectionModel);
		
		CameraModel(CameraModel const& other);

		~CameraModel();

		CameraModel& operator=(CameraModel const& other);

		Eigen::Array2Xd distortAndProject(Eigen::Array2Xd const& normalizedCameraPoints) const;

		Eigen::Array2Xd unprojectAndUndistort(Eigen::Array2Xd const& imagePoints) const;
		
		Eigen::Array2Xd transformTo(CameraModel const& other, Eigen::Array2Xd const& imagePoints) const;

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

	private:

		DistortionModel::Uptr mpDistortionModel;

		ProjectionModel::Uptr mpProjectionModel;

	};
}





