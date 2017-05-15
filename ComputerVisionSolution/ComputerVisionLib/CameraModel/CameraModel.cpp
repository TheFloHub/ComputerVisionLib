#include "CameraModel.h"
#include <ComputerVisionLib/CameraModel/ProjectionModel/PinholeModel.h>

Cvl::CameraModel::CameraModel(DistortionModel::Uptr pDistortionModel, ProjectionModel::Uptr pProjectionModel) :
mpDistortionModel(std::move(pDistortionModel)),
mpProjectionModel(std::move(pProjectionModel))
{
}

Cvl::CameraModel::CameraModel(CameraModel const & other) :
mpDistortionModel(other.mpDistortionModel->clone()),
mpProjectionModel(other.mpProjectionModel->clone())
{
}

Cvl::CameraModel::~CameraModel()
{
}

Eigen::Array2Xd Cvl::CameraModel::distortAndProject(Eigen::Array2Xd const & normalizedCameraPoints) const
{
	return mpProjectionModel->project(mpDistortionModel->distort(normalizedCameraPoints));
}

bool Cvl::CameraModel::isPinholeModel() const
{
	return (dynamic_cast<Cvl::PinholeModel*>(mpProjectionModel.get()) != nullptr);
}

Eigen::Matrix3d Cvl::CameraModel::getPinholeCameraMatrix() const
{
	return mpProjectionModel->getPinholeCameraMatrix();
}

Eigen::Matrix3d Cvl::CameraModel::getInversePinholeCameraMatrix() const
{
	return mpProjectionModel->getInversePinholeCameraMatrix();
}

Eigen::Vector4d Cvl::CameraModel::getProjectionParameters() const
{
	return mpProjectionModel->getParameters();
}

Eigen::VectorXd Cvl::CameraModel::getDistortionParameters() const
{
	return mpDistortionModel->getParameters();
}

Eigen::VectorXd Cvl::CameraModel::getAllParameters() const
{
	Eigen::VectorXd distortionParameters = mpDistortionModel->getParameters();
	return (Eigen::VectorXd(4 + distortionParameters.rows()) << mpProjectionModel->getParameters(), distortionParameters).finished();
}

void Cvl::CameraModel::setProjectionParameters(double focalLengthX, double focalLengthY, double principalPointX, double principalPointY)
{
	mpProjectionModel->setParameters(focalLengthX, focalLengthY, principalPointX, principalPointY);
}

void Cvl::CameraModel::setProjectionParameters(Eigen::Vector4d const & parameters)
{
	mpProjectionModel->setParameters(parameters);
}

void Cvl::CameraModel::setDistortionParameters(Eigen::VectorXd const & parameters)
{
	mpDistortionModel->setParameters(parameters);
}

void Cvl::CameraModel::resetDistortionParameters()
{
	mpDistortionModel->resetParameters();
}

void Cvl::CameraModel::setAllParameters(Eigen::VectorXd const & parameters)
{
	mpProjectionModel->setParameters(parameters.head(4));
	mpDistortionModel->setParameters(parameters.tail(parameters.rows() - 4));
}

Cvl::CameraModel& Cvl::CameraModel::operator=(CameraModel const & other)
{
	if (this != &other)
	{
		mpDistortionModel = other.mpDistortionModel->clone();
		mpProjectionModel = other.mpProjectionModel->clone();
	}
	return *this;
}
