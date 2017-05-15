#include "EigenHelpers.h"
#include <Eigen/Geometry>

void Cvl::fixRotationMatrix(Eigen::Matrix3d & matrix)
{
	Eigen::JacobiSVD<Eigen::Matrix3d> svd(matrix, Eigen::ComputeFullU | Eigen::ComputeFullV);
	matrix = svd.matrixU() * svd.matrixV().transpose();
}

Eigen::Matrix<double, 6, 1> Cvl::getModelViewParameters(Eigen::Affine3d const & modelView)
{
	// parameters = (angle*(r1, r2, r3)^T, t1, t2, t3)^T
	Eigen::Matrix<double, 6, 1> parameters;
	Eigen::AngleAxisd angleAxis;
	angleAxis = modelView.linear();
	parameters.head<3>() = angleAxis.axis()*angleAxis.angle();
	parameters.tail<3>() = modelView.translation();
	return parameters;
}

Eigen::Affine3d Cvl::createModelView(Eigen::Matrix<double, 6, 1> const & parameters)
{
	// the reconstructed modelview from the parameters
	Eigen::Affine3d modelView;

	// the first three parameters are the rotation and angle (angle*(r1, r2, r3)^T)
	Eigen::Vector3d axis = parameters.head<3>();
	double angle = axis.norm();
	axis *= 1.0 / angle;
	modelView.linear() = Eigen::AngleAxisd(angle, axis).toRotationMatrix();

	// the last three parameters are the translation
	modelView.translation() = parameters.tail<3>();

	return modelView;
}
