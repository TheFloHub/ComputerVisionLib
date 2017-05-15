#pragma once

#include <Eigen/Core>
#include <Eigen/Geometry>

namespace Cvl
{
	void fixRotationMatrix(Eigen::Matrix3d & matrix);

	Eigen::Matrix<double, 6, 1> getModelViewParameters(Eigen::Affine3d const & modelView);

	Eigen::Affine3d createModelView(Eigen::Matrix<double, 6, 1> const & parameters);
}





