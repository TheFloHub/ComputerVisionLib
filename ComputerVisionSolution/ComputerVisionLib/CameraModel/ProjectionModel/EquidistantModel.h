/// 
/// @author	Florian Feuerstein
/// 
/// @date   02/2017
/// 
/// @class  EquidistantModel
/// 
///	@brief  
///

#pragma once

#include "ProjectionModel.h"

namespace Cvl
{
	class EquidistantModel : public ProjectionModel
	{

	public:

		EquidistantModel(
			double focalLengthX,
			double focalLengthY,
			double principalPointX,
			double principalPointY);

		virtual ~EquidistantModel() {}

		ProjectionModel::Uptr clone() const override;

		Eigen::Array2Xd project(Eigen::Array2Xd const& distortedPoints) const override;

		Eigen::Array2Xd unproject(Eigen::Array2Xd const& imagePoints) const override;

	private:



	};
}



