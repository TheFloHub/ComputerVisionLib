/// 
/// @author	Florian Feuerstein
/// 
/// @date   02/2017
/// 
/// @class  EquisolidModel
/// 
///	@brief  
///

#pragma once

#include "ProjectionModel.h"

namespace Cvl
{
	class EquisolidModel : public ProjectionModel
	{

	public:

		EquisolidModel(
			double focalLengthX,
			double focalLengthY,
			double principalPointX,
			double principalPointY);

		virtual ~EquisolidModel() {}

		ProjectionModel::Uptr clone() const override;

		Eigen::Array2Xd project(Eigen::Array2Xd const& distortedPoints) const override;

		//Eigen::Array2Xd unproject(Eigen::Array2Xd const& imagePoints) const override;

	private:



	};
}



