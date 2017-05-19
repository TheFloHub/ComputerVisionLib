/// 
/// @author	Florian Feuerstein
/// 
/// @date   02/2017
/// 
/// @class  StereographicModel
/// 
///	@brief  
///

#pragma once

#include "ProjectionModel.h"

namespace Cvl
{
	class StereographicModel : public ProjectionModel
	{

	public:

		StereographicModel(
			double focalLengthX,
			double focalLengthY,
			double principalPointX,
			double principalPointY);

		virtual ~StereographicModel() {}

		ProjectionModel::Uptr clone() const override;

		Eigen::Array2Xd project(Eigen::Array2Xd const& distortedPoints) const override;

		//Eigen::Array2Xd unproject(Eigen::Array2Xd const& imagePoints) const override;

	private:



	};
}



