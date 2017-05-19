/// 
/// @author	Florian Feuerstein
/// 
/// @date   02/2017
/// 
/// @class  OrthographicModel
/// 
///	@brief  
///

#pragma once

#include "ProjectionModel.h"

namespace Cvl
{
	class OrthographicModel : public ProjectionModel
	{

	public:

		OrthographicModel(
			double focalLengthX,
			double focalLengthY,
			double principalPointX,
			double principalPointY);

		virtual ~OrthographicModel() {}

		ProjectionModel::Uptr clone() const override;

		Eigen::Array2Xd project(Eigen::Array2Xd const& distortedPoints) const override;

		//Eigen::Array2Xd unproject(Eigen::Array2Xd const& imagePoints) const override;

	private:



	};
}



