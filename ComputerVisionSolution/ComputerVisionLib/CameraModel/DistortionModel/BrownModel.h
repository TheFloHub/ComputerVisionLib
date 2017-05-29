/// 
/// @author	Florian Feuerstein
/// 
/// @date   02/2017
/// 
/// @class  BrownModel
/// 
///	@brief  
///

#pragma once

#include "DistortionModel.h"

namespace Cvl
{
	class BrownModel : public DistortionModel
	{

	public:

		BrownModel();
		
		BrownModel(double radialParameter1, double radialParameter2, double tangentialParameter1, double tangentialParameter2);

		virtual ~BrownModel() {}

		DistortionModel::Uptr clone() const override;

	protected:

		Eigen::Array<double, 1, Eigen::Dynamic> radialDistortionFactors(Eigen::Array<double, 1, Eigen::Dynamic> const& squaredNorms) const override;

		virtual double radialDistortionDerivative(double r) const override;

		Eigen::VectorXd getRadialDistortionParameters() const override;

		void setRadialDistortionParameters(Eigen::VectorXd const& parameters) override;

		void resetRadialDistortionParameters() override;

		double mParameter1;

		double mParameter2;

	};
}



