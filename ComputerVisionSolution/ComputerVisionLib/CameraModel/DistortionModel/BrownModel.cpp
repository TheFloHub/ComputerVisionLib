#include "BrownModel.h"

Cvl::BrownModel::BrownModel(double radialParameter1, double radialParameter2, double tangentialParameter1, double tangentialParameter2) :
DistortionModel(tangentialParameter1, tangentialParameter2),
mParameter1(radialParameter1),
mParameter2(radialParameter2)
{
}

Cvl::DistortionModel::Uptr Cvl::BrownModel::clone() const
{
	return DistortionModel::Uptr(new BrownModel(mParameter1, mParameter2, mTangentialParameter1, mTangentialParameter2));
}

Eigen::Array<double, 1, Eigen::Dynamic> Cvl::BrownModel::radialDistortionFactors(Eigen::Array<double, 1, Eigen::Dynamic> const & squaredNorms) const
{
	return squaredNorms * mParameter1 + squaredNorms.square() * mParameter2 + 1.0;
}

Eigen::VectorXd Cvl::BrownModel::getRadialDistortionParameters() const
{
	return (Eigen::VectorXd(2) << mParameter1, mParameter2).finished();
}

void Cvl::BrownModel::setRadialDistortionParameters(Eigen::VectorXd const & parameters)
{
	mParameter1 = parameters(0);
	mParameter2 = parameters(1);
}

void Cvl::BrownModel::resetRadialDistortionParameters()
{
	mParameter1 = 0.0;
	mParameter2 = 0.0;
}
