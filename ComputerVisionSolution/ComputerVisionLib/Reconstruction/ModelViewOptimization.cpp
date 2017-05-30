#include "ModelViewOptimization.h"
#include <ComputerVisionLib/Common/EigenHelpers.h>
#include <ComputerVisionLib/Reconstruction/ReconstructionError.h>
#include <unsupported/Eigen/LevenbergMarquardt>
#include <iostream>

std::tuple<bool, double, Eigen::Affine3d> Cvl::ModelViewOptimization::optimize(
	CameraModel const & cameraModel, 
	Eigen::Affine3d const & modelView, 
	Eigen::Array2Xd const & srcPoints, 
	Eigen::Array2Xd const & dstPoints)
{
	Eigen::VectorXd parameters = getModelViewParameters(modelView);

	Functor2d functor(srcPoints, dstPoints, cameraModel);
	Eigen::NumericalDiff<Functor2d> numDiff(functor);
	Eigen::LevenbergMarquardt<Eigen::NumericalDiff<Functor2d>> lm(numDiff);

	Eigen::LevenbergMarquardtSpace::Status status = lm.minimize(parameters);

	Eigen::Affine3d optimizedModelView = createModelView(parameters);
	double error = std::sqrt(lm.fvec().squaredNorm() / lm.fvec().size());
	bool success = lm.info() == Eigen::Success;

#ifdef _DEBUG
	std::cout << "ModelViewOptimization optimized parameters: " << std::endl;
	std::cout << parameters.transpose() << std::endl;
	std::cout << "ModelViewOptimization optimized RMSE: " << error << std::endl;
	std::cout << "LM- info: " << lm.info() << " / status: " << status << " / iters: " << lm.iterations() << " / nfev: " << lm.nfev() << " / njev: " << lm.njev() << std::endl << std::endl;
#endif

	return std::make_tuple(success, error, optimizedModelView);
}

Cvl::ModelViewOptimization::Functor2d::Functor2d(
	Eigen::Array2Xd const & srcPoints, 
	Eigen::Array2Xd const & dstPoints, 
	CameraModel const & cameraModel) :
FunctorBase<double>(6, (int)srcPoints.size()),
mSrcPoints(srcPoints),
mDstPoints(dstPoints),
mCameraModel(cameraModel)
{
}

int Cvl::ModelViewOptimization::Functor2d::operator()(Eigen::VectorXd const & x, Eigen::VectorXd & fvec) const
{
	fvec = ReconstructionError::calculateDiff(x, mCameraModel, mSrcPoints, mDstPoints);
	return 0;
}
