#include "IntrinsicCameraCalibration.h"
#include <ComputerVisionLib/Calibration/IntrinsicFishEyeCalibration.h>
#include <ComputerVisionLib/Reconstruction/HomographyCalculation.h>
#include <ComputerVisionLib/Reconstruction/ModelViewFromHomography.h>
#include <ComputerVisionLib/Reconstruction/ReconstructionError.h>
#include <ComputerVisionLib/Common/EigenHelpers.h>
#include <unsupported/Eigen/LevenbergMarquardt>
#include <Eigen/Dense>
#include <iostream>
#include <chrono>
#include <limits>

std::tuple<bool, double> Cvl::IntrinsicCameraCalibration::calibrate(
	Eigen::Array2Xd const & templatePoints,
	std::vector<Eigen::Array2Xd> const & imagePointsPerFrame,
	std::vector<std::vector<Match>> const & matchesPerFrame,
	CameraModel & cameraModel)
{
	cameraModel.setProjectionParameters(0.0, 0.0, 0.0, 0.0);
	cameraModel.resetDistortionParameters();

	if (imagePointsPerFrame.size() < 3 ||
		imagePointsPerFrame.size() != matchesPerFrame.size() ||
		templatePoints.size() < 4)
	{
		return std::make_tuple(false, std::numeric_limits<double>::max());
	}

	std::vector<std::tuple<Eigen::Array2Xd, Eigen::Array2Xd>> alignedPointsPerFrame = alignPoints(templatePoints, imagePointsPerFrame, matchesPerFrame);
	std::vector<Eigen::Matrix3d> homographies;

	// initialize the camera matrix
	if (cameraModel.isPinholeModel())
	{
		homographies = calculateHomographies(alignedPointsPerFrame);
		if (!initializeWithHomographies(homographies, cameraModel))
		{
			return std::make_tuple(false, std::numeric_limits<double>::max());
		}
	}
	else 
	{
		if (!IntrinsicFishEyeCalibration::calibrate(templatePoints, imagePointsPerFrame, matchesPerFrame, cameraModel))
		{
			return std::make_tuple(false, std::numeric_limits<double>::max());
		}

		std::vector<Eigen::Array2Xd> pinholePointsPerFrame;
		pinholePointsPerFrame.reserve(imagePointsPerFrame.size());
		for (Eigen::Array2Xd const & imagePoints : imagePointsPerFrame)
		{
			pinholePointsPerFrame.push_back(cameraModel.transformToPinhole(imagePoints));
		}
		std::vector<std::tuple<Eigen::Array2Xd, Eigen::Array2Xd>> alignedPinholePointsPerFrame = alignPoints(templatePoints, pinholePointsPerFrame, matchesPerFrame);
		homographies = calculateHomographies2(alignedPinholePointsPerFrame, alignedPointsPerFrame);
	}

	std::vector<Eigen::Affine3d> modelViews;
	bool success = false;
	double rmse = 0.0;
	Eigen::Affine3d modelView;
	auto alignedPointsIter = std::begin(alignedPointsPerFrame);
	auto homographyIter = std::begin(homographies);
	for (; alignedPointsIter != std::end(alignedPointsPerFrame); ++alignedPointsIter, ++homographyIter)
	{
		std::tie(success, rmse, modelView) = 
			ModelViewFromHomography::calculate(cameraModel, *homographyIter, std::get<0>(*alignedPointsIter), std::get<1>(*alignedPointsIter));
		if (success)
			modelViews.push_back(modelView);
	}

	if (modelViews.size() < 3)
	{
		return std::make_tuple(false, std::numeric_limits<double>::max());
	}

	return optimize(alignedPointsPerFrame, modelViews, cameraModel);
}

std::vector<std::tuple<Eigen::Array2Xd, Eigen::Array2Xd>> Cvl::IntrinsicCameraCalibration::alignPoints(
	Eigen::Array2Xd const & templatePoints, 
	std::vector<Eigen::Array2Xd> const & imagePointsPerFrame, 
	std::vector<std::vector<Match>> const & matchesPerFrame)
{
	std::vector<std::tuple<Eigen::Array2Xd, Eigen::Array2Xd>> alignedPointsPerFrame;
	alignedPointsPerFrame.reserve(matchesPerFrame.size());
	
	auto imagePointsIter = std::begin(imagePointsPerFrame);
	auto matchesIter = std::begin(matchesPerFrame);
	for (; imagePointsIter != std::end(imagePointsPerFrame); ++imagePointsIter, ++matchesIter)
	{
		alignedPointsPerFrame.push_back(Match::alignMatches(templatePoints, *imagePointsIter, *matchesIter));
	}
	return alignedPointsPerFrame;
}

std::vector<Eigen::Matrix3d> Cvl::IntrinsicCameraCalibration::calculateHomographies(
	std::vector<std::tuple<Eigen::Array2Xd, Eigen::Array2Xd>>& alignedPointsPerFrame)
{
	std::vector<Eigen::Matrix3d> homographies;
	homographies.reserve(alignedPointsPerFrame.size());

	bool success = false;
	Eigen::Matrix3d h = Eigen::Matrix3d::Zero();
	auto pointsPerFrameIter = std::begin(alignedPointsPerFrame);
	while (pointsPerFrameIter != std::end(alignedPointsPerFrame))
	{
		std::tie(success, h) = HomographyCalculation::calculate(std::get<0>(*pointsPerFrameIter), std::get<1>(*pointsPerFrameIter));
		if (success)
		{
			homographies.push_back(h);
			++pointsPerFrameIter;
		}
		else
		{
			pointsPerFrameIter = alignedPointsPerFrame.erase(pointsPerFrameIter);
		}
	}
	return homographies;
}

std::vector<Eigen::Matrix3d> Cvl::IntrinsicCameraCalibration::calculateHomographies2(
	std::vector<std::tuple<Eigen::Array2Xd, Eigen::Array2Xd>>& alignedPinholePointsPerFrame, 
	std::vector<std::tuple<Eigen::Array2Xd, Eigen::Array2Xd>>& alignedPointsPerFrame)
{
	std::vector<Eigen::Matrix3d> homographies;
	homographies.reserve(alignedPinholePointsPerFrame.size());

	bool success = false;
	Eigen::Matrix3d h = Eigen::Matrix3d::Zero();
	auto pinholePointsPerFrameIter = std::begin(alignedPinholePointsPerFrame);
	auto pointsPerFrameIter = std::begin(alignedPointsPerFrame);
	while (pinholePointsPerFrameIter != std::end(alignedPinholePointsPerFrame))
	{
		std::tie(success, h) = HomographyCalculation::calculate(std::get<0>(*pinholePointsPerFrameIter), std::get<1>(*pinholePointsPerFrameIter));
		if (success)
		{
			homographies.push_back(h);
			++pinholePointsPerFrameIter;
			++pointsPerFrameIter;
		}
		else
		{
			pinholePointsPerFrameIter = alignedPinholePointsPerFrame.erase(pinholePointsPerFrameIter);
			pointsPerFrameIter = alignedPointsPerFrame.erase(pointsPerFrameIter);
		}
	}
	return homographies;
}

bool Cvl::IntrinsicCameraCalibration::initializeWithHomographies(
	std::vector<Eigen::Matrix3d> const& homographies,
	CameraModel & cameraModel)
{
	if (homographies.size() < 3)
		return false;

	Eigen::MatrixXd V(homographies.size() * 2 + 1, 6);
	size_t numberOfEquations = 0;
	for (Eigen::Matrix3d const& h : homographies)
	{
		V(numberOfEquations, 0) = h(0, 0)*h(0, 1);
		V(numberOfEquations, 1) = h(0, 0)*h(1, 1) + h(1, 0)*h(0, 1);
		V(numberOfEquations, 2) = h(1, 0)*h(1, 1);
		V(numberOfEquations, 3) = h(2, 0)*h(0, 1) + h(0, 0)*h(2, 1);
		V(numberOfEquations, 4) = h(2, 0)*h(1, 1) + h(1, 0)*h(2, 1);
		V(numberOfEquations, 5) = h(2, 0)*h(2, 1);
		++numberOfEquations;

		V(numberOfEquations, 0) = h(0, 0)*h(0, 0) - h(0, 1)*h(0, 1);
		V(numberOfEquations, 1) = 2.0*(h(0, 0)*h(1, 0) - h(0, 1)*h(1, 1));
		V(numberOfEquations, 2) = h(1, 0)*h(1, 0) - h(1, 1)*h(1, 1);
		V(numberOfEquations, 3) = 2.0*(h(2, 0)*h(0, 0) - h(2, 1)*h(0, 1));
		V(numberOfEquations, 4) = 2.0*(h(2, 0)*h(1, 0) - h(2, 1)*h(1, 1));
		V(numberOfEquations, 5) = h(2, 0)*h(2, 0) - h(2, 1)*h(2, 1);
		++numberOfEquations;
	}

	V(numberOfEquations, 0) = 0.0;
	V(numberOfEquations, 1) = 1.0;
	V(numberOfEquations, 2) = 0.0;
	V(numberOfEquations, 3) = 0.0;
	V(numberOfEquations, 4) = 0.0;
	V(numberOfEquations, 5) = 0.0;
	++numberOfEquations;

	//auto startTime = std::chrono::high_resolution_clock::now();
	Eigen::Matrix<double, 6, 6> covar = V.transpose()*V;
	Eigen::SelfAdjointEigenSolver<Eigen::Matrix<double, 6, 6>> eigensolver(covar);
	if (eigensolver.info() != Eigen::Success) 
		return false;
	Eigen::VectorXd b = eigensolver.eigenvectors().col(0);
	//std::cout << "Time: " << std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::high_resolution_clock::now() - startTime).count() << std::endl;

	// another possibility
	//startTime = std::chrono::high_resolution_clock::now();
	//Eigen::VectorXd b2 = V.jacobiSvd(Eigen::ComputeFullV).matrixV().col(5);
	//std::cout << "b2: " << b2.transpose() << std::endl;
	//std::cout << "Time: " << std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::high_resolution_clock::now() - startTime).count() << std::endl;

	double temp1 = b(1) * b(3) - b(0) * b(4);
	double temp2 = b(0) * b(2) - b(1) * b(1);
	double v0 = temp1 / temp2;
	double lambda = b(5) - (b(3)*b(3) + v0*temp1) / b(0);
	double alpha = std::sqrt(lambda/b(0));
	double beta = std::sqrt(lambda*b(0)/temp2);
	double c = -b(1) * alpha * alpha * beta / lambda;
	double u0 = c * v0 / alpha - b(3) * alpha * alpha / lambda;

	cameraModel.setProjectionParameters(-alpha, -beta, u0, v0);
	return true;
}

std::tuple<bool, double> Cvl::IntrinsicCameraCalibration::optimize(
	std::vector<std::tuple<Eigen::Array2Xd, Eigen::Array2Xd>> const & alignedPointsPerFrame, 
	std::vector<Eigen::Affine3d> & modelViews, 
	CameraModel & cameraModel)
{
	Eigen::VectorXd intrinsicParamters = cameraModel.getAllParameters();
	size_t numberOfIntrinsicParameters = intrinsicParamters.size();
	Eigen::VectorXd allParameters = Eigen::VectorXd::Zero(numberOfIntrinsicParameters + 6 * modelViews.size());
	allParameters.head(numberOfIntrinsicParameters) = intrinsicParamters;

	size_t parameterIndex = numberOfIntrinsicParameters;
	for (auto const & modelView : modelViews)
	{
		allParameters.segment<6>(parameterIndex) = getModelViewParameters(modelView);
		parameterIndex += 6;
	}

#ifdef _DEBUG
	std::cout << "Initial parameters of intrinsic camera calibration: " << std::endl;
	std::cout << allParameters << std::endl;
#endif

	Functor functor(numberOfIntrinsicParameters, alignedPointsPerFrame, cameraModel);
	Eigen::NumericalDiff<Functor> numDiff(functor);
	Eigen::LevenbergMarquardt<Eigen::NumericalDiff<Functor>> lm(numDiff);
	lm.setMaxfev(1000);
	Eigen::LevenbergMarquardtSpace::Status status = lm.minimize(allParameters);

	// set the optimized parameters
	cameraModel.setAllParameters(allParameters.head(numberOfIntrinsicParameters));
	parameterIndex = numberOfIntrinsicParameters;
	for (auto & modelView : modelViews)
	{
		modelView = createModelView(allParameters.segment<6>(parameterIndex));
		parameterIndex += 6;
	}

	double error = std::sqrt(lm.fvec().squaredNorm() / lm.fvec().size());
	bool success = lm.info() == Eigen::Success || lm.info() == Eigen::NoConvergence;

#ifdef _DEBUG
	std::cout << "Optimized parameters of intrinsic camera calibration: " << std::endl;
	std::cout << allParameters << std::endl;
	std::cout << "Optimized RMSE of intrinsic camera calibration: " << error << std::endl;
	std::cout << "LM- info: " << lm.info() << " / status: " << status << " / iters: " << lm.iterations() << " / nfev: " << lm.nfev() << " / njev: " << lm.njev() << std::endl << std::endl;
#endif
	
	return std::make_tuple(success, error);
}

Cvl::IntrinsicCameraCalibration::Functor::Functor(
	size_t numberOfIntrinsicParameters, 
	std::vector<std::tuple<Eigen::Array2Xd, Eigen::Array2Xd>> const & alignedPointsPerFrame, 
	CameraModel & cameraModel) :
FunctorBase<double>((int)(numberOfIntrinsicParameters + 6 * alignedPointsPerFrame.size()), 0),
mNumberOfIntrinsicParameters(numberOfIntrinsicParameters),
mAlignedPointsPerFrame(alignedPointsPerFrame),
mCameraModel(cameraModel)
{
	for (auto const & frame : alignedPointsPerFrame)
	{
		mNumberOfValues += (int)(std::get<0>(frame).size());
	}
}

int Cvl::IntrinsicCameraCalibration::Functor::operator()(Eigen::VectorXd const & x, Eigen::VectorXd & fvec) const
{
	mCameraModel.setAllParameters(x.head(mNumberOfIntrinsicParameters));
	size_t parameterIndex = mNumberOfIntrinsicParameters;
	size_t valueIndex = 0;
	size_t numberOfValues = 0;
	for (auto const & frame : mAlignedPointsPerFrame)
	{
		Eigen::Array2Xd const & src = std::get<0>(frame);
		Eigen::Array2Xd const & dst = std::get<1>(frame);
		numberOfValues = src.size();
		fvec.segment(valueIndex, numberOfValues) = ReconstructionError::calculateDiff(x.segment<6>(parameterIndex), mCameraModel, src, dst);
		valueIndex += numberOfValues;
		parameterIndex += 6;
	}
	return 0;
}

