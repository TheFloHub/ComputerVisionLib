#include "IntrinsicFishEyeCalibration.h"
#include <Eigen/Geometry>
#include <unsupported/Eigen/LevenbergMarquardt>
#include <iostream>

bool Cvl::IntrinsicFishEyeCalibration::calibrate(
	Eigen::Array2Xd const & templatePoints, 
	std::vector<Eigen::Array2Xd> const & imagePointsPerFrame, 
	std::vector<std::vector<Match>> const & matchesPerFrame, 
	CameraModel & cameraModel)
{
	if (imagePointsPerFrame.size() == 0 || templatePoints.cols() == 0 || matchesPerFrame.size() == 0)
		return false;

	// calculate intrinsic params for each frame
	double avgFocalLength = 0;
	Eigen::Vector2d avgPrincipalPoint(0, 0);
	size_t numValidFrames = 0;
	Eigen::Array2Xd allCirclePoints;
	Eigen::Array2Xd frameCirclePoints;
	Eigen::Vector4d frameParameters;
	bool success = false;

	for (size_t frameIndex = 0; frameIndex < matchesPerFrame.size(); ++ frameIndex)
	{
		std::vector<Match> const & matchesOfFrame = matchesPerFrame[frameIndex];
		Eigen::Array2Xd const & imagePoints = imagePointsPerFrame[frameIndex];
		std::vector<Eigen::VectorXd> matches;
		matches.reserve(matchesOfFrame.size());
		for (auto const& match : matchesOfFrame)
		{
			matches.push_back((Eigen::VectorXd(4) << templatePoints.col(match.mTemplateId), imagePoints.col(match.mMeasuredId)).finished());
		}

		std::tie(success, frameCirclePoints) = calculateIntrinsicParametersOfFrame(matches, cameraModel);

		if (success)
		{
			++numValidFrames;
			frameParameters = cameraModel.getProjectionParameters();
			avgFocalLength += frameParameters(0);
			avgPrincipalPoint += frameParameters.tail<2>();
			allCirclePoints.conservativeResize(2, allCirclePoints.cols() + frameCirclePoints.cols());
			allCirclePoints.rightCols(frameCirclePoints.cols()) = frameCirclePoints;
#ifdef _DEBUG
			std::cout << "FishEyeCalibration Frame " << frameIndex << ": " << frameParameters.transpose() << std::endl;
#endif
		}
	}

	if (numValidFrames == 0)
		return false;

	avgFocalLength /= (double) numValidFrames;
	avgPrincipalPoint /= (double)numValidFrames;
#ifdef _DEBUG
	std::cout << "FishEyeCalibration average: " << avgFocalLength << " " << avgPrincipalPoint.transpose() << std::endl;
#endif

	// optimize principal point and focal length over all frames
	Functor functor(cameraModel, allCirclePoints);
	Eigen::VectorXd parameters(3);
	parameters(0) = avgFocalLength;
	parameters(1) = avgPrincipalPoint.x();
	parameters(2) = avgPrincipalPoint.y();
	Eigen::NumericalDiff<Functor> numDiff(functor);
	Eigen::LevenbergMarquardt<Eigen::NumericalDiff<Functor>> lm(numDiff);
	Eigen::LevenbergMarquardtSpace::Status status = lm.minimize(parameters);

	// set the optimized parameters
	cameraModel.setProjectionParameters(parameters(0), parameters(0), parameters(1), parameters(2));

#ifdef _DEBUG
	double error = std::sqrt(lm.fvec().squaredNorm() / lm.fvec().size());
	std::cout << "FishEyeCalibration optimized parameters: " << parameters.transpose() << std::endl;
	std::cout << "FishEyeCalibration optimized RMSE: " << error << std::endl;
	std::cout << "LM- info: " << lm.info() << " / status: " << status << " / iters: " << lm.iterations() << " / nfev: " << lm.nfev() << " / njev: " << lm.njev() << std::endl << std::endl;
#endif

	return lm.info() == Eigen::Success;
}

std::tuple<bool, Eigen::Array2Xd> Cvl::IntrinsicFishEyeCalibration::calculateIntrinsicParametersOfFrame(
	std::vector<Eigen::VectorXd> & matches,
	CameraModel & cameraModel)
{
	if (matches.size() < 3)
		return std::make_tuple(false, Eigen::Array2Xd());

	double const maxCircleError = 2.0;
	size_t const minNumberOfCircles = 4;
	std::vector<Eigen::VectorXd> circlePoints;

	// split vertical lines
	std::sort(matches.begin(), matches.end(), compareColumnMajor);
	std::vector<std::vector<Eigen::VectorXd>> verticalLines;
	verticalLines.push_back(std::vector<Eigen::VectorXd>());
	double currentX = matches[0].x();
	for (size_t i = 0; i<matches.size(); ++i)
	{
		if (matches[i].x() != currentX)
		{
			verticalLines.push_back(std::vector<Eigen::VectorXd>());
			currentX = matches[i].x();
		}
		verticalLines.back().push_back(matches[i].tail<2>());
	}
	if (verticalLines.size() < minNumberOfCircles)
		return std::make_tuple(false, Eigen::Array2Xd());

	// calculate vertical circles
	std::vector<Circle> verticalCircles;
	for (auto const & verticalLine : verticalLines)
	{
		size_t numberOfPoints = verticalLine.size();
		if (numberOfPoints > 4)
		{
			size_t index1 = 0;
			size_t index2 = numberOfPoints / 2;
			size_t index3 = numberOfPoints - 1;
			Circle circle(vectorToEigen(verticalLine), index1, index2, index3);
			if (circle.getRadius() > 0.0 && circle.getError() < maxCircleError)
			{
				circlePoints.push_back(verticalLine[index1]);
				circlePoints.push_back(verticalLine[index2]);
				circlePoints.push_back(verticalLine[index3]);
				verticalCircles.push_back(circle);
			}
		}
	}
	if (verticalCircles.size() < minNumberOfCircles)
		return std::make_tuple(false, Eigen::Array2Xd());

	// split horizontal lines
	std::sort(matches.begin(), matches.end(), compareRowMajor);
	std::vector<std::vector<Eigen::VectorXd>> horizontalLines;
	horizontalLines.push_back(std::vector<Eigen::VectorXd>());
	double currentY = matches[0].y();
	for (size_t i = 0; i<matches.size(); ++i)
	{
		if (matches[i].y() != currentY)
		{
			horizontalLines.push_back(std::vector<Eigen::VectorXd>());
			currentY = matches[i].y();
		}
		horizontalLines.back().push_back(matches[i].tail<2>());
	}
	if (horizontalLines.size() < minNumberOfCircles)
		return std::make_tuple(false, Eigen::Array2Xd());

	// calculate horizontal circles
	std::vector<Circle> horizontalCircles;
	for (auto const & horizontalLine : horizontalLines)
	{
		size_t numberOfPoints = horizontalLine.size();
		if (numberOfPoints > 4)
		{
			size_t index1 = 0;
			size_t index2 = numberOfPoints / 2;
			size_t index3 = numberOfPoints - 1;
			Circle circle(vectorToEigen(horizontalLine), index1, index2, index3);
			if (circle.getRadius() > 0.0 && circle.getError() < maxCircleError)
			{
				circlePoints.push_back(horizontalLine[index1]);
				circlePoints.push_back(horizontalLine[index2]);
				circlePoints.push_back(horizontalLine[index3]);
				horizontalCircles.push_back(circle);
			}
		}
	}
	if (horizontalCircles.size() < minNumberOfCircles)
		return std::make_tuple(false, Eigen::Array2Xd());

	// intersect circles and get the 4 vanishing points
	Eigen::Vector2d vanishingPointHorizontal1;
	Eigen::Vector2d vanishingPointHorizontal2;
	Eigen::Vector2d vanishingPointVertical1;
	Eigen::Vector2d vanishingPointVertical2;
	if (!getVanishingPoints(horizontalCircles, vanishingPointHorizontal1, vanishingPointHorizontal2))
		return std::make_tuple(false, Eigen::Array2Xd());
	if (!getVanishingPoints(verticalCircles, vanishingPointVertical1, vanishingPointVertical2))
		return std::make_tuple(false, Eigen::Array2Xd());

	// calculate distortion center
	Eigen::Vector3d line1 = vanishingPointHorizontal1.homogeneous().cross(vanishingPointHorizontal2.homogeneous());
	Eigen::Vector3d line2 = vanishingPointVertical1.homogeneous().cross(vanishingPointVertical2.homogeneous());
	Eigen::Vector2d principalPoint = line1.cross(line2).hnormalized();

	// calculate the focal length
	double lowerBound = -1500;
	double upperBound = -150;
	int numSamples = 1000;
	double stepWidth = (upperBound - lowerBound) / numSamples;
	Eigen::Array2Xd circlePointsArray = vectorToEigen(circlePoints);

	Functor functor(cameraModel, circlePointsArray);
	Eigen::VectorXd x(3);
	x(1) = principalPoint.x();
	x(2) = principalPoint.y();
	Eigen::VectorXd fvec(circlePointsArray.cols() / 3);
	double minError = std::numeric_limits<double>::max();
	double error = 0.0;
	double focalLength = 0.0;
	for (double f = lowerBound; f < upperBound; f += stepWidth)
	{
		x(0) = f;
		functor(x, fvec);
		error = fvec.sum();
		if (error < minError)
		{
			minError = error;
			focalLength = f;
		}
	}
	cameraModel.setProjectionParameters(focalLength, focalLength, principalPoint.x(), principalPoint.y());

	return std::make_tuple(true, circlePointsArray);
}


bool Cvl::IntrinsicFishEyeCalibration::getVanishingPoints(
	std::vector<Circle> const & circles, 
	Eigen::Vector2d & vanishingPoint1, 
	Eigen::Vector2d & vanishingPoint2)
{
	// intersect all circles
	Eigen::Vector2d intersection1;
	Eigen::Vector2d intersection2;
	size_t maxIntersections = circles.size()*(circles.size()+1);
	Eigen::Matrix2Xd intersections(2, maxIntersections);
	size_t numIntersections = 0;
	for (size_t i = 0; i<circles.size(); ++i)
	{
		for (size_t j = i + 1; j<circles.size(); ++j)
		{
			if (circles[i].intersect(circles[j], intersection1, intersection2))
			{
				intersections.col(numIntersections) = intersection1;
				++numIntersections;
				intersections.col(numIntersections) = intersection2;
				++numIntersections;
			}
		}
	}
	intersections.conservativeResize(2, numIntersections);

	if (numIntersections<2)
		return false;

	// separate points in two groups and average
	Eigen::Vector2d avgAll(0, 0);
	for (Eigen::Index i = 0; i < intersections.cols(); ++i)
	{
		avgAll += intersections.col(i);
	}
	avgAll /= (double)intersections.cols();


	double varianceX = 0;
	double varianceY = 0;
	Eigen::Vector2d diff;
	for (Eigen::Index i = 0; i<intersections.cols(); ++i)
	{
		diff = intersections.col(i) - avgAll;
		varianceX += diff.x()*diff.x();
		varianceY += diff.y()*diff.y();
	}
	varianceX /= (double)intersections.cols();
	varianceY /= (double)intersections.cols();

	Eigen::Vector2d avgPoint1(0.0, 0.0);
	Eigen::Vector2d avgPoint2(0.0, 0.0);
	if (varianceX>varianceY)
	{
		for (Eigen::Index i = 0; i<intersections.cols(); i += 2)
		{
			if (intersections.col(i).x() < avgAll.x())
			{
				avgPoint1 += intersections.col(i);
				avgPoint2 += intersections.col(i + 1);
			}
			else {
				avgPoint1 += intersections.col(i + 1);
				avgPoint2 += intersections.col(i);
			}
		}
	}
	else
	{
		for (Eigen::Index i = 0; i<intersections.cols(); i += 2)
		{
			if (intersections.col(i).y() < avgAll.y())
			{
				avgPoint1 += intersections.col(i);
				avgPoint2 += intersections.col(i + 1);
			}
			else {
				avgPoint1 += intersections.col(i + 1);
				avgPoint2 += intersections.col(i);
			}
		}
	}
	vanishingPoint1 = 2.0*avgPoint1 / (double)intersections.cols();
	vanishingPoint2 = 2.0*avgPoint2 / (double)intersections.cols();
	return true;
}

bool Cvl::IntrinsicFishEyeCalibration::compareColumnMajor(Eigen::VectorXd const & match1, Eigen::VectorXd const & match2)
{
	if (match1(0) < match2(0))
	{
		return true;
	}
	else if (match1(0) == match2(0) && match1(1) < match2(1))
	{
		return true;
	}
	return false;
}

bool Cvl::IntrinsicFishEyeCalibration::compareRowMajor(Eigen::VectorXd const & match1, Eigen::VectorXd const & match2)
{
	if (match1(1) < match2(1))
	{
		return true;
	}
	else if (match1(1) == match2(1) && match1(0) < match2(0))
	{
		return true;
	}
	return false;
}

Eigen::Array2Xd Cvl::IntrinsicFishEyeCalibration::vectorToEigen(std::vector<Eigen::VectorXd> const & vector)
{
	Eigen::Array2Xd eigenArray(2, vector.size());
	size_t i = 0;
	for (Eigen::VectorXd const & p : vector)
	{
		eigenArray.col(i) = p.head<2>();
		++i;
	}
	return eigenArray;
}

Cvl::IntrinsicFishEyeCalibration::Functor::Functor(CameraModel & cameraModel, Eigen::Array2Xd const & points) :
FunctorBase<double>(3, (int)points.cols()/3),
mCameraModel(cameraModel),
mPoints(points)
{
}

int Cvl::IntrinsicFishEyeCalibration::Functor::operator() (Eigen::VectorXd const & x, Eigen::VectorXd & fvec) const
{
	mCameraModel.setProjectionParameters(x(0), x(0), x(1), x(2));
	Eigen::Array2Xd pinholePoints = mCameraModel.transformToPinhole(mPoints);
	Eigen::Matrix3d matrix;

	Eigen::Index i = 0;
	Eigen::Index j = 0;
	for (; i < pinholePoints.cols(); i += 3, ++j)
	{
		matrix.col(0) = pinholePoints.col(i  ).matrix().homogeneous();
		matrix.col(1) = pinholePoints.col(i+1).matrix().homogeneous();
		matrix.col(2) = pinholePoints.col(i+2).matrix().homogeneous();
		fvec(j) = std::abs(matrix.determinant())/2.0;
	}
	return 0;
}
