#include "ChessboardSegmentation.h"
#include <opencv2/imgproc.hpp>
#include <opencv2/highgui.hpp>
#include <Eigen/Dense>
#include <limits>
#include <vector>
#include <utility>
#include <algorithm>

using namespace Eigen;

Cvl::ChessboardSegmentation::ChessboardSegmentation()
{
}

Cvl::ChessboardSegmentation::~ChessboardSegmentation()
{
}

Cvl::ChessboardSegmentation::Result Cvl::ChessboardSegmentation::match(
	cv::Mat const & image, 
	Eigen::Array2Xd const& corners, 
	size_t const cornersPerRow,
	size_t const cornersPerCol,
	size_t blockSize)
{
	Result result;

	if (corners.cols() == 0)
		return result;

	// binarize image
	if (blockSize < 5)
	{
		blockSize = (3 * image.cols) / cornersPerRow;
	}
	if (blockSize % 2 == 0)
		++blockSize;

	cv::Mat binarizedImage;
	cv::adaptiveThreshold(image, binarizedImage, 255, cv::ADAPTIVE_THRESH_MEAN_C, cv::THRESH_BINARY, (int)blockSize, 0);

	// initial connected corners
	std::vector<std::vector<Eigen::Array3i>> connectedCorners =	traverseAllCorners(corners, binarizedImage);
	if (connectedCorners.empty())
		return result;
	std::vector<Eigen::Array3i>& chessboardMatches = connectedCorners[0];

	// sort corners
	size_t detectedCornersPerRow = 0;
	size_t detectedCornersPerCol = 0;
	setOriginZero(chessboardMatches, detectedCornersPerRow, detectedCornersPerCol);
	checkOrientation(corners, binarizedImage, chessboardMatches, detectedCornersPerRow, detectedCornersPerCol);
	checkRowsAndCols(cornersPerRow, cornersPerCol, chessboardMatches, detectedCornersPerRow, detectedCornersPerCol);
	
	result.mMatches = assignTemplateIndices(chessboardMatches, cornersPerRow, cornersPerCol);
	result.mUnambiguous = cornersPerRow == detectedCornersPerRow && cornersPerCol == detectedCornersPerCol;
	result.mSuccessful = true;
	return result;
}

std::vector<std::vector<Eigen::Array3i>> Cvl::ChessboardSegmentation::traverseAllCorners(Eigen::Array2Xd const & corners, cv::Mat const & image)
{
	std::vector<bool> matchedCorners(corners.cols(), false);
	std::vector<std::vector<Eigen::Array3i>> connectedCorners;

	for (size_t c = 0; c < (size_t)corners.cols(); ++c)
	{
		if (!matchedCorners[c])
		{
			std::vector<Eigen::Array3i> matches = startTraversing(c, corners, image);
			if (!matches.empty())
			{
				for (Eigen::Array3i const& match : matches)
				{
					matchedCorners[match.z()] = true;
				}
				connectedCorners.push_back(matches);
			}
		}
	}
	std::sort(connectedCorners.begin(), connectedCorners.end(),
		[](auto const& a, auto const& b) -> bool 
		{
			return a.size() > b.size();
		});
	return connectedCorners;
}

std::vector<Eigen::Array3i> Cvl::ChessboardSegmentation::startTraversing(size_t const startIndex, Eigen::Array2Xd const & corners, cv::Mat const & image)
{
	std::vector<Eigen::Array3i> chessboardMatches;
	Array2d midpoint = corners.col(startIndex);

	// Search for principal directions
	std::vector<int> neighborship = getNeighborship(midpoint, corners);
	std::vector<int> validPoints;
	for (size_t n = 0; n<neighborship.size(); n++)
	{
		if (isPrincipalDirection(midpoint, corners.col(neighborship[n]), image))
		{
			validPoints.push_back(neighborship[n]);
		}
	}

	if (validPoints.size() != 4)
	{
		return chessboardMatches;
	}

	// Assign directions
	Array<double, 2, 4> directions;
	int cornerIndices[4] = { validPoints[0], -1, -1, -1 };

	directions.col(UP) = (corners.col(cornerIndices[UP]) - midpoint).matrix().normalized();
	Array2d temp = (corners.col(validPoints[1]) - midpoint).matrix().normalized();
	if ((directions.col(UP) + temp).matrix().norm() < 0.2)
	{
		cornerIndices[DOWN] = validPoints[1];
		directions.col(DOWN) = temp;
		cornerIndices[LEFT] = validPoints[2];
		directions.col(LEFT) = (corners.col(cornerIndices[LEFT]) - midpoint).matrix().normalized();
		cornerIndices[RIGHT] = validPoints[3];
		directions.col(RIGHT) = (corners.col(cornerIndices[RIGHT]) - midpoint).matrix().normalized();
	}
	else
	{
		cornerIndices[LEFT] = validPoints[1];
		directions.col(LEFT) = temp;
		temp = (corners.col(validPoints[2]) - midpoint).matrix().normalized();
		if ((directions.col(UP) + temp).matrix().norm() < 0.2)
		{
			cornerIndices[DOWN] = validPoints[2];
			directions.col(DOWN) = temp;
			cornerIndices[RIGHT] = validPoints[3];
			directions.col(RIGHT) = (corners.col(cornerIndices[RIGHT]) - midpoint).matrix().normalized();

		}
		else
		{
			cornerIndices[DOWN] = validPoints[3];
			directions.col(DOWN) = (corners.col(cornerIndices[DOWN]) - midpoint).matrix().normalized();
			cornerIndices[RIGHT] = validPoints[2];
			directions.col(RIGHT) = temp;
		}
	}

	// Start the show
	std::vector<bool> traversedCorners(corners.cols(), false);
	traversedCorners[startIndex] = true;
	chessboardMatches.push_back(Array3i(0, 0, (int)startIndex));
	processCorner(cornerIndices[UP], Array2i(0, 1), directions, corners, traversedCorners, chessboardMatches, image);
	processCorner(cornerIndices[DOWN], Array2i(0, -1), directions, corners, traversedCorners, chessboardMatches, image);
	processCorner(cornerIndices[LEFT], Array2i(-1, 0), directions, corners, traversedCorners, chessboardMatches, image);
	processCorner(cornerIndices[RIGHT], Array2i(1, 0), directions, corners, traversedCorners, chessboardMatches, image);

	return chessboardMatches;
}

void Cvl::ChessboardSegmentation::processCorner(
	size_t const pointIndex,
	Array2i const& relativePosition,
	Array<double, 2, 4> const& directions,
	Array2Xd const& corners,
	std::vector<bool>& traversedCorners,
	std::vector<Array3i>& chessboardMatches,
	cv::Mat const& image)
{
	// Check if already processed
	if (traversedCorners[pointIndex])
		return;

	// Copy point and set it traversed
	traversedCorners[pointIndex] = true;
	Array2d midpoint = corners.col(pointIndex);
	chessboardMatches.push_back(Array3i(relativePosition(0), relativePosition(1), (int)pointIndex));

	// Search for principal directions
	std::vector<int> neighborship = getNeighborship(midpoint, corners);
	std::vector<int> validPoints;
	Array2Xd validDirections(2, neighborship.size());
	for (size_t i = 0; i<neighborship.size(); i++)
	{
		if (isPrincipalDirection(midpoint, corners.col(neighborship[i]), image))
		{
			validDirections.col(validPoints.size()) = (corners.col(neighborship[i]) - midpoint).matrix().normalized().array();
			validPoints.push_back(neighborship[i]);
		}
	}
	validDirections.conservativeResize(2, validPoints.size());

	if (validPoints.empty() || validPoints.size()>4)
		return;

	// Sort principal directions
	int cornerIndices[4] = { -1, -1, -1, -1 };
	Array<double, 2, 4> newDirections = directions;

	for (size_t j = 0; j<4; ++j)
	{
		double minError = std::numeric_limits<double>::max();
		double error = 0;
		int index = -1;

		for (Index i = 0; i<validDirections.cols(); ++i)
		{
			error = (validDirections.col(i) - directions.col(j)).matrix().norm();
			if (error < minError)
			{
				minError = error;
				index = static_cast<int>(i);
			}
		}
		if (minError < 0.4)
		{
			newDirections.col(j) = validDirections.col(index);
			cornerIndices[j] = validPoints[index];
		}
	}

	// Further traversing
	for (size_t i = 0; i<4; ++i)
	{
		if (cornerIndices[i] != -1)
		{
			switch (i)
			{
			case UP:
				processCorner(cornerIndices[i], relativePosition + Array2i(0, 1), newDirections, corners, traversedCorners, chessboardMatches, image);
				break;
			case DOWN:
				processCorner(cornerIndices[i], relativePosition + Array2i(0, -1), newDirections, corners, traversedCorners, chessboardMatches, image);
				break;
			case LEFT:
				processCorner(cornerIndices[i], relativePosition + Array2i(-1, 0), newDirections, corners, traversedCorners, chessboardMatches, image);
				break;
			case RIGHT:
				processCorner(cornerIndices[i], relativePosition + Array2i(1, 0), newDirections, corners, traversedCorners, chessboardMatches, image);
			}
		}
	}
}

std::vector<int> Cvl::ChessboardSegmentation::getNeighborship(Eigen::Array2d const & point, Eigen::Array2Xd const & points, size_t numberOfNextNeighbors)
{
	std::vector< std::pair<double, int> > distanceIdPairs;
	distanceIdPairs.reserve(points.cols());
	double squaredDistance;
	for (int i = 0; i<points.cols(); ++i)
	{
		squaredDistance = (point - points.col(i)).matrix().squaredNorm();
		if (squaredDistance > 0.25)
			distanceIdPairs.push_back(std::pair<double, int>(squaredDistance, i));
	}

	if (distanceIdPairs.size() < numberOfNextNeighbors)
		numberOfNextNeighbors = distanceIdPairs.size();
	std::partial_sort(distanceIdPairs.begin(), distanceIdPairs.begin() + numberOfNextNeighbors, distanceIdPairs.end());

	std::vector<int> neighbors(numberOfNextNeighbors);
	for (size_t i = 0; i<numberOfNextNeighbors; ++i)
		neighbors[i] = distanceIdPairs[i].second;
	return neighbors;
}

bool Cvl::ChessboardSegmentation::isPrincipalDirection(Eigen::Array2d const & point1, Eigen::Array2d const & point2, cv::Mat const & image)
{
	Array2d direction = point2 - point1;
	double distance = direction.matrix().norm();
	Array2d normDirection = direction.matrix().normalized();
	Array2d offset = Array2d(normDirection.y(), -normDirection.x())*(distance / 8.0);
	std::vector<int> samplesRhs;
	samplesRhs.reserve(100);
	std::vector<int> samplesLhs;
	samplesLhs.reserve(100);
	double currentDistance = distance / 4.0;
	double endDistance = currentDistance*3.0;
	Array2d samplePositionRhs, samplePositionLhs, temp;
	int xLhs, yLhs, xRhs, yRhs;
	int numSamples = 0;
	int sampleValueLhs;
	int sampleValueRhs;
	double meanValueLhs = 0.0;
	double meanValueRhs = 0.0;
	int imageWidth = image.cols;
	int imageHeight = image.rows;

	// Sample gray values on either side of the line
	for (; currentDistance<endDistance; ++currentDistance)
	{
		temp = point1 + (normDirection*currentDistance);
		samplePositionLhs = temp + offset;
		samplePositionRhs = temp - offset;
		xLhs = (int)std::round(samplePositionLhs.x());
		yLhs = (int)std::round(samplePositionLhs.y());
		xRhs = (int)std::round(samplePositionRhs.x());
		yRhs = (int)std::round(samplePositionRhs.y());
		if (xLhs<0 || xLhs >= imageWidth || yLhs<0 || yLhs >= imageHeight ||
			xRhs<0 || xRhs >= imageWidth || yRhs<0 || yRhs >= imageHeight)
		{
			return false;
		}
		sampleValueLhs = (int)image.at<unsigned char>(yLhs, xLhs);
		sampleValueRhs = (int)image.at<unsigned char>(yRhs, xRhs);
		samplesLhs.push_back(sampleValueLhs);
		samplesRhs.push_back(sampleValueRhs);
		meanValueLhs += sampleValueLhs;
		meanValueRhs += sampleValueRhs;
		++numSamples;
	}

	if (numSamples<4)
		return false;

	meanValueLhs /= numSamples;
	meanValueRhs /= numSamples;

	// Is the difference of mean values great enough?
	if (fabs(meanValueRhs - meanValueLhs)<100)
		return false;

	// Get standard deviation of either side
	double summedLhs = 0;
	double summedRhs = 0;
	double diff;
	for (int i = 0; i<numSamples; ++i)
	{
		diff = samplesLhs[i] - meanValueLhs;
		summedLhs += diff*diff;
		diff = samplesRhs[i] - meanValueRhs;
		summedRhs += diff*diff;
	}

	// No principal direction if the intensity deviation on one side is too large 
	if (sqrt(summedLhs / numSamples)>30.0 || sqrt(summedRhs / numSamples)>30.0)
		return false;

	return true;
}

void Cvl::ChessboardSegmentation::setOriginZero(std::vector<Eigen::Array3i>& chessboardMatches, size_t & detectedCornersPerRow, size_t & detectedCornersPerCol)
{
	// Make all coordinates positive and relative to 0/0
	int minX = std::numeric_limits<int>::max();
	int minY = std::numeric_limits<int>::max();
	int maxX = std::numeric_limits<int>::min();
	int maxY = std::numeric_limits<int>::min();

	for (size_t i = 0; i < chessboardMatches.size(); ++i)
	{
		if (chessboardMatches[i].x() < minX)
			minX = chessboardMatches[i].x();
		if (chessboardMatches[i].y() < minY)
			minY = chessboardMatches[i].y();
		if (chessboardMatches[i].x() > maxX)
			maxX = chessboardMatches[i].x();
		if (chessboardMatches[i].y() > maxY)
			maxY = chessboardMatches[i].y();
	}

	for (size_t i = 0; i<chessboardMatches.size(); ++i)
	{
		chessboardMatches[i].x() -= minX;
		chessboardMatches[i].y() -= minY;
	}

	detectedCornersPerRow = (size_t)(maxX - minX) + 1;
	detectedCornersPerCol = (size_t)(maxY - minY) + 1;
}

void Cvl::ChessboardSegmentation::checkOrientation(
	Eigen::Array2Xd const & corners, 
	cv::Mat const & image, 
	std::vector<Eigen::Array3i>& chessboardMatches, 
	size_t & detectedCornersPerRow, 
	size_t & detectedCornersPerCol)
{
	// Check if the chessboard's z-axis is negative
	Array2d imgPoint00 = corners.col(chessboardMatches[0].z());
	Array2d imgPoint10(-1.0, -1.0);
	Array2d imgPoint01(-1.0, -1.0);
	Array2i patternPoint10 = chessboardMatches[0].head(2);
	patternPoint10.x() += 1;
	Array2i patternPoint01 = chessboardMatches[0].head(2);
	patternPoint01.y() += 1;

	for (size_t i = 0; i<chessboardMatches.size(); ++i)
	{
		if ((chessboardMatches[i].head(2) == patternPoint10).all())
			imgPoint10 = corners.col(chessboardMatches[i].z());
		else if ((chessboardMatches[i].head(2) == patternPoint01).all())
			imgPoint01 = corners.col(chessboardMatches[i].z());
	}

	if (imgPoint10.x() < 0 || imgPoint01.x() < 0)
		return;

	Vector2d xDir = imgPoint10.matrix() - imgPoint00.matrix();
	xDir.normalize();
	Vector2d yDir = imgPoint01.matrix() - imgPoint00.matrix();
	yDir.normalize();
	Vector3d xDirVec3(xDir.x(), xDir.y(), 0.0);
	Vector3d yDirVec3(yDir.x(), yDir.y(), 0.0);
	Vector3d zDirVec3 = xDirVec3.cross(yDirVec3);
	if (zDirVec3.z() > 0)
	{
		// switch x and y axis
		int tempInt = 0;
		for (size_t i = 0; i<chessboardMatches.size(); ++i)
		{
			tempInt = chessboardMatches[i].x();
			chessboardMatches[i].x() = chessboardMatches[i].y();
			chessboardMatches[i].y() = tempInt;
		}
		std::swap(xDir, yDir);
		std::swap(detectedCornersPerRow, detectedCornersPerCol);
	}

	// Check the intensity of the upper left
	// chessboard field of the initial (0,0) point
	Vector2d diagDir = (xDir + yDir);
	diagDir.normalize();
	Vector2d samplePosition = imgPoint00.matrix() - (diagDir*5.0);
	int samplePosX = (int)std::round(samplePosition.x());
	int samplePosY = (int)std::round(samplePosition.y());
	int sampleValue = (int)image.at<unsigned char>(samplePosY, samplePosX);
	bool black = sampleValue == 0;
	bool even = (chessboardMatches[0].x() + chessboardMatches[0].y()) % 2 == 0;

	if ((!black && even) || (black && !even))
	{
		// Swap x-direction and swap y-direction
		int maxXIndex = (int)detectedCornersPerRow - 1;
		int maxYIndex = (int)detectedCornersPerCol - 1;
		for (size_t i = 0; i<chessboardMatches.size(); ++i)
		{
			chessboardMatches[i].x() = maxXIndex - chessboardMatches[i].x();
			chessboardMatches[i].y() = maxYIndex - chessboardMatches[i].y();
		}
	}
}

void Cvl::ChessboardSegmentation::checkRowsAndCols(
	size_t const cornersPerRow, 
	size_t const cornersPerCol, 
	std::vector<Eigen::Array3i>& chessboardMatches, 
	size_t & detectedCornersPerRow, 
	size_t & detectedCornersPerCol)
{
	// if rows and cols are switched
	if (
		(cornersPerRow > cornersPerCol && detectedCornersPerRow < detectedCornersPerCol)
		||
		(cornersPerRow < cornersPerCol && detectedCornersPerRow > detectedCornersPerCol)
		)
	{
		int tempInt;
		// if width is even
		if (cornersPerRow % 2 == 0)
		{
			for (size_t i = 0; i<chessboardMatches.size(); ++i)
			{
				tempInt = chessboardMatches[i].y();
				chessboardMatches[i].y() = chessboardMatches[i].x();
				chessboardMatches[i].x() = (int)detectedCornersPerCol - 1 - tempInt;
			}
		}
		// if width is uneven
		else
		{
			for (size_t i = 0; i<chessboardMatches.size(); ++i)
			{
				tempInt = chessboardMatches[i].x();
				chessboardMatches[i].x() = chessboardMatches[i].y();
				chessboardMatches[i].y() = (int)detectedCornersPerRow - 1 - tempInt;
			}
		}
		std::swap(detectedCornersPerRow, detectedCornersPerCol);
	}
}

std::vector<Cvl::Match> Cvl::ChessboardSegmentation::assignTemplateIndices(
	std::vector<Eigen::Array3i> const & chessboardMatches, 
	size_t const cornersPerRow, 
	size_t const cornersPerCol)
{
	std::vector<Match> matches;
	matches.reserve(chessboardMatches.size());
	size_t templatePosition;
	for (auto const& match : chessboardMatches)
	{
		if (match.x() < (int)cornersPerRow &&
			match.y() < (int)cornersPerCol)
		{
			templatePosition = match.y()*cornersPerRow + match.x();
			matches.push_back(Match(templatePosition, (size_t)match.z()));
		}
	}

	std::sort(matches.begin(), matches.end(),
		[](auto const& a, auto const& b) -> bool
	{
		return a.mTemplateId < b.mTemplateId;
	});

	return matches;
}
