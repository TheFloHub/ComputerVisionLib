#include "ChessboardCornerDetector.h"
#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/highgui.hpp>
#include <cmath>

Eigen::Array2Xd Cvl::ChessboardCornerDetector::findCorners(cv::Mat const & image, size_t const cornersPerRow, size_t const cornersPerCol, bool const highFidelity, size_t blockSize)
{
	std::vector< cv::Point2f > cvCorners;
	cv::goodFeaturesToTrack(image, cvCorners, 0, 0.05, 10);
	if (cvCorners.empty())
		return Eigen::Array2Xd();

	int maxIterations = 8;
	double epsilon = 0.75;
	if (highFidelity)
	{
		maxIterations = 40;
		epsilon = 0.001;
	}
	cv::cornerSubPix(image, cvCorners, cv::Size(8,8), cv::Size(-1, -1), cv::TermCriteria(CV_TERMCRIT_EPS + CV_TERMCRIT_ITER, maxIterations, epsilon));
	
	// reject duplicated corners
	Eigen::Array2Xd corners(2, cvCorners.size());
	int numberOfValidCorners = 0;
	cv::Point2f delta;
	for (int i = 0; i < cvCorners.size(); i++)
	{
		bool hasDuplicate = false;
		for (size_t j = i + 1; j < cvCorners.size(); j++)
		{
			delta = cvCorners[i] - cvCorners[j];
			if (delta.x * delta.x + delta.y * delta.y < 25.0)
			{
				hasDuplicate = true;
				break;
			}
		}
		if (!hasDuplicate)
		{
			corners.col(numberOfValidCorners) = Eigen::Array2d(cvCorners[i].x, cvCorners[i].y);
			++numberOfValidCorners;
		}
	}
	corners.conservativeResize(2, numberOfValidCorners);
	return validateCorners(corners, image, cornersPerRow, cornersPerCol, blockSize);
}

Eigen::Array2Xd Cvl::ChessboardCornerDetector::validateCorners(Eigen::Array2Xd const & corners, cv::Mat const & image, size_t const cornersPerRow, size_t const cornersPerCol, size_t blockSize)
{
	// binarize image
	if (blockSize < 5)
	{
		blockSize = (3 * image.cols) / cornersPerRow;
	}
	if (blockSize % 2 == 0)
		++blockSize;

	cv::Mat binarizedImage;
	cv::adaptiveThreshold(image, binarizedImage, 255, cv::ADAPTIVE_THRESH_MEAN_C, cv::THRESH_BINARY, (int)blockSize, 0);
	//cv::imshow("binarizedImage", binarizedImage);
	//cv::waitKey(0);

	int numberOfOutputCorners = 0;
	Eigen::Array2Xd outputCorners(2, corners.cols());
	Eigen::Array2d currentPoint;
	int x, y, startX, startY;
	const double radius = 8;
	const double stepSize = 0.02;
	const double maxRadians = 6.29 + stepSize;
	int crossings;
	bool white;
	unsigned char intensity;
	int imageWidth = binarizedImage.cols;
	int imageHeight = binarizedImage.rows;

	for (int c = 0; c < corners.cols(); ++c) {

		// Init loop
		currentPoint = corners.col(c);
		crossings = 0;
		int segmentLength[4] = { 0,0,0,0 };
		bool crossingOccured = false;
		startX = x = (int)std::round(currentPoint(0) + radius);
		startY = y = (int)std::round(currentPoint(1));
		double radians = 0.0;
		if (startX<0 || startX >= imageWidth || startY<0 || startY >= imageHeight)
			continue;
		if (binarizedImage.at<unsigned char>(startY, startX) == 0) {
			white = false;
		}
		else {
			white = true;
		}

		// Loop
		do
		{
			nextPixelOnCircle(x, y, radians, currentPoint, radius, stepSize);
			if (x >= 0 && x<imageWidth && y >= 0 && y<imageHeight)
			{
				intensity = binarizedImage.at<unsigned char>(y, x);
				if (intensity == 0 && white) {
					if (!crossingOccured)
					{
						white = false;
						++crossings;
						crossingOccured = true;
					}
					else {
						white = false;
						--segmentLength[crossings % 4];
						--crossings;
						crossingOccured = false;
					}
				}
				else if (intensity == 255 && !white) {
					if (!crossingOccured)
					{
						white = true;
						++crossings;
						crossingOccured = true;
					}
					else {
						white = true;
						--segmentLength[crossings % 4];
						--crossings;
						crossingOccured = false;
					}
				}
				else {
					crossingOccured = false;
				}
				++segmentLength[crossings % 4];
			}
		} while ((x != startX || y != startY) && radians<maxRadians);

		// Evaluate results
		if (crossings == 4)
		{
			if (((double)segmentLength[0] / (double)segmentLength[2]) >= 0.5 &&
				((double)segmentLength[0] / (double)segmentLength[2]) <= 2.0 &&
				((double)segmentLength[1] / (double)segmentLength[3]) >= 0.5 &&
				((double)segmentLength[1] / (double)segmentLength[3]) <= 2.0)
			{
				outputCorners.col(numberOfOutputCorners) = currentPoint;
				++numberOfOutputCorners;
			}
		}
	}
	outputCorners.conservativeResize(2, numberOfOutputCorners);
	return outputCorners;
}

void Cvl::ChessboardCornerDetector::nextPixelOnCircle(int & x, int & y, double & radians, Eigen::Vector2d const & center, double const radius, double const stepSize)
{
	const int startX = x;
	const int startY = y;
	while (x == startX && y == startY)
	{
		radians += stepSize;
		x = (int)std::round(center(0) + radius*cos(radians));
		y = (int)std::round(center(1) + radius*sin(radians));
	}
}
