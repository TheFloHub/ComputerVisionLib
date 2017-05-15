#include "HomographyCalculation.h"
#include <opencv2/core.hpp>
#include <opencv2/calib3d.hpp>

#include <iostream>

Cvl::HomographyCalculation::HomographyCalculation()
{
}

Cvl::HomographyCalculation::~HomographyCalculation()
{
}

std::tuple<bool, Eigen::Matrix3d> Cvl::HomographyCalculation::calculate(
	Eigen::Array2Xd const & srcPoints, 
	Eigen::Array2Xd const & dstPoints)
{
	std::vector<cv::Point2d> srcPointsCv;
	srcPointsCv.reserve(srcPoints.cols());
	std::vector<cv::Point2d> dstPointsCv;
	dstPointsCv.reserve(dstPoints.cols());

	for (int i = 0; i < dstPoints.cols(); ++i)
	{
		srcPointsCv.emplace_back(srcPoints(0, i), srcPoints(1, i));
		dstPointsCv.emplace_back(dstPoints(0, i), dstPoints(1, i));
	}

	cv::Mat cvHomography = cv::findHomography(srcPointsCv, dstPointsCv);
	Eigen::Matrix3d homography = Eigen::Matrix3d::Zero();
	if (cvHomography.empty())
	{
#ifdef _DEBUG
		std::cout << "HomographyCalculation failed." << std::endl << std::endl;
#endif
		return std::make_tuple(false, homography);
	}

	homography(0, 0) = cvHomography.at<double>(0, 0);
	homography(0, 1) = cvHomography.at<double>(0, 1);
	homography(0, 2) = cvHomography.at<double>(0, 2);
	homography(1, 0) = cvHomography.at<double>(1, 0);
	homography(1, 1) = cvHomography.at<double>(1, 1);
	homography(1, 2) = cvHomography.at<double>(1, 2);
	homography(2, 0) = cvHomography.at<double>(2, 0);
	homography(2, 1) = cvHomography.at<double>(2, 1);
	homography(2, 2) = cvHomography.at<double>(2, 2);

#ifdef _DEBUG
	std::cout << "HomographyCalculation: " << std::endl;
	std::cout << homography << std::endl;
	std::cout << "RMSE: " << rmse(homography, srcPoints, dstPoints) << std::endl << std::endl;
#endif

	return std::make_tuple(true, homography);
}

double Cvl::HomographyCalculation::rmse(
	Eigen::Matrix3d const & homography, 
	Eigen::Array2Xd const & srcPoints, 
	Eigen::Array2Xd const & dstPoints)
{
	return std::sqrt(((homography * srcPoints.matrix().colwise().homogeneous()).colwise().hnormalized() - dstPoints.matrix()).squaredNorm() / srcPoints.size());
}
