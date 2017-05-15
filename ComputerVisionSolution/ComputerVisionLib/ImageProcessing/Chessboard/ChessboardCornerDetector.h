/// 
/// @author	Florian Feuerstein
/// 
/// @date   02/2017
/// 
/// @class  ChessboardCornerDetector
/// 
///	@brief  
///

#pragma once

#include <Eigen/Core>
#include <opencv2/core.hpp>

namespace Cvl
{

	class ChessboardCornerDetector
	{

	public:

		ChessboardCornerDetector();

		~ChessboardCornerDetector();

		static Eigen::Array2Xd findCorners(cv::Mat const& image, size_t const cornersPerRow, size_t const cornersPerCol, bool const highFidelity, size_t blockSize = 0);

	private:

		static Eigen::Array2Xd validateCorners(Eigen::Array2Xd const& corners, cv::Mat const& image, size_t const cornersPerRow, size_t const cornersPerCol, size_t blockSize);

		static void nextPixelOnCircle(int& x, int& y, double& radians, Eigen::Vector2d const& center, double const radius, double const stepSize);

	};
}





