/// 
/// @author	Florian Feuerstein
/// 
/// @date   02/2017
/// 
/// @class  ChessboardSegmentation
/// 
///	@brief  
///

#pragma once

#include <ComputerVisionLib/Common/Match.h>
#include <Eigen/Core>
#include <utility>
#include <opencv2/core.hpp>

namespace Cvl
{

	class ChessboardSegmentation
	{

	public:

		struct Result
		{
			Result() :
				mSuccessful(false),
				mUnambiguous(false),
				mMatches()
			{}
			bool mSuccessful;
			bool mUnambiguous;
			std::vector<Match> mMatches;
		};

		static Result match(
			cv::Mat const& image, 
			Eigen::Array2Xd const& corners, 
			size_t const cornersPerRow, 
			size_t const cornersPerCol, 
			size_t blockSize = 0);

	private:

		ChessboardSegmentation() = delete;

		~ChessboardSegmentation() = delete;

		enum Direction { UP = 0, DOWN = 1, LEFT = 2, RIGHT = 3};

		/// Traverses all corners and gathers connected corners
		static std::vector<std::vector<Eigen::Array3i>> traverseAllCorners(
			Eigen::Array2Xd const& corners,
			cv::Mat const& image);

		/// Searches for connected chessboard corners starting from startIndex
		static std::vector<Eigen::Array3i> startTraversing(
			size_t const startIndex,
			Eigen::Array2Xd const& corners,
			cv::Mat const& image);

		/// Recursive function. Assigns a relative position to a corner and searches for principal directions to proceed further. 
		static void processCorner(
			size_t const pointIndex,
			Eigen::Array2i const& relativePosition,
			Eigen::Array<double, 2, 4> const& directions,
			Eigen::Array2Xd const& corners,
			std::vector<bool>& traversedCorners,
			std::vector<Eigen::Array3i>& chessboardMatches,
			cv::Mat const& image);

		/// Returns the n nearest neighbors of a point
		static std::vector<int> getNeighborship(
			Eigen::Array2d const& point, 
			Eigen::Array2Xd const& points, 
			size_t numberOfNextNeighbors = 12);

		/// Checks if the line between point1 and point2 is a principal direction of the chessboard
		static bool isPrincipalDirection(
			Eigen::Array2d const& point1, 
			Eigen::Array2d const& point2, 
			cv::Mat const& image);

		/// Sets all relative coordinates to a positive value and returns
		/// the maximum number of detected rows and columns. 
		static void setOriginZero(
			std::vector<Eigen::Array3i>& chessboardMatches,
			size_t& detectedCornersPerRow,
			size_t& detectedCornersPerCol);

		/// Checks the orientation of the pattern and
		/// transforms the coordinates to the correct orientation
		/// After this transformation there are still two possible
		/// orientations.
		static void checkOrientation(
			Eigen::Array2Xd const& corners,
			cv::Mat const& image,
			std::vector<Eigen::Array3i>& chessboardMatches,
			size_t& detectedCornersPerRow,
			size_t& detectedCornersPerCol);

		/// Checks if rows and cols are switched
		/// and corrects the coordinates
		static void checkRowsAndCols(
			size_t const cornersPerRow,
			size_t const cornersPerCol,
			std::vector<Eigen::Array3i>& chessboardMatches,
			size_t& detectedCornersPerRow,
			size_t& detectedCornersPerCol);

		/// Assigns a template ID to each corner of the chessboard pattern.
		static std::vector<Match> assignTemplateIndices(
			std::vector<Eigen::Array3i> const& chessboardMatches,
			size_t const cornersPerRow,
			size_t const cornersPerCol);

	};
}





