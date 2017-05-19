/// 
/// @author	Florian Feuerstein
/// 
/// @date   02/2017
/// 
/// @class  Circle
/// 
///	@brief  
///

#pragma once

#include <Eigen/Core>

namespace Cvl
{
	class Circle
	{

	public:

		Circle(Eigen::Array2Xd const & points, size_t index1, size_t index2, size_t index3);

		~Circle();

		double getError() const { return mError; }

		bool intersect(Circle const & other, Eigen::Vector2d & intersection1, Eigen::Vector2d & intersection2) const;

	protected:

		double mCenterX;
		
		double mCenterY;

		double mRadius;

		double mError;

	};
}




