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

#include <ComputerVisionLib/Common/EigenFunctorBase.h>
#include <Eigen/Core>

namespace Cvl
{
	class Circle
	{

	public:

		Circle(Eigen::Array2Xd const & points, size_t index1, size_t index2, size_t index3);

		~Circle();

		Eigen::Vector2d getCenter() const;

		double getRadius() const;

		double getError() const;

		bool intersect(Circle const & other, Eigen::Vector2d & intersection1, Eigen::Vector2d & intersection2) const;

	protected:

		double mCenterX;
		
		double mCenterY;

		double mRadius;

		double mError;

		class Functor : public FunctorBase<double>
		{
		public:
			Functor(Eigen::Array2Xd const & points);
			int operator() (Eigen::VectorXd const & x, Eigen::VectorXd & fvec) const;
			int df(Eigen::VectorXd const & x, Eigen::MatrixXd & fjac) const;
		private:
			Eigen::Array2Xd const & mPoints;
		};

	};
}




