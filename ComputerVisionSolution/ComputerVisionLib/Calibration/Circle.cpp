#include "Circle.h"
#include <unsupported/Eigen/LevenbergMarquardt>
#include <limits>
#include <iostream>

Cvl::Circle::Circle(Eigen::Array2Xd const & points, size_t index1, size_t index2, size_t index3) :
mCenterX(0.0),
mCenterY(0.0),
mRadius(0.0),
mError(std::numeric_limits<double>::max())
{
	if (points.cols() < 3)
		return;

	Eigen::Vector2d p1 = points.col(index1);
	Eigen::Vector2d p2 = points.col(index2);
	Eigen::Vector2d p3 = points.col(index3);
	Eigen::Vector2d deltaA = p2 - p1;
	Eigen::Vector2d deltaB = p3 - p2;
	bool initializedCircle = false;

	// initialize circle parameters
	if (std::abs(deltaA.x())<0.0000001 && std::abs(deltaB.y())<0.0000001)
	{
		mCenterX = 0.5*(p2.x() + p3.x());
		mCenterY = 0.5*(p1.y() + p2.y());
		mRadius = (Eigen::Vector2d(mCenterX, mCenterY) - p1).norm();
		initializedCircle = true;
	}
	else if (deltaA.x() != 0.0 && deltaB.x() != 0.0)
	{
		double aSlope = deltaA.y() / deltaA.x();
		double bSlope = deltaB.y() / deltaB.x();
		if (std::abs(aSlope - bSlope) > 0.000001)
		{
			mCenterX = (aSlope*bSlope*(p1.y() - p3.y()) + bSlope*(p1.x() + p2.x()) - aSlope*(p2.x() + p3.x())) / (2.0*(bSlope - aSlope));
			mCenterY = -1.0*(mCenterX - (p1.x() + p2.x()) / 2.0) / aSlope + (p1.y() + p2.y()) / 2.0;
			mRadius = (Eigen::Vector2d(mCenterX, mCenterY) - p1).norm();
			initializedCircle = true;
		}
	}

	// optimize parameters
	if (initializedCircle)
	{
		Eigen::VectorXd parameters = Eigen::Vector3d(mCenterX, mCenterY, mRadius);

#ifdef _DEBUG
		Eigen::VectorXd errorVec = ((points.matrix().colwise() - parameters.head<2>()).colwise().norm().array() - parameters(2)).matrix().transpose();
		mError = std::sqrt(errorVec.squaredNorm() / errorVec.size());
		std::cout << "Initial circle parameters: " << std::endl;
		std::cout << parameters << std::endl;
		std::cout << "Initial RMSE of circle: " << mError << std::endl;
#endif

		Functor functor(points);
		Eigen::LevenbergMarquardt<Functor> lm(functor);
		Eigen::LevenbergMarquardtSpace::Status status = lm.minimize(parameters);

		bool success = lm.info() == Eigen::Success || lm.info() == Eigen::NoConvergence;
		if (success)
		{
			mCenterX = parameters(0);
			mCenterY = parameters(1);
			mRadius = parameters(2);
			mError = std::sqrt(lm.fvec().squaredNorm() / lm.fvec().size());
		}

#ifdef _DEBUG
		std::cout << "Optimized circle parameters: " << std::endl;
		std::cout << parameters << std::endl;
		std::cout << "Optimized RMSE of circle: " << mError << std::endl;
		std::cout << "LM- info: " << lm.info() << " / status: " << status << " / iters: " << lm.iterations() << " / nfev: " << lm.nfev() << " / njev: " << lm.njev() << std::endl << std::endl;
#endif
	}
}

Cvl::Circle::~Circle()
{
}

Eigen::Vector2d Cvl::Circle::getCenter() const
{
	return Eigen::Vector2d(mCenterX, mCenterY);
}

double Cvl::Circle::getRadius() const
{
	return mRadius;
}

double Cvl::Circle::getError() const
{
	return mError;
}


bool Cvl::Circle::intersect(Circle const & other, Eigen::Vector2d & intersection1, Eigen::Vector2d & intersection2) const
{
	Eigen::Vector2d center1 = this->getCenter();
	Eigen::Vector2d center2 = other.getCenter();
	double radius1 = this->getRadius();
	double radius2 = other.getRadius();

	// determine the straight-line distance between the centers. 
	Eigen::Vector2d delta = center2 - center1;
	double distance = delta.norm();

	// check if solvable
	if (distance > (radius1 + radius2) || distance < std::abs(radius1 - radius2) || distance == 0.0)
		return false;

	// point 2 is the point where the line through the circle
	// intersection points crosses the line between the circle centers.  
	// determine the distance from point 0 to point 2.
	double a = ((radius1*radius1) - (radius2*radius2) + (distance*distance)) / (2.0 * distance);

	// determine the coordinates of point 2.
	double x2 = center1.x() + (delta.x() * a / distance);
	double y2 = center1.y() + (delta.y() * a / distance);

	// determine the distance from point 2 to either of the
	// intersection points.
	double h = std::sqrt((radius1*radius1) - (a*a));

	// now determine the offsets of the intersection points from point 2.
	double rx = -delta.y() * (h / distance);
	double ry = delta.x() * (h / distance);

	// determine the absolute intersection points.
	intersection1.x() = x2 + rx;
	intersection1.y() = y2 + ry;
	intersection2.x() = x2 - rx;
	intersection2.y() = y2 - ry;
	return true;
}


Cvl::Circle::Functor::Functor(Eigen::Array2Xd const & points) :
FunctorBase<double>(3, (int)points.cols()),
mPoints(points)
{
}

int Cvl::Circle::Functor::operator() (Eigen::VectorXd const & x, Eigen::VectorXd & fvec) const
{
	fvec = ((mPoints.matrix().colwise() - x.head<2>()).colwise().norm().array() - x(2)).matrix().transpose();
	return 0;
}

int Cvl::Circle::Functor::df(Eigen::VectorXd const & x, Eigen::MatrixXd & fjac) const
{
	Eigen::ArrayX2d diff = (mPoints.matrix().colwise() - x.head<2>()).transpose();
	fjac.leftCols<2>() = ((-diff).colwise() / (diff.matrix().rowwise().norm()).array()).matrix();
	fjac.col(2).setConstant(-1.0);

	// just a test
	//Eigen::MatrixXd fjac2 = fjac;
	//fjac2.setZero();
	//Eigen::Vector2d center = x.head<2>();
	//double radius = x(2);
	//for (int i = 0; i < values(); ++i)
	//{
	//	double d=(mPoints.matrix().col(i) - center).norm();
	//	fjac2(i, 0) = (center.x() - mPoints(0, i))/d;
	//	fjac2(i, 1) = (center.y() - mPoints(1, i)) / d;
	//	fjac2(i, 2) = -1;
	//}
	//std::cout << fjac2 - fjac << std::endl;
	//std::cout <<std::endl;

	return 0;
}
