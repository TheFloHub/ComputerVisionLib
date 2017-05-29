#include "DistortionModel.h"
#include <Eigen/Dense>
#include <iostream>

Cvl::DistortionModel::DistortionModel() : DistortionModel(0.0, 0.0)
{
}

Cvl::DistortionModel::DistortionModel(double tangentialParameter1, double tangentialParameter2) :
mTangentialParameter1(tangentialParameter1),
mTangentialParameter2(tangentialParameter2)
{
}

Cvl::DistortionModel::~DistortionModel()
{
}

Eigen::Array2Xd Cvl::DistortionModel::distort(Eigen::Array2Xd const & normalizedCameraPoints) const
{
	Eigen::Array<double, 1, Eigen::Dynamic> squaredNorms = normalizedCameraPoints.matrix().colwise().squaredNorm();
	// (xd, yd)^T = (xu, yu)^T * radialFactor + tangentialOffsets
	return (normalizedCameraPoints.rowwise() * radialDistortionFactors(squaredNorms)) + tangentialDistortionOffsets(normalizedCameraPoints, squaredNorms);
}

Eigen::Array2Xd Cvl::DistortionModel::undistort(Eigen::Array2Xd const & distortedPoints) const
{
	// The first order Taylor approximation of the distortion function f(u)=d is
	// 
	// f(u) ~ f(u_0) + f'(u_0) * (u - u_0)
	// 
	// So we can undistort a point by iteratively distorting and updating a point u
	// so that the difference d-f(u) between the distorted point d and f(u) is small enough.
	//
	// u_i+1 = f'^-1(u_i) * (d - f(u_i)) + u_i

	static double const maxError = 0.0000001;
	static size_t const maxNumberOfIterations = 100;

	Eigen::Array2Xd undistortedPoints = Eigen::Array2Xd::Zero(2, distortedPoints.cols());
	Eigen::Vector2d undistortedPoint(0.0, 0.0);
	Eigen::Vector2d diff(0.0, 0.0);
	Eigen::Matrix2d jacobian;
	Eigen::Matrix2d inverseJacobian;
	double error = 0.0;
	size_t numberOfIterations = 0;

	for (Eigen::Index pointIndex = 0; pointIndex < distortedPoints.cols(); ++pointIndex)
	{
		Eigen::Vector2d const & distortedPoint = distortedPoints.col(pointIndex).matrix();
		undistortedPoint = distortedPoint;
		diff = distortedPoint - distort(undistortedPoint.array()).col(0).matrix();
		error = diff.norm();
		numberOfIterations = 0;

		while (numberOfIterations < maxNumberOfIterations && error > maxError)
		{
			// calculate the next improved undistorted point
			jacobian = derivative(undistortedPoint);
			inverseJacobian = jacobian.inverse(); // TODO: computeInverseAndDetWithCheck?
			undistortedPoint = inverseJacobian * diff + undistortedPoint;

			// calculate the error
			diff = distortedPoint - distort(undistortedPoint.array()).col(0).matrix();
			error = diff.norm();
			++numberOfIterations;

			//std::cout << "Iter: " << numberOfIterations << " Error: " << error << std::endl;
		}
		undistortedPoints.col(pointIndex) = undistortedPoint.array();

		//std::cout << std::endl << std::endl << std::endl;

#ifdef _DEBUG
		if (numberOfIterations >= maxNumberOfIterations)
		{
			std::cout << "Iterative undistortion reached maximum number of iterations with an error of " << error << "." << std::endl;
		}
#endif
	}
	return undistortedPoints;
}

Eigen::VectorXd Cvl::DistortionModel::getParameters() const
{
	Eigen::VectorXd radialParameters = getRadialDistortionParameters();
	return (Eigen::VectorXd(2+radialParameters.rows()) << mTangentialParameter1, mTangentialParameter2, radialParameters).finished();
}

void Cvl::DistortionModel::setParameters(Eigen::VectorXd const & parameters)
{
	mTangentialParameter1 = parameters(0);
	mTangentialParameter2 = parameters(1);
	setRadialDistortionParameters(parameters.tail(parameters.rows()-2));
}

void Cvl::DistortionModel::resetParameters()
{
	mTangentialParameter1 = 0.0;
	mTangentialParameter2 = 0.0;
	resetRadialDistortionParameters();
}

Eigen::Matrix2d Cvl::DistortionModel::derivative(Eigen::Array2d const & normalizedCameraPoint) const
{
	// the jacobian of the tangential distortion is
	// p1 * 2 * y  + p2 * 6 * x		p1 * 2 * x + p2 * 2 * y
	// p2 * 2 * y  + p1 * 2 * x		p2 * 2 * x + p1 * 6 * y

	Eigen::Matrix2d jacobi;
	jacobi(0, 0) = 2.0 * mTangentialParameter1 * normalizedCameraPoint.y() + 6.0 * mTangentialParameter2 * normalizedCameraPoint.x();
	jacobi(1, 0) = 2.0 * mTangentialParameter2 * normalizedCameraPoint.y() + 2.0 * mTangentialParameter1 * normalizedCameraPoint.x();
	jacobi(0, 1) = 2.0 * mTangentialParameter1 * normalizedCameraPoint.x() + 2.0 * mTangentialParameter2 * normalizedCameraPoint.y();
	jacobi(1, 1) = 2.0 * mTangentialParameter2 * normalizedCameraPoint.x() + 6.0 * mTangentialParameter1 * normalizedCameraPoint.y();

	// the jacobian of the radial distortion is
	// ( (d/dx) x*f(r)  (d/dy) x*f(r) )  =
	// ( (d/dx) y*f(r)  (d/dy) y*f(r) )
	//
	// f(r) * ( 1 0 )  +  ((d/dr) f(r)) * ( x*(dr/dx)  y*(dr/dy) )  =
	//        ( 0 1 )                     ( y*(dr/dx)  x*(dr/dy) )
	//
	// f(r) * ( 1 0 )  +  1/r * ((d/dr) f(r)) * ( x^2  x*y )    
	//        ( 0 1 )                           ( x*y  y^2 )

	double rSquared = normalizedCameraPoint.matrix().squaredNorm();
	double r = std::sqrt(rSquared);
	double dr = radialDistortionDerivative(r);
	double fr = radialDistortionFactors((Eigen::Array<double, 1, Eigen::Dynamic>(1, 1) << rSquared).finished())(0);

	jacobi(0, 0) += fr + dr * (normalizedCameraPoint.x() * normalizedCameraPoint.x()) / r;
	jacobi(1, 0) +=				   dr * (normalizedCameraPoint.x() * normalizedCameraPoint.y()) / r;
	jacobi(0, 1) +=				   dr * (normalizedCameraPoint.x() * normalizedCameraPoint.y()) / r;
	jacobi(1, 1) += fr + dr * (normalizedCameraPoint.y() * normalizedCameraPoint.y()) / r;

	return jacobi;
}

Eigen::Array2Xd Cvl::DistortionModel::tangentialDistortionOffsets(
	Eigen::Array2Xd const & undistortedPoints,
	Eigen::Array<double, 1, Eigen::Dynamic> const & squaredNorms) const
{
	// x' = p1 * 2 * x * y  + p2 * (r^2 + 2 * x^2)
	// y' = p2 * 2 * x * y  + p1 * (r^2 + 2 * y^2)
	return ((2.0 * Eigen::Vector2d(mTangentialParameter1, mTangentialParameter2)) * (undistortedPoints.row(0) * undistortedPoints.row(1)).matrix()).array()
		+ ((2.0 * undistortedPoints.square()).rowwise() + squaredNorms).colwise() * Eigen::Array2d(mTangentialParameter2, mTangentialParameter1);
}

