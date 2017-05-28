#include "DistortionModel.h"
#include <Eigen/Dense>

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
	
	static double const maxError = 0.0000001;
	static size_t const maxNumberOfIterations = 100;

	Eigen::Array2Xd undistortedPoints = Eigen::Array2Xd::Zero(2, distortedPoints.cols());
	Eigen::Array<double, 1, Eigen::Dynamic> squaredNorms1D(1);
	Eigen::Vector2d bi(0.0, 0.0);
	Eigen::Vector2d aiPlus1(0.0, 0.0);
	Eigen::Vector2d di(0.0, 0.0);
	Eigen::Matrix2d jacobian;
	Eigen::Matrix2d inverseJacobian;
	double radialFactor = 0.0;
	double error = 0.0;
	bool stopIteration = false;
	size_t numberOfIterations = 0;

	for (Eigen::Index pointIndex = 0; pointIndex < distortedPoints.cols(); ++pointIndex)
	{

		Eigen::Vector2d const & distortedPoint = distortedPoints.col(pointIndex).matrix();
		bi = distortedPoint;
		squaredNorms1D(0) = bi.squaredNorm();
		radialFactor = radialDistortionFactors(squaredNorms1D)(0);
		aiPlus1 = radialFactor * bi + tangentialDistortionOffsets(bi, squaredNorms1D).matrix();
		di = aiPlus1 - distortedPoint;
		error = di.norm();
		stopIteration = error < maxError;

		numberOfIterations = 0;

		while (!stopIteration)
		{
			++numberOfIterations;

			// step 1:
			jacobian = derivative(bi, radialFactor);
			inverseJacobian = jacobian.inverse(); // computeInverseAndDetWithCheck ???
			bi = inverseJacobian*(distortedPoint - aiPlus1) + bi;

			// step 2:
			// calculate analytic function from b_(i+1)
			squaredNorms1D(0) = bi.squaredNorm();
			radialFactor = radialDistortionFactors(squaredNorms1D)(0);
			aiPlus1 = radialFactor * bi + tangentialDistortionOffsets(bi, squaredNorms1D).matrix();
			di = aiPlus1 - distortedPoint;
			error = di.norm();

			// step 3: check the norm of l_di
			stopIteration = (numberOfIterations >= maxNumberOfIterations) || (error < maxError);
		}

		//if (m_numIterations >= m_maxNumberOfIterations)
		//{
		//	if (m_logIterativeWarnings)
		//	{
		//		std::ostringstream l_warningSstream;
		//		l_warningSstream << "iterative (un-)distortion reached max num iterations "
		//			<< "for input radius r=" << fcr_in.norm() << " ("
		//			<< atan2(fcr_in.norm(), 1.)*RAD2DEG << " degrees)"
		//			<< "; remaining error: " << m_error;
		//		logwarn << __FUNCTION__ << ": " << l_warningSstream.str() << endlog;
		//		m_messageSignal(Vu::MESSAGE_WARN,
		//			std::string("VipDistortionFunctions: ") + l_warningSstream.str());
		//	}

		//	fr_status |= vip::DISTORTION_STATUS_NO_CONVERGENCE;
		//}

		undistortedPoints.col(pointIndex) = bi.array();
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

Eigen::Matrix2d Cvl::DistortionModel::derivative(Eigen::Array2d const & normalizedCameraPoint, double radialFactor) const
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
	// ( (d/dx) x*g(r)  (d/dy) x*g(r) )  =
	// ( (d/dx) y*g(r)  (d/dy) y*g(r) )
	//
	// g(r) * ( 1 0 )  +  ((d/dr) g(r)) * ( x*(dr/dx)  y*(dr/dy) )  =
	//        ( 0 1 )                     ( y*(dr/dx)  x*(dr/dy) )
	//
	// g(r) * ( 1 0 )  +  1/r * ((d/dr) g(r)) * ( x^2  x*y )    
	//        ( 0 1 )                           ( x*y  y^2 )

	double r = normalizedCameraPoint.matrix().norm();
	double dr = radialDistortionDerivative(r);

	jacobi(0, 0) += radialFactor + dr * (normalizedCameraPoint.x() * normalizedCameraPoint.x()) / r;
	jacobi(1, 0) +=				   dr * (normalizedCameraPoint.x() * normalizedCameraPoint.y()) / r;
	jacobi(0, 1) +=				   dr * (normalizedCameraPoint.x() * normalizedCameraPoint.y()) / r;
	jacobi(1, 1) += radialFactor + dr * (normalizedCameraPoint.y() * normalizedCameraPoint.y()) / r;

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

