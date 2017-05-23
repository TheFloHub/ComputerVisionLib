#include "ReconstructionError.h"
#include <ComputerVisionLib/Common/EigenHelpers.h>

Eigen::VectorXd Cvl::ReconstructionError::calculateDiff(
	Eigen::Affine3d const & modelView, 
	CameraModel const & cameraModel, 
	Eigen::Array2Xd const & srcPoints, 
	Eigen::Array2Xd const & dstPoints)
{
	// Since we have only 2d template points points with with z=0,
	// we can skip the z rotation axis here.
	Eigen::Matrix3d planarModelView;
	planarModelView.col(0) = modelView.linear().col(0);
	planarModelView.col(1) = modelView.linear().col(1);
	planarModelView.col(2) = modelView.translation();

	// transformation to normalized camera coordinates
	Eigen::Array2Xd normalizedCameraPoints = (planarModelView * srcPoints.matrix().colwise().homogeneous()).colwise().hnormalized();
	// camera projection to image coordinates
	Eigen::Array2Xd estimatedPoints = cameraModel.distortAndProject(normalizedCameraPoints);
	// difference of estimated and measured points
	Eigen::Array2Xd diff = estimatedPoints - dstPoints;
	Eigen::Map<Eigen::VectorXd> diffMap(diff.data(), diff.size());
	return diffMap;
}

Eigen::VectorXd Cvl::ReconstructionError::calculateDiff(
	Eigen::Matrix<double, 6, 1> const & modelViewParameter, 
	CameraModel const & cameraModel, 
	Eigen::Array2Xd const & srcPoints, 
	Eigen::Array2Xd const & dstPoints)
{	
	return calculateDiff(createModelView(modelViewParameter), cameraModel, srcPoints, dstPoints);
}

double Cvl::ReconstructionError::calculateRMSE(
	Eigen::Affine3d const & modelView, 
	CameraModel const & cameraModel, 
	Eigen::Array2Xd const & srcPoints, 
	Eigen::Array2Xd const & dstPoints)
{
	return std::sqrt(calculateDiff(modelView, cameraModel, srcPoints, dstPoints).squaredNorm() / srcPoints.size());
}
