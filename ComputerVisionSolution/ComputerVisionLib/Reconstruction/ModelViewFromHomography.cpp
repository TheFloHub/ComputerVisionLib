#include "ModelViewFromHomography.h"
#include <ComputerVisionLib/Common/EigenHelpers.h>
#include <ComputerVisionLib/Reconstruction/ReconstructionError.h>
#include <unsupported/Eigen/LevenbergMarquardt>
#include <iostream>

Cvl::ModelViewFromHomography::ModelViewFromHomography()
{
}

Cvl::ModelViewFromHomography::~ModelViewFromHomography()
{
}

Eigen::Affine3d Cvl::ModelViewFromHomography::algebraic(
	CameraModel const& cameraModel,
	Eigen::Matrix3d const & homography)
{
	/** 
	* Calculate modelview (rotation and translation) from homography matrix H.
	* (see G. Bradski and A. Kaehler, Learning OpenCV, Chapter 11)
	* The modelview M is given by :  r1_x  r2_x  r3_x  t_x
	*                                r1_y  r2_y  r3_y  t_y
	*                                r1_z  r2_z  r3_z  t_z
	*                                0     0     0     1
	* where the 3x3 matrix [r1 r2 r3] is the rotation and t the translation.
	* H can be written as H = s*K.[r1 r2 t] where K is the 3x3 matrix of the
	* camera intrinsics and s is a scaling factor. It is assumed that the
	* coordinate system of the object plane is chosen such that z=0. Then any
	* rotation [r1 r2 r3] (where r1, r2 and r3 are unit column vectors) applied
	* to that plane does not make use of r3 and it can be written as
	* [r1 r2].[x y]^T. Using homogeneous coordinates for the object plane this
	* becomes [r1 r2 t].[x y 1]^T including the translation t. [r1 r2 t] can be
	* calculated from H via [r1 r2 t] = 1/s*K^(-1).H. The normalization factor
	* is lambda = 1/s = 1/ |K^(-1).h1|.
	*/

	Eigen::Matrix3d r1r2t = cameraModel.getInversePinholeCameraMatrix()*homography;
	double lambda = r1r2t.col(0).norm();
	r1r2t /= lambda;

	Eigen::Matrix3d rotation;
	rotation.col(0) = r1r2t.col(0);
	rotation.col(1) = r1r2t.col(1);
	rotation.col(2) = r1r2t.col(0).cross(r1r2t.col(1));
	fixRotationMatrix(rotation);

	// The resulting transformation is the rotation and translation from the
	// coordinate system of the first image plane (i.e. the object plane with
	// z=0) into the coordinate system of the camera. For any two corresponding
	// points, p_obj in image 1, and p_camera in image 2, we get:
	// p_camera = R * p_obj + t
	// Consequently, t is the position of the object origin in the camera system,
	// and the columns of R describe the object system axes in the camera system.

	// Typically, the camera coordinate system (not the sensor system!) is chosen
	// such that the camera "looks" at the positive z-direction. Using a right
	// handed coordinate system x then faces left if y faces upwards.

	// Since the homography H is only defined up to a scaling factor s, the
	// transformation we get is not unique, but there's always two solutions.
	// The reason is that the norm of s is factored out above, but the sign of s
	// remains. The second solution has the same components as the first one, but
	// with inverted sign: the components of the x-column and the y-column of the
	// rotation change sign, while the z-column, calculated as (x cross y),
	// remains the same. The components of the translation change sign as well.

	// Graphically, the two solutions are related as follows: the position of
	// the second camera is obtained by a reflection about the object plane. The
	// orientation of the second camera is obtained by rotation of the first cam
	// about the object systems z-axis. Algebraically this can be seen as well:
	// Assuming the first orientation and translation are:
	//        x1  y1 (x2*y3-x3*y2)           t1
	// R_1 =  x2  y2 (x3*y1-x1*y3),   t_1 =  t2
	//        x3  y3 (x1*y2-x2*y1)           t3
	// then the second solution is by construction:
	//             -1  0  0
	// R_2 = R_1 *  0 -1  0,          t_2 = -t_1
	//              0  0  1
	//
	//       -x1 -y1 (x2*y3-x3*y2)          -t1
	// R_2 = -x2 -y2 (x3*y1-x1*y3),   t_1 = -t2
	//       -x3 -y3 (x1*y2-x2*y1)          -t3
	// Remember, R_1, t_1 and R_2, t_2 describe the position of the object plane
	// (root) and the orientation of the object system in the camera system.
	// Now, let's look at the position of the cameras and their orientation in
	// the object system. For this we have to invert the transformations using:
	// If M = ( R_i   t_i ),    M^(-1) = ( R_i^T  -R_i^T.t_i )
	//	      ( 0 0 0   1 )              ( 0 0 0           1 )
	// For camera 1 we get:
	//             x1            x2            x3
	// R_1^T =     y1            y2            y3
	//        (x2*y3-x3*y2) (x3*y1-x1*y3) (x1*y2-x2*y1)
	//
	//              -(x1*t1 + x2*t2 + x3*t3)
	// -R_1^T.t_1 = -(y1*t1 + y2*t2 + y3*t3)
	//              -((x2*y3-x3*y2)*t1+(x3*y1-x1*y3)*t2+(x1*y2-x2*y1)*t3)
	// For camera 2 we get:
	//            -x1           -x2           -x3
	// R_2^T =    -y1           -y2           -y3
	//        (x2*y3-x3*y2) (x3*y1-x1*y3) (x1*y2-x2*y1)
	//
	//              -(x1*t1 + x2*t2 + x3*t3)
	// -R_2^T.t_2 = -(y1*t1 + y2*t2 + y3*t3)
	//               ((x2*y3-x3*y2)*t1+(x3*y1-x1*y3)*t2+(x1*y2-x2*y1)*t3)
	// 
	//         -1  0  0                 t_21 = t_11
	// R_2^T =  0 -1  0 * R_1,          t_22 = t_12
	//          0  0  1                 t_23 = -t_13

	// Looking at the translation vectors -R_1^T.t_1 and -R_2^T.t_2 (that
	// describe the position of the camera in the object system) we see that the
	// components are identical. Only the z component changes sign, i.e. we
	// have one solution in front of the object plane and one behind. We always
	// choose the solution with z>0.

	// However, by normalizing H such that h_33>0 we made sure that t3>0. This
	// means that the object root is always at z>0 in the first camera system.
	// In most cases this solution is also the one with (-R_1^T.t_1)>0 but not
	// always.

	double cameraZPosInWorldSys = -(rotation.col(2).dot(r1r2t.col(2)));

	// If the camera center has a negative z-component (lies behind any pattern),
	// invert x, y and the translation to get the positive solution.
	if (cameraZPosInWorldSys < 0)
	{
		rotation.col(0) = -rotation.col(0);
		rotation.col(1) = -rotation.col(1);
		r1r2t.col(2) = -r1r2t.col(2);
	}

	Eigen::Affine3d modelView;
	modelView.linear() = rotation;
	modelView.translation() = r1r2t.col(2);

	return modelView;
}

std::tuple<bool, double, Eigen::Affine3d> Cvl::ModelViewFromHomography::calculate(
	CameraModel const & cameraModel, 
	Eigen::Matrix3d const & homography, 
	Eigen::Array2Xd const & srcPoints, 
	Eigen::Array2Xd const & dstPoints)
{
	Eigen::Affine3d modelView = algebraic(cameraModel, homography);
	Eigen::VectorXd parameters = getModelViewParameters(modelView);

#ifdef _DEBUG
	std::cout << "Initial parameters of modelview from homography: " << parameters.transpose() << std::endl;
	std::cout << "Initial RMSE of of modelview from homography: " << ReconstructionError::calculateRMSE(modelView, cameraModel, srcPoints, dstPoints) << std::endl;
#endif

	Functor functor(srcPoints, dstPoints, cameraModel);
	Eigen::NumericalDiff<Functor> numDiff(functor);
	Eigen::LevenbergMarquardt<Eigen::NumericalDiff<Functor>> lm(numDiff);
	
	Eigen::LevenbergMarquardtSpace::Status status = lm.minimize(parameters);

	modelView = createModelView(parameters);
	double error = std::sqrt(lm.fvec().squaredNorm() / lm.fvec().size());
	bool success = lm.info() == Eigen::Success;

#ifdef _DEBUG
	std::cout << "Optimized parameters of modelview from homography: " << parameters.transpose() << std::endl;
	std::cout << "Optimized RMSE of of modelview from homography: " << error << std::endl;
	std::cout << "LM- info: " << lm.info() << " / status: " << status <<  " / iters: " << lm.iterations() << " / nfev: " << lm.nfev() << " / njev: " << lm.njev() << std::endl << std::endl;
#endif

	return std::make_tuple(success, error, modelView);
}

Cvl::ModelViewFromHomography::Functor::Functor(
	Eigen::Array2Xd const & srcPoints,
	Eigen::Array2Xd const & dstPoints,
	CameraModel const & cameraModel) :
mNumberOfInputs(6),
mNumberOfValues((int)srcPoints.size()),
mSrcPoints(srcPoints),
mDstPoints(dstPoints),
mCameraModel(cameraModel)
{
}

int Cvl::ModelViewFromHomography::Functor::operator()(
	Eigen::VectorXd const & x, 
	Eigen::VectorXd & fvec) const
{
	fvec = ReconstructionError::calculateDiff(x, mCameraModel, mSrcPoints, mDstPoints);
	return 0;
}

