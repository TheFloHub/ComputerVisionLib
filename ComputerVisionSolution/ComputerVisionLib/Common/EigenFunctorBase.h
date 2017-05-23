#pragma once

#include <Eigen/Core>

namespace Cvl
{
	template <typename _Scalar, int NX = Eigen::Dynamic, int NY = Eigen::Dynamic>
	struct FunctorBase
	{
		typedef _Scalar Scalar;
		enum {
			InputsAtCompileTime = NX,
			ValuesAtCompileTime = NY
		};
		typedef Eigen::Matrix<Scalar, InputsAtCompileTime, 1> InputType;
		typedef Eigen::Matrix<Scalar, ValuesAtCompileTime, 1> ValueType;
		typedef Eigen::Matrix<Scalar, ValuesAtCompileTime, InputsAtCompileTime> JacobianType;
		typedef Eigen::ColPivHouseholderQR<JacobianType> QRSolver;

		FunctorBase(int numberOfInputs, int numberOfValues) : mNumberOfInputs(numberOfInputs), mNumberOfValues(numberOfValues) {}

		virtual ~FunctorBase() {}

		int inputs() const { return mNumberOfInputs; }

		int values() const { return mNumberOfValues; }

	protected:

		int mNumberOfInputs;

		int mNumberOfValues;

		//int operator()(const InputType &x, ValueType& fvec) { }
		// should be defined in derived classes

		//int df(const InputType &x, JacobianType& fjac) { }
		// should be defined in derived classes
	};
}





