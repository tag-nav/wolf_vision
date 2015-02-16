
#ifndef COMPLEX_ANGLE_PARAMETRIZATION_H_
#define COMPLEX_ANGLE_PARAMETRIZATION_H_

//Ceres includes
#include "ceres/ceres.h"

class ComplexAngleParameterization : public ceres::LocalParameterization
{
	public:

		//ComplexAngleParameterization();

		virtual ~ComplexAngleParameterization();

		virtual bool Plus(const double* x_raw, const double* delta_raw, double* x_plus_delta_raw) const;

		virtual bool ComputeJacobian(const double* x, double* jacobian) const;

		virtual int GlobalSize() const;

		virtual int LocalSize() const;
};
#endif
