#include "complex_angle_parametrization.h"

//ComplexAngleParameterization::ComplexAngleParameterization()
//{
//    //
//}
ComplexAngleParameterization::~ComplexAngleParameterization()
{
	//
}

bool ComplexAngleParameterization::Plus(const double* x_raw, const double* delta_raw, double* x_plus_delta_raw) const
{
	x_plus_delta_raw[0] = x_raw[0] * cos(delta_raw[0]) - x_raw[1] * sin(delta_raw[0]);
	x_plus_delta_raw[1] = x_raw[1] * cos(delta_raw[0]) + x_raw[0] * sin(delta_raw[0]);

	//normalize
	//double norm = sqrt(x_plus_delta_raw[0] * x_plus_delta_raw[0] + x_plus_delta_raw[1] * x_plus_delta_raw[1]);
	//std::cout << "(before normalization) norm = " << norm << std::endl;
	//x_plus_delta_raw[0] /= norm;
	//x_plus_delta_raw[1] /= norm;

	return true;
}

bool ComplexAngleParameterization::ComputeJacobian(const double* x, double* jacobian) const
{
	jacobian[0] = -x[1];
	jacobian[1] =  x[0];
	return true;
}

int ComplexAngleParameterization::GlobalSize() const
{
	return 2;
}

int ComplexAngleParameterization::LocalSize() const
{
	return 1;
}




