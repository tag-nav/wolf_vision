#include "local_parametrization_wrapper.h"

namespace wolf {

bool LocalParametrizationWrapper::Plus(const double* x_raw, const double* delta_raw, double* x_plus_delta_raw) const
{
    Eigen::Map<const Eigen::VectorXs> x_raw_map((Scalar*)x_raw, GlobalSize());
    Eigen::Map<const Eigen::VectorXs> delta_raw_map((Scalar*)delta_raw, LocalSize());
    Eigen::Map<Eigen::VectorXs> x_plus_map((Scalar*)x_plus_delta_raw, GlobalSize());
    return local_parametrization_ptr_->plus(x_raw_map, delta_raw_map, x_plus_map);
};

bool LocalParametrizationWrapper::ComputeJacobian(const double* x, double* jacobian) const
{
    Eigen::Map<const Eigen::VectorXs> x_map((Scalar*)x, GlobalSize());
    Eigen::Map<Eigen::MatrixXs> jacobian_map((Scalar*)jacobian, GlobalSize(), LocalSize());
    return local_parametrization_ptr_->computeJacobian(x_map, jacobian_map);
};

} // namespace wolf

