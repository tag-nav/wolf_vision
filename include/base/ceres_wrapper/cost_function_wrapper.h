#ifndef TRUNK_SRC_COST_FUNCTION_WRAPPER_H_
#define TRUNK_SRC_COST_FUNCTION_WRAPPER_H_

// WOLF
#include "base/wolf.h"
#include "base/factor/factor_analytic.h"

// CERES
#include "ceres/cost_function.h"

// EIGEN
#include <Eigen/StdVector>

namespace wolf {

WOLF_PTR_TYPEDEFS(CostFunctionWrapper);

class CostFunctionWrapper : public ceres::CostFunction
{
    private:

        FactorBasePtr factor_ptr_;

    public:

        CostFunctionWrapper(FactorBasePtr _factor_ptr);

        virtual ~CostFunctionWrapper();

        virtual bool Evaluate(const double* const * parameters, double* residuals, double** jacobians) const;

        FactorBasePtr getFactor() const;
};

inline CostFunctionWrapper::CostFunctionWrapper(FactorBasePtr _factor_ptr) :
        ceres::CostFunction(), factor_ptr_(_factor_ptr)
{
    for (auto st_block_size : factor_ptr_->getStateSizes())
        mutable_parameter_block_sizes()->push_back(st_block_size);
    set_num_residuals(factor_ptr_->getSize());
}

inline CostFunctionWrapper::~CostFunctionWrapper()
{
}

inline bool CostFunctionWrapper::Evaluate(const double* const * parameters, double* residuals, double** jacobians) const
{
    return factor_ptr_->evaluate(parameters, residuals, jacobians);
}

inline FactorBasePtr CostFunctionWrapper::getFactor() const
{
    return factor_ptr_;
}

} // namespace wolf

#endif /* TRUNK_SRC_COST_FUNCTION_WRAPPER_H_ */
