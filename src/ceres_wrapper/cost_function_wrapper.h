#ifndef TRUNK_SRC_COST_FUNCTION_WRAPPER_H_
#define TRUNK_SRC_COST_FUNCTION_WRAPPER_H_

// WOLF
#include "../wolf.h"
#include "../constraint_analytic.h"

// CERES
#include "ceres/cost_function.h"

// EIGEN
#include <Eigen/StdVector>

namespace wolf {

WOLF_PTR_TYPEDEFS(CostFunctionWrapper);

class CostFunctionWrapper : public ceres::CostFunction
{
    private:

        ConstraintBasePtr constraint_ptr_;

    public:

        CostFunctionWrapper(ConstraintBasePtr _constraint_ptr);

        virtual ~CostFunctionWrapper();

        virtual bool Evaluate(const double* const * parameters, double* residuals, double** jacobians) const;

        ConstraintBasePtr getConstraintPtr() const;
};

inline CostFunctionWrapper::CostFunctionWrapper(ConstraintBasePtr _constraint_ptr) :
        ceres::CostFunction(), constraint_ptr_(_constraint_ptr)
{
    for (auto st_block_size : constraint_ptr_->getStateSizes())
        mutable_parameter_block_sizes()->push_back(st_block_size);
    set_num_residuals(constraint_ptr_->getSize());
}

inline CostFunctionWrapper::~CostFunctionWrapper()
{
}

inline bool CostFunctionWrapper::Evaluate(const double* const * parameters, double* residuals, double** jacobians) const
{
    return constraint_ptr_->evaluate(parameters, residuals, jacobians);
}

inline ConstraintBasePtr CostFunctionWrapper::getConstraintPtr() const
{
    return constraint_ptr_;
}

} // namespace wolf



#endif /* TRUNK_SRC_COST_FUNCTION_WRAPPER_H_ */
