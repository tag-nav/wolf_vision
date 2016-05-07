#ifndef TRUNK_SRC_COST_FUNCTION_WRAPPER_H_
#define TRUNK_SRC_COST_FUNCTION_WRAPPER_H_

// WOLF
#include "../wolf.h"
#include "../constraint_analytic.h"

// CERES
#include "ceres/cost_function.h"

namespace wolf {

class CostFunctionWrapper : public ceres::CostFunction
{
    protected:
        ConstraintAnalytic* constraint_ptr_;
        std::vector<unsigned int> state_blocks_sizes_;

    public:

        CostFunctionWrapper(ConstraintAnalytic* _constraint_ptr) :
            ceres::CostFunction(),
            constraint_ptr_(_constraint_ptr),
            state_blocks_sizes_(constraint_ptr_->getStateSizes())
        {
            for (unsigned int i = 0; i < constraint_ptr_->getStatePtrVector().size(); i++)
                mutable_parameter_block_sizes()->push_back(state_blocks_sizes_[i]);

            set_num_residuals(constraint_ptr_->getSize());
        };

        virtual ~CostFunctionWrapper()
        {

        };

        virtual bool Evaluate(double const* const* parameters, double* residuals, double** jacobians) const
        {
            // load parameters evaluation value
            std::vector<Eigen::Map<const Eigen::VectorXs>> state_blocks_map_;
            for (unsigned int i = 0; i < state_blocks_sizes_.size(); i++)
                state_blocks_map_.push_back(Eigen::Map<const Eigen::VectorXs>((Scalar*)parameters[i], state_blocks_sizes_[i]));

            // residuals
            Eigen::Map<Eigen::VectorXs> residuals_map((Scalar*)residuals, constraint_ptr_->getSize());
            residuals_map = constraint_ptr_->evaluateResiduals(state_blocks_map_);

            // also compute jacobians
            if (jacobians != nullptr)
            {
                std::vector<Eigen::Map<Eigen::MatrixXs>> jacobians_map_;
                std::vector<bool> compute_jacobians_(state_blocks_sizes_.size());

                for (unsigned int i = 0; i < state_blocks_sizes_.size(); i++)
                {
                    compute_jacobians_[i] = (jacobians[i] != nullptr);
                    if (jacobians[i] != nullptr)
                        jacobians_map_.push_back(Eigen::Map<Eigen::MatrixXs>((Scalar*)jacobians[i], constraint_ptr_->getSize(), state_blocks_sizes_[i]));
                    else
                        jacobians_map_.push_back(Eigen::Map<Eigen::MatrixXs>(nullptr, 0, 0)); //TODO: check if it can be done
                }

                // evaluate jacobians
                constraint_ptr_->evaluateJacobians(state_blocks_map_, jacobians_map_, compute_jacobians_);
            }
            return true;
        }
};

} // namespace wolf



#endif /* TRUNK_SRC_COST_FUNCTION_WRAPPER_H_ */
