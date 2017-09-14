#include "constraint_analytic.h"
#include "state_block.h"

namespace wolf {

ConstraintAnalytic::ConstraintAnalytic(ConstraintType _tp, bool _apply_loss_function, ConstraintStatus _status,
                                       StateBlockPtr _state0Ptr, StateBlockPtr _state1Ptr, StateBlockPtr _state2Ptr, StateBlockPtr _state3Ptr, StateBlockPtr _state4Ptr,
                                       StateBlockPtr _state5Ptr, StateBlockPtr _state6Ptr, StateBlockPtr _state7Ptr, StateBlockPtr _state8Ptr, StateBlockPtr _state9Ptr ) :
            ConstraintBase(_tp, _apply_loss_function, _status),
            state_ptr_vector_({_state0Ptr, _state1Ptr, _state2Ptr, _state3Ptr, _state4Ptr,
                               _state5Ptr, _state6Ptr, _state7Ptr, _state8Ptr, _state9Ptr})
{
    resizeVectors();
}

ConstraintAnalytic::ConstraintAnalytic(ConstraintType _tp, const FrameBasePtr& _frame_ptr, const ProcessorBasePtr& _processor_ptr,
                                       bool _apply_loss_function, ConstraintStatus _status,
                                       StateBlockPtr _state0Ptr, StateBlockPtr _state1Ptr, StateBlockPtr _state2Ptr, StateBlockPtr _state3Ptr, StateBlockPtr _state4Ptr,
                                       StateBlockPtr _state5Ptr, StateBlockPtr _state6Ptr, StateBlockPtr _state7Ptr, StateBlockPtr _state8Ptr, StateBlockPtr _state9Ptr ) :
            ConstraintBase(_tp, _frame_ptr, nullptr, nullptr, _processor_ptr, _apply_loss_function, _status),
            state_ptr_vector_({_state0Ptr, _state1Ptr, _state2Ptr, _state3Ptr, _state4Ptr,
                               _state5Ptr, _state6Ptr, _state7Ptr, _state8Ptr, _state9Ptr})
{
    resizeVectors();
}

ConstraintAnalytic::ConstraintAnalytic(ConstraintType _tp, const FeatureBasePtr& _feature_ptr, const ProcessorBasePtr& _processor_ptr,
                                       bool _apply_loss_function, ConstraintStatus _status,
                                       StateBlockPtr _state0Ptr, StateBlockPtr _state1Ptr, StateBlockPtr _state2Ptr, StateBlockPtr _state3Ptr, StateBlockPtr _state4Ptr,
                                       StateBlockPtr _state5Ptr, StateBlockPtr _state6Ptr, StateBlockPtr _state7Ptr, StateBlockPtr _state8Ptr, StateBlockPtr _state9Ptr ) :
            ConstraintBase( _tp, nullptr, _feature_ptr, nullptr, _processor_ptr, _apply_loss_function, _status),
            state_ptr_vector_({_state0Ptr, _state1Ptr, _state2Ptr, _state3Ptr, _state4Ptr,
                               _state5Ptr, _state6Ptr, _state7Ptr, _state8Ptr, _state9Ptr})
{
    resizeVectors();
}


ConstraintAnalytic::ConstraintAnalytic(ConstraintType _tp, const LandmarkBasePtr& _landmark_ptr, const ProcessorBasePtr& _processor_ptr,
                                       bool _apply_loss_function, ConstraintStatus _status,
                                       StateBlockPtr _state0Ptr, StateBlockPtr _state1Ptr, StateBlockPtr _state2Ptr, StateBlockPtr _state3Ptr, StateBlockPtr _state4Ptr,
                                       StateBlockPtr _state5Ptr, StateBlockPtr _state6Ptr, StateBlockPtr _state7Ptr, StateBlockPtr _state8Ptr, StateBlockPtr _state9Ptr ) :
            ConstraintBase( _tp, nullptr, nullptr, _landmark_ptr, _processor_ptr, _apply_loss_function, _status),
            state_ptr_vector_({_state0Ptr, _state1Ptr, _state2Ptr, _state3Ptr, _state4Ptr,
                               _state5Ptr, _state6Ptr, _state7Ptr, _state8Ptr, _state9Ptr})
{
    resizeVectors();
}

std::vector<Scalar*> ConstraintAnalytic::getStateScalarPtrVector() const
{
    assert(state_ptr_vector_.size() > 0 && state_ptr_vector_.size() <= 10 && "Wrong state vector size in constraint, it should be between 1 and 10");

    std::vector<Scalar*> state_scalar_ptr_vector(state_ptr_vector_.size());

    for (auto i = 0; i < state_scalar_ptr_vector.size(); i++)
        state_scalar_ptr_vector[i] = state_ptr_vector_[i]->getPtr();

    return state_scalar_ptr_vector;
}

std::vector<StateBlockPtr> ConstraintAnalytic::getStateBlockPtrVector() const
{
    return state_ptr_vector_;
}

std::vector<unsigned int> ConstraintAnalytic::getStateSizes() const
{
    return state_block_sizes_vector_;
}

JacobianMethod ConstraintAnalytic::getJacobianMethod() const
{
    return JAC_ANALYTIC;
}

void ConstraintAnalytic::resizeVectors()
{
    for (unsigned int ii = 1; ii<state_ptr_vector_.size(); ii++)
    {
        if (state_ptr_vector_.at(ii) != nullptr)
            state_block_sizes_vector_.push_back(state_ptr_vector_.at(ii)->getSize());

        else
        {
            state_ptr_vector_.resize(ii);
            break;
        }
    }
}

} // namespace wolf
