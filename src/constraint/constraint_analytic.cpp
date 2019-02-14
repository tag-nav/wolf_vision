#include "base/constraint/constraint_analytic.h"
#include "base/state_block.h"

namespace wolf {

ConstraintAnalytic::ConstraintAnalytic(const std::string&  _tp,
                                       bool _apply_loss_function, ConstraintStatus _status,
                                       StateBlockPtr _state0Ptr, StateBlockPtr _state1Ptr, StateBlockPtr _state2Ptr, StateBlockPtr _state3Ptr, StateBlockPtr _state4Ptr,
                                       StateBlockPtr _state5Ptr, StateBlockPtr _state6Ptr, StateBlockPtr _state7Ptr, StateBlockPtr _state8Ptr, StateBlockPtr _state9Ptr ) :
        ConstraintBase(_tp, _apply_loss_function, _status),
        state_ptr_vector_({_state0Ptr, _state1Ptr, _state2Ptr, _state3Ptr, _state4Ptr,
                           _state5Ptr, _state6Ptr, _state7Ptr, _state8Ptr, _state9Ptr}),
        state_block_sizes_vector_({_state0Ptr ? (unsigned int) _state0Ptr->getSize() : 0,
                                   _state1Ptr ? (unsigned int) _state1Ptr->getSize() : 0,
                                   _state2Ptr ? (unsigned int) _state2Ptr->getSize() : 0,
                                   _state3Ptr ? (unsigned int) _state3Ptr->getSize() : 0,
                                   _state4Ptr ? (unsigned int) _state4Ptr->getSize() : 0,
                                   _state5Ptr ? (unsigned int) _state5Ptr->getSize() : 0,
                                   _state6Ptr ? (unsigned int) _state6Ptr->getSize() : 0,
                                   _state7Ptr ? (unsigned int) _state7Ptr->getSize() : 0,
                                   _state8Ptr ? (unsigned int) _state8Ptr->getSize() : 0,
                                   _state9Ptr ? (unsigned int) _state9Ptr->getSize() : 0})
{
    resizeVectors();
}

ConstraintAnalytic::ConstraintAnalytic(const std::string&  _tp,
                                       const FrameBasePtr& _frame_other_ptr,
                                       const CaptureBasePtr& _capture_other_ptr,
                                       const FeatureBasePtr& _feature_other_ptr,
                                       const LandmarkBasePtr& _landmark_other_ptr,
                                       const ProcessorBasePtr& _processor_ptr,
                                       bool _apply_loss_function, ConstraintStatus _status,
                                       StateBlockPtr _state0Ptr, StateBlockPtr _state1Ptr, StateBlockPtr _state2Ptr, StateBlockPtr _state3Ptr, StateBlockPtr _state4Ptr,
                                       StateBlockPtr _state5Ptr, StateBlockPtr _state6Ptr, StateBlockPtr _state7Ptr, StateBlockPtr _state8Ptr, StateBlockPtr _state9Ptr ) :
        ConstraintBase(_tp,  _frame_other_ptr, _capture_other_ptr, _feature_other_ptr, _landmark_other_ptr, _processor_ptr, _apply_loss_function, _status),
        state_ptr_vector_({_state0Ptr, _state1Ptr, _state2Ptr, _state3Ptr, _state4Ptr,
                           _state5Ptr, _state6Ptr, _state7Ptr, _state8Ptr, _state9Ptr}),
        state_block_sizes_vector_({_state0Ptr ? (unsigned int) _state0Ptr->getSize() : 0,
                                   _state1Ptr ? (unsigned int) _state1Ptr->getSize() : 0,
                                   _state2Ptr ? (unsigned int) _state2Ptr->getSize() : 0,
                                   _state3Ptr ? (unsigned int) _state3Ptr->getSize() : 0,
                                   _state4Ptr ? (unsigned int) _state4Ptr->getSize() : 0,
                                   _state5Ptr ? (unsigned int) _state5Ptr->getSize() : 0,
                                   _state6Ptr ? (unsigned int) _state6Ptr->getSize() : 0,
                                   _state7Ptr ? (unsigned int) _state7Ptr->getSize() : 0,
                                   _state8Ptr ? (unsigned int) _state8Ptr->getSize() : 0,
                                   _state9Ptr ? (unsigned int) _state9Ptr->getSize() : 0})
{
    resizeVectors();
}
/*
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
*/
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
    assert(state_ptr_vector_[0] != nullptr && "at least one not null state block pointer required");

    for (unsigned int ii = 1; ii<state_ptr_vector_.size(); ii++)
        if (state_ptr_vector_.at(ii) == nullptr)
        {
            state_ptr_vector_.resize(ii);
            state_block_sizes_vector_.resize(ii);
            break;
        }
}

} // namespace wolf
