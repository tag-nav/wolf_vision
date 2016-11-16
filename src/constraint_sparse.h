
#ifndef CONSTRAINT_SPARSE_H_
#define CONSTRAINT_SPARSE_H_

//Wolf includes
#include "constraint_base.h"
#include "state_block.h"

namespace wolf {

//TODO: change class name (and file name!->includes) to ConstraintNumericalAutoDiff 
//template class ConstraintSparse
template <const unsigned int MEASUREMENT_SIZE,
                unsigned int BLOCK_0_SIZE,
                unsigned int BLOCK_1_SIZE = 0,
                unsigned int BLOCK_2_SIZE = 0,
                unsigned int BLOCK_3_SIZE = 0,
                unsigned int BLOCK_4_SIZE = 0,
                unsigned int BLOCK_5_SIZE = 0,
                unsigned int BLOCK_6_SIZE = 0,
                unsigned int BLOCK_7_SIZE = 0,
                unsigned int BLOCK_8_SIZE = 0,
                unsigned int BLOCK_9_SIZE = 0>
class ConstraintSparse: public ConstraintBase
{
    protected:
        std::vector<StateBlockPtr> state_ptr_vector_;
        std::vector<unsigned int> state_block_sizes_vector_;

    public:
        static const unsigned int measurementSize = MEASUREMENT_SIZE;
        static const unsigned int block0Size = BLOCK_0_SIZE;
        static const unsigned int block1Size = BLOCK_1_SIZE;
        static const unsigned int block2Size = BLOCK_2_SIZE;
        static const unsigned int block3Size = BLOCK_3_SIZE;
        static const unsigned int block4Size = BLOCK_4_SIZE;
        static const unsigned int block5Size = BLOCK_5_SIZE;
        static const unsigned int block6Size = BLOCK_6_SIZE;
        static const unsigned int block7Size = BLOCK_7_SIZE;
        static const unsigned int block8Size = BLOCK_8_SIZE;
        static const unsigned int block9Size = BLOCK_9_SIZE;

        /** \brief Constructor of category CTR_ABSOLUTE
         *
         * Constructor of category CTR_ABSOLUTE
         *
         **/
        ConstraintSparse(ConstraintType _tp, bool _apply_loss_function, ConstraintStatus _status,
                         StateBlockPtr _state0Ptr,
                         StateBlockPtr _state1Ptr = nullptr,
                         StateBlockPtr _state2Ptr = nullptr,
                         StateBlockPtr _state3Ptr = nullptr,
                         StateBlockPtr _state4Ptr = nullptr,
                         StateBlockPtr _state5Ptr = nullptr,
                         StateBlockPtr _state6Ptr = nullptr,
                         StateBlockPtr _state7Ptr = nullptr,
                         StateBlockPtr _state8Ptr = nullptr,
                         StateBlockPtr _state9Ptr = nullptr ) ;

        /** \brief Constructor valid for all categories (FRAME, FEATURE, LANDMARK)
         *
         * Constructor of category CTR_FRAME
         *
         **/
        ConstraintSparse(ConstraintType _tp, FrameBasePtr _frame_other_ptr, FeatureBasePtr _feature_other_ptr, LandmarkBasePtr _landmark_other_ptr, bool _apply_loss_function, ConstraintStatus _status,
                         StateBlockPtr _state0Ptr,
                         StateBlockPtr _state1Ptr = nullptr,
                         StateBlockPtr _state2Ptr = nullptr,
                         StateBlockPtr _state3Ptr = nullptr,
                         StateBlockPtr _state4Ptr = nullptr,
                         StateBlockPtr _state5Ptr = nullptr,
                         StateBlockPtr _state6Ptr = nullptr,
                         StateBlockPtr _state7Ptr = nullptr,
                         StateBlockPtr _state8Ptr = nullptr,
                         StateBlockPtr _state9Ptr = nullptr );

        /** \brief Constructor of category CTR_FRAME
         *
         * Constructor of category CTR_FRAME
         *
         **/
        ConstraintSparse(ConstraintType _tp, FrameBasePtr _frame_other_ptr, bool _apply_loss_function, ConstraintStatus _status,
                         StateBlockPtr _state0Ptr,
                         StateBlockPtr _state1Ptr = nullptr,
                         StateBlockPtr _state2Ptr = nullptr,
                         StateBlockPtr _state3Ptr = nullptr,
                         StateBlockPtr _state4Ptr = nullptr,
                         StateBlockPtr _state5Ptr = nullptr,
                         StateBlockPtr _state6Ptr = nullptr,
                         StateBlockPtr _state7Ptr = nullptr,
                         StateBlockPtr _state8Ptr = nullptr,
                         StateBlockPtr _state9Ptr = nullptr );

        /** \brief Constructor of category CTR_FEATURE
         *
         * Constructor of category CTR_FEATURE
         *
         **/
        ConstraintSparse(ConstraintType _tp, FeatureBasePtr _feature_other_ptr, bool _apply_loss_function, ConstraintStatus _status,
                         StateBlockPtr _state0Ptr,
                         StateBlockPtr _state1Ptr = nullptr,
                         StateBlockPtr _state2Ptr = nullptr,
                         StateBlockPtr _state3Ptr = nullptr,
                         StateBlockPtr _state4Ptr = nullptr,
                         StateBlockPtr _state5Ptr = nullptr,
                         StateBlockPtr _state6Ptr = nullptr,
                         StateBlockPtr _state7Ptr = nullptr,
                         StateBlockPtr _state8Ptr = nullptr,
                         StateBlockPtr _state9Ptr = nullptr ) ;

        /** \brief Constructor of category CTR_LANDMARK
         *
         * Constructor of category CTR_LANDMARK
         *
         **/
        ConstraintSparse(ConstraintType _tp, LandmarkBasePtr _landmark_other_ptr, bool _apply_loss_function, ConstraintStatus _status,
                         StateBlockPtr _state0Ptr,
                         StateBlockPtr _state1Ptr = nullptr,
                         StateBlockPtr _state2Ptr = nullptr,
                         StateBlockPtr _state3Ptr = nullptr,
                         StateBlockPtr _state4Ptr = nullptr,
                         StateBlockPtr _state5Ptr = nullptr,
                         StateBlockPtr _state6Ptr = nullptr,
                         StateBlockPtr _state7Ptr = nullptr,
                         StateBlockPtr _state8Ptr = nullptr,
                         StateBlockPtr _state9Ptr = nullptr ) ;

        virtual ~ConstraintSparse();

        /** \brief Returns a vector of pointers to the state blocks
         *
         * Returns a vector of pointers to the state blocks in which this constraint depends
         *
         **/
        virtual const std::vector<Scalar*> getStateBlockPtrVector();

        /** \brief Returns a vector of pointers to the states
         *
         * Returns a vector of pointers to the state in which this constraint depends
         *
         **/
        virtual const std::vector<StateBlockPtr> getStatePtrVector() const;

        /** \brief Returns the constraint residual size
         *
         * Returns the constraint residual size
         *
         **/
        virtual unsigned int getSize() const;

    private:
        void resizeVectors();
};


//////////////////////////////////////////
//          IMPLEMENTATION
//////////////////////////////////////////
template <const unsigned int MEASUREMENT_SIZE,
                unsigned int BLOCK_0_SIZE,
                unsigned int BLOCK_1_SIZE,
                unsigned int BLOCK_2_SIZE,
                unsigned int BLOCK_3_SIZE,
                unsigned int BLOCK_4_SIZE,
                unsigned int BLOCK_5_SIZE,
                unsigned int BLOCK_6_SIZE,
                unsigned int BLOCK_7_SIZE,
                unsigned int BLOCK_8_SIZE,
                unsigned int BLOCK_9_SIZE>
ConstraintSparse<MEASUREMENT_SIZE,
                 BLOCK_0_SIZE,
                 BLOCK_1_SIZE,
                 BLOCK_2_SIZE,
                 BLOCK_3_SIZE,
                 BLOCK_4_SIZE,
                 BLOCK_5_SIZE,
                 BLOCK_6_SIZE,
                 BLOCK_7_SIZE,
                 BLOCK_8_SIZE,
                 BLOCK_9_SIZE>::ConstraintSparse(ConstraintType _tp, bool _apply_loss_function, ConstraintStatus _status,
                                                 StateBlockPtr _state0Ptr,
                                                 StateBlockPtr _state1Ptr,
                                                 StateBlockPtr _state2Ptr,
                                                 StateBlockPtr _state3Ptr,
                                                 StateBlockPtr _state4Ptr,
                                                 StateBlockPtr _state5Ptr,
                                                 StateBlockPtr _state6Ptr,
                                                 StateBlockPtr _state7Ptr,
                                                 StateBlockPtr _state8Ptr,
                                                 StateBlockPtr _state9Ptr ) :
            ConstraintBase(_tp, _apply_loss_function, _status),
            state_ptr_vector_({_state0Ptr,_state1Ptr,_state2Ptr,_state3Ptr,_state4Ptr,_state5Ptr,_state6Ptr,_state7Ptr,_state8Ptr,_state9Ptr}),
            state_block_sizes_vector_({BLOCK_0_SIZE,BLOCK_1_SIZE,BLOCK_2_SIZE,BLOCK_3_SIZE,BLOCK_4_SIZE,BLOCK_5_SIZE,BLOCK_6_SIZE,BLOCK_7_SIZE,BLOCK_8_SIZE,BLOCK_9_SIZE})
        {
            resizeVectors();
        }

template <const unsigned int MEASUREMENT_SIZE,
                unsigned int BLOCK_0_SIZE,
                unsigned int BLOCK_1_SIZE,
                unsigned int BLOCK_2_SIZE,
                unsigned int BLOCK_3_SIZE,
                unsigned int BLOCK_4_SIZE,
                unsigned int BLOCK_5_SIZE,
                unsigned int BLOCK_6_SIZE,
                unsigned int BLOCK_7_SIZE,
                unsigned int BLOCK_8_SIZE,
                unsigned int BLOCK_9_SIZE>
ConstraintSparse<MEASUREMENT_SIZE,
                 BLOCK_0_SIZE,
                 BLOCK_1_SIZE,
                 BLOCK_2_SIZE,
                 BLOCK_3_SIZE,
                 BLOCK_4_SIZE,
                 BLOCK_5_SIZE,
                 BLOCK_6_SIZE,
                 BLOCK_7_SIZE,
                 BLOCK_8_SIZE,
                 BLOCK_9_SIZE>::ConstraintSparse(ConstraintType _tp, FrameBasePtr _frame_other_ptr, FeatureBasePtr _feature_other_ptr, LandmarkBasePtr _landmark_other_ptr, bool _apply_loss_function, ConstraintStatus _status,
                                                 StateBlockPtr _state0Ptr,
                                                 StateBlockPtr _state1Ptr,
                                                 StateBlockPtr _state2Ptr,
                                                 StateBlockPtr _state3Ptr,
                                                 StateBlockPtr _state4Ptr,
                                                 StateBlockPtr _state5Ptr,
                                                 StateBlockPtr _state6Ptr,
                                                 StateBlockPtr _state7Ptr,
                                                 StateBlockPtr _state8Ptr,
                                                 StateBlockPtr _state9Ptr ) :
            ConstraintBase(_tp, _frame_other_ptr, _feature_other_ptr, _landmark_other_ptr, _apply_loss_function, _status),
            state_ptr_vector_({_state0Ptr,_state1Ptr,_state2Ptr,_state3Ptr,_state4Ptr,_state5Ptr,_state6Ptr,_state7Ptr,_state8Ptr,_state9Ptr}),
            state_block_sizes_vector_({BLOCK_0_SIZE,BLOCK_1_SIZE,BLOCK_2_SIZE,BLOCK_3_SIZE,BLOCK_4_SIZE,BLOCK_5_SIZE,BLOCK_6_SIZE,BLOCK_7_SIZE,BLOCK_8_SIZE,BLOCK_9_SIZE})
        {
            resizeVectors();
        }

template <const unsigned int MEASUREMENT_SIZE,
                unsigned int BLOCK_0_SIZE,
                unsigned int BLOCK_1_SIZE,
                unsigned int BLOCK_2_SIZE,
                unsigned int BLOCK_3_SIZE,
                unsigned int BLOCK_4_SIZE,
                unsigned int BLOCK_5_SIZE,
                unsigned int BLOCK_6_SIZE,
                unsigned int BLOCK_7_SIZE,
                unsigned int BLOCK_8_SIZE,
                unsigned int BLOCK_9_SIZE>
ConstraintSparse<MEASUREMENT_SIZE,
                 BLOCK_0_SIZE,
                 BLOCK_1_SIZE,
                 BLOCK_2_SIZE,
                 BLOCK_3_SIZE,
                 BLOCK_4_SIZE,
                 BLOCK_5_SIZE,
                 BLOCK_6_SIZE,
                 BLOCK_7_SIZE,
                 BLOCK_8_SIZE,
                 BLOCK_9_SIZE>::ConstraintSparse(ConstraintType _tp, FrameBasePtr _frame_other_ptr, bool _apply_loss_function, ConstraintStatus _status,
                                                 StateBlockPtr _state0Ptr,
                                                 StateBlockPtr _state1Ptr,
                                                 StateBlockPtr _state2Ptr,
                                                 StateBlockPtr _state3Ptr,
                                                 StateBlockPtr _state4Ptr,
                                                 StateBlockPtr _state5Ptr,
                                                 StateBlockPtr _state6Ptr,
                                                 StateBlockPtr _state7Ptr,
                                                 StateBlockPtr _state8Ptr,
                                                 StateBlockPtr _state9Ptr ) :
            ConstraintBase(_tp, _frame_other_ptr, _apply_loss_function, _status),
            state_ptr_vector_({_state0Ptr,_state1Ptr,_state2Ptr,_state3Ptr,_state4Ptr,_state5Ptr,_state6Ptr,_state7Ptr,_state8Ptr,_state9Ptr}),
            state_block_sizes_vector_({BLOCK_0_SIZE,BLOCK_1_SIZE,BLOCK_2_SIZE,BLOCK_3_SIZE,BLOCK_4_SIZE,BLOCK_5_SIZE,BLOCK_6_SIZE,BLOCK_7_SIZE,BLOCK_8_SIZE,BLOCK_9_SIZE})
        {
            resizeVectors();
        }

template <const unsigned int MEASUREMENT_SIZE,
                unsigned int BLOCK_0_SIZE,
                unsigned int BLOCK_1_SIZE,
                unsigned int BLOCK_2_SIZE,
                unsigned int BLOCK_3_SIZE,
                unsigned int BLOCK_4_SIZE,
                unsigned int BLOCK_5_SIZE,
                unsigned int BLOCK_6_SIZE,
                unsigned int BLOCK_7_SIZE,
                unsigned int BLOCK_8_SIZE,
                unsigned int BLOCK_9_SIZE>
ConstraintSparse<MEASUREMENT_SIZE,
                 BLOCK_0_SIZE,
                 BLOCK_1_SIZE,
                 BLOCK_2_SIZE,
                 BLOCK_3_SIZE,
                 BLOCK_4_SIZE,
                 BLOCK_5_SIZE,
                 BLOCK_6_SIZE,
                 BLOCK_7_SIZE,
                 BLOCK_8_SIZE,
                 BLOCK_9_SIZE>::ConstraintSparse(ConstraintType _tp, FeatureBasePtr _feature_other_ptr, bool _apply_loss_function, ConstraintStatus _status,
                                                 StateBlockPtr _state0Ptr,
                                                 StateBlockPtr _state1Ptr,
                                                 StateBlockPtr _state2Ptr,
                                                 StateBlockPtr _state3Ptr,
                                                 StateBlockPtr _state4Ptr,
                                                 StateBlockPtr _state5Ptr,
                                                 StateBlockPtr _state6Ptr,
                                                 StateBlockPtr _state7Ptr,
                                                 StateBlockPtr _state8Ptr,
                                                 StateBlockPtr _state9Ptr ) :
            ConstraintBase( _tp, _feature_other_ptr, _apply_loss_function, _status),
            state_ptr_vector_({_state0Ptr,_state1Ptr,_state2Ptr,_state3Ptr,_state4Ptr,_state5Ptr,_state6Ptr,_state7Ptr,_state8Ptr,_state9Ptr}),
            state_block_sizes_vector_({BLOCK_0_SIZE,BLOCK_1_SIZE,BLOCK_2_SIZE,BLOCK_3_SIZE,BLOCK_4_SIZE,BLOCK_5_SIZE,BLOCK_6_SIZE,BLOCK_7_SIZE,BLOCK_8_SIZE,BLOCK_9_SIZE})
        {
            resizeVectors();
        }

template <const unsigned int MEASUREMENT_SIZE,
                unsigned int BLOCK_0_SIZE,
                unsigned int BLOCK_1_SIZE,
                unsigned int BLOCK_2_SIZE,
                unsigned int BLOCK_3_SIZE,
                unsigned int BLOCK_4_SIZE,
                unsigned int BLOCK_5_SIZE,
                unsigned int BLOCK_6_SIZE,
                unsigned int BLOCK_7_SIZE,
                unsigned int BLOCK_8_SIZE,
                unsigned int BLOCK_9_SIZE>
ConstraintSparse<MEASUREMENT_SIZE,
                 BLOCK_0_SIZE,
                 BLOCK_1_SIZE,
                 BLOCK_2_SIZE,
                 BLOCK_3_SIZE,
                 BLOCK_4_SIZE,
                 BLOCK_5_SIZE,
                 BLOCK_6_SIZE,
                 BLOCK_7_SIZE,
                 BLOCK_8_SIZE,
                 BLOCK_9_SIZE>::ConstraintSparse(ConstraintType _tp, LandmarkBasePtr _landmark_other_ptr, bool _apply_loss_function, ConstraintStatus _status,
                                                 StateBlockPtr _state0Ptr,
                                                 StateBlockPtr _state1Ptr,
                                                 StateBlockPtr _state2Ptr,
                                                 StateBlockPtr _state3Ptr,
                                                 StateBlockPtr _state4Ptr,
                                                 StateBlockPtr _state5Ptr,
                                                 StateBlockPtr _state6Ptr,
                                                 StateBlockPtr _state7Ptr,
                                                 StateBlockPtr _state8Ptr,
                                                 StateBlockPtr _state9Ptr ) :
            ConstraintBase( _tp, _landmark_other_ptr, _apply_loss_function, _status),
            state_ptr_vector_({_state0Ptr,_state1Ptr,_state2Ptr,_state3Ptr,_state4Ptr,_state5Ptr,_state6Ptr,_state7Ptr,_state8Ptr,_state9Ptr}),
            state_block_sizes_vector_({BLOCK_0_SIZE,BLOCK_1_SIZE,BLOCK_2_SIZE,BLOCK_3_SIZE,BLOCK_4_SIZE,BLOCK_5_SIZE,BLOCK_6_SIZE,BLOCK_7_SIZE,BLOCK_8_SIZE,BLOCK_9_SIZE})
        {
            resizeVectors();
        }

template <const unsigned int MEASUREMENT_SIZE,
                unsigned int BLOCK_0_SIZE,
                unsigned int BLOCK_1_SIZE,
                unsigned int BLOCK_2_SIZE,
                unsigned int BLOCK_3_SIZE,
                unsigned int BLOCK_4_SIZE,
                unsigned int BLOCK_5_SIZE,
                unsigned int BLOCK_6_SIZE,
                unsigned int BLOCK_7_SIZE,
                unsigned int BLOCK_8_SIZE,
                unsigned int BLOCK_9_SIZE>
ConstraintSparse<MEASUREMENT_SIZE,
                 BLOCK_0_SIZE,
                 BLOCK_1_SIZE,
                 BLOCK_2_SIZE,
                 BLOCK_3_SIZE,
                 BLOCK_4_SIZE,
                 BLOCK_5_SIZE,
                 BLOCK_6_SIZE,
                 BLOCK_7_SIZE,
                 BLOCK_8_SIZE,
                 BLOCK_9_SIZE>::~ConstraintSparse()
{
    //
}

template <const unsigned int MEASUREMENT_SIZE,
                unsigned int BLOCK_0_SIZE,
                unsigned int BLOCK_1_SIZE,
                unsigned int BLOCK_2_SIZE,
                unsigned int BLOCK_3_SIZE,
                unsigned int BLOCK_4_SIZE,
                unsigned int BLOCK_5_SIZE,
                unsigned int BLOCK_6_SIZE,
                unsigned int BLOCK_7_SIZE,
                unsigned int BLOCK_8_SIZE,
                unsigned int BLOCK_9_SIZE>
const std::vector<Scalar*> ConstraintSparse<MEASUREMENT_SIZE,
                                                BLOCK_0_SIZE,
                                                BLOCK_1_SIZE,
                                                BLOCK_2_SIZE,
                                                BLOCK_3_SIZE,
                                                BLOCK_4_SIZE,
                                                BLOCK_5_SIZE,
                                                BLOCK_6_SIZE,
                                                BLOCK_7_SIZE,
                                                BLOCK_8_SIZE,
                                                BLOCK_9_SIZE>::getStateBlockPtrVector()
{
    assert(state_ptr_vector_.size() > 0 && state_ptr_vector_.size() <= 10 && "Wrong state vector size in constraint, it should be between 1 and 10");

    switch (state_ptr_vector_.size())
    {
        case 1:
        {
            return std::vector<Scalar*>({state_ptr_vector_[0]->getPtr()});
        }
        case 2:
        {
            return std::vector<Scalar*>({state_ptr_vector_[0]->getPtr(),
                                             state_ptr_vector_[1]->getPtr()});
        }
        case 3:
        {
            return std::vector<Scalar*>({state_ptr_vector_[0]->getPtr(),
                                             state_ptr_vector_[1]->getPtr(),
                                             state_ptr_vector_[2]->getPtr()});
        }
        case 4:
        {
            return std::vector<Scalar*>({state_ptr_vector_[0]->getPtr(),
                                             state_ptr_vector_[1]->getPtr(),
                                             state_ptr_vector_[2]->getPtr(),
                                             state_ptr_vector_[3]->getPtr()});
        }
        case 5:
        {
            return std::vector<Scalar*>({state_ptr_vector_[0]->getPtr(),
                                             state_ptr_vector_[1]->getPtr(),
                                             state_ptr_vector_[2]->getPtr(),
                                             state_ptr_vector_[3]->getPtr(),
                                             state_ptr_vector_[4]->getPtr()});
        }
        case 6:
        {
            return std::vector<Scalar*>({state_ptr_vector_[0]->getPtr(),
                                             state_ptr_vector_[1]->getPtr(),
                                             state_ptr_vector_[2]->getPtr(),
                                             state_ptr_vector_[3]->getPtr(),
                                             state_ptr_vector_[4]->getPtr(),
                                             state_ptr_vector_[5]->getPtr()});
        }
        case 7:
        {
            return std::vector<Scalar*>({state_ptr_vector_[0]->getPtr(),
                                             state_ptr_vector_[1]->getPtr(),
                                             state_ptr_vector_[2]->getPtr(),
                                             state_ptr_vector_[3]->getPtr(),
                                             state_ptr_vector_[4]->getPtr(),
                                             state_ptr_vector_[5]->getPtr(),
                                             state_ptr_vector_[6]->getPtr()});
        }
        case 8:
        {
            return std::vector<Scalar*>({state_ptr_vector_[0]->getPtr(),
                                             state_ptr_vector_[1]->getPtr(),
                                             state_ptr_vector_[2]->getPtr(),
                                             state_ptr_vector_[3]->getPtr(),
                                             state_ptr_vector_[4]->getPtr(),
                                             state_ptr_vector_[5]->getPtr(),
                                             state_ptr_vector_[6]->getPtr(),
                                             state_ptr_vector_[7]->getPtr()});
        }
        case 9:
        {
            return std::vector<Scalar*>({state_ptr_vector_[0]->getPtr(),
                                             state_ptr_vector_[1]->getPtr(),
                                             state_ptr_vector_[2]->getPtr(),
                                             state_ptr_vector_[3]->getPtr(),
                                             state_ptr_vector_[4]->getPtr(),
                                             state_ptr_vector_[5]->getPtr(),
                                             state_ptr_vector_[6]->getPtr(),
                                             state_ptr_vector_[7]->getPtr(),
                                             state_ptr_vector_[8]->getPtr()});
        }
        case 10:
        {
            return std::vector<Scalar*>({state_ptr_vector_[0]->getPtr(),
                                             state_ptr_vector_[1]->getPtr(),
                                             state_ptr_vector_[2]->getPtr(),
                                             state_ptr_vector_[3]->getPtr(),
                                             state_ptr_vector_[4]->getPtr(),
                                             state_ptr_vector_[5]->getPtr(),
                                             state_ptr_vector_[6]->getPtr(),
                                             state_ptr_vector_[7]->getPtr(),
                                             state_ptr_vector_[8]->getPtr(),
                                             state_ptr_vector_[9]->getPtr()});
        }
    }

    return std::vector<Scalar*>(0); //Not going to happen
}

template <const unsigned int MEASUREMENT_SIZE,
                unsigned int BLOCK_0_SIZE,
                unsigned int BLOCK_1_SIZE,
                unsigned int BLOCK_2_SIZE,
                unsigned int BLOCK_3_SIZE,
                unsigned int BLOCK_4_SIZE,
                unsigned int BLOCK_5_SIZE,
                unsigned int BLOCK_6_SIZE,
                unsigned int BLOCK_7_SIZE,
                unsigned int BLOCK_8_SIZE,
                unsigned int BLOCK_9_SIZE>
const std::vector<StateBlockPtr> ConstraintSparse<MEASUREMENT_SIZE,
                                                BLOCK_0_SIZE,
                                                BLOCK_1_SIZE,
                                                BLOCK_2_SIZE,
                                                BLOCK_3_SIZE,
                                                BLOCK_4_SIZE,
                                                BLOCK_5_SIZE,
                                                BLOCK_6_SIZE,
                                                BLOCK_7_SIZE,
                                                BLOCK_8_SIZE,
                                                BLOCK_9_SIZE>::getStatePtrVector() const
{
    return state_ptr_vector_;
}

template <const unsigned int MEASUREMENT_SIZE,
                unsigned int BLOCK_0_SIZE,
                unsigned int BLOCK_1_SIZE,
                unsigned int BLOCK_2_SIZE,
                unsigned int BLOCK_3_SIZE,
                unsigned int BLOCK_4_SIZE,
                unsigned int BLOCK_5_SIZE,
                unsigned int BLOCK_6_SIZE,
                unsigned int BLOCK_7_SIZE,
                unsigned int BLOCK_8_SIZE,
                unsigned int BLOCK_9_SIZE>
unsigned int ConstraintSparse<MEASUREMENT_SIZE,
                              BLOCK_0_SIZE,
                              BLOCK_1_SIZE,
                              BLOCK_2_SIZE,
                              BLOCK_3_SIZE,
                              BLOCK_4_SIZE,
                              BLOCK_5_SIZE,
                              BLOCK_6_SIZE,
                              BLOCK_7_SIZE,
                              BLOCK_8_SIZE,
                              BLOCK_9_SIZE>::getSize() const
{
    return MEASUREMENT_SIZE;
}


template <const unsigned int MEASUREMENT_SIZE,
                unsigned int BLOCK_0_SIZE,
                unsigned int BLOCK_1_SIZE,
                unsigned int BLOCK_2_SIZE,
                unsigned int BLOCK_3_SIZE,
                unsigned int BLOCK_4_SIZE,
                unsigned int BLOCK_5_SIZE,
                unsigned int BLOCK_6_SIZE,
                unsigned int BLOCK_7_SIZE,
                unsigned int BLOCK_8_SIZE,
                unsigned int BLOCK_9_SIZE>
void ConstraintSparse<MEASUREMENT_SIZE,
                      BLOCK_0_SIZE,
                      BLOCK_1_SIZE,
                      BLOCK_2_SIZE,
                      BLOCK_3_SIZE,
                      BLOCK_4_SIZE,
                      BLOCK_5_SIZE,
                      BLOCK_6_SIZE,
                      BLOCK_7_SIZE,
                      BLOCK_8_SIZE,
                      BLOCK_9_SIZE>::resizeVectors()
{
    for (unsigned int ii = 1; ii<state_block_sizes_vector_.size(); ii++)
    {
        if (state_block_sizes_vector_.at(ii) != 0)
        {
        	for (unsigned int jj = 0; jj < ii; jj++)
        		assert(state_ptr_vector_.at(ii) != state_ptr_vector_.at(jj) && "ConstraintSparse: Repeated state block.");
        	assert(state_ptr_vector_.at(ii) != nullptr && "ConstraintSparse: Null state pointer in a non-zero sized block!");
        	assert(state_ptr_vector_[ii]->getSize() == state_block_sizes_vector_[ii] && "incoherent state block size and template block size");
        }

        else
        {
            assert(state_ptr_vector_.at(ii) == nullptr && "ConstraintSparse: Non-null state pointer in a zero sized block!");
            state_ptr_vector_.resize(ii);
            state_block_sizes_vector_.resize(ii);
            break;
        }
    }
}

} // namespace wolf

#endif
