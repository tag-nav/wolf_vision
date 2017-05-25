
#ifndef CONSTRAINT_SPARSE_H_
#define CONSTRAINT_SPARSE_H_

//Wolf includes
#include "constraint_base.h"
#include "state_block.h"

namespace wolf {

//TODO: change class name (and file name!->includes) to ConstraintNumericalAutoDiff 
//template class ConstraintSparse
template <const unsigned int RESIDUAL_SIZE,
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

    public:
        static const unsigned int residualSize = RESIDUAL_SIZE;
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
        static const unsigned int n_blocks = (block0Size != 0 ? 1 : 0) +
                                             (block1Size != 0 ? 1 : 0) +
                                             (block2Size != 0 ? 1 : 0) +
                                             (block3Size != 0 ? 1 : 0) +
                                             (block4Size != 0 ? 1 : 0) +
                                             (block5Size != 0 ? 1 : 0) +
                                             (block6Size != 0 ? 1 : 0) +
                                             (block7Size != 0 ? 1 : 0) +
                                             (block8Size != 0 ? 1 : 0) +
                                             (block9Size != 0 ? 1 : 0);

        const static std::vector<unsigned int> state_block_sizes_vector;

    protected:
        std::vector<StateBlockPtr> state_ptr_vector_;

    public:
        /** \brief Constructor of category ABSOLUTE
         *
         * Constructor of category ABSOLUTE
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

        virtual ~ConstraintSparse();

        /** \brief Returns a vector of pointers to the state blocks
         *
         * Returns a vector of pointers to the state blocks in which this constraint depends
         *
         **/
        virtual const std::vector<Scalar*> getStateScalarPtrVector();

        /** \brief Returns a vector of pointers to the states
         *
         * Returns a vector of pointers to the state in which this constraint depends
         *
         **/
        virtual const std::vector<StateBlockPtr> getStateBlockPtrVector() const;

        /** \brief Returns the residual size
         *
         * Returns the residual size
         *
         **/
        virtual unsigned int getSize() const;
};


//////////////////////////////////////////
//          IMPLEMENTATION
//////////////////////////////////////////
template <const unsigned int RESIDUAL_SIZE,
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
const std::vector<unsigned int> ConstraintSparse<RESIDUAL_SIZE,
                                                 BLOCK_0_SIZE,
                                                 BLOCK_1_SIZE,
                                                 BLOCK_2_SIZE,
                                                 BLOCK_3_SIZE,
                                                 BLOCK_4_SIZE,
                                                 BLOCK_5_SIZE,
                                                 BLOCK_6_SIZE,
                                                 BLOCK_7_SIZE,
                                                 BLOCK_8_SIZE,
                                                 BLOCK_9_SIZE>::state_block_sizes_vector({BLOCK_0_SIZE
                                                                                           #if BLOCK_1_SIZE > 0
                                                                                           , BLOCK_1_SIZE
                                                                                           #if BLOCK_2_SIZE > 0
                                                                                           , BLOCK_2_SIZE
                                                                                           #if BLOCK_3_SIZE > 0
                                                                                           , BLOCK_3_SIZE
                                                                                           #if BLOCK_4_SIZE > 0
                                                                                           , BLOCK_4_SIZE
                                                                                           #if BLOCK_5_SIZE > 0
                                                                                           , BLOCK_5_SIZE
                                                                                           #if BLOCK_6_SIZE > 0
                                                                                           , BLOCK_6_SIZE
                                                                                           #if BLOCK_7_SIZE > 0
                                                                                           , BLOCK_7_SIZE
                                                                                           #if BLOCK_8_SIZE > 0
                                                                                           , BLOCK_8_SIZE
                                                                                           #if BLOCK_9_SIZE > 0
                                                                                           , BLOCK_9_SIZE
                                                                                           #endif
                                                                                           #endif
                                                                                           #endif
                                                                                           #endif
                                                                                           #endif
                                                                                           #endif
                                                                                           #endif
                                                                                           #endif
                                                                                           #endif
                                                                                           });


template <const unsigned int RESIDUAL_SIZE,
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
ConstraintSparse<RESIDUAL_SIZE,
                 BLOCK_0_SIZE,
                 BLOCK_1_SIZE,
                 BLOCK_2_SIZE,
                 BLOCK_3_SIZE,
                 BLOCK_4_SIZE,
                 BLOCK_5_SIZE,
                 BLOCK_6_SIZE,
                 BLOCK_7_SIZE,
                 BLOCK_8_SIZE,
                 BLOCK_9_SIZE>::ConstraintSparse(ConstraintType     _tp,
                                                 bool               _apply_loss_function,
                                                 ConstraintStatus   _status,
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
            state_ptr_vector_({_state0Ptr,_state1Ptr,_state2Ptr,_state3Ptr,_state4Ptr,_state5Ptr,_state6Ptr,_state7Ptr,_state8Ptr,_state9Ptr})
        {
            state_ptr_vector_.resize(n_blocks);
        }

template <const unsigned int RESIDUAL_SIZE,
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
ConstraintSparse<RESIDUAL_SIZE,
                 BLOCK_0_SIZE,
                 BLOCK_1_SIZE,
                 BLOCK_2_SIZE,
                 BLOCK_3_SIZE,
                 BLOCK_4_SIZE,
                 BLOCK_5_SIZE,
                 BLOCK_6_SIZE,
                 BLOCK_7_SIZE,
                 BLOCK_8_SIZE,
                 BLOCK_9_SIZE>::ConstraintSparse(ConstraintType     _tp,
                                                 FrameBasePtr       _frame_other_ptr,
                                                 FeatureBasePtr     _feature_other_ptr,
                                                 LandmarkBasePtr    _landmark_other_ptr,
                                                 bool               _apply_loss_function,
                                                 ConstraintStatus   _status,
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
            state_ptr_vector_({_state0Ptr,_state1Ptr,_state2Ptr,_state3Ptr,_state4Ptr,_state5Ptr,_state6Ptr,_state7Ptr,_state8Ptr,_state9Ptr})
        {
            state_ptr_vector_.resize(n_blocks);
        }


template <const unsigned int RESIDUAL_SIZE,
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
ConstraintSparse<RESIDUAL_SIZE,
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

template <const unsigned int RESIDUAL_SIZE,
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
const std::vector<Scalar*> ConstraintSparse<RESIDUAL_SIZE,
                                            BLOCK_0_SIZE,
                                            BLOCK_1_SIZE,
                                            BLOCK_2_SIZE,
                                            BLOCK_3_SIZE,
                                            BLOCK_4_SIZE,
                                            BLOCK_5_SIZE,
                                            BLOCK_6_SIZE,
                                            BLOCK_7_SIZE,
                                            BLOCK_8_SIZE,
                                            BLOCK_9_SIZE>::getStateScalarPtrVector()
{
    return std::vector<Scalar*>({state_ptr_vector_[0]->getPtr()
                                 #if n_blocks > 1
                                 , state_ptr_vector_[1]->getPtr()
                                 #if n_blocks > 2
                                 , state_ptr_vector_[2]->getPtr()
                                 #if n_blocks > 3
                                 , state_ptr_vector_[3]->getPtr()
                                 #if n_blocks > 4
                                 , state_ptr_vector_[4]->getPtr()
                                 #if n_blocks > 5
                                 , state_ptr_vector_[5]->getPtr()
                                 #if n_blocks > 6
                                 , state_ptr_vector_[6]->getPtr()
                                 #if n_blocks > 7
                                 , state_ptr_vector_[7]->getPtr()
                                 #if n_blocks > 8
                                 , state_ptr_vector_[8]->getPtr()
                                 #if n_blocks > 9
                                 , state_ptr_vector_[9]->getPtr()
                                 #endif
                                 #endif
                                 #endif
                                 #endif
                                 #endif
                                 #endif
                                 #endif
                                 #endif
                                 #endif
                                 });
}

template <const unsigned int RESIDUAL_SIZE,
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
const std::vector<StateBlockPtr> ConstraintSparse<RESIDUAL_SIZE,
                                                BLOCK_0_SIZE,
                                                BLOCK_1_SIZE,
                                                BLOCK_2_SIZE,
                                                BLOCK_3_SIZE,
                                                BLOCK_4_SIZE,
                                                BLOCK_5_SIZE,
                                                BLOCK_6_SIZE,
                                                BLOCK_7_SIZE,
                                                BLOCK_8_SIZE,
                                                BLOCK_9_SIZE>::getStateBlockPtrVector() const
{
    return state_ptr_vector_;
}

template <const unsigned int RESIDUAL_SIZE,
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
unsigned int ConstraintSparse<RESIDUAL_SIZE,
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
    return RESIDUAL_SIZE;
}

} // namespace wolf

#endif
