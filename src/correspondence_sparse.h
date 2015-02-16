
#ifndef CORRESPONDENCE_SPARSE_H_
#define CORRESPONDENCE_SPARSE_H_

//Wolf includes
#include "wolf.h"
#include "correspondence_base.h"

//TODO: 
// - public static const may be are not necessary, since sizes are already kept in CorrespondenceBase::state_block_sizes_vector_
// 	 JVN: Yes, they are necessary for the ceres cost function constructor. Maybe, the state_block_sizes_vector_ is not necessary (it can be useful in filtering...)
// - measurement_ptr can be set from FeatureBase::measurement_, once this correspondence is up-linked to a feature. 
//   May be a simple get is enough to access this data.
// - 

//template class CorrespondenceBase
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
class CorrespondenceSparse: public CorrespondenceBase
{
    protected:
        std::vector<WolfScalar*> state_block_ptr_vector_;
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

         /** \brief Contructor with state pointer array
         * 
         * Constructor with state pointer array
         * JVN: Potser aquest constructor no l'utilitzarem mai.. no?
         * 
         **/               
        CorrespondenceSparse(const FeatureBasePtr& _ftr_ptr, CorrespondenceType _tp, WolfScalar** _blockPtrArray) :
            CorrespondenceBase(_ftr_ptr,_tp),
            state_block_ptr_vector_(10),
            state_block_sizes_vector_({BLOCK_0_SIZE,BLOCK_1_SIZE,BLOCK_2_SIZE,BLOCK_3_SIZE,BLOCK_4_SIZE,BLOCK_5_SIZE,BLOCK_6_SIZE,BLOCK_7_SIZE,BLOCK_8_SIZE,BLOCK_9_SIZE})
        {
            for (uint ii = 0; ii<state_block_sizes_vector_.size(); ii++)
            {
                if (state_block_sizes_vector_.at(ii) != 0)
                {
                    state_block_ptr_vector_.at(ii) = _blockPtrArray[ii];
                }
                else //at the end the vector is cropped to just relevant components
                {
                    state_block_ptr_vector_.resize(ii); 
                    break;
                }
            }
        }

        /** \brief Contructor with state pointer separated
         * 
         * Constructor with state pointers separated
         * 
         **/        
        CorrespondenceSparse(const FeatureBasePtr& _ftr_ptr,
        					 CorrespondenceType _tp,
                             WolfScalar* _state0Ptr,
                             WolfScalar* _state1Ptr = nullptr,
                             WolfScalar* _state2Ptr = nullptr,
                             WolfScalar* _state3Ptr = nullptr,
                             WolfScalar* _state4Ptr = nullptr,
                             WolfScalar* _state5Ptr = nullptr,
                             WolfScalar* _state6Ptr = nullptr,
                             WolfScalar* _state7Ptr = nullptr,
                             WolfScalar* _state8Ptr = nullptr,
                             WolfScalar* _state9Ptr = nullptr ) :
            CorrespondenceBase(_ftr_ptr,_tp),
            state_block_ptr_vector_({_state0Ptr,_state1Ptr,_state2Ptr,_state3Ptr,_state4Ptr,_state5Ptr,_state6Ptr,_state7Ptr,_state8Ptr,_state9Ptr}),
            state_block_sizes_vector_({BLOCK_0_SIZE,BLOCK_1_SIZE,BLOCK_2_SIZE,BLOCK_3_SIZE,BLOCK_4_SIZE,BLOCK_5_SIZE,BLOCK_6_SIZE,BLOCK_7_SIZE,BLOCK_8_SIZE,BLOCK_9_SIZE})
        {
            for (uint ii = 0; ii<state_block_sizes_vector_.size(); ii++)
            {
                if ( (state_block_ptr_vector_.at(ii) == nullptr) && (state_block_sizes_vector_.at(ii) == 0) )
                {
                    state_block_sizes_vector_.resize(ii);
                    state_block_ptr_vector_.resize(ii);
                    break;
                }
                else // check error cases
                {
                    assert(state_block_ptr_vector_.at(ii) != nullptr);
                    assert(state_block_sizes_vector_.at(ii) != 0);
                }
            }
        }

        /** \brief Destructor
         * 
         * Destructor
         * 
         **/        
        virtual ~CorrespondenceSparse()
        {
            //
        }

        /** \brief Returns a vector of pointers to the state blocks
         * 
         * Returns a vector of pointers to the state blocks in which this correspondence depends
         * 
         **/
        virtual const std::vector<WolfScalar*> getStateBlockPtrVector()
        {
            return state_block_ptr_vector_;
        }

        // Ja és a correspondence_base...
        /** \brief Returns a pointer to the mesaurement associated to this correspondence
         *
         * Returns a pointer to the mesaurement associated to this correspondence.
         * Measurement is owned by upper-level feature
         **/
//        const Eigen::VectorXs * getMeasurement() const
//        {
//            return upperNode().getMeasurement();
//        }
};
#endif
