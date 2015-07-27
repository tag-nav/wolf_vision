
#ifndef CONSTRAINT_SPARSE_H_
#define CONSTRAINT_SPARSE_H_

//Wolf includes
#include "wolf.h"
#include "constraint_base.h"

//TODO: 
// - public static const may be are not necessary, since sizes are already kept in ConstraintBase::state_block_sizes_vector_
// 	 JVN: Yes, they are necessary for the ceres cost function constructor. Maybe, the state_block_sizes_vector_ is not necessary (it can be useful in filtering...)
// - measurement_ptr can be set from FeatureBase::measurement_, once this constraint is up-linked to a feature.
//   May be a simple get is enough to access this data.
// - 

//template class ConstraintBase
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
        std::vector<StateBase*> state_ptr_vector_;
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

        /** \brief Contructor with state pointer separated
         *
         * Constructor with state pointers separated
         *
         **/
        ConstraintSparse(FeatureBase* _ftr_ptr,
                             ConstraintType _tp,
                             StateBase* _state0Ptr,
                             StateBase* _state1Ptr = nullptr,
                             StateBase* _state2Ptr = nullptr,
                             StateBase* _state3Ptr = nullptr,
                             StateBase* _state4Ptr = nullptr,
                             StateBase* _state5Ptr = nullptr,
                             StateBase* _state6Ptr = nullptr,
                             StateBase* _state7Ptr = nullptr,
                             StateBase* _state8Ptr = nullptr,
                             StateBase* _state9Ptr = nullptr ) :
            ConstraintBase(_ftr_ptr,_tp),
            state_ptr_vector_({_state0Ptr,_state1Ptr,_state2Ptr,_state3Ptr,_state4Ptr,_state5Ptr,_state6Ptr,_state7Ptr,_state8Ptr,_state9Ptr}),
            state_block_sizes_vector_({BLOCK_0_SIZE,BLOCK_1_SIZE,BLOCK_2_SIZE,BLOCK_3_SIZE,BLOCK_4_SIZE,BLOCK_5_SIZE,BLOCK_6_SIZE,BLOCK_7_SIZE,BLOCK_8_SIZE,BLOCK_9_SIZE})
        {
            for (unsigned int ii = 1; ii<state_block_sizes_vector_.size(); ii++)
            {
                if (state_ptr_vector_.at(ii) != nullptr)
                    assert(state_block_sizes_vector_.at(ii) != 0 && "Too many non-null state pointers in ConstraintSparse constructor");

                else
                {
                    assert(state_block_sizes_vector_.at(ii) == 0 && "No non-null state pointers enough in ConstraintSparse constructor");
                    state_ptr_vector_.resize(ii);
                    state_block_sizes_vector_.resize(ii);
                    break;
                }
            }
        }

        /** \brief Destructor
         * 
         * Destructor
         * 
         **/        
        virtual ~ConstraintSparse()
        {
            //
        }

        /** \brief Returns a vector of pointers to the state blocks
         * 
         * Returns a vector of pointers to the state blocks in which this constraint depends
         * 
         **/
        virtual const std::vector<WolfScalar*> getStateBlockPtrVector()
        {
            assert(state_ptr_vector_.size() > 0 && state_ptr_vector_.size() <= 10 && "Wrong state vector size in constraint, it should be between 1 and 10");

            switch (state_ptr_vector_.size())
            {
                case 1:
                {
                    return std::vector<WolfScalar*>({state_ptr_vector_[0]->getPtr()});
                }
                case 2:
                {
                    return std::vector<WolfScalar*>({state_ptr_vector_[0]->getPtr(),
                                                     state_ptr_vector_[1]->getPtr()});
                }
                case 3:
                {
                    return std::vector<WolfScalar*>({state_ptr_vector_[0]->getPtr(),
                                                     state_ptr_vector_[1]->getPtr(),
                                                     state_ptr_vector_[2]->getPtr()});
                }
                case 4:
                {
                    return std::vector<WolfScalar*>({state_ptr_vector_[0]->getPtr(),
                                                     state_ptr_vector_[1]->getPtr(),
                                                     state_ptr_vector_[2]->getPtr(),
                                                     state_ptr_vector_[3]->getPtr()});
                }
                case 5:
                {
                    return std::vector<WolfScalar*>({state_ptr_vector_[0]->getPtr(),
                                                     state_ptr_vector_[1]->getPtr(),
                                                     state_ptr_vector_[2]->getPtr(),
                                                     state_ptr_vector_[3]->getPtr(),
                                                     state_ptr_vector_[4]->getPtr()});
                }
                case 6:
                {
                    return std::vector<WolfScalar*>({state_ptr_vector_[0]->getPtr(),
                                                     state_ptr_vector_[1]->getPtr(),
                                                     state_ptr_vector_[2]->getPtr(),
                                                     state_ptr_vector_[3]->getPtr(),
                                                     state_ptr_vector_[4]->getPtr(),
                                                     state_ptr_vector_[5]->getPtr()});
                }
                case 7:
                {
                    return std::vector<WolfScalar*>({state_ptr_vector_[0]->getPtr(),
                                                     state_ptr_vector_[1]->getPtr(),
                                                     state_ptr_vector_[2]->getPtr(),
                                                     state_ptr_vector_[3]->getPtr(),
                                                     state_ptr_vector_[4]->getPtr(),
                                                     state_ptr_vector_[5]->getPtr(),
                                                     state_ptr_vector_[6]->getPtr()});
                }
                case 8:
                {
                    return std::vector<WolfScalar*>({state_ptr_vector_[0]->getPtr(),
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
                    return std::vector<WolfScalar*>({state_ptr_vector_[0]->getPtr(),
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
                    return std::vector<WolfScalar*>({state_ptr_vector_[0]->getPtr(),
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

            return std::vector<WolfScalar*>(0); //Not going to happen
        }

        /** \brief Returns a vector of pointers to the states
         *
         * Returns a vector of pointers to the state in which this constraint depends
         *
         **/
        virtual const std::vector<StateBase*> getStatePtrVector() const
        {
            return state_ptr_vector_;
        }

        /** \brief Returns the constraint residual size
         *
         * Returns the constraint residual size
         *
         **/
        virtual unsigned int getSize() const
        {
            return MEASUREMENT_SIZE;
        }

        virtual void print(unsigned int _ntabs = 0, std::ostream& _ost = std::cout) const
        {
        	NodeLinked::printSelf(_ntabs, _ost);
        	for (unsigned int ii = 0; ii<state_block_sizes_vector_.size(); ii++)
        	{
        		printTabs(_ntabs);
        		_ost << "block " << ii << ": ";
        		for (unsigned int jj = 0; jj<state_block_sizes_vector_.at(ii); jj++)
        			_ost << *(state_ptr_vector_.at(ii)->getPtr()+jj) << " ";
        		_ost << std::endl;
        	}
        }
};
#endif
