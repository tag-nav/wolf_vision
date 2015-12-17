
#ifndef CONSTRAINT_ANALYTIC_H_
#define CONSTRAINT_ANALYTIC_H_

//Wolf includes
#include "wolf.h"
#include "constraint_base.h"

class ConstraintAnalytic: public ConstraintBase
{
    protected:
        std::vector<StateBlock*> state_ptr_vector_;
        std::vector<unsigned int> state_block_sizes_vector_;

    public:

        /** \brief Constructor of category CTR_ABSOLUTE
         *
         * Constructor of category CTR_ABSOLUTE
         *
         **/
        ConstraintAnalytic(FeatureBase* _ftr_ptr, ConstraintType _tp, ConstraintStatus _status,
                         StateBlock* _state0Ptr,
                         StateBlock* _state1Ptr = nullptr,
                         StateBlock* _state2Ptr = nullptr,
                         StateBlock* _state3Ptr = nullptr,
                         StateBlock* _state4Ptr = nullptr,
                         StateBlock* _state5Ptr = nullptr,
                         StateBlock* _state6Ptr = nullptr,
                         StateBlock* _state7Ptr = nullptr,
                         StateBlock* _state8Ptr = nullptr,
                         StateBlock* _state9Ptr = nullptr ) ;

        /** \brief Constructor of category CTR_FRAME
         *
         * Constructor of category CTR_FRAME
         *
         **/
        ConstraintAnalytic(FeatureBase* _ftr_ptr, ConstraintType _tp, FrameBase* _frame_ptr, ConstraintStatus _status,
                         StateBlock* _state0Ptr,
                         StateBlock* _state1Ptr = nullptr,
                         StateBlock* _state2Ptr = nullptr,
                         StateBlock* _state3Ptr = nullptr,
                         StateBlock* _state4Ptr = nullptr,
                         StateBlock* _state5Ptr = nullptr,
                         StateBlock* _state6Ptr = nullptr,
                         StateBlock* _state7Ptr = nullptr,
                         StateBlock* _state8Ptr = nullptr,
                         StateBlock* _state9Ptr = nullptr );

        /** \brief Constructor of category CTR_FEATURE
         *
         * Constructor of category CTR_FEATURE
         *
         **/
        ConstraintAnalytic(FeatureBase* _ftr_ptr, ConstraintType _tp, FeatureBase* _feature_ptr, ConstraintStatus _status,
                         StateBlock* _state0Ptr,
                         StateBlock* _state1Ptr = nullptr,
                         StateBlock* _state2Ptr = nullptr,
                         StateBlock* _state3Ptr = nullptr,
                         StateBlock* _state4Ptr = nullptr,
                         StateBlock* _state5Ptr = nullptr,
                         StateBlock* _state6Ptr = nullptr,
                         StateBlock* _state7Ptr = nullptr,
                         StateBlock* _state8Ptr = nullptr,
                         StateBlock* _state9Ptr = nullptr ) ;

        /** \brief Constructor of category CTR_LANDMARK
         *
         * Constructor of category CTR_LANDMARK
         *
         **/
        ConstraintAnalytic(FeatureBase* _ftr_ptr, ConstraintType _tp, LandmarkBase* _landmark_ptr, ConstraintStatus _status,
                         StateBlock* _state0Ptr,
                         StateBlock* _state1Ptr = nullptr,
                         StateBlock* _state2Ptr = nullptr,
                         StateBlock* _state3Ptr = nullptr,
                         StateBlock* _state4Ptr = nullptr,
                         StateBlock* _state5Ptr = nullptr,
                         StateBlock* _state6Ptr = nullptr,
                         StateBlock* _state7Ptr = nullptr,
                         StateBlock* _state8Ptr = nullptr,
                         StateBlock* _state9Ptr = nullptr ) ;

        /** \brief Default destructor (not recommended)
         *
         * Default destructor (please use destruct() instead of delete for guaranteeing the wolf tree integrity)
         * 
         **/        
        virtual ~ConstraintAnalytic();

        /** \brief Returns a vector of pointers to the state blocks
         *
         * Returns a vector of pointers to the state blocks in which this constraint depends
         *
         **/
        virtual const std::vector<WolfScalar*> getStateBlockPtrVector();

        /** \brief Returns a vector of pointers to the states
         *
         * Returns a vector of pointers to the state in which this constraint depends
         *
         **/
        virtual const std::vector<StateBlock*> getStatePtrVector() const;

        /** \brief Returns the constraint residual size
         *
         * Returns the constraint residual size
         *
         **/
        virtual unsigned int getSize() const = 0;

        /** \brief Returns the residual evaluated in the states provided
         *
         * Returns the residual evaluated in the states provided in std::vector of Eigen::VectorXs
         *
         **/
        virtual void evaluateResiduals(const std::vector<Eigen::VectorXs>& _st_vector) const = 0;

        /** \brief Returns the jacobians evaluated in the states provided
         *
         * Returns the jacobians evaluated in the states provided in std::vector of Eigen::VectorXs
         *
         **/
        virtual void evaluateJacobians(const std::vector<Eigen::VectorXs>& _st_vector, std::vector<Eigen::MatrixXs>& jacobians) const = 0;

        virtual void print(unsigned int _ntabs = 0, std::ostream& _ost = std::cout) const;

    private:
        void resizeVectors();
};


//////////////////////////////////////////
//          IMPLEMENTATION
//////////////////////////////////////////
ConstraintAnalytic::ConstraintAnalytic(FeatureBase* _ftr_ptr, ConstraintType _tp, ConstraintStatus _status,
                                       StateBlock* _state0Ptr, StateBlock* _state1Ptr, StateBlock* _state2Ptr, StateBlock* _state3Ptr, StateBlock* _state4Ptr,
                                       StateBlock* _state5Ptr, StateBlock* _state6Ptr, StateBlock* _state7Ptr, StateBlock* _state8Ptr, StateBlock* _state9Ptr ) :
            ConstraintBase(_tp, _status),
            state_ptr_vector_({_state0Ptr, _state1Ptr, _state2Ptr, _state3Ptr, _state4Ptr,
                               _state5Ptr, _state6Ptr, _state7Ptr, _state8Ptr, _state9Ptr})
        {
            resizeVectors();
        }

ConstraintAnalytic::ConstraintAnalytic(FeatureBase* _ftr_ptr, ConstraintType _tp, FrameBase* _frame_ptr, ConstraintStatus _status,
                                       StateBlock* _state0Ptr, StateBlock* _state1Ptr, StateBlock* _state2Ptr, StateBlock* _state3Ptr, StateBlock* _state4Ptr,
                                       StateBlock* _state5Ptr, StateBlock* _state6Ptr, StateBlock* _state7Ptr, StateBlock* _state8Ptr, StateBlock* _state9Ptr ) :
            ConstraintBase(_tp, _frame_ptr, _status),
            state_ptr_vector_({_state0Ptr, _state1Ptr, _state2Ptr, _state3Ptr, _state4Ptr,
                               _state5Ptr, _state6Ptr, _state7Ptr, _state8Ptr, _state9Ptr})
        {
            resizeVectors();
        }

ConstraintAnalytic::ConstraintAnalytic(FeatureBase* _ftr_ptr, ConstraintType _tp, FeatureBase* _feature_ptr, ConstraintStatus _status,
                                       StateBlock* _state0Ptr, StateBlock* _state1Ptr, StateBlock* _state2Ptr, StateBlock* _state3Ptr, StateBlock* _state4Ptr,
                                       StateBlock* _state5Ptr, StateBlock* _state6Ptr, StateBlock* _state7Ptr, StateBlock* _state8Ptr, StateBlock* _state9Ptr ) :
            ConstraintBase( _tp, _feature_ptr, _status),
            state_ptr_vector_({_state0Ptr, _state1Ptr, _state2Ptr, _state3Ptr, _state4Ptr,
                               _state5Ptr, _state6Ptr, _state7Ptr, _state8Ptr, _state9Ptr})
        {
            resizeVectors();
        }


ConstraintAnalytic::ConstraintAnalytic(FeatureBase* _ftr_ptr, ConstraintType _tp, LandmarkBase* _landmark_ptr, ConstraintStatus _status,
                                       StateBlock* _state0Ptr, StateBlock* _state1Ptr, StateBlock* _state2Ptr, StateBlock* _state3Ptr, StateBlock* _state4Ptr,
                                       StateBlock* _state5Ptr, StateBlock* _state6Ptr, StateBlock* _state7Ptr, StateBlock* _state8Ptr, StateBlock* _state9Ptr ) :
            ConstraintBase( _tp, _landmark_ptr, _status),
            state_ptr_vector_({_state0Ptr, _state1Ptr, _state2Ptr, _state3Ptr, _state4Ptr,
                               _state5Ptr, _state6Ptr, _state7Ptr, _state8Ptr, _state9Ptr})
        {
            resizeVectors();
        }


ConstraintAnalytic::~ConstraintAnalytic()
{
    //
}


const std::vector<WolfScalar*> ConstraintAnalytic::getStateBlockPtrVector()
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

const std::vector<StateBlock*> ConstraintAnalytic::getStatePtrVector() const
{
    return state_ptr_vector_;
}

void ConstraintAnalytic::print(unsigned int _ntabs, std::ostream& _ost) const
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

void ConstraintAnalytic::resizeVectors()
{
    for (unsigned int ii = 1; ii<state_ptr_vector_.size(); ii++)
    {
        if (state_ptr_vector_.at(ii) != nullptr)
        {
            assert(state_block_sizes_vector_.at(ii) != 0 && "Too many non-null state pointers in ConstraintSparse constructor");
            state_block_sizes_vector_.push_back(state_ptr_vector_.at(ii)->getSize());
        }

        else
        {
            assert(state_block_sizes_vector_.at(ii) == 0 && "No non-null state pointers enough in ConstraintSparse constructor");
            state_ptr_vector_.resize(ii);
            break;
        }
    }
}

#endif
