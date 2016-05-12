
#ifndef STATE_BLOCK_H_
#define STATE_BLOCK_H_

// Fwd references
namespace wolf{
class LocalParametrizationBase;
}

//Wolf includes
#include "wolf.h"

//std includes
#include <iostream>


namespace wolf {

/** \brief class StateBlock
 *
 * This class implements a state block for general nonlinear optimization. It offers the following functionality:
 *  - A state vector storing the state values.
 *  - A key to indicate whether the state is fixed or not.
 *     - Fixed state blocks are not estimated and treated by the estimator as fixed parameters.
 *     - Non-fixed state blocks are estimated.
 *  - A local parametrization useful for optimizing in the tangent space to the manifold.
 */
class StateBlock
{
    protected:
        Eigen::VectorXs state_; ///< State vector storing the state values
        bool fixed_; ///< Key to indicate whether the state is fixed or not
        LocalParametrizationBase* local_param_ptr_; ///< Local parametrization useful for optimizing in the tangent space to the manifold
        
    public:
        /** \brief Constructor from size
         *
         * \param _size is state size
         * \param _fixed Indicates this state is not estimated and thus acts as a fixed parameter
         * \param _local_param_ptr pointer to the local parametrization for the block
         */
        StateBlock(const unsigned int _size, bool _fixed = false, LocalParametrizationBase* _local_param_ptr = nullptr);

        /** \brief Constructor from vector
         * 
         * \param _state is state vector
         * \param _fixed Indicates this state is not estimated and thus acts as a fixed parameter
         * \param _local_param_ptr pointer to the local parametrization for the block
         **/
        StateBlock(const Eigen::VectorXs _state, bool _fixed = false, LocalParametrizationBase* _local_param_ptr = nullptr);
        
        /** \brief Destructor
         **/
        virtual ~StateBlock();

        /** \brief Returns the pointer to the first element of the state
         **/
        Scalar* getPtr();
        
        /** \brief Returns the state vector
         **/
        Eigen::VectorXs getVector();

        /** \brief Sets the state vector
         **/
        void setVector(const Eigen::VectorXs& _state);

        /** \brief Returns the state size
         **/
        unsigned int getSize() const;

        /** \brief Returns if the state is fixed (not estimated)
         **/
        bool isFixed() const;

        /** \brief Sets the state as fixed
         **/
        void fix();

        /** \brief Sets the state as estimated
         **/
        void unfix();

        bool hasLocalParametrization();

        LocalParametrizationBase* getLocalParametrizationPtr();

};

} // namespace wolf

// IMPLEMENTATION
#include "local_parametrization_base.h"
namespace wolf {

inline StateBlock::StateBlock(const Eigen::VectorXs _state, bool _fixed, LocalParametrizationBase* _local_param_ptr) :
        state_(_state), fixed_(_fixed), local_param_ptr_(_local_param_ptr)
{
    //
}

inline StateBlock::StateBlock(const unsigned int _size, bool _fixed, LocalParametrizationBase* _local_param_ptr) :
        state_(_size), fixed_(_fixed), local_param_ptr_(_local_param_ptr)
{
    state_.setZero();
    //
}

inline StateBlock::~StateBlock()
{
    if (local_param_ptr_ != nullptr)
        delete local_param_ptr_;
}

inline Scalar* StateBlock::getPtr()
{
    return state_.data();
}

inline Eigen::VectorXs StateBlock::getVector()
{
    return state_;
}

inline void StateBlock::setVector(const Eigen::VectorXs& _state)
{
    assert(_state.size() == state_.size());
    state_ = _state;
}

inline unsigned int StateBlock::getSize() const
{
    return state_.size();
}

inline bool StateBlock::isFixed() const
{
    return fixed_;
}

inline void StateBlock::fix()
{
    fixed_ = true;
}

inline void StateBlock::unfix()
{
    fixed_ = false;
}

inline bool StateBlock::hasLocalParametrization()
{
    return (local_param_ptr_ != nullptr);
}

inline LocalParametrizationBase* StateBlock::getLocalParametrizationPtr()
{
    return local_param_ptr_;
}

} // namespace wolf

#endif
