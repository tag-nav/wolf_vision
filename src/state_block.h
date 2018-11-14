
#ifndef STATE_BLOCK_H_
#define STATE_BLOCK_H_

// Fwd references
namespace wolf{
class NodeBase;
class LocalParametrizationBase;
}

//Wolf includes
#include "wolf.h"
#include "local_parametrization_base.h"

//std includes
#include <iostream>
#include <mutex>


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
class StateBlock : public std::enable_shared_from_this<StateBlock>
{
public:

  enum class Notification : std::size_t
  {
    ADD = 0,
    REMOVE,
    UPDATE_STATE,
    UPDATE_FIX
  };

  using Notifications = std::list<Notification>;

    protected:

        mutable Notifications notifications_;
        mutable std::mutex notifictions_mut_;

        ProblemWPtr  problem_ptr_; ///< pointer to the wolf problem

        std::atomic_bool fixed_; ///< Key to indicate whether the state is fixed or not

        std::atomic<SizeEigen> state_size_; ///< State vector size
        Eigen::VectorXs state_; ///< State vector storing the state values
        mutable std::mutex mut_state_; ///< State vector mutex

        LocalParametrizationBasePtr local_param_ptr_; ///< Local parametrization useful for optimizing in the tangent space to the manifold

    public:
        /** \brief Constructor from size
         *
         * \param _size is state size
         * \param _fixed Indicates this state is not estimated and thus acts as a fixed parameter
         * \param _local_param_ptr pointer to the local parametrization for the block
         */
        StateBlock(const SizeEigen _size, bool _fixed = false, LocalParametrizationBasePtr _local_param_ptr = nullptr);

        /** \brief Constructor from vector
         * 
         * \param _state is state vector
         * \param _fixed Indicates this state is not estimated and thus acts as a fixed parameter
         * \param _local_param_ptr pointer to the local parametrization for the block
         **/
        StateBlock(const Eigen::VectorXs& _state, bool _fixed = false, LocalParametrizationBasePtr _local_param_ptr = nullptr);

        ///< Explicitly not copyable/movable
        StateBlock(const StateBlock& o) = delete;
        StateBlock(StateBlock&& o) = delete;
        StateBlock& operator=(const StateBlock& o) = delete;

        /** \brief Destructor
         **/
        virtual ~StateBlock();

        /** \brief Returns the state vector
         **/
        Eigen::VectorXs getState() const;

        /** \brief Sets the state vector
         **/
        void setState(const Eigen::VectorXs& _state, const bool _notify = true);

        /** \brief Set the pointer to Problem
         */
        void setProblem(const ProblemPtr _problem);

        /** \brief Return the problem pointer
         */
        ProblemPtr getProblem();

        /** \brief Returns the state size
         **/
        SizeEigen getSize() const;

        /**\brief Returns the size of the local parametrization
         *
         * @return the size of the local parametrization
         */
        SizeEigen getLocalSize() const;

        /** \brief Returns if the state is fixed (not estimated)
         **/
        bool isFixed() const;

        /** \brief Sets the state as fixed
         **/
        void fix();

        /** \brief Sets the state as estimated
         **/
        void unfix();

        void setFixed(bool _fixed);

        bool hasLocalParametrization() const;

        LocalParametrizationBasePtr getLocalParametrizationPtr() const;

        void setLocalParametrizationPtr(LocalParametrizationBasePtr _local_param);

        void removeLocalParametrization();

        void addNotification(const StateBlock::Notification _new_notification);

        StateBlock::Notifications consumeNotifications() const;

        /** \brief Check if exist any notification
         **/
        bool hasNotifications() const;

        /** \brief Return list of notifications
         **/
        StateBlock::Notifications getNotifications() const;

        /** \brief Print list of notifications
         **/
        void printNotifications() const;

};

} // namespace wolf

// IMPLEMENTATION
#include "problem.h"
#include "local_parametrization_base.h"
#include "node_base.h"

namespace wolf {

inline StateBlock::StateBlock(const Eigen::VectorXs& _state, bool _fixed, LocalParametrizationBasePtr _local_param_ptr) :
//        notifications_{Notification::ADD},
        fixed_(_fixed),
        state_size_(_state.size()),
        state_(_state),
        local_param_ptr_(_local_param_ptr)
{
//    std::cout << "constructed           +sb" << std::endl;
}

inline StateBlock::StateBlock(const SizeEigen _size, bool _fixed, LocalParametrizationBasePtr _local_param_ptr) :
//        notifications_{Notification::ADD},
        fixed_(_fixed),
        state_size_(_size),
        state_(Eigen::VectorXs::Zero(_size)),
        local_param_ptr_(_local_param_ptr)
{
    //
//    std::cout << "constructed           +sb" << std::endl;
}

inline StateBlock::~StateBlock()
{
//    std::cout << "destructed            -sb" << std::endl;
}

inline Eigen::VectorXs StateBlock::getState() const
{
    std::lock_guard<std::mutex> lock(mut_state_);
    return state_;
}

inline SizeEigen StateBlock::getSize() const
{
    return state_size_.load();
}

inline SizeEigen StateBlock::getLocalSize() const
{
    if(local_param_ptr_)
        return local_param_ptr_->getLocalSize();
    return getSize();
}

inline bool StateBlock::isFixed() const
{
    return fixed_.load();
}

inline void StateBlock::fix()
{
    setFixed(true);
}

inline void StateBlock::unfix()
{
    setFixed(false);
}

inline bool StateBlock::hasLocalParametrization() const
{
    return (local_param_ptr_ != nullptr);
}

inline LocalParametrizationBasePtr StateBlock::getLocalParametrizationPtr() const
{
    return local_param_ptr_;
}

inline void StateBlock::removeLocalParametrization()
{
	assert(local_param_ptr_ != nullptr && "state block without local parametrization");
    local_param_ptr_.reset();
}

inline void StateBlock::setLocalParametrizationPtr(LocalParametrizationBasePtr _local_param)
{
	assert(_local_param != nullptr && "setting a null local parametrization");
    local_param_ptr_ = _local_param;
}

inline void StateBlock::addNotification(const StateBlock::Notification _new_notification)
{
  std::lock_guard<std::mutex> lock(notifictions_mut_);
  notifications_.emplace_back(_new_notification);
}

inline void StateBlock::setProblem(const ProblemPtr _problem)
{
    problem_ptr_ = _problem;
}

inline ProblemPtr StateBlock::getProblem()
{
    return problem_ptr_.lock();
}

inline bool StateBlock::hasNotifications() const
{
  std::lock_guard<std::mutex> lock(notifictions_mut_);
  return !notifications_.empty();
}

} // namespace wolf

#endif
