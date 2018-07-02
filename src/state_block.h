
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
class StateBlock
{
public:

  enum class Notification : std::size_t
  {
    ADD = 0,
    REMOVE,
    STATE_UPDATE,
    FIX_UPDATE
  };

  using Notifications = std::list<Notification>;

    protected:

        mutable Notifications notifications_;
        mutable std::mutex notifictions_mut_;

        NodeBaseWPtr node_ptr_; //< pointer to the wolf Node owning this StateBlock

        std::atomic_bool fixed_; ///< Key to indicate whether the state is fixed or not

        std::atomic<Size> state_size_; ///< State vector size
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
        StateBlock(const Size _size, bool _fixed = false, LocalParametrizationBasePtr _local_param_ptr = nullptr);

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
        void setState(const Eigen::VectorXs& _state);

        /** \brief Returns the state size
         **/
        Size getSize() const;

        /**\brief Returns the size of the local parametrization
         *
         * @return the size of the local parametrization
         */
        Size getLocalSize() const;

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

        bool hasNotifications() const;
};

} // namespace wolf

// IMPLEMENTATION
#include "local_parametrization_base.h"

namespace wolf {

inline StateBlock::StateBlock(const Eigen::VectorXs& _state, bool _fixed, LocalParametrizationBasePtr _local_param_ptr) :
//        notifications_{Notification::ADD},
        node_ptr_(), // nullptr
        fixed_(_fixed),
        state_size_(_state.size()),
        state_(_state),
        local_param_ptr_(_local_param_ptr)
{
//    std::cout << "constructed           +sb" << std::endl;
}

inline StateBlock::StateBlock(const Size _size, bool _fixed, LocalParametrizationBasePtr _local_param_ptr) :
//        notifications_{Notification::ADD},
        node_ptr_(), // nullptr
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

inline void StateBlock::setState(const Eigen::VectorXs& _state)
{
    assert(_state.size() == state_.size());

    {
      std::lock_guard<std::mutex> lock(mut_state_);
      state_ = _state;
      state_size_ = state_.size();
    }

    addNotification(Notification::STATE_UPDATE);
}

inline Size StateBlock::getSize() const
{
    return state_size_.load();
}

inline Size StateBlock::getLocalSize() const
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

inline void StateBlock::setFixed(bool _fixed)
{
    fixed_.store(_fixed);
    addNotification(Notification::FIX_UPDATE);
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

inline StateBlock::Notifications StateBlock::consumeNotifications() const
{
  std::lock_guard<std::mutex> lock(notifictions_mut_);
  return std::move(notifications_);
}

inline bool StateBlock::hasNotifications() const
{
  std::lock_guard<std::mutex> lock(notifictions_mut_);
  return !notifications_.empty();
}

} // namespace wolf

#endif
