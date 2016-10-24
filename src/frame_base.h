
#ifndef FRAME_BASE_H_
#define FRAME_BASE_H_

// Fwd refs
namespace wolf{
class TrajectoryBase;
class CaptureBase;
class StateBlock;
}

//Wolf includes
#include "wolf.h"
#include "time_stamp.h"
#include "node_base.h"

//std includes

namespace wolf {

//class FrameBase
class FrameBase : public NodeBase, public std::enable_shared_from_this<FrameBase>
{
    private:
        TrajectoryBaseWPtr trajectory_ptr_;
        CaptureBaseList capture_list_;
        ConstraintBaseList constrained_by_list_;
        std::vector<StateBlockPtr> state_block_vec_; ///< vector of state blocks, in the order P, O, V.

        static unsigned int frame_id_count_;

    protected:
        unsigned int frame_id_;
        FrameKeyType type_id_;     ///< type of frame. Either NON_KEY_FRAME or KEY_FRAME. (types defined at wolf.h)
        StateStatus status_;       ///< status of the estimation of the frame state
        TimeStamp time_stamp_;     ///< frame time stamp
        
    public:
        /** \brief Constructor of non-key Frame with only time stamp
         *
         * Constructor with only time stamp
         * \param _ts is the time stamp associated to this frame, provided in seconds
         * \param _p_ptr StateBlock pointer to the position (default: nullptr)
         * \param _o_ptr StateBlock pointer to the orientation (default: nullptr). Pass a StateQuaternion if needed.
         * \param _v_ptr StateBlock pointer to the velocity (default: nullptr).
         **/
        FrameBase(const TimeStamp& _ts, StateBlockPtr _p_ptr, StateBlockPtr _o_ptr = nullptr, StateBlockPtr _v_ptr = nullptr);
        
        /** \brief Constructor with type, time stamp and state pointer
         * 
         * Constructor with type, time stamp and state pointer
         * \param _tp indicates frame type. Generally either NON_KEY_FRAME or KEY_FRAME. (types defined at wolf.h)
         * \param _ts is the time stamp associated to this frame, provided in seconds
         * \param _p_ptr StateBlock pointer to the position (default: nullptr)
         * \param _o_ptr StateBlock pointer to the orientation (default: nullptr)
         * \param _v_ptr StateBlock pointer to the velocity (default: nullptr).
         **/        
        FrameBase(const FrameKeyType & _tp, const TimeStamp& _ts, StateBlockPtr _p_ptr, StateBlockPtr _o_ptr = nullptr, StateBlockPtr _v_ptr = nullptr);

        virtual ~FrameBase();
        void remove();

        unsigned int id();



        // Frame properties -----------------------------------------------
    public:
        // KeyFrame / NonKeyFrame
        bool isKey() const;
        void setKey();

        // Fixed / Estimated
        void fix();
        void unfix();
        bool isFixed() const;

        // Frame values ------------------------------------------------
    public:
        void setTimeStamp(const TimeStamp& _ts);
        TimeStamp getTimeStamp() const;
        void getTimeStamp(TimeStamp& _ts) const;

        // State blocks
    public:
        const std::vector<StateBlockPtr>& getStateBlockVec() const
        {
            return state_block_vec_;
        }
        std::vector<StateBlockPtr>& getStateBlockVec()
        {
            return state_block_vec_;
        }
        StateBlockPtr getPPtr() const;
        StateBlockPtr getOPtr() const;
        StateBlockPtr getVPtr() const;
        void setPPtr(const StateBlockPtr _p_ptr);
        void setOPtr(const StateBlockPtr _o_ptr);
        void setVPtr(const StateBlockPtr _v_ptr);

    protected:
        StateBlockPtr getStateBlockPtr(unsigned int _i) const
        {
            assert (_i < state_block_vec_.size() && "Requested a state block pointer out of the vector range!");
            return state_block_vec_[_i];
        }
        void setStateBlockPtr(unsigned int _i, const StateBlockPtr _sb_ptr)
        {
            state_block_vec_[_i] = _sb_ptr;
        }

        // states
    public:
        void setState(const Eigen::VectorXs& _st);
        virtual Eigen::VectorXs getState() const;
        void getState(Eigen::VectorXs& state) const;


        // Wolf tree access ---------------------------------------------------
    public:
        ProblemPtr getProblem();

        TrajectoryBasePtr getTrajectoryPtr() const;
        void setTrajectoryPtr(TrajectoryBasePtr _trj_ptr);

        FrameBasePtr getPreviousFrame() const;
        FrameBasePtr getNextFrame() const;

        CaptureBaseList& getCaptureList();
        CaptureBasePtr addCapture(CaptureBasePtr _capt_ptr);
        CaptureBasePtr getCaptureOf(const SensorBasePtr _sensor_ptr);
        void unlinkCapture(CaptureBasePtr _cap_ptr);

        void getConstraintList(ConstraintBaseList& _ctr_list);
        virtual void addConstrainedBy(ConstraintBasePtr _ctr_ptr);
        unsigned int getHits() const;
        ConstraintBaseList& getConstrainedByList();


        /** \brief Adds all stateBlocks of the frame to the wolfProblem list of new stateBlocks
         **/
        void registerNewStateBlocks();

    private:
        StateStatus getStatus() const;
        void setStatus(StateStatus _st);
        void removeStateBlocks();
};

} // namespace wolf

// IMPLEMENTATION //

#include "capture_base.h"
#include "trajectory_base.h"
#include "state_block.h"

namespace wolf {

inline ProblemPtr FrameBase::getProblem()
{
    ProblemPtr prb = problem_ptr_.lock();
    if (!prb)
    {
        TrajectoryBasePtr trj = trajectory_ptr_.lock();
        if (trj)
        {
            prb = trj->getProblem();
            problem_ptr_ = prb;
        }
    }

    return prb;
}


inline unsigned int FrameBase::id()
{
    return frame_id_;
}

inline bool FrameBase::isKey() const
{
    return (type_id_ == KEY_FRAME);
}

inline void FrameBase::fix()
{
    this->setStatus(ST_FIXED);
}

inline void FrameBase::unfix()
{
    //std::cout << "Unfixing frame " << nodeId() << std::endl;
    this->setStatus(ST_ESTIMATED);
}

inline bool FrameBase::isFixed() const
{
    return status_ == ST_FIXED;
}

inline void FrameBase::setTimeStamp(const TimeStamp& _ts)
{
    time_stamp_ = _ts;
}

inline void FrameBase::getTimeStamp(TimeStamp& _ts) const
{
    _ts = time_stamp_.get();
}

inline TimeStamp FrameBase::getTimeStamp() const
{
    return time_stamp_.get();
}

inline StateBlockPtr FrameBase::getPPtr() const
{
    return state_block_vec_[0];
}
inline void FrameBase::setPPtr(const StateBlockPtr _p_ptr)
{
    state_block_vec_[0] = _p_ptr;
}

inline StateBlockPtr FrameBase::getOPtr() const
{
    return state_block_vec_[1];
}
inline void FrameBase::setOPtr(const StateBlockPtr _o_ptr)
{
    state_block_vec_[1] = _o_ptr;
}

inline StateBlockPtr FrameBase::getVPtr() const
{
    return state_block_vec_[2];
}
inline void FrameBase::setVPtr(const StateBlockPtr _v_ptr)
{
    state_block_vec_[2] = _v_ptr;
}

inline TrajectoryBasePtr FrameBase::getTrajectoryPtr() const
{
    return trajectory_ptr_.lock();
}

inline CaptureBaseList& FrameBase::getCaptureList()
{
    return capture_list_;
}

inline CaptureBasePtr FrameBase::addCapture(CaptureBasePtr _capt_ptr)
{
    capture_list_.push_back(_capt_ptr);
    _capt_ptr->setFramePtr(shared_from_this());
    _capt_ptr->setProblem(getProblem());
    return _capt_ptr;
}

inline StateStatus FrameBase::getStatus() const
{
    return status_;
}


inline CaptureBasePtr FrameBase::getCaptureOf(const SensorBasePtr _sensor_ptr)
{
    for (CaptureBasePtr capture_ptr : getCaptureList())
        if (capture_ptr->getSensorPtr() == _sensor_ptr)
            return capture_ptr;
    return nullptr;
}

inline void FrameBase::unlinkCapture(CaptureBasePtr _cap_ptr)
{
    _cap_ptr->unlinkFromFrame();
    capture_list_.remove(_cap_ptr);
}

inline void FrameBase::getConstraintList(ConstraintBaseList& _ctr_list)
{
    for (auto c_ptr : getCaptureList())
        c_ptr->getConstraintList(_ctr_list);
}


inline void FrameBase::addConstrainedBy(ConstraintBasePtr _ctr_ptr)
{
    constrained_by_list_.push_back(_ctr_ptr);
}

inline unsigned int FrameBase::getHits() const
{
    return constrained_by_list_.size();
}

inline ConstraintBaseList& FrameBase::getConstrainedByList()
{
    return constrained_by_list_;
}

inline void FrameBase::setTrajectoryPtr(TrajectoryBasePtr _trj_ptr)
{
    trajectory_ptr_ = _trj_ptr;
}

} // namespace wolf

#endif
