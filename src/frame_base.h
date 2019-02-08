
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

/** \brief Enumeration of frame types: key-frame or non-key-frame
 */
typedef enum
{
    NON_KEY_FRAME = 0,  ///< Regular frame. It does not play at optimization.
    KEY_FRAME = 1       ///< key frame. It plays at optimizations.
} FrameType;


//class FrameBase
class FrameBase : public NodeBase, public std::enable_shared_from_this<FrameBase>
{
    private:
        TrajectoryBaseWPtr trajectory_ptr_;
        CaptureBaseList capture_list_;
        ConstraintBaseList constrained_by_list_;
        std::vector<StateBlockPtr> state_block_vec_; ///< vector of state blocks, in the order: Position, Orientation, Velocity.

        static unsigned int frame_id_count_;

    protected:
        unsigned int frame_id_;
        FrameType type_;     ///< type of frame. Either NON_KEY_FRAME or KEY_FRAME. (types defined at wolf.h)
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
        FrameBase(const FrameType & _tp, const TimeStamp& _ts, StateBlockPtr _p_ptr, StateBlockPtr _o_ptr = nullptr, StateBlockPtr _v_ptr = nullptr);

        virtual ~FrameBase();
        virtual void remove();




        // Frame properties -----------------------------------------------
    public:
        unsigned int id();

        // KeyFrame / NonKeyFrame
        bool isKey() const;
        void setKey();

        // Frame values ------------------------------------------------
    public:
        void        setTimeStamp(const TimeStamp& _ts);
        TimeStamp   getTimeStamp() const;
        void        getTimeStamp(TimeStamp& _ts) const;

        // State blocks
    public:
        const std::vector<StateBlockPtr>& getStateBlockVec() const;
        std::vector<StateBlockPtr>& getStateBlockVec();
    protected:
        StateBlockPtr getStateBlockPtr(unsigned int _i) const;
        void setStateBlockPtr(unsigned int _i, const StateBlockPtr _sb_ptr);
        void resizeStateBlockVec(unsigned int _size);

    public:
        StateBlockPtr getPPtr() const;
        StateBlockPtr getOPtr() const;
        StateBlockPtr getVPtr() const;
        void setPPtr(const StateBlockPtr _p_ptr);
        void setOPtr(const StateBlockPtr _o_ptr);
        void setVPtr(const StateBlockPtr _v_ptr);
        void registerNewStateBlocks();
        void removeStateBlocks();

        // Fixed / Estimated
    public:
        void fix();
        void unfix();
        bool isFixed() const;

        // States
    public:
        void setState(const Eigen::VectorXs& _state);
        Eigen::VectorXs getState() const;
        void getState(Eigen::VectorXs& _state) const;
        unsigned int getSize() const;
        bool getCovariance(Eigen::MatrixXs& _cov) const;
        Eigen::MatrixXs getCovariance() const;

        // Wolf tree access ---------------------------------------------------
    public:
        virtual void setProblem(ProblemPtr _problem) final;

        TrajectoryBasePtr getTrajectoryPtr() const;
        void setTrajectoryPtr(TrajectoryBasePtr _trj_ptr);

        FrameBasePtr getPreviousFrame() const;
        FrameBasePtr getNextFrame() const;

        CaptureBaseList& getCaptureList();
        CaptureBasePtr addCapture(CaptureBasePtr _capt_ptr);
        CaptureBasePtr getCaptureOf(const SensorBasePtr _sensor_ptr);
        CaptureBasePtr getCaptureOf(const SensorBasePtr _sensor_ptr, const std::string& type);
        CaptureBaseList getCapturesOf(const SensorBasePtr _sensor_ptr);
        void unlinkCapture(CaptureBasePtr _cap_ptr);

        ConstraintBasePtr getConstraintOf(const ProcessorBasePtr _processor_ptr);
        ConstraintBasePtr getConstraintOf(const ProcessorBasePtr _processor_ptr, const std::string& type);

        void getConstraintList(ConstraintBaseList& _ctr_list);
        virtual ConstraintBasePtr addConstrainedBy(ConstraintBasePtr _ctr_ptr);
        unsigned int getHits() const;
        ConstraintBaseList& getConstrainedByList();

    public:
        static FrameBasePtr create_PO_2D (const FrameType & _tp,
                                          const TimeStamp& _ts,
                                          const Eigen::VectorXs& _x = Eigen::VectorXs::Zero(3));
        static FrameBasePtr create_PO_3D (const FrameType & _tp,
                                          const TimeStamp& _ts,
                                          const Eigen::VectorXs& _x = Eigen::VectorXs::Zero(7));
        static FrameBasePtr create_POV_3D(const FrameType & _tp,
                                          const TimeStamp& _ts,
                                          const Eigen::VectorXs& _x = Eigen::VectorXs::Zero(10));
};

} // namespace wolf

// IMPLEMENTATION //

#include "trajectory_base.h"
#include "capture_base.h"
#include "constraint_base.h"
#include "state_block.h"

namespace wolf {


inline unsigned int FrameBase::id()
{
    return frame_id_;
}

inline bool FrameBase::isKey() const
{
    return (type_ == KEY_FRAME);
}

inline void FrameBase::getTimeStamp(TimeStamp& _ts) const
{
    _ts = time_stamp_;
}

inline TimeStamp FrameBase::getTimeStamp() const
{
    return time_stamp_;
}

inline const std::vector<StateBlockPtr>& FrameBase::getStateBlockVec() const
{
    return state_block_vec_;
}

inline std::vector<StateBlockPtr>& FrameBase::getStateBlockVec()
{
    return state_block_vec_;
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

inline StateBlockPtr FrameBase::getStateBlockPtr(unsigned int _i) const
{
    assert (_i < state_block_vec_.size() && "Requested a state block pointer out of the vector range!");
    return state_block_vec_[_i];
}

inline void FrameBase::setStateBlockPtr(unsigned int _i, const StateBlockPtr _sb_ptr)
{
    assert (_i < state_block_vec_.size() && "Requested a state block pointer out of the vector range!");
    state_block_vec_[_i] = _sb_ptr;
}

inline TrajectoryBasePtr FrameBase::getTrajectoryPtr() const
{
    return trajectory_ptr_.lock();
}

inline CaptureBaseList& FrameBase::getCaptureList()
{
    return capture_list_;
}

inline void FrameBase::resizeStateBlockVec(unsigned int _size)
{
    if (_size > state_block_vec_.size())
        state_block_vec_.resize(_size);
}

inline unsigned int FrameBase::getHits() const
{
    return constrained_by_list_.size();
}

inline void FrameBase::setProblem(ProblemPtr _problem)
{
    NodeBase::setProblem(_problem);
    for (auto cap : capture_list_)
        cap->setProblem(_problem);
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
