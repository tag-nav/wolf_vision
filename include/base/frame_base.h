
#ifndef FRAME_BASE_H_
#define FRAME_BASE_H_

// Fwd refs
namespace wolf{
class TrajectoryBase;
class CaptureBase;
class StateBlock;
}

//Wolf includes
#include "base/wolf.h"
#include "base/time_stamp.h"
#include "base/node_base.h"

//std includes

namespace wolf {

/** \brief Enumeration of frame types
 */
typedef enum
{
    KEY_ESTIMATED = 2, ///< estimated key frame. It plays at optimizations.
    ESTIMATED = 1,     ///< estimated frame. It plays at optimizations.
    NON_ESTIMATED = 0  ///< Regular frame. It does not play at optimization.
} FrameType;

//class FrameBase
class FrameBase : public NodeBase, public std::enable_shared_from_this<FrameBase>
{
    private:
        TrajectoryBaseWPtr trajectory_ptr_;
        CaptureBasePtrList capture_list_;
        FactorBasePtrList constrained_by_list_;
        std::vector<StateBlockPtr> state_block_vec_; ///< vector of state blocks, in the order: Position, Orientation, Velocity.

        static unsigned int frame_id_count_;

    protected:
        unsigned int frame_id_; ///< frame id
        FrameType type_;        ///< type of frame. Either KEY_ESTIMATED, KEY_ESTIMATED or NON_ESTIMATED. (types defined above)
        TimeStamp time_stamp_;  ///< frame time stamp
        
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

        // Estimated / Non estimated
        bool isEstimated() const;
        void setEstimated();

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
        StateBlockPtr getStateBlock(unsigned int _i) const;
        void setStateBlock(unsigned int _i, const StateBlockPtr _sb_ptr);
        void resizeStateBlockVec(unsigned int _size);

    public:
        StateBlockPtr getP() const;
        StateBlockPtr getO() const;
        StateBlockPtr getV() const;
        void setP(const StateBlockPtr _p_ptr);
        void setO(const StateBlockPtr _o_ptr);
        void setV(const StateBlockPtr _v_ptr);
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

        // Wolf tree access ---------------------------------------------------
    public:
        virtual void setProblem(ProblemPtr _problem) final;

        TrajectoryBasePtr getTrajectory() const;
        void setTrajectory(TrajectoryBasePtr _trj_ptr);

        FrameBasePtr getPreviousFrame() const;
        FrameBasePtr getNextFrame() const;

        CaptureBasePtrList& getCaptureList();
        CaptureBasePtr addCapture(CaptureBasePtr _capt_ptr);
        CaptureBasePtr getCaptureOf(const SensorBasePtr _sensor_ptr);
        CaptureBasePtr getCaptureOf(const SensorBasePtr _sensor_ptr, const std::string& type);
        CaptureBasePtrList getCapturesOf(const SensorBasePtr _sensor_ptr);
        void unlinkCapture(CaptureBasePtr _cap_ptr);

        FactorBasePtr getFactorOf(const ProcessorBasePtr _processor_ptr);
        FactorBasePtr getFactorOf(const ProcessorBasePtr _processor_ptr, const std::string& type);

        void getFactorList(FactorBasePtrList& _fac_list);
        virtual FactorBasePtr addConstrainedBy(FactorBasePtr _fac_ptr);
        unsigned int getHits() const;
        FactorBasePtrList& getConstrainedByList();

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

#include "base/trajectory_base.h"
#include "base/capture/capture_base.h"
#include "base/factor/factor_base.h"
#include "base/state_block.h"

namespace wolf {

inline unsigned int FrameBase::id()
{
    return frame_id_;
}

inline bool FrameBase::isEstimated() const
{
    return (type_ == ESTIMATED || type_ == KEY_ESTIMATED);
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

inline StateBlockPtr FrameBase::getP() const
{
    return state_block_vec_[0];
}
inline void FrameBase::setP(const StateBlockPtr _p_ptr)
{
    state_block_vec_[0] = _p_ptr;
}

inline StateBlockPtr FrameBase::getO() const
{
    return state_block_vec_[1];
}
inline void FrameBase::setO(const StateBlockPtr _o_ptr)
{
    state_block_vec_[1] = _o_ptr;
}

inline StateBlockPtr FrameBase::getV() const
{
    return state_block_vec_[2];
}

inline void FrameBase::setV(const StateBlockPtr _v_ptr)
{
    state_block_vec_[2] = _v_ptr;
}

inline StateBlockPtr FrameBase::getStateBlock(unsigned int _i) const
{
    assert (_i < state_block_vec_.size() && "Requested a state block pointer out of the vector range!");
    return state_block_vec_[_i];
}

inline void FrameBase::setStateBlock(unsigned int _i, const StateBlockPtr _sb_ptr)
{
    assert (_i < state_block_vec_.size() && "Requested a state block pointer out of the vector range!");
    state_block_vec_[_i] = _sb_ptr;
}

inline TrajectoryBasePtr FrameBase::getTrajectory() const
{
    return trajectory_ptr_.lock();
}

inline CaptureBasePtrList& FrameBase::getCaptureList()
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

inline FactorBasePtrList& FrameBase::getConstrainedByList()
{
    return constrained_by_list_;
}

inline void FrameBase::setTrajectory(TrajectoryBasePtr _trj_ptr)
{
    trajectory_ptr_ = _trj_ptr;
}

} // namespace wolf

#endif
