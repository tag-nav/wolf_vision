#ifndef CAPTURE_BASE_H_
#define CAPTURE_BASE_H_

// Forward declarations for node templates
namespace wolf{
class FrameBase;
class FeatureBase;
}

//Wolf includes
#include "wolf.h"
#include "node_base.h"
#include "time_stamp.h"

//std includes

namespace wolf{

//class CaptureBase
class CaptureBase : public NodeBase, public std::enable_shared_from_this<CaptureBase>
{
    private:
        FrameBaseWPtr   frame_ptr_;
        FeatureBaseList feature_list_;
        SensorBaseWPtr  sensor_ptr_; ///< Pointer to sensor
        // Deal with sensors with dynamic extrinsics (check dynamic_extrinsic_ in SensorBase)
        std::vector<StateBlockPtr> state_block_vec_; ///< vector of state blocks, in the order P, O, intrinsic.
        Size calib_size_;           ///< size of the calibration parameters (dynamic or static sensor params that are not fixed)

        static unsigned int capture_id_count_;
        bool is_removing_;          ///< A flag for safely removing nodes from the Wolf tree. See remove().

    protected:
        unsigned int capture_id_;
        TimeStamp time_stamp_;      ///< Time stamp

    public:

        CaptureBase(const std::string& _type,
                    const TimeStamp& _ts,
                    SensorBasePtr _sensor_ptr   = nullptr,
                    StateBlockPtr _p_ptr        = nullptr,
                    StateBlockPtr _o_ptr        = nullptr,
                    StateBlockPtr _intr_ptr     = nullptr
                    );

        virtual ~CaptureBase();
        void remove();

        unsigned int id();
        TimeStamp getTimeStamp() const;
        void setTimeStamp(const TimeStamp& _ts);
        void setTimeStampToNow();

        ProblemPtr getProblem();

        FrameBasePtr getFramePtr() const;
        void setFramePtr(const FrameBasePtr _frm_ptr);
        void unlinkFromFrame(){frame_ptr_.reset();}

        FeatureBasePtr addFeature(FeatureBasePtr _ft_ptr);
        FeatureBaseList& getFeatureList();
        void addFeatureList(FeatureBaseList& _new_ft_list);

        void getConstraintList(ConstraintBaseList& _ctr_list);

        SensorBasePtr getSensorPtr() const;
        virtual void setSensorPtr(const SensorBasePtr sensor_ptr);

        // State blocks
        const std::vector<StateBlockPtr>& getStateBlockVec() const;
        std::vector<StateBlockPtr>& getStateBlockVec();
        StateBlockPtr getStateBlockPtr(unsigned int _i) const;
        void setStateBlockPtr(unsigned int _i, const StateBlockPtr _sb_ptr);

        StateBlockPtr getSensorPPtr() const;
        StateBlockPtr getSensorOPtr() const;
        StateBlockPtr getSensorIntrinsicPtr() const;
        void removeStateBlocks();
        virtual void registerNewStateBlocks();

        void fix();
        void unfix();
        void fixExtrinsics();
        void unfixExtrinsics();
        void fixIntrinsics();
        void unfixIntrinsics();

        Size getCalibSize() const;
        virtual Eigen::VectorXs getCalibration() const;
        void setCalibration(const Eigen::VectorXs& _calib);

    protected:
        Size computeCalibSize() const;

    private:
        void updateCalibSize();
};

inline wolf::Size CaptureBase::getCalibSize() const
{
    return calib_size_;
}

inline void CaptureBase::updateCalibSize()
{
    calib_size_ = computeCalibSize();
}

}

#include "sensor_base.h"
#include "frame_base.h"
#include "feature_base.h"

namespace wolf{

inline ProblemPtr CaptureBase::getProblem()
{
    ProblemPtr prb = problem_ptr_.lock();
    if (!prb)
    {
        FrameBasePtr frm = frame_ptr_.lock();
        if (frm)
        {
            prb = frm->getProblem();
            problem_ptr_ = prb;
        }
    }
    return prb;
}

inline const std::vector<StateBlockPtr>& CaptureBase::getStateBlockVec() const
{
    return state_block_vec_;
}

inline std::vector<StateBlockPtr>& CaptureBase::getStateBlockVec()
{
    return state_block_vec_;
}

inline StateBlockPtr CaptureBase::getStateBlockPtr(unsigned int _i) const
{
    if (getSensorPtr())
    {
        if (_i < 2) // _i == 0 is position, 1 is orientation, 2 and onwards are intrinsics
            if (getSensorPtr()->extrinsicsInCaptures())
            {
                assert (_i < state_block_vec_.size() && "Requested a state block pointer out of the vector range!");
                return state_block_vec_[_i];
            }
            else
                return getSensorPtr()->getStateBlockPtr(_i);

        else // 2 and onwards are intrinsics
            if (getSensorPtr()->intrinsicsInCaptures())
            {
                assert (_i < state_block_vec_.size() && "Requested a state block pointer out of the vector range!");
                return state_block_vec_[_i];
            }
            else
                return getSensorPtr()->getStateBlockPtr(_i);
    }
    else // No sensor associated: assume sensor params are here
    {
        assert (_i < state_block_vec_.size() && "Requested a state block pointer out of the vector range!");
        return state_block_vec_[_i];
    }
}

inline void CaptureBase::setStateBlockPtr(unsigned int _i, const StateBlockPtr _sb_ptr)
{
    state_block_vec_[_i] = _sb_ptr;
}


inline StateBlockPtr CaptureBase::getSensorPPtr() const
{
    return getStateBlockPtr(0);
}

inline StateBlockPtr CaptureBase::getSensorOPtr() const
{
    return getStateBlockPtr(1);
}

inline StateBlockPtr CaptureBase::getSensorIntrinsicPtr() const
{
    return getStateBlockPtr(2);
}


inline unsigned int CaptureBase::id()
{
    return capture_id_;
}

inline FrameBasePtr CaptureBase::getFramePtr() const
{
    return frame_ptr_.lock();
}

inline void CaptureBase::setFramePtr(const FrameBasePtr _frm_ptr)
{
    frame_ptr_ = _frm_ptr;
}

inline FeatureBaseList& CaptureBase::getFeatureList()
{
    return feature_list_;
}

inline void CaptureBase::getConstraintList(ConstraintBaseList& _ctr_list)
{
    for (auto f_ptr : getFeatureList())
        f_ptr->getConstraintList(_ctr_list);
}

inline TimeStamp CaptureBase::getTimeStamp() const
{
    return time_stamp_;
}

inline SensorBasePtr CaptureBase::getSensorPtr() const
{
    return sensor_ptr_.lock();
}

inline void CaptureBase::setSensorPtr(const SensorBasePtr sensor_ptr)
{
  sensor_ptr_ = sensor_ptr;
}

inline void CaptureBase::setTimeStamp(const TimeStamp& _ts)
{
    time_stamp_ = _ts;
}

inline void CaptureBase::setTimeStampToNow()
{
    time_stamp_.setToNow();
}

} // namespace wolf

#endif
