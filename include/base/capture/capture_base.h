#ifndef CAPTURE_BASE_H_
#define CAPTURE_BASE_H_

// Forward declarations for node templates
namespace wolf{
class FrameBase;
class FeatureBase;
}

//Wolf includes
#include "base/wolf.h"
#include "base/node_base.h"
#include "base/time_stamp.h"

//std includes

namespace wolf{

//class CaptureBase
class CaptureBase : public NodeBase, public std::enable_shared_from_this<CaptureBase>
{
    private:
        FrameBaseWPtr   frame_ptr_;
        FeatureBasePtrList feature_list_;
        FactorBasePtrList constrained_by_list_;
        SensorBaseWPtr  sensor_ptr_; ///< Pointer to sensor
        // Deal with sensors with dynamic extrinsics (check dynamic_extrinsic_ in SensorBase)
        std::vector<StateBlockPtr> state_block_vec_; ///< vector of state blocks, in the order P, O, intrinsic.
        SizeEigen calib_size_;           ///< size of the calibration parameters (dynamic or static sensor params that are not fixed)

        static unsigned int capture_id_count_;

    protected:
        unsigned int capture_id_;
        TimeStamp time_stamp_;      ///< Time stamp

    public:

        CaptureBase(const std::string& _type,
                    const TimeStamp& _ts,
                    SensorBasePtr _sensor_ptr   = nullptr,
                    StateBlockPtr _p_ptr        = nullptr,
                    StateBlockPtr _o_ptr        = nullptr,
                    StateBlockPtr _intr_ptr     = nullptr);

        virtual ~CaptureBase();
        virtual void remove();

        // Type
        virtual bool isMotion() const { return false; }

        bool process();

        unsigned int id();
        TimeStamp getTimeStamp() const;
        void setTimeStamp(const TimeStamp& _ts);
        void setTimeStampToNow();

        FrameBasePtr getFrame() const;
        void setFramePtr(const FrameBasePtr _frm_ptr);
        void unlinkFromFrame(){frame_ptr_.reset();}

        virtual void setProblem(ProblemPtr _problem) final;

        FeatureBasePtr addFeature(FeatureBasePtr _ft_ptr);
        FeatureBasePtrList& getFeatureList();
        void addFeatureList(FeatureBasePtrList& _new_ft_list);

        void getFactorList(FactorBasePtrList& _fac_list);

        SensorBasePtr getSensor() const;
        virtual void setSensorPtr(const SensorBasePtr sensor_ptr);

        // constrained by
        virtual FactorBasePtr addConstrainedBy(FactorBasePtr _fac_ptr);
        unsigned int getHits() const;
        FactorBasePtrList& getConstrainedByList();

        // State blocks
        const std::vector<StateBlockPtr>& getStateBlockVec() const;
        std::vector<StateBlockPtr>& getStateBlockVec();
        StateBlockPtr getStateBlockPtr(unsigned int _i) const;
        void setStateBlockPtr(unsigned int _i, const StateBlockPtr _sb_ptr);

        StateBlockPtr getSensorP() const;
        StateBlockPtr getSensorO() const;
        StateBlockPtr getSensorIntrinsic() const;
        void removeStateBlocks();
        virtual void registerNewStateBlocks();

        void fix();
        void unfix();
        void fixExtrinsics();
        void unfixExtrinsics();
        void fixIntrinsics();
        void unfixIntrinsics();

        bool hasCalibration() {return calib_size_ > 0;}
        SizeEigen getCalibSize() const;
        virtual Eigen::VectorXs getCalibration() const;
        void setCalibration(const Eigen::VectorXs& _calib);
        void link(FrameBasePtr);
        template<typename classType, typename... T>
        static std::shared_ptr<CaptureBase> emplace(FrameBasePtr _frm_ptr, T&&... all);

    protected:
        SizeEigen computeCalibSize() const;

    private:
        void updateCalibSize();
};

}

#include "base/sensor/sensor_base.h"
#include "base/frame_base.h"
#include "base/feature/feature_base.h"
#include "base/state_block.h"

namespace wolf{

template<typename classType, typename... T>
std::shared_ptr<CaptureBase> CaptureBase::emplace(FrameBasePtr _frm_ptr, T&&... all)
{
    CaptureBasePtr cpt = std::make_shared<classType>(std::forward<T>(all)...);
    cpt->link(_frm_ptr);
    return cpt;
}

inline SizeEigen CaptureBase::getCalibSize() const
{
    return calib_size_;
}

inline void CaptureBase::setProblem(ProblemPtr _problem)
{
    NodeBase::setProblem(_problem);
    for (auto ft : feature_list_)
        ft->setProblem(_problem);
}

inline void CaptureBase::updateCalibSize()
{
    calib_size_ = computeCalibSize();
}

inline const std::vector<StateBlockPtr>& CaptureBase::getStateBlockVec() const
{
    return state_block_vec_;
}

inline std::vector<StateBlockPtr>& CaptureBase::getStateBlockVec()
{
    return state_block_vec_;
}

inline void CaptureBase::setStateBlockPtr(unsigned int _i, const StateBlockPtr _sb_ptr)
{
    state_block_vec_[_i] = _sb_ptr;
}

inline StateBlockPtr CaptureBase::getSensorP() const
{
    return getStateBlockPtr(0);
}

inline StateBlockPtr CaptureBase::getSensorO() const
{
    return getStateBlockPtr(1);
}

inline StateBlockPtr CaptureBase::getSensorIntrinsic() const
{
    return getStateBlockPtr(2);
}

inline unsigned int CaptureBase::id()
{
    return capture_id_;
}

inline FrameBasePtr CaptureBase::getFrame() const
{
    return frame_ptr_.lock();
}

inline void CaptureBase::setFramePtr(const FrameBasePtr _frm_ptr)
{
    frame_ptr_ = _frm_ptr;
}

inline FeatureBasePtrList& CaptureBase::getFeatureList()
{
    return feature_list_;
}

inline unsigned int CaptureBase::getHits() const
{
    return constrained_by_list_.size();
}

inline FactorBasePtrList& CaptureBase::getConstrainedByList()
{
    return constrained_by_list_;
}

inline TimeStamp CaptureBase::getTimeStamp() const
{
    return time_stamp_;
}

inline SensorBasePtr CaptureBase::getSensor() const
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

inline bool CaptureBase::process()
{
    assert (getSensor() != nullptr && "Attempting to process a capture with no associated sensor. Either set the capture's sensor or call sensor->process(capture) instead.");

    return getSensor()->process(shared_from_this());
}


} // namespace wolf

#endif
