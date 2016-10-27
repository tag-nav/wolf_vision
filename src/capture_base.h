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
        FrameBaseWPtr frame_ptr_;
        FeatureBaseList feature_list_;

        static unsigned int capture_id_count_;

    protected:
        unsigned int capture_id_;
        TimeStamp time_stamp_; ///< Time stamp
        SensorBaseWPtr sensor_ptr_; ///< Pointer to sensor

        // Deal with sensors with dynamic extrinsics (check dynamic_extrinsic_ in SensorBase)
        StateBlockPtr sensor_p_ptr_;
        StateBlockPtr sensor_o_ptr_;

    public:

        CaptureBase(const std::string& _type, const TimeStamp& _ts, SensorBasePtr _sensor_ptr = nullptr);

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
        StateBlockPtr getSensorPPtr() const;
        StateBlockPtr getSensorOPtr() const;

        virtual void setSensorPtr(const SensorBasePtr sensor_ptr);

};

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

inline wolf::StateBlockPtr CaptureBase::getSensorPPtr() const
{
    if (getSensorPtr()->isExtrinsicDynamic())
        return sensor_p_ptr_;
    else
        return getSensorPtr()->getPPtr();
}

inline wolf::StateBlockPtr CaptureBase::getSensorOPtr() const
{
    if (getSensorPtr()->isExtrinsicDynamic())
        return sensor_o_ptr_;
    else
        return getSensorPtr()->getOPtr();
}


inline unsigned int CaptureBase::id()
{
    return capture_id_;
}

inline FeatureBasePtr CaptureBase::addFeature(FeatureBasePtr _ft_ptr)
{
    //std::cout << "Adding feature" << std::endl;
    feature_list_.push_back(_ft_ptr);
    _ft_ptr->setCapturePtr(shared_from_this());
    _ft_ptr->setProblem(getProblem());
    return _ft_ptr;
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
