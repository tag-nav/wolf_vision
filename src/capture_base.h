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
//

namespace wolf{

//class CaptureBase
class CaptureBase : public NodeBase
{
    private:
        ProblemWPtr problem_ptr_;
        FrameBaseWPtr frame_ptr_;
        FeatureBaseList feature_list_;

        static unsigned int capture_id_count_;

    protected:
        unsigned int capture_id_;
        TimeStamp time_stamp_; ///< Time stamp
        SensorBasePtr sensor_ptr_; ///< Pointer to sensor

        // Deal with sensors with dynamic extrinsics (check dynamic_extrinsic_ in SensorBase)
        StateBlock* sensor_p_ptr_;
        StateBlock* sensor_o_ptr_;

    public:

        CaptureBase(const std::string& _type, const TimeStamp& _ts, SensorBasePtr _sensor_ptr);

        /** \brief Default destructor (not recommended)
         *
         * Default destructor (please use destruct() instead of delete for guaranteeing the wolf tree integrity)
         **/
        virtual ~CaptureBase();
        void destruct();

        unsigned int id();
        TimeStamp getTimeStamp() const;
        void setTimeStamp(const TimeStamp& _ts);
        void setTimeStampToNow();

        ProblemPtr getProblem();
        void setProblem(ProblemPtr _prob_ptr){problem_ptr_ = _prob_ptr;}

        FrameBasePtr getFramePtr() const;
        void setFramePtr(const FrameBasePtr _frm_ptr);
        void unlinkFromFrame(){frame_ptr_ = nullptr;}

        FeatureBasePtr addFeature(FeatureBasePtr _ft_ptr);
        FeatureBaseList* getFeatureListPtr();
        void addFeatureList(FeatureBaseList& _new_ft_list);
        void removeFeature(FeatureBasePtr _ft_ptr);

        void getConstraintList(ConstraintBaseList& _ctr_list);

        SensorBasePtr getSensorPtr() const;
        StateBlock* getSensorPPtr() const;
        StateBlock* getSensorOPtr() const;

        /** \brief Call all the processors for this Capture
         */
        virtual void process();

};

}

#include "sensor_base.h"
#include "frame_base.h"
#include "feature_base.h"

namespace wolf{

inline ProblemPtr CaptureBase::getProblem()
{
    if (problem_ptr_ == nullptr && frame_ptr_ != nullptr)
        setProblem(frame_ptr_->getProblem());

    return problem_ptr_;
}

inline wolf::StateBlock* CaptureBase::getSensorPPtr() const
{
    if (getSensorPtr()->isExtrinsicDynamic())
        return sensor_p_ptr_;
    else
        return getSensorPtr()->getPPtr();
}

inline wolf::StateBlock* CaptureBase::getSensorOPtr() const
{
    if (getSensorPtr()->isExtrinsicDynamic())
        return sensor_o_ptr_;
    else
        return getSensorPtr()->getOPtr();
}


inline void CaptureBase::removeFeature(FeatureBasePtr _ft_ptr)
{
    feature_list_.remove(_ft_ptr);
    delete _ft_ptr;
}

inline unsigned int CaptureBase::id()
{
    return capture_id_;
}

inline FeatureBasePtr CaptureBase::addFeature(FeatureBasePtr _ft_ptr)
{
    //std::cout << "Adding feature" << std::endl;
    feature_list_.push_back(_ft_ptr);
    _ft_ptr->setCapturePtr(this);
    _ft_ptr->setProblem(getProblem());
//    addDownNode(_ft_ptr);
    return _ft_ptr;
}

inline FrameBasePtr CaptureBase::getFramePtr() const
{
    return frame_ptr_;
//    return upperNodePtr();
}

inline void CaptureBase::setFramePtr(const FrameBasePtr _frm_ptr)
{
    frame_ptr_ = _frm_ptr;
}

inline FeatureBaseList* CaptureBase::getFeatureListPtr()
{
    return & feature_list_;
//    return getDownNodeListPtr();
}

inline void CaptureBase::getConstraintList(ConstraintBaseList& _ctr_list)
{
    for (FeatureBasePtr f_ptr : *getFeatureListPtr())
        f_ptr->getConstraintList(_ctr_list);
}

inline TimeStamp CaptureBase::getTimeStamp() const
{
    return time_stamp_;
}

inline SensorBasePtr CaptureBase::getSensorPtr() const
{
    return sensor_ptr_;
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
