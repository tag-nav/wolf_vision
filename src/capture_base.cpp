#include "capture_base.h"
#include "sensor_base.h"
#include "node_base.h"

namespace wolf{

unsigned int CaptureBase::capture_id_count_ = 0;

CaptureBase::CaptureBase(const std::string& _type, const TimeStamp& _ts, SensorBasePtr _sensor_ptr) :
        NodeBase("CAPTURE", _type),
        problem_ptr_(nullptr),
        frame_ptr_(nullptr),
        capture_id_(++capture_id_count_),
        time_stamp_(_ts),
        sensor_ptr_(_sensor_ptr),
        sensor_p_ptr_(sensor_ptr_->getPPtr()),
        sensor_o_ptr_(sensor_ptr_->getOPtr())
{
    //
}


CaptureBase::~CaptureBase()
{
    //std::cout << "deleting CaptureBase " << nodeId() << std::endl;
    is_removing_ = true;
    while (!feature_list_.empty())
    {
        delete feature_list_.front();
        feature_list_.pop_front();
    }
}

void CaptureBase::destruct()
{
    if (!is_removing_)
    {
        if (frame_ptr_ != nullptr)
            frame_ptr_->removeCapture(this);
        else
            delete this;
    }
}

void CaptureBase::process()
{
    // Call all processors assigned to the sensor that captured this data
    for (auto processor_iter = sensor_ptr_->getProcessorListPtr()->begin(); processor_iter != sensor_ptr_->getProcessorListPtr()->end(); ++processor_iter)
    {
        (*processor_iter)->process(this);
    }
}

void CaptureBase::addFeatureList(FeatureBaseList& _new_ft_list)
{
    for (FeatureBasePtr feature_ptr : _new_ft_list)
    {
        feature_ptr->setCapturePtr(this);
        if (getProblem() != nullptr)
            feature_ptr->setProblem(getProblem());
    }
    feature_list_.splice(feature_list_.end(), _new_ft_list);
}


} // namespace wolf

