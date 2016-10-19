#include "capture_base.h"
#include "sensor_base.h"
#include "node_base.h"

namespace wolf{

unsigned int CaptureBase::capture_id_count_ = 0;

CaptureBase::CaptureBase(const std::string& _type, const TimeStamp& _ts, SensorBasePtr _sensor_ptr) :
        NodeBase("CAPTURE", _type),
        frame_ptr_(), // nullptr
        capture_id_(++capture_id_count_),
        time_stamp_(_ts),
        sensor_ptr_(_sensor_ptr),
        sensor_p_ptr_(sensor_ptr_.lock()->getPPtr()),
        sensor_o_ptr_(sensor_ptr_.lock()->getOPtr())
{
    //
    std::cout << "constructed     C" << id() << std::endl;
}


CaptureBase::~CaptureBase()
{
//    remove();
    std::cout << "destructed      C" << id() << std::endl;
}

//void CaptureBase::destruct()
//{
//    if (!is_removing_)
//    {
//        if (frame_ptr_ != nullptr)
//            frame_ptr_->removeCapture(this);
//        else
//            delete this;
//    }
//}

void CaptureBase::process()
{
    // Call all processors assigned to the sensor that captured this data
    auto cap = shared_from_this();
    auto sen = sensor_ptr_.lock();
    if (sen)
        for (auto prc : sen->getProcessorList())
            prc->process(cap);
}

void CaptureBase::remove()
{
    std::cout << "Remove          C" << id() << std::endl;
//    std::cout << "C" <<  this_C->id() << " count: " << this_C.use_count() << std::endl;
    if (!is_removing_)
    {
        std::cout << "Removing        C" << id() << std::endl;
        is_removing_ = true;
        CaptureBasePtr this_C = shared_from_this();  // keep this alive while removing it
        FrameBasePtr frm = frame_ptr_.lock();
        if (frm)
        {
            frm->getCaptureList().remove(this_C); // remove from upstream
            if (frm->getCaptureList().empty() && frm->getConstrainedByList().empty())
                frm->remove(); // remove upstream
        }
        while (!feature_list_.empty())
        {
            feature_list_.front()->remove(); // remove downstream
            feature_list_.pop_front();
        }
        std::cout << "Removed         C" << id() << std::endl;
    }
}

void CaptureBase::addFeatureList(FeatureBaseList& _new_ft_list)
{
    for (FeatureBasePtr feature_ptr : _new_ft_list)
    {
        feature_ptr->setCapturePtr(shared_from_this());
        if (getProblem() != nullptr)
            feature_ptr->setProblem(getProblem());
    }
    feature_list_.splice(feature_list_.end(), _new_ft_list);
}


} // namespace wolf

