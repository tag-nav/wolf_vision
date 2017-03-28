#include "capture_base.h"
#include "sensor_base.h"

namespace wolf{

unsigned int CaptureBase::capture_id_count_ = 0;

CaptureBase::CaptureBase(const std::string& _type, const TimeStamp& _ts, SensorBasePtr _sensor_ptr) :
        NodeBase("CAPTURE", _type),
        frame_ptr_(), // nullptr
        is_removing_(false),
        capture_id_(++capture_id_count_),
        time_stamp_(_ts),
        sensor_ptr_(_sensor_ptr),
        sensor_p_ptr_(!sensor_ptr_.expired() ? sensor_ptr_.lock()->getPPtr() : nullptr),
        sensor_o_ptr_(!sensor_ptr_.expired() ? sensor_ptr_.lock()->getOPtr() : nullptr)
{
    //
//    std::cout << "constructed    +C" << id() << std::endl;
}


CaptureBase::~CaptureBase()
{
//    std::cout << "destructed     -C" << id() << std::endl;
}

void CaptureBase::remove()
{
//    std::cout << "Remove          C" << id() << std::endl;
    if (!is_removing_)
    {
//        std::cout << "Removing        C" << id() << std::endl;
        is_removing_ = true;
        CaptureBasePtr this_C = shared_from_this();  // keep this alive while removing it

        // remove from upstream
        FrameBasePtr F = frame_ptr_.lock();
        if (F)
        {
            F->getCaptureList().remove(this_C);
            if (F->getCaptureList().empty() && F->getConstrainedByList().empty())
                F->remove(); // remove upstream
        }

        // remove downstream
        while (!feature_list_.empty())
        {
            feature_list_.front()->remove(); // remove downstream
        }

//        std::cout << "Removed         C" << id() << std::endl;
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

