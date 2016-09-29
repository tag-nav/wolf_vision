#include "capture_base.h"
#include "sensor_base.h"
#include "node_base.h"

namespace wolf{

unsigned int CaptureBase::capture_id_count_ = 0;

CaptureBase::CaptureBase(const std::string& _type, const TimeStamp& _ts, SensorBasePtr _sensor_ptr) :
        NodeBase("CAPTURE", _type),
//        NodeLinked(MID, "CAPTURE", _type),
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
}

void CaptureBase::destruct()
{
    // TODO fill code
    if (!is_deleting_)
    {
        if (frame_ptr_ != nullptr) // && !up_node_ptr_->isTop())
        {
            //std::cout << "upper node is not WolfProblem " << std::endl;
            frame_ptr_->removeCapture(this);
        }
        else
        {
            //std::cout << "upper node is WolfProblem or nullptr" << std::endl;
            delete this;
        }
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

} // namespace wolf

