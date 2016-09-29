#include "capture_base.h"
#include "sensor_base.h"
#include "node_base.h"

namespace wolf{

unsigned int CaptureBase::capture_id_count_ = 0;

CaptureBase::CaptureBase(const std::string& _type, const TimeStamp& _ts, SensorBase* _sensor_ptr) :
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

void CaptureBase::getConstraintList(ConstraintBaseList & _ctr_list)
{
	for(auto f_it = getFeatureListPtr()->begin(); f_it != getFeatureListPtr()->end(); ++f_it)
		(*f_it)->getConstraintList(_ctr_list);
}

void CaptureBase::addFeatureList(FeatureBaseList& _new_ft_list)
{
    for (auto feature_ptr : _new_ft_list)
        feature_ptr->setCapturePtr(this);
    feature_list_.splice(feature_list_.end(), _new_ft_list);
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

void CaptureBase::removeFeature(FeatureBasePtr _ft_ptr)
{
    feature_list_.remove(_ft_ptr);
    delete _ft_ptr;
}

void CaptureBase::process()
{
    // Call all processors assigned to the sensor that captured this data
    for (auto processor_iter = sensor_ptr_->getProcessorListPtr()->begin(); processor_iter != sensor_ptr_->getProcessorListPtr()->end(); ++processor_iter)
    {
        (*processor_iter)->process(this);
    }
}


StateBlock* CaptureBase::getSensorPPtr() const {
	if (getSensorPtr()->isExtrinsicDynamic())
		return sensor_p_ptr_;
	else
		return getSensorPtr()->getPPtr();
}

StateBlock* CaptureBase::getSensorOPtr() const {
	if (getSensorPtr()->isExtrinsicDynamic())
		return sensor_o_ptr_;
	else
		return getSensorPtr()->getOPtr();
}

}

