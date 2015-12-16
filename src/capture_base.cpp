#include "capture_base.h"

CaptureBase::CaptureBase(const TimeStamp& _ts, SensorBase* _sensor_ptr) :
        NodeLinked(MID, "CAPTURE"),
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

// TODO: Why the linker throws an error when this function is inline...
void CaptureBase::addFeature(FeatureBase* _ft_ptr)
{
	//std::cout << "Adding feature" << std::endl;
	addDownNode(_ft_ptr);
}

FrameBase* CaptureBase::getFramePtr() const
{
    return upperNodePtr();
}

FeatureBaseList* CaptureBase::getFeatureListPtr()
{
    return getDownNodeListPtr();
}

void CaptureBase::getConstraintList(ConstraintBaseList & _ctr_list)
{
	for(auto f_it = getFeatureListPtr()->begin(); f_it != getFeatureListPtr()->end(); ++f_it)
		(*f_it)->getConstraintFromList(_ctr_list);
}

TimeStamp CaptureBase::getTimeStamp() const
{
    return time_stamp_;
}

SensorBase* CaptureBase::getSensorPtr() const
{
    return sensor_ptr_;
}


void CaptureBase::setTimeStamp(const TimeStamp & _ts)
{
    time_stamp_ = _ts;
}

void CaptureBase::setTimeStampToNow()
{
    time_stamp_.setToNow();
}

void CaptureBase::processCapture()
{
    // Call all processors assigned to the sensor that captured this data
	//TODO jsola: derive SensorBase::getProcessorList
    for (auto processor_iter = sensor_ptr_->getDownNodeListPtr()->begin(); processor_iter != sensor_ptr_->getDownNodeListPtr()->end(); ++processor_iter)
    {
        (*processor_iter)->extractFeatures(this);
        (*processor_iter)->establishConstraints(this);
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

void CaptureBase::printSelf(unsigned int _ntabs, std::ostream & _ost) const
{
    NodeLinked::printSelf(_ntabs, _ost);
    //printTabs(_ntabs);
    //_ost << "\tSensor pose : ( " << sensor_ptr_->pose().x().transpose() << " )" << std::endl;
    //printNTabs(_ntabs);
    //_ost << "\tSensor intrinsic : ( " << sensor_ptr_->intrinsic().transpose() << " )" << std::endl;
}



