#include "capture_base.h"

CaptureBase::CaptureBase(const TimeStamp& _ts, SensorBase* _sensor_ptr) :
    NodeLinked(MID, "CAPTURE"),
    time_stamp_(_ts),
    sensor_ptr_(_sensor_ptr)
{
    //
}

CaptureBase::CaptureBase(const TimeStamp& _ts, SensorBase* _sensor_ptr, const Eigen::VectorXs& _data) :
	NodeLinked(MID, "CAPTURE"),
	time_stamp_(_ts),
	sensor_ptr_(_sensor_ptr),
	data_(_data)
{
	//
}

CaptureBase::CaptureBase(const TimeStamp& _ts, SensorBase* _sensor_ptr, const Eigen::VectorXs& _data, const Eigen::MatrixXs& _data_covariance) :
	NodeLinked(MID, "CAPTURE"),
	time_stamp_(_ts),
	sensor_ptr_(_sensor_ptr),
	data_(_data),
	data_covariance_(_data_covariance)
{
    //std::cout << "created CaptureBase " << nodeId() << std::endl << "covariance: " << std::endl << data_covariance_ << std::endl;
}

CaptureBase::~CaptureBase()
{
	//std::cout << "deleting CaptureBase " << nodeId() << std::endl;
}

void CaptureBase::linkToFrame(FrameBase* _frm_ptr)
{
    linkToUpperNode(_frm_ptr);
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
		(*f_it)->getConstraintList(_ctr_list);
}

TimeStamp CaptureBase::getTimeStamp() const
{
    return time_stamp_;
}

SensorBase* CaptureBase::getSensorPtr() const
{
    return sensor_ptr_;
}

SensorType CaptureBase::getSensorType() const
{
	return sensor_ptr_->getSensorType();
}

void CaptureBase::setTimeStamp(const TimeStamp & _ts)
{
    time_stamp_ = _ts;
}

void CaptureBase::setTimeStampToNow()
{
    time_stamp_.setToNow();
}

Eigen::VectorXs CaptureBase::getData()
{
	return data_;
}

Eigen::MatrixXs CaptureBase::getDataCovariance()
{
	return data_covariance_;
}

void CaptureBase::setData(unsigned int _size, const WolfScalar *_data)
{
    data_.resize(_size);
    for (unsigned int ii=0; ii<_size; ii++) data_(ii) = *(&_data[ii]);
}

void CaptureBase::setData(const Eigen::VectorXs& _data)
{
    data_=_data;
}

void CaptureBase::setDataCovariance(const Eigen::MatrixXs& _data_cov)
{
    data_covariance_ = _data_cov;
}

void CaptureBase::processCapture()
{
    std::cout << "CaptureBase::processCapture()... processing capture" << std::endl;
}

void CaptureBase::printSelf(unsigned int _ntabs, std::ostream & _ost) const
{
    NodeLinked::printSelf(_ntabs, _ost);
    //printTabs(_ntabs);
    //_ost << "\tSensor pose : ( " << sensor_ptr_->pose().x().transpose() << " )" << std::endl;
    //printNTabs(_ntabs);
    //_ost << "\tSensor intrinsic : ( " << sensor_ptr_->intrinsic().transpose() << " )" << std::endl;
}



