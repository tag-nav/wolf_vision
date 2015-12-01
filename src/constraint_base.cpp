#include "constraint_base.h"


ConstraintBase::ConstraintBase(FeatureBase* _ftr_ptr, ConstraintType _tp, ConstraintStatus _status) :
    NodeLinked(BOTTOM, "CONSTRAINT"),
    type_(_tp),
    category_(CTR_ABSOLUTE),
    status_(_status),
    measurement_(_ftr_ptr->getMeasurement()),
    measurement_covariance_(_ftr_ptr->getMeasurementCovariance())
{
    //std::cout << "creating ConstraintBase " << std::endl;
}


ConstraintBase::ConstraintBase(FeatureBase* _ftr_ptr, ConstraintType _tp, FrameBase* _frame_ptr, ConstraintStatus _status) :
    NodeLinked(BOTTOM, "CONSTRAINT"),
    type_(_tp),
    category_(CTR_FRAME),
    status_(_status),
    measurement_(_ftr_ptr->getMeasurement()),
    measurement_covariance_(_ftr_ptr->getMeasurementCovariance()),
    frame_ptr_(_frame_ptr),
    feature_ptr_(nullptr),
    landmark_ptr_(nullptr)
{
    // add constraint to frame
    frame_ptr_->addConstraintTo(this);
}


ConstraintBase::ConstraintBase(FeatureBase* _ftr_ptr, ConstraintType _tp, FeatureBase* _feature_ptr, ConstraintStatus _status) :
    NodeLinked(BOTTOM, "CONSTRAINT"),
    type_(_tp),
    category_(CTR_FEATURE),
    status_(_status),
    measurement_(_ftr_ptr->getMeasurement()),
    measurement_covariance_(_ftr_ptr->getMeasurementCovariance()),
    frame_ptr_(nullptr),
    feature_ptr_(_feature_ptr),
    landmark_ptr_(nullptr)
{
    // add constraint to feature
    feature_ptr_->addConstraintTo(this);
}


ConstraintBase::ConstraintBase(FeatureBase* _ftr_ptr, ConstraintType _tp, LandmarkBase* _landmark_ptr, ConstraintStatus _status) :
    NodeLinked(BOTTOM, "CONSTRAINT"),
    type_(_tp),
    category_(CTR_LANDMARK),
    status_(_status),
    measurement_(_ftr_ptr->getMeasurement()),
    measurement_covariance_(_ftr_ptr->getMeasurementCovariance()),
    frame_ptr_(nullptr),
    feature_ptr_(nullptr),
    landmark_ptr_(_landmark_ptr)
{
    // add constraint to landmark
    landmark_ptr_->addConstraintTo(this);
}

ConstraintBase::~ConstraintBase()
{
	//std::cout << "deleting ConstraintBase " << nodeId() << std::endl;
    is_deleting_ = true;

    // add constraint to be removed from solver
    if (getTop() != nullptr)
        getTop()->removeConstraintPtr(this);

    // remove constraint to frame/landmark/feature
    if (frame_ptr_ != nullptr)
        frame_ptr_->removeConstraintTo(this);
    if (feature_ptr_ != nullptr)
        feature_ptr_->removeConstraintTo(this);
    if (landmark_ptr_ != nullptr)
        landmark_ptr_->removeConstraintTo(this);
}

void ConstraintBase::destruct()
{
    if (!is_deleting_)
        up_node_ptr_->removeDownNode(this);
}

ConstraintType ConstraintBase::getType() const
{
    return type_;
}

const Eigen::VectorXs& ConstraintBase::getMeasurement()
{
	return measurement_;
}

const Eigen::MatrixXs& ConstraintBase::getMeasurementCovariance()
{
    return measurement_covariance_;
}

FeatureBase* ConstraintBase::getFeaturePtr() const
{
	return upperNodePtr();
}

CaptureBase* ConstraintBase::getCapturePtr() const
{
	return upperNodePtr()->upperNodePtr();
}

ConstraintCategory ConstraintBase::getCategory() const
{
    return category_;
}

ConstraintStatus ConstraintBase::getStatus() const
{
	return status_;
}

void ConstraintBase::setStatus(ConstraintStatus _status)
{
	if (getTop() != nullptr && _status == CTR_INACTIVE && status_ == CTR_ACTIVE)
	    getTop()->addConstraintPtr(this);
	else if (getTop() != nullptr && _status == CTR_ACTIVE && status_ == CTR_INACTIVE)
        getTop()->removeConstraintPtr(this);

    status_ = _status;
}

FrameBase* ConstraintBase::getFrameToPtr()
{
    return frame_ptr_;
}

FeatureBase* ConstraintBase::getFeatureToPtr()
{
    return feature_ptr_;
}

LandmarkBase* ConstraintBase::getLandmarkToPtr()
{
    return landmark_ptr_;
}
