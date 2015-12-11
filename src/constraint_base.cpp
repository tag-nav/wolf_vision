#include "constraint_base.h"


ConstraintBase::ConstraintBase(ConstraintType _tp, ConstraintStatus _status) :
    NodeLinked(BOTTOM, "CONSTRAINT"),
    type_(_tp),
    category_(CTR_ABSOLUTE),
    status_(_status)
{
    //std::cout << "creating ConstraintBase " << std::endl;
}


ConstraintBase::ConstraintBase(ConstraintType _tp, FrameBase* _frame_ptr, ConstraintStatus _status) :
    NodeLinked(BOTTOM, "CONSTRAINT"),
    type_(_tp),
    category_(CTR_FRAME),
    status_(_status),
    frame_ptr_(_frame_ptr),
    feature_ptr_(nullptr),
    landmark_ptr_(nullptr)
{
    // add constraint to frame
    frame_ptr_->addConstraintTo(this);
}


ConstraintBase::ConstraintBase(ConstraintType _tp, FeatureBase* _feature_ptr, ConstraintStatus _status) :
    NodeLinked(BOTTOM, "CONSTRAINT"),
    type_(_tp),
    category_(CTR_FEATURE),
    status_(_status),
    frame_ptr_(nullptr),
    feature_ptr_(_feature_ptr),
    landmark_ptr_(nullptr)
{
    // add constraint to feature
    feature_ptr_->addConstraintTo(this);
}


ConstraintBase::ConstraintBase(ConstraintType _tp, LandmarkBase* _landmark_ptr, ConstraintStatus _status) :
    NodeLinked(BOTTOM, "CONSTRAINT"),
    type_(_tp),
    category_(CTR_LANDMARK),
    status_(_status),
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

ConstraintType ConstraintBase::getType() const
{
    return type_;
}

const Eigen::VectorXs& ConstraintBase::getMeasurement() const
{
	return getFeaturePtr()->getMeasurement();
}

const Eigen::MatrixXs& ConstraintBase::getMeasurementCovariance() const
{
    return getFeaturePtr()->getMeasurementCovariance();
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

FrameBase* ConstraintBase::getFrameOtherPtr()
{
    return frame_ptr_;
}

FeatureBase* ConstraintBase::getFeatureOtherPtr()
{
    return feature_ptr_;
}

LandmarkBase* ConstraintBase::getLandmarkOtherPtr()
{
    return landmark_ptr_;
}
