#include "constraint_base.h"
#include "frame_base.h"
#include "node_terminus.h"
#include "landmark_base.h"


ConstraintBase::ConstraintBase(ConstraintType _tp, ConstraintStatus _status) :
    NodeLinked(BOTTOM, "CONSTRAINT"),
    type_(_tp),
    category_(CTR_ABSOLUTE),
    status_(_status),
    frame_ptr_(nullptr),
    feature_ptr_(nullptr),
    landmark_ptr_(nullptr)
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
    frame_ptr_->addConstrainedBy(this);
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
    feature_ptr_->addConstrainedBy(this);
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
    landmark_ptr_->addConstrainedBy(this);
}

ConstraintBase::~ConstraintBase()
{
	//std::cout << "deleting ConstraintBase " << nodeId() << std::endl;
    is_deleting_ = true;

    // add constraint to be removed from solver
    if (getTop() != nullptr)
        getTop()->removeConstraintPtr(this);

    //std::cout << "removeConstraintPtr " << std::endl;

    // remove constraint to frame/landmark/feature
    if (frame_ptr_ != nullptr)
        frame_ptr_->removeConstrainedBy(this);
    if (feature_ptr_ != nullptr)
        feature_ptr_->removeConstrainedBy(this);
    if (landmark_ptr_ != nullptr)
        landmark_ptr_->removeConstrainedBy(this);

    //std::cout << "removed constraints to " << std::endl;
}

const Eigen::VectorXs& ConstraintBase::getMeasurement() const
{
    return getFeaturePtr()->getMeasurement();
}

const Eigen::MatrixXs& ConstraintBase::getMeasurementCovariance() const
{
    return getFeaturePtr()->getMeasurementCovariance();
}

const Eigen::MatrixXs& ConstraintBase::getMeasurementSquareRootInformation() const
{
    return getFeaturePtr()->getMeasurementSquareRootInformation();
}

CaptureBase* ConstraintBase::getCapturePtr() const
{
	return upperNodePtr()->upperNodePtr();
}

void ConstraintBase::setStatus(ConstraintStatus _status)
{
    if (getTop() == nullptr)
        std::cout << "constraint not linked with 'top', only status changed" << std::endl;
    else if (_status != status_)
    {
        if (_status == CTR_ACTIVE)
            getTop()->addConstraintPtr(this);
        else if (_status == CTR_INACTIVE)
            getTop()->removeConstraintPtr(this);
    }
    status_ = _status;
}
