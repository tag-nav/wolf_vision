#include "constraint_base.h"
#include "frame_base.h"
#include "landmark_base.h"

namespace wolf {

unsigned int ConstraintBase::constraint_id_count_ = 0;

ConstraintBase::ConstraintBase(ConstraintType _tp, bool _apply_loss_function, ConstraintStatus _status) :
    NodeBase("CONSTRAINT", "Base"),
    problem_ptr_(nullptr),
    feature_ptr_(nullptr),
    constraint_id_(++constraint_id_count_),
    type_id_(_tp),
    category_(CTR_ABSOLUTE),
    status_(_status),
    apply_loss_function_(_apply_loss_function),
    frame_other_ptr_(nullptr),
    feature_other_ptr_(nullptr),
    landmark_other_ptr_(nullptr)
{
    //std::cout << "creating ConstraintBase " << std::endl;
}


ConstraintBase::ConstraintBase(ConstraintType _tp, FrameBasePtr _frame_ptr, bool _apply_loss_function, ConstraintStatus _status) :
    NodeBase("CONSTRAINT", "Base"),
    problem_ptr_(nullptr),
    feature_ptr_(nullptr),
    constraint_id_(++constraint_id_count_),
    type_id_(_tp),
    category_(CTR_FRAME),
    status_(_status),
    apply_loss_function_(_apply_loss_function),
    frame_other_ptr_(_frame_ptr),
    feature_other_ptr_(nullptr),
    landmark_other_ptr_(nullptr)
{
    // add constraint to frame
    frame_other_ptr_->addConstrainedBy(this);
}


ConstraintBase::ConstraintBase(ConstraintType _tp, FeatureBasePtr _feature_ptr, bool _apply_loss_function, ConstraintStatus _status) :
    NodeBase("CONSTRAINT"),
    problem_ptr_(nullptr),
    feature_ptr_(nullptr),
    constraint_id_(++constraint_id_count_),
    type_id_(_tp),
    category_(CTR_FEATURE),
    status_(_status),
    apply_loss_function_(_apply_loss_function),
    frame_other_ptr_(nullptr),
    feature_other_ptr_(_feature_ptr),
    landmark_other_ptr_(nullptr)
{
    // add constraint to feature
    feature_other_ptr_->addConstrainedBy(this);
}


ConstraintBase::ConstraintBase(ConstraintType _tp, LandmarkBasePtr _landmark_ptr, bool _apply_loss_function, ConstraintStatus _status) :
    NodeBase("CONSTRAINT"),
    problem_ptr_(nullptr),
    feature_ptr_(nullptr),
    constraint_id_(++constraint_id_count_),
    type_id_(_tp),
    category_(CTR_LANDMARK),
    status_(_status),
    apply_loss_function_(_apply_loss_function),
    frame_other_ptr_(nullptr),
    feature_other_ptr_(nullptr),
    landmark_other_ptr_(_landmark_ptr.get()) // TODO remove line
    //landmark_other_ptr_(_landmark_ptr) // TODO uncomment line
{
    // add constraint to landmark
    landmark_other_ptr_->addConstrainedBy(this);
}

ConstraintBase::~ConstraintBase()
{

    std::cout << "destructing         c" << id() << std::endl;
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

CaptureBasePtr ConstraintBase::getCapturePtr() const
{
    return getFeaturePtr()->getCapturePtr();
}

void ConstraintBase::setStatus(ConstraintStatus _status)
{
    if (getProblem() == nullptr)
        std::cout << "constraint not linked with 'top', only status changed" << std::endl;
    else if (_status != status_)
    {
        if (_status == CTR_ACTIVE)
            getProblem()->addConstraintPtr(this);
        else if (_status == CTR_INACTIVE)
            getProblem()->removeConstraintPtr(this);
    }
    status_ = _status;
}

} // namespace wolf
