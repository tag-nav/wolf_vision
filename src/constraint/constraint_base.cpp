#include "base/constraint/constraint_base.h"
#include "base/frame_base.h"
#include "base/landmark/landmark_base.h"

namespace wolf {

unsigned int ConstraintBase::constraint_id_count_ = 0;

ConstraintBase::ConstraintBase(const std::string&  _tp,
                               bool _apply_loss_function,
                               ConstraintStatus _status) :
    NodeBase("CONSTRAINT", _tp),
    feature_ptr_(), // nullptr
    constraint_id_(++constraint_id_count_),
    status_(_status),
    apply_loss_function_(_apply_loss_function),
    frame_other_ptr_(), // nullptr
    feature_other_ptr_(), // nullptr
    landmark_other_ptr_(), // nullptr
    processor_ptr_() // nullptr
{
//    std::cout << "constructed        +c" << id() << std::endl;
}

ConstraintBase::ConstraintBase(const std::string&  _tp,
                               const FrameBasePtr& _frame_other_ptr,
                               const CaptureBasePtr& _capture_other_ptr,
                               const FeatureBasePtr& _feature_other_ptr,
                               const LandmarkBasePtr& _landmark_other_ptr,
                               const ProcessorBasePtr& _processor_ptr,
                               bool _apply_loss_function, ConstraintStatus _status) :
    NodeBase("CONSTRAINT", _tp),
    feature_ptr_(),
    constraint_id_(++constraint_id_count_),
    status_(_status),
    apply_loss_function_(_apply_loss_function),
    frame_other_ptr_(_frame_other_ptr),
    capture_other_ptr_(_capture_other_ptr),
    feature_other_ptr_(_feature_other_ptr),
    landmark_other_ptr_(_landmark_other_ptr),
    processor_ptr_(_processor_ptr)
{
//    std::cout << "constructed        +c" << id() << std::endl;
}

void ConstraintBase::remove()
{
    if (!is_removing_)
    {
        is_removing_ = true;
        ConstraintBasePtr this_c = shared_from_this(); // keep this alive while removing it
        FeatureBasePtr f = feature_ptr_.lock();
        if (f)
        {
            f->getConstraintList().remove(shared_from_this()); // remove from upstream
            if (f->getConstraintList().empty() && f->getConstrainedByList().empty())
                f->remove(); // remove upstream
        }
        // add constraint to be removed from solver
        if (getProblem() != nullptr)
            getProblem()->removeConstraint(shared_from_this());

        // remove other: {Frame, Capture, Feature, Landmark}
        FrameBasePtr frm_o = frame_other_ptr_.lock();
        if (frm_o)
        {
            frm_o->getConstrainedByList().remove(shared_from_this());
            if (frm_o->getConstrainedByList().empty() && frm_o->getCaptureList().empty())
                frm_o->remove();
        }

        CaptureBasePtr cap_o = capture_other_ptr_.lock();
        if (cap_o)
        {
            cap_o->getConstrainedByList().remove(shared_from_this());
            if (cap_o->getConstrainedByList().empty() && cap_o->getFeatureList().empty())
                cap_o->remove();
        }

        FeatureBasePtr ftr_o = feature_other_ptr_.lock();
        if (ftr_o)
        {
            ftr_o->getConstrainedByList().remove(shared_from_this());
            if (ftr_o->getConstrainedByList().empty() && ftr_o->getConstraintList().empty())
                ftr_o->remove();
        }

        LandmarkBasePtr lmk_o = landmark_other_ptr_.lock();
        if (lmk_o)
        {
            lmk_o->getConstrainedByList().remove(shared_from_this());
            if (lmk_o->getConstrainedByList().empty())
                lmk_o->remove();
        }

        //        std::cout << "Removed             c" << id() << std::endl;
    }
}

const Eigen::VectorXs& ConstraintBase::getMeasurement() const
{
    return getFeaturePtr()->getMeasurement();
}

const Eigen::MatrixXs& ConstraintBase::getMeasurementCovariance() const
{
    return getFeaturePtr()->getMeasurementCovariance();
}

const Eigen::MatrixXs& ConstraintBase::getMeasurementSquareRootInformationUpper() const
{
    return getFeaturePtr()->getMeasurementSquareRootInformationUpper();
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
            getProblem()->addConstraint(shared_from_this());
        else if (_status == CTR_INACTIVE)
            getProblem()->removeConstraint(shared_from_this());
    }
    status_ = _status;
}

void ConstraintBase::setApplyLossFunction(const bool _apply)
{
    if (apply_loss_function_ != _apply)
    {
        if (getProblem() == nullptr)
            std::cout << "constraint not linked with Problem, apply loss function change not notified" << std::endl;
        else
        {
            ConstraintBasePtr this_c = shared_from_this();
            getProblem()->removeConstraint(this_c);
            getProblem()->addConstraint(this_c);
        }
    }
}

} // namespace wolf
