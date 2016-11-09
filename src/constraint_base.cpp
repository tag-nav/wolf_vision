#include "constraint_base.h"
#include "frame_base.h"
#include "landmark_base.h"

namespace wolf {

unsigned int ConstraintBase::constraint_id_count_ = 0;

ConstraintBase::ConstraintBase(ConstraintType _tp, bool _apply_loss_function, ConstraintStatus _status) :
    NodeBase("CONSTRAINT", "Base"),
    feature_ptr_(), // nullptr
    constraint_id_(++constraint_id_count_),
    type_id_(_tp),
    category_(CTR_ABSOLUTE),
    status_(_status),
    apply_loss_function_(_apply_loss_function),
    frame_other_ptr_(), // nullptr
    feature_other_ptr_(), // nullptr
    landmark_other_ptr_() // nullptr
{
    //std::cout << "creating ConstraintBase " << std::endl;
    std::cout << "constructed        +c" << id() << std::endl;
}


ConstraintBase::ConstraintBase(ConstraintType _tp, FrameBasePtr _frame_ptr, bool _apply_loss_function, ConstraintStatus _status) :
    NodeBase("CONSTRAINT", "Base"),
    feature_ptr_(),
    constraint_id_(++constraint_id_count_),
    type_id_(_tp),
    category_(CTR_FRAME),
    status_(_status),
    apply_loss_function_(_apply_loss_function),
    frame_other_ptr_(_frame_ptr),
    feature_other_ptr_(),
    landmark_other_ptr_()
{
    std::cout << "constructed        +c" << id() << std::endl;
}


ConstraintBase::ConstraintBase(ConstraintType _tp, FeatureBasePtr _feature_ptr, bool _apply_loss_function, ConstraintStatus _status) :
    NodeBase("CONSTRAINT"),
    feature_ptr_(),
    constraint_id_(++constraint_id_count_),
    type_id_(_tp),
    category_(CTR_FEATURE),
    status_(_status),
    apply_loss_function_(_apply_loss_function),
    frame_other_ptr_(),
    feature_other_ptr_(_feature_ptr),
    landmark_other_ptr_()
{
    std::cout << "constructed        +c" << id() << std::endl;
}


ConstraintBase::ConstraintBase(ConstraintType _tp, LandmarkBasePtr _landmark_ptr, bool _apply_loss_function, ConstraintStatus _status) :
    NodeBase("CONSTRAINT"),
    feature_ptr_(),
    constraint_id_(++constraint_id_count_),
    type_id_(_tp),
    category_(CTR_LANDMARK),
    status_(_status),
    apply_loss_function_(_apply_loss_function),
    frame_other_ptr_(),
    feature_other_ptr_(),
    landmark_other_ptr_(_landmark_ptr)
{
    std::cout << "constructed        +c" << id() << std::endl;
}

ConstraintBase::~ConstraintBase()
{
    std::cout << "destructed         -c" << id() << std::endl;
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
            getProblem()->removeConstraintPtr(shared_from_this());

        // remove other: {Frame, feature, Landmark}
        switch (category_)
        {
            case CTR_FRAME:
            {
                FrameBasePtr frm_o = frame_other_ptr_.lock();
                if (frm_o)
                {
                    frm_o->getConstrainedByList().remove(shared_from_this());
                    if (frm_o->getConstrainedByList().empty() && frm_o->getCaptureList().empty())
                        frm_o->remove();
                }
            }
                break;
            case CTR_FEATURE:
            {
                FeatureBasePtr ftr_o = feature_other_ptr_.lock();
                if (ftr_o)
                {
                    ftr_o->getConstrainedByList().remove(shared_from_this());
                    if (ftr_o->getConstrainedByList().empty() && ftr_o->getConstraintList().empty())
                        ftr_o->remove();
                }
                break;
            }
            case CTR_LANDMARK:
            {
                LandmarkBasePtr lmk_o = landmark_other_ptr_.lock();
                if (lmk_o)
                {
                    lmk_o->getConstrainedByList().remove(shared_from_this());
                    if (lmk_o->getConstrainedByList().empty())
                        lmk_o->remove();
                }
                break;
            }
            case CTR_ABSOLUTE:
                break;
            default:
                break;
        }
        std::cout << "Removed             c" << id() << std::endl;
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
            getProblem()->addConstraintPtr(shared_from_this());
        else if (_status == CTR_INACTIVE)
            getProblem()->removeConstraintPtr(shared_from_this());
    }
    status_ = _status;
}

} // namespace wolf
