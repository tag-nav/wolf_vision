#include "feature_base.h"
#include "constraint_base.h"
#include "capture_base.h"

namespace wolf {

unsigned int FeatureBase::feature_id_count_ = 0;

FeatureBase::FeatureBase(const std::string& _type, const Eigen::VectorXs& _measurement, const Eigen::MatrixXs& _meas_covariance) :
	NodeBase("FEATURE", _type),
    capture_ptr_(),
    is_removing_(false),
    feature_id_(++feature_id_count_),
    track_id_(0),
    landmark_id_(0),
	measurement_(_measurement)
{
    setMeasurementCovariance(_meas_covariance);
//    std::cout << "constructed      +f" << id() << std::endl;
}

FeatureBase::~FeatureBase()
{
//    std::cout << "destructed       -f" << id() << std::endl;
}

void FeatureBase::remove()
{
    if (!is_removing_)
    {
        is_removing_ = true;
        FeatureBasePtr this_f = shared_from_this(); // keep this alive while removing it
        CaptureBasePtr C = capture_ptr_.lock();
        if (C)
        {
            C->getFeatureList().remove(this_f); // remove from upstream
            if (C->getFeatureList().empty())
                C->remove(); // remove upstream
        }
        while (!constraint_list_.empty())
        {
            constraint_list_.front()->remove(); // remove downstream
        }
        while (!constrained_by_list_.empty())
        {
            constrained_by_list_.front()->remove(); // remove constrained
        }
//        std::cout << "Removed           f" << id() << std::endl;
    }
}

ConstraintBasePtr FeatureBase::addConstraint(ConstraintBasePtr _co_ptr)
{
    constraint_list_.push_back(_co_ptr);
    _co_ptr->setFeaturePtr(shared_from_this());
    _co_ptr->setProblem(getProblem());
    // add constraint to be added in solver
    if (getProblem() != nullptr)
        getProblem()->addConstraintPtr(_co_ptr);
    else
        std::cout << "WARNING: ADDING CONSTRAINT TO A FEATURE NOT CONNECTED WITH PROBLEM." << std::endl;
    return _co_ptr;
}

FrameBasePtr FeatureBase::getFramePtr() const
{
    return capture_ptr_.lock()->getFramePtr();
}

ConstraintBaseList& FeatureBase::getConstraintList()
{
    return constraint_list_;
}

void FeatureBase::getConstraintList(ConstraintBaseList & _ctr_list)
{
    _ctr_list.insert(_ctr_list.end(), constraint_list_.begin(), constraint_list_.end());
}

void FeatureBase::setMeasurementCovariance(const Eigen::MatrixXs & _meas_cov)
{
    if (_meas_cov.determinant() < Constants::EPS_SMALL)
    {
        Eigen::MatrixXs eps = Eigen::MatrixXs::Identity(_meas_cov.rows(), _meas_cov.cols()) * 1e-10;
        measurement_covariance_ = _meas_cov + eps; // avoid singular covariance
    }
    else
        measurement_covariance_ = _meas_cov;

	measurement_sqrt_information_upper_ = computeSqrtUpper(_meas_cov);
}

void FeatureBase::setMeasurementInfo(const Eigen::MatrixXs & _meas_info)
{
    assert(_meas_info.determinant() > 0 && "Not positive definite measurement information");

    measurement_covariance_ = _meas_info.inverse();
    measurement_sqrt_information_upper_ = computeSqrtUpper(_meas_info.inverse());
}

Eigen::MatrixXs FeatureBase::computeSqrtUpper(const Eigen::MatrixXs & _covariance) const
{
    if (_covariance.determinant() < Constants::EPS_SMALL)
    {
        // Avoid singular covariances matrix
        Eigen::MatrixXs cov = _covariance + 1e-8 * Eigen::MatrixXs::Identity(_covariance.rows(), _covariance.cols());
        Eigen::LLT<Eigen::MatrixXs> llt_of_info(cov.inverse());
        return llt_of_info.matrixU();
    }
    else
    {
        // Normal operation
        Eigen::LLT<Eigen::MatrixXs> llt_of_info(_covariance.inverse());
        return llt_of_info.matrixU();
    }
}

} // namespace wolf
