#include "feature_base.h"
#include "constraint_base.h"
#include "capture_base.h"

namespace wolf {

unsigned int FeatureBase::feature_id_count_ = 0;

FeatureBase::FeatureBase(FeatureType _tp, const std::string& _type, unsigned int _dim_measurement) :
    NodeBase("FEATURE", _type),
    capture_ptr_(),
    feature_id_(++feature_id_count_),
    track_id_(0),
    landmark_id_(0),
    type_id_(_tp),
    measurement_(_dim_measurement)
{
    //
    std::cout << "constructed      +f" << id() << std::endl;
}

FeatureBase::FeatureBase(FeatureType _tp, const std::string& _type, const Eigen::VectorXs& _measurement, const Eigen::MatrixXs& _meas_covariance) :
	NodeBase("FEATURE", _type),
    capture_ptr_(),
    feature_id_(++feature_id_count_),
    track_id_(0),
    landmark_id_(0),
    type_id_(_tp),
	measurement_(_measurement),
	measurement_covariance_(_meas_covariance)
{
    assert(_meas_covariance.determinant() > 0 && "Not positive definite measurement covariance");
    Eigen::LLT<Eigen::MatrixXs> lltOfA(measurement_covariance_); // compute the Cholesky decomposition of A
    Eigen::MatrixXs measurement_sqrt_covariance = lltOfA.matrixU();
    measurement_sqrt_information_ = measurement_sqrt_covariance.inverse().transpose(); // retrieve factor U  in the decomposition

    std::cout << "constructed      +f" << id() << std::endl;
}

FeatureBase::~FeatureBase()
{
    std::cout << "destructed       -f" << id() << std::endl;
}

void FeatureBase::remove()
{
    if (!is_removing_)
    {
        is_removing_ = true;
        std::shared_ptr<FeatureBase> this_f = shared_from_this(); // keep this alive while removing it
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
        std::cout << "Removed           f" << id() << std::endl;
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
//	for(ConstraintBasePtr c_ptr : constraint_list_)
//		_ctr_list.push_back(c_ptr);
}

void FeatureBase::setMeasurementCovariance(const Eigen::MatrixXs & _meas_cov)
{
    assert(_meas_cov.determinant() > 0 && "Not positive definite measurement covariance");
	measurement_covariance_ = _meas_cov;
	Eigen::LLT<Eigen::MatrixXs> lltOfA(measurement_covariance_); // compute the Cholesky decomposition of A
    Eigen::MatrixXs measurement_sqrt_covariance = lltOfA.matrixU();
    measurement_sqrt_information_ = measurement_sqrt_covariance.inverse(); // retrieve factor U  in the decomposition
}

} // namespace wolf
