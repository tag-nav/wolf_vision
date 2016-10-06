#include "feature_base.h"
#include "constraint_base.h"
#include "capture_base.h"

namespace wolf {

unsigned int FeatureBase::feature_id_count_ = 0;

FeatureBase::FeatureBase(FeatureType _tp, const std::string& _type, unsigned int _dim_measurement) :
    NodeBase("FEATURE", _type),
    problem_ptr_(nullptr),
    capture_ptr_(nullptr),
    feature_id_(++feature_id_count_),
    track_id_(0),
    landmark_id_(0),
    type_id_(_tp),
    measurement_(_dim_measurement)
{
    //
}

FeatureBase::FeatureBase(FeatureType _tp, const std::string& _type, const Eigen::VectorXs& _measurement, const Eigen::MatrixXs& _meas_covariance) :
	NodeBase("FEATURE", _type),
    problem_ptr_(nullptr),
    capture_ptr_(nullptr),
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
}

FeatureBase::~FeatureBase()
{
	//std::cout << "deleting FeatureBase " << nodeId() << std::endl;
    is_deleting_ = true;

    while (!constrained_by_list_.empty())
    {
        //std::cout << "destruct() constraint " << (*constrained_by_list_.begin())->nodeId() << std::endl;
        constrained_by_list_.front()->destruct();
        //std::cout << "deleted " << std::endl;
    }
    //std::cout << "constraints deleted" << std::endl;

    while (!constraint_list_.empty())
    {
        delete constraint_list_.front();
        constraint_list_.pop_front();
    }

}

void FeatureBase::setProblem(ProblemPtr _prob_ptr)
{
    problem_ptr_ = _prob_ptr;
}



ConstraintBasePtr FeatureBase::addConstraint(ConstraintBasePtr _co_ptr)
{
    constraint_list_.push_back(_co_ptr);
    _co_ptr->setFeaturePtr(this);
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
    return capture_ptr_->getFramePtr();
}

ConstraintBaseList* FeatureBase::getConstraintListPtr()
{
    return & constraint_list_;
}

void FeatureBase::getConstraintList(ConstraintBaseList & _ctr_list)
{
	for(ConstraintBasePtr c_ptr : *getConstraintListPtr())
		_ctr_list.push_back(c_ptr);
}

void FeatureBase::destruct()
{
    if (!is_deleting_)
    {
        if (capture_ptr_ != nullptr) // && !up_node_ptr_->isTop())
        {
            //std::cout << "upper node is not WolfProblem " << std::endl;
            capture_ptr_->removeFeature(this);
        }
        else
        {
            //std::cout << "upper node is WolfProblem or nullptr" << std::endl;
            delete this;
        }
    }
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
