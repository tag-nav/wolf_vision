#include "feature_base.h"
#include "constraint_base.h"
#include "capture_base.h"

FeatureBase::FeatureBase(unsigned int _dim_measurement) :
    NodeLinked(MID, "FEATURE"),
    measurement_(_dim_measurement)
{
    //
}

FeatureBase::FeatureBase(const Eigen::VectorXs& _measurement, const Eigen::MatrixXs& _meas_covariance) :
	NodeLinked(MID, "FEATURE"),
	measurement_(_measurement),
	measurement_covariance_(_meas_covariance)
{
    Eigen::LLT<Eigen::MatrixXs> lltOfA(measurement_covariance_); // compute the Cholesky decomposition of A
    Eigen::MatrixXs measurement_sqrt_covariance = lltOfA.matrixU();
    measurement_sqrt_information_ = measurement_sqrt_covariance.inverse(); // retrieve factor U  in the decomposition
}

FeatureBase::~FeatureBase()
{
	//std::cout << "deleting FeatureBase " << nodeId() << std::endl;
    is_deleting_ = true;

    while (!constrained_by_list_.empty())
    {
        //std::cout << "destruct() constraint " << (*constraint_to_list_.begin())->nodeId() << std::endl;
        constrained_by_list_.front()->destruct();
        //std::cout << "deleted " << std::endl;
    }
    //std::cout << "constraints deleted" << std::endl;
}

void FeatureBase::addConstraint(ConstraintBase* _co_ptr)
{
    addDownNode(_co_ptr);
    // add constraint to be added in solver
    getTop()->addConstraintPtr(_co_ptr);
}

void FeatureBase::addConstrainedBy(ConstraintBase* _ctr_ptr)
{
    constrained_by_list_.push_back(_ctr_ptr);
}

void FeatureBase::removeConstrainedBy(ConstraintBase* _ctr_ptr)
{
    constrained_by_list_.remove(_ctr_ptr);
}

unsigned int FeatureBase::getHits() const
{
    return constrained_by_list_.size();
}

FrameBase* FeatureBase::getFramePtr() const
{
    return upperNodePtr()->upperNodePtr();
}

ConstraintBaseList* FeatureBase::getConstraintListPtr()
{
    return getDownNodeListPtr();
}

void FeatureBase::getConstraintList(ConstraintBaseList & _ctr_list)
{
	for(auto c_it = getConstraintListPtr()->begin(); c_it != getConstraintListPtr()->end(); ++c_it)
		_ctr_list.push_back((*c_it));
}

void FeatureBase::setMeasurementCovariance(const Eigen::MatrixXs & _meas_cov)
{
	measurement_covariance_ = _meas_cov;
	Eigen::LLT<Eigen::MatrixXs> lltOfA(measurement_covariance_); // compute the Cholesky decomposition of A
    Eigen::MatrixXs measurement_sqrt_covariance = lltOfA.matrixU();
    measurement_sqrt_information_ = measurement_sqrt_covariance.inverse(); // retrieve factor U  in the decomposition
}

