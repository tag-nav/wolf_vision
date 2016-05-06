#include "feature_base.h"
#include "constraint_base.h"
#include "capture_base.h"

namespace wolf {

unsigned int FeatureBase::feature_id_count_ = 0;

FeatureBase::FeatureBase(FeatureType _tp, unsigned int _dim_measurement) :
    NodeConstrained(MID, "FEATURE"),
    feature_id_(++feature_id_count_),
    type_id_(_tp),
    measurement_(_dim_measurement)
{
    //
}

FeatureBase::FeatureBase(FeatureType _tp, const Eigen::VectorXs& _measurement, const Eigen::MatrixXs& _meas_covariance) :
	NodeConstrained(MID, "FEATURE"),
    feature_id_(++feature_id_count_),
    type_id_(_tp),
	measurement_(_measurement),
	measurement_covariance_(_meas_covariance)
{
    Eigen::LLT<Eigen::MatrixXs> lltOfA(measurement_covariance_); // compute the Cholesky decomposition of A
    Eigen::MatrixXs measurement_sqrt_covariance = lltOfA.matrixU();
    measurement_sqrt_information_ = measurement_sqrt_covariance.inverse().transpose(); // retrieve factor U  in the decomposition
}

FeatureBase::~FeatureBase()
{
	//std::cout << "deleting FeatureBase " << nodeId() << std::endl;
    is_deleting_ = true;

    while (!getConstrainedByListPtr()->empty())
    {
        //std::cout << "destruct() constraint " << (*constrained_by_list_.begin())->nodeId() << std::endl;
        getConstrainedByListPtr()->front()->destruct();
        //std::cout << "deleted " << std::endl;
    }
    //std::cout << "constraints deleted" << std::endl;
}

ConstraintBase* FeatureBase::addConstraint(ConstraintBase* _co_ptr)
{
    addDownNode(_co_ptr);
    // add constraint to be added in solver
    if (getProblem() != nullptr)
        getProblem()->addConstraintPtr(_co_ptr);

    return _co_ptr;
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


} // namespace wolf
