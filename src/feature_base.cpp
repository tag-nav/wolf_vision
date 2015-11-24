#include "feature_base.h"

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
	//
}

FeatureBase::~FeatureBase()
{
	//std::cout << "deleting FeatureBase " << nodeId() << std::endl;
    is_deleting_ = true;

    while (!constraint_to_list_.empty())
    {
        //std::cout << "deleting constraint " << (*constraints_list_.begin())->nodeId() << std::endl;
        constraint_to_list_.front()->destruct();
        //std::cout << "deleted " << std::endl;
    }
    //std::cout << "constraints deleted" << std::endl;
}

void FeatureBase::addConstraintFrom(ConstraintBase* _co_ptr)
{
    addDownNode(_co_ptr);
    // add constraint to be added in solver
    getTop()->addConstraintPtr(_co_ptr);
}

void FeatureBase::addConstraintTo(ConstraintBase* _ctr_ptr)
{
    constraint_to_list_.push_back(_ctr_ptr);
}

void FeatureBase::removeConstraintTo(ConstraintBase* _ctr_ptr)
{
    constraint_to_list_.remove(_ctr_ptr);
}

unsigned int FeatureBase::getHits() const
{
    return constraint_to_list_.size();
}

std::list<ConstraintBase*>* FeatureBase::getConstraintToListPtr()
{
    return &constraint_to_list_;
}

CaptureBase* FeatureBase::getCapturePtr() const
{
    return upperNodePtr();    
}

FrameBase* FeatureBase::getFramePtr() const
{
    return upperNodePtr()->upperNodePtr();
}

ConstraintBaseList* FeatureBase::getConstraintFromListPtr()
{
    return getDownNodeListPtr();
}

void FeatureBase::getConstraintFromList(ConstraintBaseList & _ctr_list)
{
	for(auto c_it = getConstraintFromListPtr()->begin(); c_it != getConstraintFromListPtr()->end(); ++c_it)
		_ctr_list.push_back((*c_it));
}

//Eigen::VectorXs * FeatureBase::getMeasurementPtr()
//{
//    return &measurement_;
//}
//
//Eigen::MatrixXs * FeatureBase::getMeasurementCovariancePtr()
//{
//    return &measurement_covariance_;
//}

const Eigen::VectorXs& FeatureBase::getMeasurement() const
{
    return measurement_;
}

WolfScalar FeatureBase::getMeasurement(unsigned int _ii) const
{
    return measurement_(_ii);
}

const Eigen::MatrixXs& FeatureBase::getMeasurementCovariance() const
{
    return measurement_covariance_;
}

void FeatureBase::setMeasurement(const Eigen::VectorXs & _meas)
{
    measurement_ = _meas;
}

void FeatureBase::setMeasurementCovariance(const Eigen::MatrixXs & _meas_cov)
{
	measurement_covariance_ = _meas_cov;
}

void FeatureBase::printSelf(unsigned int _ntabs, std::ostream & _ost) const
{
    NodeLinked::printSelf(_ntabs, _ost);
    printTabs(_ntabs);
    _ost << "\tMeasurement: ( " << measurement_.transpose() << " )" << std::endl;
    printTabs(_ntabs);
    _ost << "\tMeasurement covariance: ( " << measurement_covariance_ << " )" << std::endl;
}
