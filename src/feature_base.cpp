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
}

void FeatureBase::addConstraint(ConstraintBase* _co_ptr)
{
    addDownNode(_co_ptr);
}

CaptureBase* FeatureBase::getCapturePtr() const
{
    return upperNodePtr();    
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
