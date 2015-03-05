#include "feature_base.h"

//FeatureBase::FeatureBase(const CaptureBasePtr& _capt_ptr, unsigned int _dim_measurement) :
FeatureBase::FeatureBase(unsigned int _dim_measurement) :
    //NodeLinked(MID, "FEATURE", _capt_ptr),
    NodeLinked(MID, "FEATURE"),
    measurement_(_dim_measurement)
{
    //
}

//FeatureBase::FeatureBase(const CaptureBasePtr& _capt_ptr, const Eigen::VectorXs& _measurement) :
//	NodeLinked(MID, "FEATURE", _capt_ptr),
//	measurement_(_measurement)
//{
//	//
//}

//FeatureBase::FeatureBase(const CaptureBasePtr& _capt_ptr, const Eigen::VectorXs& _measurement, const Eigen::MatrixXs& _meas_covariance) :
FeatureBase::FeatureBase(const Eigen::VectorXs& _measurement, const Eigen::MatrixXs& _meas_covariance) :
	//NodeLinked(MID, "FEATURE", _capt_ptr),
	NodeLinked(MID, "FEATURE"),
	measurement_(_measurement),
	measurement_covariance_(_meas_covariance)
{
	//
}

FeatureBase::~FeatureBase()
{
    //
}

//inline void FeatureBase::linkToCapture(const CaptureBaseShPtr& _capt_ptr)
//{
//    linkToUpperNode(_capt_ptr.get());
//}

void FeatureBase::addConstraint(ConstraintBaseShPtr& _co_ptr)
{
    addDownNode(_co_ptr);
}

const CaptureBasePtr FeatureBase::getCapturePtr() const
{
    return upperNodePtr();    
}

const FrameBasePtr FeatureBase::getFramePtr() const
{
    return upperNodePtr()->upperNodePtr();
}

// inline const ConstraintBaseList & FeatureBase::getConstraintList() const
// {
//     return downNodeList();
// }

ConstraintBaseList* FeatureBase::getConstraintListPtr()
{
    return getDownNodeListPtr();
}

void FeatureBase::getConstraintList(ConstraintBasePtrList & _ctr_list)
{
	for(auto c_it = getConstraintListPtr()->begin(); c_it != getConstraintListPtr()->end(); ++c_it)
		_ctr_list.push_back((*c_it).get());
}

Eigen::VectorXs * FeatureBase::getMeasurementPtr()
{
    return &measurement_;
}

Eigen::MatrixXs * FeatureBase::getMeasurementCovariancePtr()
{
    return &measurement_covariance_;
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
