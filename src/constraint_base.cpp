#include "constraint_base.h"

ConstraintBase::ConstraintBase(const FeatureBasePtr& _ftr_ptr, ConstraintType _tp) :
//ConstraintBase::ConstraintBase(ConstraintType _tp) :
//     NodeLinked(BOTTOM, "CORRESPONDENCE", _ftr_ptr),
    NodeLinked(BOTTOM, "CONSTRAINT"),
    type_(_tp),
	measurement_ptr_(_ftr_ptr->getMeasurementPtr()),
	measurement_covariance_ptr_(_ftr_ptr->getMeasurementCovariancePtr())
{
	//
}

ConstraintBase::~ConstraintBase()
{
	//
}

ConstraintType ConstraintBase::getConstraintType() const
{
    return type_;
}

const Eigen::VectorXs * ConstraintBase::getMeasurementPtr()
{
	return upperNodePtr()->getMeasurementPtr();
}

FeatureBasePtr ConstraintBase::getFeaturePtr() const
{
	return upperNodePtr();
}

CaptureBasePtr ConstraintBase::getCapturePtr() const
{
	return upperNodePtr()->upperNodePtr();
}
