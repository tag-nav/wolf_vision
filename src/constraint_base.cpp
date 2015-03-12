#include "constraint_base.h"

ConstraintBase::ConstraintBase(FeatureBase* _ftr_ptr, ConstraintType _tp) :
    NodeLinked(BOTTOM, "CONSTRAINT"),
    type_(_tp),
	measurement_(_ftr_ptr->getMeasurement()),
	measurement_covariance_(_ftr_ptr->getMeasurementCovariance())
{
	//
}

ConstraintBase::~ConstraintBase()
{
	//std::cout << "deleting ConstraintBase " << nodeId() << std::endl;
}

ConstraintType ConstraintBase::getConstraintType() const
{
    return type_;
}

const Eigen::VectorXs& ConstraintBase::getMeasurement()
{
	return measurement_;
}

FeatureBase* ConstraintBase::getFeaturePtr() const
{
	return upperNodePtr();
}

CaptureBase* ConstraintBase::getCapturePtr() const
{
	return upperNodePtr()->upperNodePtr();
}
