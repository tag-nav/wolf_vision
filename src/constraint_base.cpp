#include "constraint_base.h"

ConstraintBase::ConstraintBase(FeatureBase* _ftr_ptr, ConstraintType _tp) :
    NodeLinked(BOTTOM, "CONSTRAINT"),
    type_(_tp),
	measurement_(_ftr_ptr->getMeasurement()),
	measurement_covariance_(_ftr_ptr->getMeasurementCovariance()),
	pending_status_(ADD_PENDING)
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

PendingStatus ConstraintBase::getPendingStatus() const
{
	return pending_status_;
}

void ConstraintBase::setPendingStatus(PendingStatus _pending)
{
	pending_status_ = _pending;
}
