
#include "state_complex_angle.h"

StateComplexAngle::StateComplexAngle(Eigen::VectorXs& _st_remote, const unsigned int _idx) :
		StateOrientation(_st_remote, _idx)
{
	//
}

StateComplexAngle::StateComplexAngle(WolfScalar* _st_ptr) :
		StateOrientation(_st_ptr)
{
	//
}

StateComplexAngle::~StateComplexAngle()
{
	//
}

StateType StateComplexAngle::getStateType() const
{
	return ST_COMPLEX_ANGLE;
}

unsigned int StateComplexAngle::getStateSize() const
{
	return BLOCK_SIZE;
}

void StateComplexAngle::rotationMatrix(Eigen::Matrix3s& R) const
{
	R = Eigen::Matrix3s::Identity();

    R(0,0) = *state_ptr_;
    R(1,0) = *(state_ptr_+1);
    
    R(0,1) = -*(state_ptr_+1);
	R(1,1) = *state_ptr_;
}

Eigen::Map<const Eigen::VectorXs> StateComplexAngle::getVector() const
{
    return Eigen::Map<const Eigen::VectorXs>(state_ptr_, BLOCK_SIZE);
}

WolfScalar StateComplexAngle::getYaw() const
{
    return atan2(*(state_ptr_+1), *state_ptr_);
}

void StateComplexAngle::print(unsigned int _ntabs, std::ostream& _ost) const
{
	printTabs(_ntabs);
	_ost << nodeLabel() << " " << nodeId() << std::endl;
	printTabs(_ntabs);
	_ost << *state_ptr_<< " " << *(state_ptr_+1) << std::endl;
}
