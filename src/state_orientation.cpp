
#include "state_orientation.h"

StateOrientation::StateOrientation(Eigen::VectorXs& _st_remote, const unsigned int _idx) :
	StateBase(_st_remote, _idx)
{
	//
}


StateOrientation::StateOrientation(WolfScalar* _st_ptr) :
	StateBase(_st_ptr)
{
	//
}

StateOrientation::~StateOrientation()
{
	//
}

Eigen::Matrix3s StateOrientation::getRotationMatrix() const
{
    Eigen::Matrix3s R(Eigen::Matrix3s::Identity());
    rotationMatrix(R); 
    return R;
}
