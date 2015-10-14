#include "state_theta.h"

StateTheta::StateTheta(Eigen::VectorXs& _st_remote, const unsigned int _idx) :
        StateOrientation(_st_remote, _idx)
{
    //
}

StateTheta::StateTheta(WolfScalar* _st_ptr) :
        StateOrientation(_st_ptr)
{
    //
}

StateTheta::~StateTheta()
{
    //
}

StateType StateTheta::getStateType() const
{
    return ST_THETA;
}

unsigned int StateTheta::getStateSize() const
{
    return BLOCK_SIZE;
}

// Eigen::Matrix3s StateTheta::getRotationMatrix() const
// {
//     Eigen::Matrix3s R(Eigen::Matrix3s::Identity());
//     this->getRotationMatrix(R); 
//     return R;
// }

void StateTheta::rotationMatrix(Eigen::Matrix3s& R) const
{
    R = Eigen::Matrix3s::Identity();
    R.block<2,2>(0,0) = Eigen::Rotation2D<WolfScalar>(*state_ptr_).matrix();
//    R(0, 0) = cos(*state_ptr_);
//    R(1, 1) = cos(*state_ptr_);
//    R(0, 1) = -sin(*state_ptr_);
//    R(1, 0) = sin(*state_ptr_);
    //std::cout << "StateTheta::getRotationMatrix()" << R << std::endl;
}

WolfScalar StateTheta::getYaw() const
{
    return *state_ptr_;
}

Eigen::Map<const Eigen::VectorXs> StateTheta::getVector() const
{
    return Eigen::Map<const Eigen::VectorXs>(state_ptr_, 1);
}

void StateTheta::print(unsigned int _ntabs, std::ostream& _ost) const
{
    printTabs(_ntabs);
    _ost << nodeLabel() << " " << nodeId() << std::endl;
    printTabs(_ntabs);
    for (unsigned int i = 0; i < BLOCK_SIZE; i++)
        _ost << *(this->state_ptr_ + i) << " ";
    _ost << std::endl;
}
