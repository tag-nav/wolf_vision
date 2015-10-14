#include "state_quaternion.h"

StateQuaternion::StateQuaternion(Eigen::VectorXs& _st_remote, const unsigned int _idx) :
        StateOrientation(_st_remote, _idx)
{
    //
}

StateQuaternion::StateQuaternion(WolfScalar* _st_ptr) :
        StateOrientation(_st_ptr)
{
    //
}

StateQuaternion::~StateQuaternion()
{
    //
}

StateType StateQuaternion::getStateType() const
{
    return ST_QUATERNION;
}

unsigned int StateQuaternion::getStateSize() const
{
    return BLOCK_SIZE;
}

// Eigen::Matrix3s StateQuaternion::getRotationMatrix() const
// {
//     Eigen::Matrix3s R(Eigen::Matrix3s::Identity());
//     this->getRotationMatrix(R); 
//     return R;
// }

void StateQuaternion::rotationMatrix(Eigen::Matrix3s& R) const
{
//    WolfScalar qi,qj,qk,qr;
//    qi = *state_ptr_;
//    qj = *(state_ptr_+1);
//    qk = *(state_ptr_+2);
//    qr = *(state_ptr_+3);
//
//    R(0,0) = 1 - 2*qj*qj - 2*qk*qk;
//    R(1,0) = 2*(qi*qj + qk*qr);
//    R(2,0) = 2*(qi*qk - qj*qr);
//
//    R(0,1) = 2*(qi*qj - qk*qr);
//    R(1,1) = 1 - 2*qi*qi - 2*qk*qk;
//    R(2,1) = 2*(qi*qr + qj*qk);
//
//    R(0,2) = 2*(qi*qk + qj*qr);
//    R(1,2) = 2*(qj*qk - qi*qr);
//    R(2,2) = 1 - 2*qi*qi - 2*qj*qj;
    
    R = Eigen::Map<Eigen::Quaternions>(state_ptr_).toRotationMatrix();

    //std::cout << "StateQuaternion::getRotationMatrix()" << R << std::endl;
}

WolfScalar StateQuaternion::getYaw() const
{
    //return atan2(2.0*(qj*qk + qr*qi), qr*qr - qi*qi - qj*qj + qk*qk);
    //return atan2(2.0*(qi*qr + qj*qk), 1 - 2 * (qk*qk + qr*qr));
    //return angles(2);
    return Eigen::Map<Eigen::Quaternions>(state_ptr_).toRotationMatrix().eulerAngles(0, 1, 2)(2);
}


Eigen::Map<const Eigen::VectorXs> StateQuaternion::getVector() const
{
    return Eigen::Map<const Eigen::VectorXs>(state_ptr_, 4);
}

Eigen::Map<const Eigen::Quaternions> StateQuaternion::getQuaternion() const
{
    return Eigen::Map<const Eigen::Quaternion<WolfScalar> >(state_ptr_);
}

void StateQuaternion::normalize()
{
    Eigen::Map<Eigen::Quaternions> q_map(state_ptr_);
    q_map.normalize(); 
}

void StateQuaternion::print(unsigned int _ntabs, std::ostream& _ost) const
{
    printTabs(_ntabs);
    _ost << nodeLabel() << " " << nodeId() << std::endl;
    printTabs(_ntabs);
    for (unsigned int i = 0; i < BLOCK_SIZE; i++)
        _ost << *(this->state_ptr_ + i) << " ";
    _ost << std::endl;
}
