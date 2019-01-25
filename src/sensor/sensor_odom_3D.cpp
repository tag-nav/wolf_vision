/**
 * \file sensor_odom_3D.cpp
 *
 *  Created on: Oct 7, 2016
 *      \author: jsola
 */

#include "sensor_odom_3D.h"

#include "state_block.h"
#include "state_quaternion.h"

namespace wolf {

SensorOdom3D::SensorOdom3D(StateBlockPtr _p_ptr, StateQuaternionPtr _o_ptr, IntrinsicsOdom3DPtr _params) :
        SensorBase("ODOM 3D", _p_ptr, _o_ptr, nullptr, 6),
        k_disp_to_disp_(_params->k_disp_to_disp),
        k_disp_to_rot_(_params->k_disp_to_rot),
        k_rot_to_rot_(_params->k_rot_to_rot),
        min_disp_var_(_params->min_disp_var),
        min_rot_var_(_params->min_rot_var)
{
    noise_cov_ = (Eigen::Vector6s() << min_disp_var_, min_disp_var_, min_disp_var_, min_rot_var_, min_rot_var_, min_rot_var_).finished().asDiagonal();
    setNoiseCov(noise_cov_); // sets also noise_std_
}

SensorOdom3D::SensorOdom3D(const Eigen::VectorXs& _extrinsics_pq, const IntrinsicsOdom3D& _intrinsics) :
                        SensorBase("ODOM 3D", std::make_shared<StateBlock>(_extrinsics_pq.head(3), true), std::make_shared<StateQuaternion>(_extrinsics_pq.tail(4), true), nullptr, 6),
                        k_disp_to_disp_(_intrinsics.k_disp_to_disp),
                        k_disp_to_rot_(_intrinsics.k_disp_to_rot),
                        k_rot_to_rot_(_intrinsics.k_rot_to_rot),
                        min_disp_var_(_intrinsics.min_disp_var),
                        min_rot_var_(_intrinsics.min_rot_var)
{
    assert(_extrinsics_pq.size() == 7 && "Bad extrinsics vector size! Should be 7 for 3D.");

    noise_cov_ = (Eigen::Vector6s() << min_disp_var_, min_disp_var_, min_disp_var_, min_rot_var_, min_rot_var_, min_rot_var_).finished().asDiagonal();
    setNoiseCov(noise_cov_); // sets also noise_std_
}

SensorOdom3D::SensorOdom3D(const Eigen::VectorXs& _extrinsics_pq, IntrinsicsOdom3DPtr _intrinsics) :
        SensorOdom3D(_extrinsics_pq, *_intrinsics)
{
    //
}


SensorOdom3D::~SensorOdom3D()
{
    //
}

// Define the factory method
SensorBasePtr SensorOdom3D::create(const std::string& _unique_name, const Eigen::VectorXs& _extrinsics_pq,
                                 const IntrinsicsBasePtr _intrinsics)
{
    // decode extrinsics vector
    assert(_extrinsics_pq.size() == 7 && "Bad extrinsics vector length. Should be 7 for 3D.");

    // cast intrinsics into derived type
    IntrinsicsOdom3DPtr params = std::static_pointer_cast<IntrinsicsOdom3D>(_intrinsics);

    // Call constructor and finish
    SensorBasePtr odo = std::make_shared<SensorOdom3D>(_extrinsics_pq, params);
    odo->setName(_unique_name);

    return odo;
}

} // namespace wolf


// Register in the SensorFactory
#include "sensor_factory.h"
namespace wolf {
WOLF_REGISTER_SENSOR("ODOM 3D", SensorOdom3D)
}
