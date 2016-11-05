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

SensorOdom3D::SensorOdom3D(StateBlockPtr _p_ptr, StateQuaternionPtr _o_ptr, IntrinsicsOdom3D::Ptr params) :
        SensorBase("ODOM 3D", _p_ptr, _o_ptr, nullptr, 6),
        k_disp_to_disp_(params->k_disp_to_disp),
        k_disp_to_rot_(params->k_disp_to_rot),
        k_rot_to_rot_(params->k_rot_to_rot),
        min_disp_var_(params->min_disp_var),
        min_rot_var_(params->min_rot_var)
{
    //
}

SensorOdom3D::~SensorOdom3D()
{
    //
}

// Define the factory method
SensorBasePtr SensorOdom3D::create(const std::string& _unique_name, const Eigen::VectorXs& _extrinsics_po,
                                 const IntrinsicsBasePtr _intrinsics)
{
    // decode extrinsics vector
    assert(_extrinsics_po.size() == 7 && "Bad extrinsics vector length. Should be 7 for 3D.");
    StateBlockPtr pos_ptr = std::make_shared<StateBlock>(_extrinsics_po.head(3), true);
    StateQuaternionPtr ori_ptr = std::make_shared<StateQuaternion>(_extrinsics_po.tail(4), true);
    // cast intrinsics into derived type
    IntrinsicsOdom3D::Ptr params = std::static_pointer_cast<IntrinsicsOdom3D>(_intrinsics);
    SensorBasePtr odo = std::make_shared<SensorOdom3D>(pos_ptr, ori_ptr, params);
    odo->setName(_unique_name);
    return odo;
}

} // namespace wolf


// Register in the SensorFactory
#include "sensor_factory.h"
namespace wolf {
WOLF_REGISTER_SENSOR("ODOM 3D", SensorOdom3D)
}
