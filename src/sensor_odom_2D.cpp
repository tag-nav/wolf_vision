#include "sensor_odom_2D.h"
#include "state_block.h"
#include "state_angle.h"

namespace wolf {

SensorOdom2D::SensorOdom2D(StateBlockPtr _p_ptr, StateBlockPtr _o_ptr, const Scalar& _disp_noise_factor, const Scalar&  _rot_noise_factor) :
        SensorBase("ODOM 2D", _p_ptr, _o_ptr, nullptr, 2),
        k_disp_to_disp_(_disp_noise_factor),
        k_rot_to_rot_(_rot_noise_factor)
{
    //
}

SensorOdom2D::SensorOdom2D(Eigen::VectorXs _extrinsics, const IntrinsicsOdom2D& _intrinsics) :
        SensorBase("ODOM 2D", std::make_shared<StateBlock>(_extrinsics.head(2), true), std::make_shared<StateAngle>(_extrinsics(2), true), nullptr, 2),
        k_disp_to_disp_(_intrinsics.k_disp_to_disp),
        k_rot_to_rot_(_intrinsics.k_rot_to_rot)
{
    assert(_extrinsics.size() == 3 && "Wrong extrinsics vector size! Should be 3 for 2D.");
    //
}

SensorOdom2D::SensorOdom2D(Eigen::VectorXs _extrinsics, IntrinsicsOdom2DPtr _intrinsics) :
        SensorOdom2D(_extrinsics, *_intrinsics)
{
    //
}


SensorOdom2D::~SensorOdom2D()
{
    //
}

Scalar SensorOdom2D::getDispVarToDispNoiseFactor() const
{
    return k_disp_to_disp_;
}

Scalar SensorOdom2D::getRotVarToRotNoiseFactor() const
{
    return k_rot_to_rot_;
}

// Define the factory method
SensorBasePtr SensorOdom2D::create(const std::string& _unique_name, const Eigen::VectorXs& _extrinsics_po,
                                 const IntrinsicsBasePtr _intrinsics)
{
    // decode extrinsics vector
    assert(_extrinsics_po.size() == 3 && "Bad extrinsics vector length. Should be 3 for 2D.");

    SensorOdom2DPtr odo;
    if (_intrinsics)
    {
        std::shared_ptr<IntrinsicsOdom2D> params = std::static_pointer_cast<IntrinsicsOdom2D>(_intrinsics);
        odo = std::make_shared<SensorOdom2D>(_extrinsics_po, params);
    }
    else
    {
        IntrinsicsOdom2D params;
        params.k_disp_to_disp = 1;
        params.k_rot_to_rot   = 1;
        odo = std::make_shared<SensorOdom2D>(_extrinsics_po, params);
    }
    odo->setName(_unique_name);
    return odo;
}

} // namespace wolf


// Register in the SensorFactory
#include "sensor_factory.h"
namespace wolf {
WOLF_REGISTER_SENSOR("ODOM 2D", SensorOdom2D)
} // namespace wolf
