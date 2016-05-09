#include "sensor_odom_2D.h"
#include "state_block.h"

namespace wolf {

SensorOdom2D::SensorOdom2D(StateBlock* _p_ptr, StateBlock* _o_ptr, const Scalar& _disp_noise_factor, const Scalar&  _rot_noise_factor) :
        SensorBase(SEN_ODOM_2D, _p_ptr, _o_ptr, nullptr, 2), k_disp_to_disp_(_disp_noise_factor), k_rot_to_rot_(_rot_noise_factor)
{
    setType("ODOM 2D");
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
SensorBase* SensorOdom2D::create(const std::string& _unique_name, const Eigen::VectorXs& _extrinsics_po,
                                 const IntrinsicsBase* _intrinsics)
{
    // decode extrinsics vector
    assert(_extrinsics_po.size() == 3 && "Bad extrinsics vector length. Should be 3 for 2D.");
    StateBlock* pos_ptr = new StateBlock(_extrinsics_po.head(2), true);
    StateBlock* ori_ptr = new StateBlock(_extrinsics_po.tail(1), true);
    // cast intrinsics into derived type
    IntrinsicsOdom2D* params = (IntrinsicsOdom2D*)(_intrinsics);
    SensorBase* odo = new SensorOdom2D(pos_ptr, ori_ptr, params->k_disp_to_disp, params->k_rot_to_rot);
    odo->setName(_unique_name);
    return odo;
}

} // namespace wolf


// Register in the SensorFactory
#include "sensor_factory.h"
namespace wolf {
namespace
{
const bool registered_odom_2d = SensorFactory::get()->registerCreator("ODOM 2D", SensorOdom2D::create);
}
} // namespace wolf
