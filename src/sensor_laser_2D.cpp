#include "sensor_laser_2D.h"
#include "state_block.h"

namespace wolf {

SensorLaser2D::SensorLaser2D(StateBlockPtr _p_ptr, StateBlockPtr _o_ptr) :
    SensorBase("LASER 2D", _p_ptr, _o_ptr, nullptr, 8)
{
    setDefaultScanParams();
}

SensorLaser2D::SensorLaser2D(StateBlockPtr _p_ptr, StateBlockPtr _o_ptr, const double& _angle_min, const double& _angle_max, const double& _angle_step, const double& _scan_time, const double& _range_min, const double& _range_max, const double& _range_std_dev, const double& _angle_std_dev) :
        SensorBase("LASER 2D", _p_ptr, _o_ptr, nullptr, 8),
        scan_params_({ _angle_min, _angle_max, _angle_step, _scan_time, _range_min, _range_max, _range_std_dev, _angle_std_dev })
{
    //
}

SensorLaser2D::SensorLaser2D(StateBlockPtr _p_ptr, StateBlockPtr _o_ptr, const laserscanutils::LaserScanParams& _params) :
        SensorBase("LASER 2D", _p_ptr, _o_ptr, nullptr, 8),
        scan_params_(_params)
{
    //
}

SensorLaser2D::~SensorLaser2D()
{
    //
}

void SensorLaser2D::setDefaultScanParams()
{
    scan_params_.angle_min_ = M_PI_2;
    scan_params_.angle_max_ = -M_PI_2;
    scan_params_.angle_step_ = -M_PI/720;
    scan_params_.scan_time_ = 0.01;//not relevant
    scan_params_.range_min_ = 0.2;
    scan_params_.range_max_ = 100;
    scan_params_.range_std_dev_ = 0.01;
    
    setNoise(Eigen::VectorXs::Constant(1,scan_params_.range_std_dev_));
}

void SensorLaser2D::setScanParams(const laserscanutils::LaserScanParams & _params)
{
    scan_params_ = _params;
}

const laserscanutils::LaserScanParams& SensorLaser2D::getScanParams() const
{
    return scan_params_;
}

// Define the factory method
SensorBasePtr SensorLaser2D::create(const std::string& _unique_name, const Eigen::VectorXs& _extrinsics_po,
                                  const IntrinsicsBasePtr _intrinsics)
{
    // decode extrinsics vector
    assert(_extrinsics_po.size() == 3 && "Bad extrinsics vector length. Should be 3 for 2D.");
    StateBlockPtr pos_ptr = std::make_shared<StateBlock>(_extrinsics_po.head(2), true);
    StateBlockPtr ori_ptr = std::make_shared<StateBlock>(_extrinsics_po.tail(1), true);
    // cast intrinsics into derived type
    IntrinsicsLaser2DPtr params = std::static_pointer_cast<IntrinsicsLaser2D>(_intrinsics);
    SensorLaser2DPtr sen = std::make_shared<SensorLaser2D>(pos_ptr, ori_ptr, params->scan_params);
    sen->setName(_unique_name);
    return sen;
}


} // namespace wolf




// Register in the SensorFactory and the ParameterFactory
#include "sensor_factory.h"
//#include "intrinsics_factory.h"
namespace wolf {
WOLF_REGISTER_SENSOR("LASER 2D", SensorLaser2D)
//const bool registered_laser_params = IntrinsicsFactory::get().registerCreator("LASER 2D", createIntrinsicsLaser2D);
} // namespace wolf
