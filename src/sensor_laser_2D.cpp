#include "sensor_laser_2D.h"
#include "state_block.h"

namespace wolf {

SensorLaser2D::SensorLaser2D(StateBlock* _p_ptr, StateBlock* _o_ptr) :
    SensorBase(SEN_LIDAR, _p_ptr, _o_ptr, nullptr, 8)
{
    setType("LASER 2D");
    setDefaultScanParams();
}

SensorLaser2D::SensorLaser2D(StateBlock* _p_ptr, StateBlock* _o_ptr, const double& _angle_min, const double& _angle_max, const double& _angle_step, const double& _scan_time, const double& _range_min, const double& _range_max, const double& _range_std_dev, const double& _angle_std_dev) :
        SensorBase(SEN_LIDAR, _p_ptr, _o_ptr, nullptr, 8),
        scan_params_({ _angle_min, _angle_max, _angle_step, _scan_time, _range_min, _range_max, _range_std_dev, _angle_std_dev })
{
    setType("LASER 2D");
}

SensorLaser2D::SensorLaser2D(StateBlock* _p_ptr, StateBlock* _o_ptr, const laserscanutils::LaserScanParams& _params) :
        SensorBase(SEN_LIDAR, _p_ptr, _o_ptr, nullptr, 8),
        scan_params_(_params)
{
    setType("LASER 2D");
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
SensorBase* SensorLaser2D::create(const std::string& _unique_name, const Eigen::VectorXs& _extrinsics_po,
                                  const IntrinsicsBase* _intrinsics)
{
    // decode extrinsics vector
    assert(_extrinsics_po.size() == 3 && "Bad extrinsics vector length. Should be 3 for 2D.");
    StateBlock* pos_ptr = new StateBlock(_extrinsics_po.head(2), true);
    StateBlock* ori_ptr = new StateBlock(_extrinsics_po.tail(1), true);
    // cast intrinsics into derived type
    IntrinsicsLaser2D* params = (IntrinsicsLaser2D*)(_intrinsics);
    SensorLaser2D* sen = new SensorLaser2D(pos_ptr, ori_ptr, params->scan_params);
    sen->setName(_unique_name);
    return sen;
}

} // namespace wolf


// Register in the SensorFactory
#include "sensor_factory.h"
namespace wolf {
namespace
{
const bool registered_laser = SensorFactory::get()->registerCreator("LASER 2D", SensorLaser2D::create);
}
} // namespace wolf
